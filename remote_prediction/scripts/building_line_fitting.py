"""Constrained piecewise total-least-squares fitting of polygon boundaries.

No SPT/torch imports: this module only processes already predicted geometry.
"""
from dataclasses import dataclass
import math

import numpy as np
from shapely.geometry import LineString, Point, Polygon


@dataclass(frozen=True)
class FitOptions:
    tolerance: float = 2.0
    max_area_change: float = 0.20
    merge_angle: float = 35.0
    fill_holes_area: float = 100.0
    min_building_area: float = 20.0


def vertex_count(polygon):
    return len(polygon.exterior.coords) - 1 + sum(len(r.coords) - 1 for r in polygon.interiors)


def _line(points):
    """Arc-length-weighted TLS, independent of a wall's XY orientation."""
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    weights = np.zeros(len(points))
    weights[:-1] += lengths / 2
    weights[1:] += lengths / 2
    if weights.sum() <= 1e-12:
        raise ValueError("Degenerate boundary chain")
    center = np.average(points, axis=0, weights=weights)
    delta = points - center
    covariance = (delta * weights[:, None]).T @ delta / weights.sum()
    _, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, -1]
    if np.dot(direction, points[-1] - points[0]) < 0:
        direction = -direction
    normal = np.array([-direction[1], direction[0]])
    residual = np.abs(delta @ normal)
    return {"points": points, "center": center, "direction": direction,
            "normal": normal, "error": float(residual.max()), "residual": residual,
            "span": float(np.ptp(delta @ direction))}


def _split(points, tolerance):
    stack, result = [points], []
    while stack:
        chain = stack.pop()
        fitted = _line(chain)
        if fitted["error"] <= tolerance or len(chain) <= 2:
            result.append(fitted)
            continue
        index = int(np.argmax(fitted["residual"][1:-1])) + 1
        stack.append(chain[index:])
        stack.append(chain[:index + 1])
    return result


def _angle(a, b):
    return math.degrees(math.acos(float(np.clip(abs(np.dot(a, b)), 0, 1))))


def _contact_points(geometry):
    if geometry.is_empty:
        return
    if geometry.geom_type in ("Point", "LineString", "LinearRing"):
        yield from geometry.coords
    elif hasattr(geometry, "geoms"):
        for part in geometry.geoms:
            yield from _contact_points(part)


def _insert_contact_vertices(coordinates, contacts):
    """Pin shared junctions without freezing an entire noisy building wall."""
    pins = set()
    extra = {}
    following = np.roll(coordinates, -1, axis=0)
    delta = following - coordinates
    squared = np.sum(delta * delta, axis=1)
    for contact in contacts:
        for xy in _contact_points(contact):
            point = np.asarray(xy[:2], dtype=np.float64)
            t = np.divide(np.sum((point - coordinates) * delta, axis=1), squared,
                          out=np.zeros(len(delta)), where=squared > 1e-20)
            t = np.clip(t, 0, 1)
            distance = np.linalg.norm(coordinates + t[:, None] * delta - point, axis=1)
            index = int(np.argmin(distance))
            if distance[index] > 1e-7:
                continue
            pins.add(tuple(np.round(point, 7)))
            if 1e-9 < t[index] < 1 - 1e-9:
                extra.setdefault(index, {})[float(t[index])] = point
    expanded = []
    for i, xy in enumerate(coordinates):
        expanded.append(xy)
        expanded.extend(point for _, point in sorted(extra.get(i, {}).items()))
    return np.asarray(expanded), pins


def fit_ring(ring, tolerance, merge_angle, protected=(), forbidden=(), contacts=()):
    coordinates = np.asarray(ring.coords, dtype=np.float64)[:-1, :2]
    coordinates, pins = _insert_contact_vertices(coordinates, contacts)
    # Work in local metres to avoid precision loss at national-grid coordinates.
    origin = coordinates[0].copy()
    points = coordinates - origin
    if len(points) <= 4:
        return coordinates.copy()
    if len({tuple(p) for p in points}) != len(points):
        return coordinates.copy()
    closed = np.vstack((points, points[0]))
    # DP only proposes chain breaks. Exported wall positions come from TLS of
    # ALL original vertices in each chain, not from deleting the short edges.
    anchors = np.asarray(LineString(closed).simplify(tolerance, preserve_topology=False).coords)[:-1]
    if len(anchors) < 3:
        return coordinates.copy()
    lookup = {tuple(p): i for i, p in enumerate(points)}
    indices = sorted(set([lookup[tuple(p)] for p in anchors] +
                         [i for i, p in enumerate(coordinates) if tuple(np.round(p, 7)) in pins]))
    segments = []
    n = len(points)
    for k, start in enumerate(indices):
        end = indices[(k + 1) % len(indices)]
        if end <= start:
            end += n
        chain = points[np.arange(start, end + 1) % n]
        segments.extend(_split(chain, tolerance))
    # Merge adjacent chains when one TLS line explains their entire original
    # support. Include the wrap-around pair so a ring's first vertex is not a
    # permanent artificial corner on a long wall.
    changed = True
    while changed and len(segments) > 3:
        changed = False
        for i in range(len(segments)):
            j = (i + 1) % len(segments)
            a, b = segments[i], segments[j]
            if tuple(np.round(b["points"][0] + origin, 7)) in pins:
                continue
            angle = _angle(a["direction"], b["direction"])
            if angle > merge_angle and min(a["span"], b["span"]) > 2 * tolerance:
                continue
            merged = _line(np.vstack((a["points"][:-1], b["points"])))
            if merged["error"] > tolerance:
                continue
            if j == 0:
                segments = [merged] + segments[1:i]
            else:
                segments[i:j + 1] = [merged]
            changed = True
            break
    corners = []
    for i, current in enumerate(segments):
        previous = segments[i - 1]
        anchor = current["points"][0]
        matrix = np.vstack((previous["normal"], current["normal"]))
        if abs(np.linalg.det(matrix)) < math.sin(math.radians(10)):
            # Adjacent near-parallel offset segments: a bounded transition,
            # not an intersection at a huge distance from the building.
            pa = anchor - np.dot(anchor - previous["center"], previous["normal"]) * previous["normal"]
            pb = anchor - np.dot(anchor - current["center"], current["normal"]) * current["normal"]
            corner = (pa + pb) / 2
        else:
            rhs = np.array([np.dot(previous["normal"], previous["center"]),
                            np.dot(current["normal"], current["center"])])
            corner = np.linalg.solve(matrix, rhs)
        if np.linalg.norm(corner - anchor) > 2 * tolerance + 1e-8:
            # Keep this difficult junction instead of discarding every fitted
            # wall of a large building because of one tiny notch/spike.
            corner = anchor.copy()
        if tuple(np.round(anchor + origin, 7)) in pins:
            corner = anchor.copy()
        # One joint per chain is required by the local fallback bookkeeping,
        # even when two adjacent joints happen to coincide.
        corners.append(corner)
    if len(corners) < 3:
        raise ValueError("Too few fitted corners")
    from shapely.strtree import STRtree
    # Preserve only locally problematic chains. A narrow notch or a courtyard
    # close to a wall must not veto all the good long-wall fits on this ring.
    # 0=TLS line, 1=bounded chord approximation, 2=original local chain.
    # A rejection no longer immediately brings back every raster stair step.
    level = np.zeros(len(segments), dtype=np.uint8)
    raw_line = LineString(np.vstack((coordinates, coordinates[0])))
    raw_buffer = raw_line.buffer(tolerance + 1e-7, quad_segs=16)
    raw_shell = Polygon(coordinates)
    forbidden_baselines = [raw_shell.intersection(g).area for g in forbidden]
    for _ in range(2 * len(segments) + 1):
        joints = [corners[i] if level[i - 1] == 0 and level[i] == 0 else segments[i]["points"][0]
                  for i in range(len(segments))]
        chains = []
        for i, segment in enumerate(segments):
            if level[i] == 0:
                chain = np.vstack((joints[i], joints[(i + 1) % len(segments)]))
            elif level[i] == 1:
                chain = np.asarray(LineString(segment["points"]).simplify(
                    tolerance * 0.5, preserve_topology=False).coords)
            else:
                chain = segment["points"]
            chains.append(LineString(chain + origin))
        output = np.vstack([np.asarray(chain.coords)[:-1] for chain in chains])
        keep = np.r_[True, np.linalg.norm(np.diff(output, axis=0), axis=1) > 1e-9]
        output = output[keep]
        if len(output) < 3:
            return coordinates.copy()
        candidate = Polygon(output)
        bad = set()
        if not candidate.is_valid:
            tree = STRtree(chains)
            for i, chain in enumerate(chains):
                for j in tree.query(chain):
                    j = int(j)
                    if j <= i:
                        continue
                    intersection = chain.intersection(chains[j])
                    if intersection.is_empty:
                        continue
                    adjacent = j == i + 1 or (i == 0 and j == len(chains) - 1)
                    if adjacent and intersection.geom_type == "Point":
                        continue
                    bad.update(k for k in (i, j) if level[k] < 2)
        if candidate.is_valid:
            candidate_buffer = candidate.boundary.buffer(tolerance + 1e-7, quad_segs=16)
            for i, chain in enumerate(chains):
                if level[i] == 2:
                    continue
                support = LineString(segments[i]["points"] + origin)
                if not chain.difference(raw_buffer).is_empty or not support.difference(candidate_buffer).is_empty:
                    bad.add(i)
            for hole in protected:
                if not candidate.contains(hole):
                    distances = [chain.distance(hole) if chain.length > 1e-10
                                 else Point(chain.coords[0]).distance(hole) for chain in chains]
                    closest = min(distances)
                    bad.update(i for i, d in enumerate(distances) if level[i] < 2 and d <= closest + 1e-7)
            for neighbor, baseline in zip(forbidden, forbidden_baselines):
                if candidate.intersection(neighbor).area > baseline + 1e-7:
                    crossing = [i for i, chain in enumerate(chains)
                                if level[i] < 2 and chain.intersects(neighbor)]
                    if crossing:
                        bad.update(crossing)
                    else:
                        choices = [(chain.distance(neighbor) if chain.length > 1e-10
                                    else Point(chain.coords[0]).distance(neighbor), i)
                                   for i, chain in enumerate(chains) if level[i] < 2]
                        if choices:
                            bad.add(min(choices)[1])
        if not bad:
            return output if candidate.is_valid else coordinates.copy()
        # Also freeze incident junctions, so a raw chain remains attached to
        # its original corners without distorting adjacent tiny features.
        for i in bad:
            level[i] += 1
    return coordinates.copy()


def polygon_candidate(original, options, tolerance, forbidden=(), contacts=()):
    def ring_candidate(ring, protected=(), obstacles=None, anchors=None, inside=None):
        before = Polygon(ring)
        obstacles = forbidden if obstacles is None else obstacles
        anchors = contacts if anchors is None else anchors
        for scale in (1.0, 0.5, 0.25):
            limit = tolerance * scale + 1e-7
            try:
                coords = fit_ring(ring, tolerance * scale, options.merge_angle, protected, obstacles, anchors)
                after = Polygon(coords)
            except (ValueError, np.linalg.LinAlgError):
                continue
            if (not after.is_valid or after.is_empty or
                    len(after.exterior.coords) >= len(before.exterior.coords) or
                    abs(after.area - before.area) > options.max_area_change * before.area + 1e-8 or
                    not before.boundary.difference(after.boundary.buffer(limit, quad_segs=16)).is_empty or
                    not after.boundary.difference(before.boundary.buffer(limit, quad_segs=16)).is_empty):
                continue
            if inside is not None and not inside.covers(after):
                continue
            return coords
        return np.asarray(ring.coords)[:, :2]
    shell = ring_candidate(original.exterior, [Polygon(r) for r in original.interiors])
    holes = [np.asarray(ring.coords)[:, :2] for ring in original.interiors]
    if not Polygon(shell, holes).is_valid:
        # A difficult courtyard must not invalidate otherwise useful fits.
        shell = np.asarray(original.exterior.coords)[:, :2]
    shell_polygon = Polygon(shell)
    for i, old_ring in enumerate(original.interiors):
        old_hole = Polygon(old_ring)
        other_holes = [Polygon(h) for j, h in enumerate(holes) if j != i]
        islands = [old_hole.intersection(neighbor) for neighbor in forbidden
                   if old_hole.intersection(neighbor).area > 1e-7]
        hole_contacts = [old_hole.boundary.intersection(original.exterior)]
        hole_contacts.extend(old_hole.boundary.intersection(neighbor.boundary) for neighbor in forbidden)
        hole_contacts.extend(old_hole.boundary.intersection(h.boundary) for h in other_holes)
        hole_contacts = [g for g in hole_contacts if not g.is_empty]
        fitted_hole = ring_candidate(old_ring, protected=islands, obstacles=other_holes,
                                    anchors=hole_contacts, inside=shell_polygon)
        trial = list(holes)
        trial[i] = fitted_hole
        if Polygon(shell, trial).is_valid:
            holes[i] = fitted_hole
    candidate = Polygon(shell, holes)
    if candidate.is_empty or not candidate.is_valid or candidate.area <= 0:
        return None, "invalid_geometry"
    if candidate.area < options.min_building_area:
        return None, "below_min_building_area_after_fit"
    if len(candidate.interiors) != len(original.interiors):
        return None, "hole_count"
    if vertex_count(candidate) >= vertex_count(original):
        return None, "no_reduction"
    if abs(candidate.area - original.area) > options.max_area_change * original.area + 1e-8:
        return None, "area_change"
    if candidate.symmetric_difference(original).area > options.max_area_change * original.area + 1e-8:
        return None, "coverage_change"
    for old_ring, new_ring in zip(original.interiors, candidate.interiors):
        old_hole, new_hole = Polygon(old_ring), Polygon(new_ring)
        if abs(old_hole.area - new_hole.area) > options.max_area_change * old_hole.area + 1e-8:
            return None, "hole_area_change"
    # Two-sided continuous boundary constraint. Checking buffers includes the
    # interiors of edges, not just vertex-to-vertex distances.
    limit = tolerance + 1e-7
    if (not original.boundary.difference(candidate.boundary.buffer(limit, quad_segs=16)).is_empty or
            not candidate.boundary.difference(original.boundary.buffer(limit, quad_segs=16)).is_empty):
        return None, "boundary_offset"
    return candidate, "fitted"


def fit_polygons(originals, options=FitOptions(), progress=None):
    """Filter tiny buildings, fill small holes, fit outer AND retained inner rings.

    Returned geometries/reports are aligned with the retained features. Each
    report's source_index maps back to the input sequence.
    """
    from shapely.strtree import STRtree
    input_originals = list(originals)
    source_indices = [i for i, p in enumerate(input_originals) if p.area >= options.min_building_area]
    originals = [input_originals[i] for i in source_indices]
    tree = STRtree(originals)
    cleaned, holes_removed = [], []
    for i, original in enumerate(originals):
        keep, removed = [], 0
        for ring in original.interiors:
            hole = Polygon(ring)
            occupied = False
            if options.fill_holes_area > 0 and hole.area <= options.fill_holes_area:
                occupied = any(hole.intersection(originals[int(j)]).area > 1e-7
                               for j in tree.query(hole) if int(j) != i)
                if not occupied:
                    removed += 1
                    continue
            keep.append(list(ring.coords))
        candidate = Polygon(original.exterior.coords, keep)
        if not candidate.is_valid:
            candidate, removed = original, 0
        cleaned.append(candidate)
        holes_removed.append(removed)
    tree = STRtree(cleaned)
    current = list(cleaned)
    reports = []
    for index, original in enumerate(cleaned):
        report = {"changed": False, "reason": "unchanged", "tolerance": 0.0,
                  "source_index": source_indices[index], "holes_fitted": 0,
                  "vertices_old": vertex_count(originals[index]), "vertices_new": vertex_count(original),
                  "holes_removed": holes_removed[index], "hole_fill_area": original.area - originals[index].area,
                  "attempt_reasons": []}
        if original.is_valid and not original.is_empty:
            neighbors = [int(j) for j in tree.query(original.buffer(2 * options.tolerance)) if int(j) != index]
            contacts = [original.boundary.intersection(cleaned[j].boundary) for j in neighbors]
            contacts = [c for c in contacts if not c.is_empty]
            for scale in (1.0, 0.5, 0.25):
                tolerance = options.tolerance * scale
                try:
                    candidate, reason = polygon_candidate(original, options, tolerance,
                                                          [current[j] for j in neighbors], contacts)
                except (ValueError, np.linalg.LinAlgError):
                    candidate, reason = None, "unstable_fit"
                report["reason"] = reason
                if candidate is None:
                    report["attempt_reasons"].append(reason)
                    continue
                for j in neighbors:
                    baseline = original.intersection(cleaned[j]).area
                    if candidate.intersection(current[j]).area > baseline + 1e-7:
                        candidate, reason = None, "neighbor_overlap"
                        break
                    contact = original.boundary.intersection(cleaned[j].boundary)
                    if not contact.is_empty:
                        if not contact.difference(candidate.boundary.buffer(1e-7)).is_empty:
                            candidate, reason = None, "shared_boundary"
                            break
                report["reason"] = reason
                report["attempt_reasons"].append(reason)
                if candidate is not None:
                    current[index] = candidate
                    report.update(changed=True, reason="fitted", tolerance=tolerance,
                                  vertices_new=vertex_count(candidate),
                                  holes_fitted=sum(not Polygon(a).equals(Polygon(b))
                                                   for a, b in zip(original.interiors, candidate.interiors)))
                    break
        else:
            report["reason"] = "invalid_input"
        reports.append(report)
        if progress is not None:
            progress(index + 1, len(originals), report)
    return current, reports
