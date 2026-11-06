"""One isolated remote task; only writes inside its newly allocated job directory."""
from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
from contextlib import ExitStack
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[1]
WORKER_REVISION = "roof-v3-20261106"


def write_json(path, value):
    temporary = path.with_name(path.name + "." + uuid4().hex + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    os.replace(temporary, path)


def progress(job, phase, message, percent, **extra):
    write_json(job / "progress.json", {"phase": phase, "message": "[" + WORKER_REVISION + "] " + message,
                                      "percent": percent, "worker_revision": WORKER_REVISION, **extra})


class CopcNodeCache:
    """Bounded decoded-node LRU. Preserve laspy's physical-offset point order.

    Installed only on the height reader, never on the model's input pipeline.
    The hook is checked at runtime; unsupported laspy versions keep native I/O.
    """
    def __init__(self, reader, max_bytes=128 * 1024**2):
        self.reader = reader
        self.max_bytes = max_bytes
        self.arrays = OrderedDict()
        self.bytes = self.hits = self.misses = 0
        self.original = getattr(reader, "_fetch_and_decompress_points_of_nodes", None)
        self.enabled = callable(self.original)
        if self.enabled:
            reader._fetch_and_decompress_points_of_nodes = self.fetch

    def stats(self):
        return {"enabled": self.enabled, "hits": self.hits, "misses": self.misses,
                "bytes": self.bytes, "limit_bytes": self.max_bytes}

    def fetch(self, nodes):
        import numpy as np
        from laspy import ScaleAwarePointRecord
        if not nodes:
            return self.original(nodes)
        ordered = sorted(nodes, key=lambda node: node.offset)
        keys = [(node.offset, node.byte_size, node.point_count) for node in ordered]
        parts, missing = {}, []
        for key, node in zip(keys, ordered):
            if key in self.arrays:
                self.hits += 1
                self.arrays.move_to_end(key)
                parts[key] = self.arrays[key]
            else:
                self.misses += 1
                missing.append(node)
        if missing:
            decoded = self.original(missing).array
            if len(decoded) != sum(node.point_count for node in missing):
                raise ValueError("COPC node point count mismatch")
            offset = 0
            for node in missing:
                key = (node.offset, node.byte_size, node.point_count)
                array = decoded[offset:offset + node.point_count].copy()
                offset += node.point_count
                parts[key] = array
                if array.nbytes <= self.max_bytes:
                    while self.arrays and self.bytes + array.nbytes > self.max_bytes:
                        _, old = self.arrays.popitem(last=False)
                        self.bytes -= old.nbytes
                    self.arrays[key] = array
                    self.bytes += array.nbytes
        # A fresh array lets laspy's bounds filter mutate the returned record
        # without affecting cached nodes or including points outside a polygon.
        array = np.concatenate([parts[key] for key in keys])
        header = self.reader.header
        return ScaleAwarePointRecord(array, header.point_format, header.scales, header.offsets)


def roof_elevation(xyz, progress_callback=None):
    """Existing CreateTool rule, in double/local coordinates with indexed XY lookup.

    This returns an absolute roof elevation, not height above ground. Failure is
    explicit rather than a fictitious zero. No fallback estimator is substituted.
    """
    import numpy as np
    from scipy.spatial import cKDTree
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if len(xyz) < 5:
        return None, "insufficient_points"
    candidates = np.flatnonzero(xyz[:, 2] >= xyz[:, 2].min() + 2.5)
    if not len(candidates):
        return None, "no_roof_candidates"
    local = xyz - xyz[0]
    xy_tree = cKDTree(local[:, :2])
    roof = []
    # Bounded batches avoid storing neighborhoods for the entire building.
    # Sorted ORIGINAL indices preserve the first-16 rule, not nearest-16 kNN.
    for start in range(0, len(candidates), 128):
        selected = candidates[start:start + 128]
        neighborhoods = xy_tree.query_ball_point(local[selected, :2], 1.0, return_sorted=True)
        indices = np.zeros((len(selected), 16), dtype=np.intp)
        counts = np.zeros(len(selected), dtype=np.intp)
        for row, neighbors in enumerate(neighborhoods):
            # Usually only examine 16 entries. At the exact radius boundary,
            # continue until 16 STRICTLY-inside entries have been collected.
            kept = []
            for offset in range(0, len(neighbors), 16):
                chunk = np.asarray(neighbors[offset:offset + 16], dtype=np.intp)
                delta = local[chunk, :2] - local[selected[row], :2]
                kept.extend(chunk[np.sum(delta * delta, axis=1) < 1.0].tolist())
                if len(kept) >= 16:
                    break
            count = min(len(kept), 16)
            counts[row] = count
            indices[row, :count] = kept[:count]
        valid = counts >= 5
        if valid.any():
            n = counts[valid]
            values = local[indices[valid]]
            mask = np.arange(16)[None, :] < n[:, None]
            centers = (values * mask[:, :, None]).sum(axis=1) / n[:, None]
            centered = (values - centers[:, None, :]) * mask[:, :, None]
            covariance = np.matmul(centered.transpose(0, 2, 1), centered)
            xx, xy, xz = covariance[:, 0, 0], covariance[:, 0, 1], covariance[:, 0, 2]
            yy, yz, zz = covariance[:, 1, 1], covariance[:, 1, 2], covariance[:, 2, 2]
            determinants = np.column_stack((yy * zz - yz * yz, xx * zz - xz * xz, xx * yy - xy * xy))
            axis = determinants.argmax(axis=1)
            normals = np.empty((len(n), 3), dtype=float)
            choose = axis == 0
            normals[choose] = np.column_stack((determinants[:, 0], xz * yz - xy * zz, xy * yz - xz * yy))[choose]
            choose = axis == 1
            normals[choose] = np.column_stack((xz * yz - xy * zz, determinants[:, 1], xy * xz - yz * xx))[choose]
            choose = axis == 2
            normals[choose] = np.column_stack((xy * yz - xz * yy, xy * xz - yz * xx, determinants[:, 2]))[choose]
            lengths = np.linalg.norm(normals, axis=1)
            nz = np.divide(np.abs(normals[:, 2]), lengths, out=np.zeros(len(n)), where=lengths > 0)
            nz[determinants.max(axis=1) <= 0] = 1.0
            roof.extend(xyz[selected[valid][nz > 0.90], 2].tolist())
        if progress_callback is not None:
            progress_callback(min(start + 128, len(candidates)), len(candidates))
    return (math.fsum(roof) / len(roof), "ok") if roof else (None, "no_roof_candidates")


class RoofReader:
    def __init__(self, path):
        import laspy
        self._resources = ExitStack()
        self.reader = self._resources.enter_context(laspy.CopcReader.open(path))
        self.cache = CopcNodeCache(self.reader)
        self.timings = {"read_seconds": 0.0, "filter_seconds": 0.0, "normal_seconds": 0.0, "buildings": 0}
        self.report = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        return self._resources.__exit__(*args)

    def close(self):
        # CopcReader is a context manager and has no public close() method.
        # ExitStack invokes its supported __exit__, and is safe to close twice.
        self._resources.close()

    def __call__(self, polygon):
        import numpy as np
        from laspy.copc import Bounds
        from shapely import contains_xy
        xmin, ymin, xmax, ymax = polygon.bounds
        low = np.array([xmin, ymin, self.reader.header.mins[2]], dtype=float)
        high = np.array([xmax, ymax, self.reader.header.maxs[2]], dtype=float)
        started = time.perf_counter()
        points = self.reader.query(bounds=Bounds(low, high))
        self.timings["read_seconds"] += time.perf_counter() - started
        started = time.perf_counter()
        x, y = np.asarray(points.x), np.asarray(points.y)
        # The polygon's retained courtyard holes are automatically excluded.
        inside = contains_xy(polygon, x, y)
        xyz = np.column_stack((x[inside], y[inside], np.asarray(points.z)[inside]))
        self.timings["filter_seconds"] += time.perf_counter() - started
        started = time.perf_counter()
        result = roof_elevation(xyz, self.report)
        self.timings["normal_seconds"] += time.perf_counter() - started
        self.timings["buildings"] += 1
        return result


def typed_value(value, field_type):
    if value is None:
        return None
    if field_type.startswith("int"):
        return int(round(value))
    if field_type.startswith("float"):
        return float(value)
    return value


def build_result(job, predictions, options, height_reader=None):
    """Append to a COPY of the uploaded target; expose new features separately.

    'target' keeps the submitted existing features and receives new buildings.
    'predicted_buildings' contains only new features for duplicate-free import
    into the original QGIS layer. Neither uploaded input is ever modified.
    """
    import fiona
    from pyproj import CRS, Transformer
    from shapely.geometry import mapping, shape
    from shapely.ops import transform

    result = job / "result.gpkg"
    if result.exists() or (job / "result.json").exists():
        raise ValueError("This task already has output; use a new job, never overwrite it")
    with fiona.open(job / "target.gpkg", layer="target") as source:
        target_crs = CRS.from_user_input(source.crs_wkt or source.crs)
        schema = {"geometry": source.schema["geometry"], "properties": dict(source.schema["properties"])}
        existing_count = len(source)
    if schema["geometry"].replace("3D ", "") not in ("Polygon", "MultiPolygon"):
        raise ValueError("Target layer must contain polygons")
    field_names = options["target_fields"]
    if not set(field_names).issubset(schema["properties"]):
        raise ValueError("Uploaded target field names do not match the request")
    height_field = options.get("height_field", "")
    if height_field and (height_field not in field_names or
                         not schema["properties"][height_field].startswith(("float", "int"))):
        raise ValueError("Height field must be a numeric target field")
    # Copy to an exclusive new filename; preserve the original snapshot exactly.
    with (job / "target.gpkg").open("rb") as source, result.open("xb") as destination:
        shutil.copyfileobj(source, destination, 1024 * 1024)
    if "predicted_buildings" in fiona.listlayers(result):
        raise ValueError("Unexpected additional layer in target snapshot")
    height_reasons, total = Counter(), 0
    height_seconds = 0.0
    last_height_report = [0.0]
    progress(job, "height_and_write", "正在计算屋顶高程并写入" if height_field else "正在写入目标图层", 80)
    private_fid = "__spt_result_id"
    while private_fid in schema["properties"]:
        private_fid += "_"
    with fiona.open(predictions, layer="buildings") as predicted:
        prediction_crs = CRS.from_user_input(predicted.crs_wkt or predicted.crs)
        project = Transformer.from_crs(prediction_crs, target_crs, always_xy=True).transform
        total_expected = len(predicted)
        with fiona.open(result, "w", driver="GPKG", layer="predicted_buildings", schema=schema,
                        crs_wkt=target_crs.to_wkt(), FID=private_fid) as incremental, \
                (job / "new_features.jsonl").open("x", encoding="utf-8") as spool:
            pending_records = []
            for feature in predicted:
                polygon = shape(feature["geometry"])
                values = {name: None for name in schema["properties"]}
                for name in field_names:
                    if name in feature["properties"]:
                        values[name] = typed_value(feature["properties"][name], schema["properties"][name])
                if height_field:
                    if isinstance(height_reader, RoofReader):
                        def report_height(done, point_total):
                            now = time.monotonic()
                            if done != point_total and now - last_height_report[0] < 1.0:
                                return
                            last_height_report[0] = now
                            fraction = done / max(point_total, 1)
                            progress(job, "height_and_write", "屋顶高程 %d/%d：候选点 %d/%d" %
                                     (total + 1, total_expected, done, point_total),
                                     80 + 18 * (total + fraction) / max(total_expected, 1))
                        height_reader.report = report_height
                    started = time.perf_counter()
                    elevation, reason = height_reader(polygon)
                    height_seconds += time.perf_counter() - started
                    height_reasons[reason] += 1
                    values[height_field] = typed_value(elevation, schema["properties"][height_field])
                geometry = transform(project, polygon)
                if schema["geometry"].replace("3D ", "") == "MultiPolygon":
                    from shapely.geometry import MultiPolygon
                    if geometry.geom_type == "Polygon":
                        geometry = MultiPolygon([geometry])
                record = {"geometry": mapping(geometry), "properties": values}
                pending_records.append(record)
                if len(pending_records) >= 128:
                    incremental.writerecords(pending_records)
                    pending_records.clear()
                spool.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                total += 1
                if total % 10 == 0 or total == total_expected:
                    progress(job, "height_and_write", "计算屋顶高程并写入目标图层" if height_field else "写入目标图层",
                             80 + 18 * total / max(total_expected, 1), buildings_done=total, buildings_total=total_expected)
            if pending_records:
                incremental.writerecords(pending_records)
    # No concurrent read/write handles to one SQLite file, and no all-features
    # list in RAM. The spool contains only the already fitted final features.
    with (job / "new_features.jsonl").open(encoding="utf-8") as spool, fiona.open(result, "a", layer="target") as target:
        target.writerecords(json.loads(line) for line in spool)
    with fiona.open(result, layer="target") as check:
        if len(check) != existing_count + total:
            raise ValueError("Target feature count verification failed")
    digest = hashlib.sha256()
    with result.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    manifest = {"complete": True, "api_version": 1, "job_id": job.name, "new_layer": "predicted_buildings",
                "target_layer": "target", "feature_count": total, "existing_feature_count": existing_count,
                "target_fields": field_names, "target_crs_wkt": target_crs.to_wkt(), "height_field": height_field,
                "height_reasons": dict(height_reasons), "height_failed": sum(v for k, v in height_reasons.items() if k != "ok"),
                "height_seconds": height_seconds,
                "worker_revision": WORKER_REVISION,
                "height_timings": height_reader.timings if isinstance(height_reader, RoofReader) else {},
                "height_cache": height_reader.cache.stats() if isinstance(height_reader, RoofReader) else {},
                "bytes": result.stat().st_size, "sha256": digest.hexdigest()}
    write_json(job / "result.json", manifest)
    progress(job, "completed", "预测、拟合和目标图层写入完成", 100)
    return manifest


def run(job, checkpoint):
    if not (job / "job.json").is_file() or job.parent.name == "testresult":
        raise ValueError("Expected a dedicated remote job directory")
    options = json.loads((job / "job.json").read_text(encoding="utf-8"))["options"]
    # Fail malformed input before consuming GPU time, without changing input.
    import fiona
    import laspy
    from pyproj import CRS
    with laspy.CopcReader.open(job / "input.copc.laz") as cloud:
        if not cloud.header.parse_crs():
            raise ValueError("COPC has no CRS metadata")
    with fiona.open(job / "target.gpkg", layer="target", enabled_drivers=["GPKG"]) as target:
        CRS.from_user_input(target.crs_wkt or target.crs)
        if target.schema["geometry"].replace("3D ", "") not in ("Polygon", "MultiPolygon"):
            raise ValueError("Target is not a polygon layer")
        if not set(options["target_fields"]).issubset(target.schema["properties"]):
            raise ValueError("Target fields are missing")
        height = options.get("height_field", "")
        if height and not target.schema["properties"].get(height, "").startswith(("float", "int")):
            raise ValueError("Height field is not numeric")
    output = job / "inference"
    output.mkdir(exist_ok=False)
    progress(job, "inference", "开始实验室分块预测", 0)
    command = [sys.executable, "-u", str(ROOT / "scripts/predict_copc_buildings.py"),
               "--input", str(job / "input.copc.laz"), "--checkpoint", str(checkpoint),
               "--output", str(output), "--run-name", "prediction"]
    with (job / "inference.log").open("xb") as log:
        process = subprocess.Popen(command, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT)
        while process.poll() is None:
            state_file = output / "prediction_work/run.json"
            if state_file.is_file():
                state = json.loads(state_file.read_text(encoding="utf-8"))
                done, total = state.get("completed_tiles", 0), state.get("total_tiles", 0)
                phase = "fitting" if total and done == total else "inference"
                message = "正在拟合内外轮廓" if phase == "fitting" else "分块预测 %d/%d" % (done, total)
                progress(job, phase, message, min(78, 78 * done / max(total, 1)), tiles_done=done, tiles_total=total)
            time.sleep(1)
        if process.returncode:
            raise RuntimeError("SPT prediction failed; see inference.log (原始文件和成功缓存均保留)")
    summary = json.loads((output / "prediction_work/vector_summary.json").read_text(encoding="utf-8"))
    if not summary.get("complete"):
        raise ValueError("Full-area fitting/export is incomplete")
    with ExitStack() as resources:
        reader = resources.enter_context(RoofReader(job / "input.copc.laz")) if options.get("height_field") else None
        build_result(job, output / "prediction_buildings.gpkg", options, reader)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=WORKER_REVISION)
    parser.add_argument("--job-dir", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    args = parser.parse_args()
    job = args.job_dir.resolve()
    try:
        run(job, args.checkpoint.resolve())
    except Exception as exc:
        progress(job, "failed", str(exc), 0)
        raise


if __name__ == "__main__":
    main()
