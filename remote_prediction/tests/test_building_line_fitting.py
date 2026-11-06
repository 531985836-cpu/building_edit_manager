"""Geometry checks; does not import SPT, torch or read point clouds."""
import sys
from pathlib import Path
import unittest

import numpy as np
from shapely import affinity
from shapely.geometry import Polygon, Point

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from building_line_fitting import FitOptions, fit_polygons, vertex_count


def noisy_ring(vertices, amplitude=0.2):
    out = []
    for a, b in zip(vertices, vertices[1:] + vertices[:1]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        delta = b - a
        normal = np.array([-delta[1], delta[0]]) / np.linalg.norm(delta)
        count = max(2, int(np.ceil(np.linalg.norm(delta) / 0.5)))
        out.append(a)
        for i in range(1, count):
            out.append(a + delta * i / count + normal * amplitude * (1 if i % 2 else -1))
    return out


class LineFitTests(unittest.TestCase):
    def test_jagged_walls_become_fitted_lines(self):
        raw = Polygon(noisy_ring([(0, 0), (24, 0), (24, 12), (0, 12)]))
        result, reports = fit_polygons([raw])
        fitted = result[0]
        self.assertTrue(reports[0]["changed"])
        self.assertLessEqual(vertex_count(fitted), 6)
        self.assertTrue(fitted.is_valid)
        # New intersections come from regression, rather than a subset of the
        # input vertices (which plain Douglas-Peucker would return).
        old = set(raw.exterior.coords)
        self.assertTrue(any(tuple(p) not in old for p in fitted.exterior.coords))
        self.assertLess(raw.boundary.hausdorff_distance(fitted.boundary), 0.75)

    def test_rotation_and_large_coordinates(self):
        raw = Polygon(noisy_ring([(0, 0), (24, 0), (24, 12), (0, 12)]))
        raw = affinity.translate(affinity.rotate(raw, 31, origin=(0, 0)), 155000, 464000)
        fitted, report = fit_polygons([raw])
        self.assertTrue(report[0]["changed"])
        self.assertLessEqual(vertex_count(fitted[0]), 6)
        self.assertLess(raw.boundary.hausdorff_distance(fitted[0].boundary), 0.75)

    def test_concave_footprint_and_courtyard(self):
        shell = noisy_ring([(0, 0), (24, 0), (24, 7), (10, 7), (10, 20), (0, 20)])
        hole = noisy_ring([(2, 3), (7, 3), (7, 10), (2, 10)], amplitude=0.1)
        raw = Polygon(shell, [hole])
        result, report = fit_polygons([raw], FitOptions(fill_holes_area=4, min_building_area=0))
        fitted = result[0]
        self.assertTrue(report[0]["changed"])
        self.assertEqual(len(fitted.interiors), 1)
        self.assertLessEqual(len(fitted.interiors[0].coords), len(raw.interiors[0].coords))
        self.assertFalse(fitted.covers(Point(4, 5)))
        self.assertFalse(fitted.covers(Point(18, 15)))
        self.assertTrue(fitted.is_valid)

    def test_no_new_neighbor_overlap(self):
        a = Polygon(noisy_ring([(0, 0), (20, 0), (20, 10), (0, 10)], amplitude=0.15))
        b = affinity.translate(a, xoff=20.31)
        self.assertLess(a.intersection(b).area, 1e-7)
        result, reports = fit_polygons([a, b])
        self.assertLess(result[0].intersection(result[1]).area, 1e-7)
        self.assertEqual(len(result), 2)

    def test_shared_wall_remains_shared(self):
        a = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
        b = Polygon([(10, 0), (20, 0), (20, 10), (10, 10)])
        result, _ = fit_polygons([a, b])
        self.assertAlmostEqual(result[0].boundary.intersection(result[1].boundary).length, 10)
        self.assertEqual([vertex_count(p) for p in result], [4, 4])

    def test_tiny_component_is_not_removed(self):
        raw = Polygon([(0, 0), (0.5, 0), (0.5, 0.5), (0, 0.5)])
        result, _ = fit_polygons([raw], FitOptions(tolerance=1.0, min_building_area=0))
        self.assertTrue(result[0].equals(raw))

    def test_narrow_notch_does_not_veto_other_walls(self):
        vertices = [(0, 0), (24, 0), (24, 12), (13, 12), (13, 10), (12.6, 10), (12.6, 12), (0, 12)]
        raw = Polygon(noisy_ring(vertices, amplitude=0.08))
        self.assertTrue(raw.is_valid)
        result, report = fit_polygons([raw], FitOptions(tolerance=1.0))
        self.assertTrue(report[0]["changed"])
        self.assertLess(vertex_count(result[0]), vertex_count(raw) / 2)
        self.assertTrue(result[0].is_valid)
        self.assertFalse(result[0].contains(Point(12.8, 11)))

    def test_small_holes_removed_large_courtyard_kept(self):
        raw = Polygon([(0, 0), (30, 0), (30, 30), (0, 30)], [
            [(2, 2), (3, 2), (3, 3), (2, 3)],
            [(5, 2), (7, 2), (7, 4), (5, 4)],
            [(10, 10), (15, 10), (15, 15), (10, 15)]])
        result, reports = fit_polygons([raw], FitOptions(fill_holes_area=4, min_building_area=0))
        self.assertEqual(reports[0]["holes_removed"], 2)
        self.assertEqual(len(result[0].interiors), 1)
        self.assertTrue(result[0].contains(Point(2.5, 2.5)))
        self.assertFalse(result[0].contains(Point(12, 12)))
        preserved, _ = fit_polygons([raw], FitOptions(fill_holes_area=0, min_building_area=0))
        self.assertEqual(len(preserved[0].interiors), 3)

    def test_hole_occupied_by_another_building_is_kept(self):
        shell = [(0, 0), (20, 0), (20, 20), (0, 20)]
        hole = [(5, 5), (7, 5), (7, 7), (5, 7)]
        raw = Polygon(shell, [hole])
        island = Polygon([(5.5, 5.5), (6.5, 5.5), (6.5, 6.5), (5.5, 6.5)])
        result, reports = fit_polygons([raw, island], FitOptions(fill_holes_area=4, min_building_area=0))
        self.assertEqual(reports[0]["holes_removed"], 0)
        self.assertEqual(len(result[0].interiors), 1)
        self.assertLess(result[0].intersection(result[1]).area, 1e-7)

    def test_raster_stair_wall_with_contact_still_forms_long_segments(self):
        coords = [(0, 0), (20, 0), (20, 20)]
        for i in range(19, -1, -1):
            coords.extend([(i, i + 1), (i, i)])
        raw = Polygon(coords)
        neighbor = Polygon([(9.5, 11), (10, 11), (10, 11.5), (9.5, 11.5)])
        self.assertTrue(raw.is_valid)
        result, reports = fit_polygons([raw, neighbor], FitOptions(min_building_area=0))
        self.assertTrue(reports[0]["changed"])
        self.assertLess(len(result[0].exterior.coords), 14)
        self.assertLess(result[0].intersection(result[1]).area, 1e-7)
        self.assertLess(result[0].boundary.distance(Point(10, 11)), 1e-7)

    def test_tiny_buildings_removed_and_source_mapping_preserved(self):
        tiny = Polygon([(0, 0), (2, 0), (2, 2), (0, 2)])
        kept = Polygon([(10, 0), (20, 0), (20, 10), (10, 10)])
        result, reports = fit_polygons([tiny, kept])
        self.assertEqual(len(result), 1)
        self.assertEqual(reports[0]["source_index"], 1)
        self.assertTrue(result[0].equals(kept))

    def test_tiny_island_removed_before_hole_fill(self):
        shell = [(0, 0), (30, 0), (30, 30), (0, 30)]
        hole = [(5, 5), (10, 5), (10, 10), (5, 10)]
        raw = Polygon(shell, [hole])
        island = Polygon([(6, 6), (7, 6), (7, 7), (6, 7)])
        result, reports = fit_polygons([raw, island])
        self.assertEqual(len(result), 1)
        self.assertEqual(reports[0]["holes_removed"], 1)
        self.assertEqual(len(result[0].interiors), 0)

    def test_large_inner_ring_is_fitted_into_long_edges(self):
        shell = noisy_ring([(0, 0), (80, 0), (80, 60), (0, 60)], amplitude=0.5)
        hole = noisy_ring([(15, 15), (55, 15), (55, 35), (15, 35)], amplitude=0.5)
        raw = Polygon(shell, [hole])
        result, reports = fit_polygons([raw])
        self.assertEqual(len(result[0].interiors), 1)
        self.assertGreater(reports[0]["holes_fitted"], 0)
        self.assertLessEqual(len(result[0].interiors[0].coords), 10)
        self.assertFalse(result[0].covers(Point(35, 25)))


if __name__ == "__main__":
    unittest.main()
