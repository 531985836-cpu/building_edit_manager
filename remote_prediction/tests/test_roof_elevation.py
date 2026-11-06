"""Reference-equivalence and resource-lifetime tests using synthetic points only."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from prediction_worker import CopcNodeCache, RoofReader, roof_elevation

import math
def legacy_roof_elevation(xyz):
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
    local = xyz - xyz[0]
    xy_tree = cKDTree(local[:, :2])
    candidates = np.flatnonzero(xyz[:, 2] >= xyz[:, 2].min() + 2.5)
    roof = []
    for i in candidates:
        # Preserve input order of the first 16 neighbors, matching the existing
        # loop which stops once neighbors.size() > 15; this is not a kNN change.
        indices = sorted(xy_tree.query_ball_point(local[i, :2], 1.0))
        indices = [j for j in indices if np.sum((local[j, :2] - local[i, :2]) ** 2) < 1.0][:16]
        if len(indices) < 5:
            continue
        points = local[indices]
        centered = points - points.mean(axis=0)
        covariance = centered.T @ centered
        xx, xy, xz = covariance[0]
        yy, yz, zz = covariance[1, 1], covariance[1, 2], covariance[2, 2]
        det_x, det_y, det_z = yy * zz - yz * yz, xx * zz - xz * xz, xx * yy - xy * xy
        maximum = max(det_x, det_y, det_z)
        if maximum <= 0:
            nz = 1.0
        else:
            if maximum == det_x:
                normal = np.array([det_x, xz * yz - xy * zz, xy * yz - xz * yy])
            elif maximum == det_y:
                normal = np.array([xz * yz - xy * zz, det_y, xy * xz - yz * xx])
            else:
                normal = np.array([xy * yz - xz * yy, xy * xz - yz * xx, det_z])
            length = np.linalg.norm(normal)
            nz = abs(normal[2] / length) if length > 0 else 0.0
        if nz > 0.90:
            roof.append(float(xyz[i, 2]))
    return (math.fsum(roof) / len(roof), "ok") if roof else (None, "no_roof_candidates")


class HeightTests(unittest.TestCase):
    def test_decoded_cache_keeps_order_and_avoids_repeat_reads(self):
        import laspy
        from types import SimpleNamespace
        header = laspy.LasHeader(point_format=6, version="1.4")
        records = {}
        nodes = []
        for i in range(4):
            node = SimpleNamespace(offset=100 + i * 100, byte_size=100, point_count=10)
            points = laspy.ScaleAwarePointRecord.zeros(10, header=header)
            points.X = np.arange(i * 10, i * 10 + 10)
            points.Y = points.X + 10
            points.Z = points.X + 20
            records[node.offset] = points.array
            nodes.append(node)
        class Reader:
            def __init__(self): self.header, self.calls = header, []
            def _fetch_and_decompress_points_of_nodes(self, requested):
                self.calls.append([n.offset for n in requested])
                data = np.concatenate([records[n.offset] for n in sorted(requested, key=lambda n: n.offset)])
                return laspy.ScaleAwarePointRecord(data, header.point_format, header.scales, header.offsets)
        reader = Reader()
        cache = CopcNodeCache(reader, max_bytes=100000)
        first = reader._fetch_and_decompress_points_of_nodes([nodes[2], nodes[0]])
        np.testing.assert_array_equal(first.X, np.r_[np.arange(10), np.arange(20, 30)])
        first.X[:] = -999  # modifying a query result must not alter the cache
        second = reader._fetch_and_decompress_points_of_nodes([nodes[1], nodes[2]])
        np.testing.assert_array_equal(second.X, np.arange(10, 30))
        self.assertEqual(reader.calls, [[100, 300], [200]])
        again = reader._fetch_and_decompress_points_of_nodes([nodes[0]])
        np.testing.assert_array_equal(again.X, np.arange(10))
        self.assertEqual(len(reader.calls), 2)
        self.assertLessEqual(cache.bytes, cache.max_bytes)
        self.assertGreater(cache.hits, 0)
        tiny = Reader()
        bounded = CopcNodeCache(tiny, max_bytes=records[100].nbytes)
        for n in nodes:
            tiny._fetch_and_decompress_points_of_nodes([n])
            self.assertLessEqual(bounded.bytes, bounded.max_bytes)
        self.assertEqual(len(bounded.arrays), 1)

    def assert_equivalent(self, points):
        expected, reason = legacy_roof_elevation(points)
        actual, status = roof_elevation(points)
        self.assertEqual(status, reason)
        if expected is None:
            self.assertIsNone(actual)
        else:
            self.assertAlmostEqual(actual, expected, places=9)

    def test_mixed_slopes_dense_clusters_and_large_coordinates(self):
        rng = np.random.default_rng(190)
        for slope in (0., .2, .46, .7, 3.):
            xy = rng.uniform(0, 5, (900, 2))
            z = 115 + slope * xy[:, 0] + rng.normal(0, .01, len(xy))
            points = np.column_stack((xy, z))
            points = np.vstack(([0., 0., 90.], points, [np.nan, 0, 0]))
            points[:, :2] += [155000., 463000.]
            self.assert_equivalent(points)

    def test_radius_boundary_duplicates_and_degenerate_neighbors(self):
        points = np.array([[x, y, 115 + x * .1] for x in range(4) for y in range(4)], dtype=float)
        self.assert_equivalent(np.vstack(([0, 0, 90], points)))
        self.assert_equivalent(np.vstack(([0, 0, 90], np.tile([1, 1, 115.], (40, 1)))))
        self.assert_equivalent(points[:3])
        self.assert_equivalent(points)

    def test_input_order_first16_preserved_and_progress_finishes(self):
        rng = np.random.default_rng(612)
        points = np.column_stack((rng.uniform(0, 2, (750, 2)), rng.uniform(110, 116, 750)))
        points = np.vstack(([0, 0, 80], points))
        for _ in range(3):
            self.assert_equivalent(points)
            rng.shuffle(points)
        events = []
        roof_elevation(points, lambda done, total: events.append((done, total)))
        self.assertTrue(events)
        self.assertEqual(events[-1][0], events[-1][1])

    def test_copc_context_exit_without_public_close_and_close_is_idempotent(self):
        class Reader:
            def __init__(self): self.entered, self.exited = 0, 0
            def __enter__(self): self.entered += 1; return self
            def __exit__(self, *args): self.exited += 1
        reader = Reader()
        with patch("laspy.CopcReader.open", return_value=reader):
            wrapper = RoofReader("synthetic.copc.laz")
            wrapper.close()
            wrapper.close()
        self.assertEqual(reader.entered, 1)
        self.assertEqual(reader.exited, 1)

    def test_real_laspy_context_exit_closes_owned_stream(self):
        import io
        from laspy.copc import CopcReader
        reader = CopcReader.__new__(CopcReader)
        reader.source = io.BytesIO(b"synthetic stream; no COPC parsing")
        reader.close_fd = True
        with patch("laspy.CopcReader.open", return_value=reader):
            wrapper = RoofReader("unused.copc.laz")
            wrapper.close()
        self.assertTrue(reader.source.closed)

if __name__ == "__main__":
    unittest.main()
