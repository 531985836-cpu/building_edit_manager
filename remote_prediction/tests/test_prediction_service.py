"""Local synthetic API/vector tests. Never load SPT or run point-cloud prediction."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from uuid import uuid4

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from fastapi.testclient import TestClient
from prediction_service import create_app, write_json
from prediction_worker import build_result, roof_elevation

TOKEN = "local-test-token-not-a-real-secret"
AUTH = {"Authorization": "Bearer " + TOKEN}


def fake_runner(path):
    content = b"synthetic result transport fixture"
    (path / "result.gpkg").write_bytes(content)
    write_json(path / "result.json", {"complete": True, "job_id": path.name, "bytes": len(content),
                                      "sha256": hashlib.sha256(content).hexdigest(), "feature_count": 2,
                                      "new_layer": "predicted_buildings"})


class ApiTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix="spt-api-test-")
        self.root = Path(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def create(self, client):
        response = client.post("/v1/jobs", headers=AUTH, json={"target_fields": ["height"]})
        self.assertEqual(response.status_code, 201, response.text)
        return response.json()["job_id"]

    def upload(self, client, job):
        for kind in ("pointcloud", "target"):
            response = client.put(f"/v1/jobs/{job}/{kind}", headers=AUTH, content=(kind + " fixture").encode())
            self.assertEqual(response.status_code, 200, response.text)

    def wait_complete(self, client, job):
        for _ in range(200):
            status = client.get(f"/v1/jobs/{job}/status", headers=AUTH).json()
            if status["status"] in ("completed", "failed"):
                self.assertEqual(status["status"], "completed", status)
                return
            time.sleep(0.01)
        self.fail("Synthetic job did not complete")

    def test_authenticated_upload_queue_download_preserves_inputs(self):
        sentinel = self.root / "existing-user-file.txt"
        sentinel.write_text("must not change")
        app = create_app(self.root / "jobs", TOKEN, runner=fake_runner)
        with TestClient(app) as client:
            self.assertEqual(client.get("/v1/health").status_code, 401)
            job = self.create(client)
            self.assertEqual(client.post(f"/v1/jobs/{job}/start", headers=AUTH).status_code, 400)
            self.upload(client, job)
            self.assertEqual(client.get(f"/v1/jobs/{job}/result", headers=AUTH).status_code, 409)
            response = client.post(f"/v1/jobs/{job}/start", headers=AUTH)
            self.assertEqual(response.status_code, 202, response.text)
            self.wait_complete(client, job)
            info = client.get(f"/v1/jobs/{job}/result-info", headers=AUTH).json()
            content = client.get(f"/v1/jobs/{job}/result", headers=AUTH).content
            self.assertEqual(hashlib.sha256(content).hexdigest(), info["sha256"])
            self.assertEqual(client.post(f"/v1/jobs/{job}/start", headers=AUTH).status_code, 409)
            self.assertEqual(client.put(f"/v1/jobs/{job}/pointcloud", headers=AUTH, content=b"replace").status_code, 409)
            self.assertEqual((self.root / "jobs" / job / "input.copc.laz").read_bytes(), b"pointcloud fixture")
        self.assertEqual(sentinel.read_text(), "must not change")

    def test_invalid_field_path_and_incomplete_upload_rejected(self):
        with TestClient(create_app(self.root / "jobs", TOKEN, runner=fake_runner, max_upload_bytes=32)) as client:
            self.assertEqual(client.post("/v1/jobs", headers=AUTH, json={"height_field": "bad", "target_fields": ["height"]}).status_code, 400)
            self.assertEqual(client.get("/v1/jobs/not-a-uuid/status", headers=AUTH).status_code, 400)
            job = self.create(client)
            response = client.put(f"/v1/jobs/{job}/pointcloud", headers={**AUTH, "Content-Length": "8"}, content=b"abc")
            self.assertEqual(response.status_code, 400)
            self.assertFalse((self.root / "jobs" / job / "input.copc.laz").exists())
            self.assertEqual(client.put(f"/v1/jobs/{job}/target", headers=AUTH, content=b"x" * 33).status_code, 413)

    def test_recover_published_result_but_reject_corrupt_result(self):
        with TestClient(create_app(self.root / "jobs", TOKEN, runner=fake_runner)) as client:
            for corrupt in (False, True):
                job = self.create(client)
                self.upload(client, job)
                client.post(f"/v1/jobs/{job}/start", headers=AUTH)
                self.wait_complete(client, job)
                path = self.root / "jobs" / job
                old = json.loads((path / "job.json").read_text(encoding="utf-8"))
                old.update(status="failed", message="CopcReader cleanup failed")
                write_json(path / "job.json", old)
                original_manifest = (path / "result.json").read_bytes()
                if corrupt:
                    (path / "result.gpkg").write_bytes(b"corrupt")
                state = client.get(f"/v1/jobs/{job}/status", headers=AUTH).json()
                self.assertEqual(state["status"], "failed" if corrupt else "completed")
                self.assertEqual((path / "result.json").read_bytes(), original_manifest)
                backups = list(path.glob("job.before_recovery.*.json"))
                self.assertEqual(len(backups), 0 if corrupt else 1)
                if not corrupt:
                    self.assertEqual(json.loads(backups[0].read_text(encoding="utf-8"))["status"], "failed")
                    self.assertEqual(client.get(f"/v1/jobs/{job}/result", headers=AUTH).status_code, 200)
                else:
                    self.assertEqual(client.get(f"/v1/jobs/{job}/result", headers=AUTH).status_code, 409)

    def test_gpu_jobs_are_serial_and_failures_not_exposed_as_results(self):
        active, maximum = 0, 0
        lock = threading.Lock()
        def runner(path):
            nonlocal active, maximum
            with lock:
                active += 1
                maximum = max(maximum, active)
            time.sleep(0.05)
            fake_runner(path)
            with lock:
                active -= 1
        with TestClient(create_app(self.root / "jobs", TOKEN, runner=runner)) as client:
            jobs = [self.create(client) for _ in range(2)]
            for job in jobs:
                self.upload(client, job)
                client.post(f"/v1/jobs/{job}/start", headers=AUTH)
            for job in jobs:
                self.wait_complete(client, job)
            self.assertEqual(maximum, 1)
        def fail(path):
            raise ValueError("synthetic failure")
        with TestClient(create_app(self.root / "failed", TOKEN, runner=fail)) as client:
            job = self.create(client)
            self.upload(client, job)
            client.post(f"/v1/jobs/{job}/start", headers=AUTH)
            for _ in range(100):
                if client.get(f"/v1/jobs/{job}/status", headers=AUTH).json()["status"] == "failed":
                    break
                time.sleep(0.01)
            self.assertEqual(client.get(f"/v1/jobs/{job}/result", headers=AUTH).status_code, 409)


class ResultTests(unittest.TestCase):
    def test_final_vectors_append_preserving_original_and_height_nulls(self):
        import fiona
        from shapely.geometry import Polygon, mapping, shape
        with tempfile.TemporaryDirectory(prefix="spt-result-test-") as directory:
            job = Path(directory) / str(uuid4())
            job.mkdir()
            target = job / "target.gpkg"
            schema = {"geometry": "Polygon", "properties": {"fid": "int", "name": "str", "height": "float", "area_m2": "float"}}
            with fiona.open(target, "w", driver="GPKG", layer="target", crs="EPSG:4326", schema=schema, FID="__snapshot_id") as dst:
                dst.write({"geometry": mapping(Polygon([(1, 1), (2, 1), (2, 2), (1, 2)])),
                           "properties": {"fid": 7, "name": "existing", "height": 42.0, "area_m2": 99.0}})
            before = hashlib.sha256(target.read_bytes()).hexdigest()
            predicted = job / "predicted.gpkg"
            with fiona.open(predicted, "w", driver="GPKG", layer="buildings", crs="EPSG:28992",
                            schema={"geometry": "Polygon", "properties": {"area_m2": "float"}}) as dst:
                for x in (155000, 155100):
                    dst.write({"geometry": mapping(Polygon([(x, 463000), (x + 10, 463000), (x + 10, 463010), (x, 463010)])),
                               "properties": {"area_m2": 100.0}})
            calls = []
            def heights(polygon):
                calls.append(polygon.bounds)
                return (115.25, "ok") if len(calls) == 1 else (None, "no_roof_candidates")
            manifest = build_result(job, predicted, {"target_fields": list(schema["properties"]), "height_field": "height"}, heights)
            self.assertEqual(hashlib.sha256(target.read_bytes()).hexdigest(), before)
            self.assertEqual(manifest["feature_count"], 2)
            self.assertEqual(manifest["height_failed"], 1)
            self.assertGreater(calls[0][0], 100000)  # height queried in point-cloud CRS
            with fiona.open(job / "result.gpkg", layer="target") as final:
                records = list(final)
                self.assertEqual(len(records), 3)
                self.assertEqual(records[0]["properties"]["name"], "existing")
            with fiona.open(job / "result.gpkg", layer="predicted_buildings") as incremental:
                records = list(incremental)
                self.assertEqual(len(records), 2)
                self.assertAlmostEqual(records[0]["properties"]["height"], 115.25)
                self.assertIsNone(records[1]["properties"]["height"])
                self.assertLess(shape(records[0]["geometry"]).bounds[0], 10)  # reprojected to longitude
            with self.assertRaises(ValueError):
                build_result(job, predicted, {"target_fields": [], "height_field": ""})

    def test_roof_elevation_absolute_and_failure_explicit(self):
        import numpy as np
        roof = np.array([[10 + x / 10, 10 + y / 10, 115.0] for x in range(9) for y in range(9)])
        points = np.vstack(([0.0, 0.0, 100.0], roof))
        elevation, status = roof_elevation(points)
        self.assertEqual(status, "ok")
        self.assertAlmostEqual(elevation, 115.0)
        value, status = roof_elevation(roof)
        self.assertIsNone(value)
        self.assertEqual(status, "no_roof_candidates")


if __name__ == "__main__":
    unittest.main()
