"""Authenticated file/job API around the existing SPT command.

Run manually on the lab computer. No SSH, system configuration, file cleanup,
or changes to existing input/model/result files are performed by this service.
"""
from __future__ import annotations

import argparse
import ast
import asyncio
from concurrent.futures import ThreadPoolExecutor
from contextlib import asynccontextmanager
import hashlib
import hmac
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from uuid import UUID, uuid4

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parents[1]
API_VERSION = 1
SERVICE_REVISION = "roof-service-v3-20261106"


def worker_identity():
    path = ROOT / "scripts/prediction_worker.py"
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    revision = "legacy"
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "WORKER_REVISION" for t in node.targets):
            revision = ast.literal_eval(node.value)
    return {"worker_revision": revision, "worker_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest()}


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path, value):
    temporary = path.with_name(path.name + "." + uuid4().hex + ".tmp")
    with temporary.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2, allow_nan=False)
    os.replace(temporary, path)


def file_sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def verified_result(path):
    """Only accept a published, immutable final result; never rerun prediction."""
    manifest = read_json(path / "result.json")
    result = path / "result.gpkg"
    if (manifest.get("complete") is not True or manifest.get("job_id") != path.name or
            manifest.get("new_layer") != "predicted_buildings" or
            not isinstance(manifest.get("feature_count"), int) or manifest["feature_count"] < 0 or
            not result.is_file() or result.stat().st_size != manifest.get("bytes") or
            file_sha256(result) != manifest.get("sha256")):
        raise ValueError("Result integrity check failed")
    return manifest


class JobOptions(BaseModel):
    pointcloud_name: str = Field(default="input.copc.laz", max_length=255)
    height_field: str = Field(default="", max_length=255)
    target_fields: list[str] = Field(default_factory=list, max_length=1000)


def create_app(jobs_root, token, checkpoint=None, runner=None, max_upload_bytes=200 * 1024**3, predictor_python=None):
    """runner is a test injection; production always runs the separate worker."""
    jobs_root = Path(jobs_root).resolve()
    checkpoint = Path(checkpoint or ROOT / "spt-2_dales.ckpt").resolve()
    predictor_python = Path(predictor_python or sys.executable).resolve()
    if runner is None and not predictor_python.is_file():
        raise ValueError("Prediction Python interpreter does not exist")
    if not token or len(token) < 16 or not token.isascii() or any(c.isspace() for c in token):
        raise ValueError("SPT_API_TOKEN must contain at least 16 ASCII characters without whitespace")
    # The root is dedicated to new remote jobs, never the ordinary testresult.
    if jobs_root in (ROOT, ROOT / "testdata", ROOT / "testdata/testresult"):
        raise ValueError("Choose a dedicated prediction_jobs directory")
    jobs_root.mkdir(parents=True, exist_ok=True)
    mutex = threading.RLock()
    uploading = set()
    executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="spt-job")

    def job_dir(job_id):
        try:
            if str(UUID(job_id)) != job_id:
                raise ValueError()
        except ValueError:
            raise HTTPException(400, "Invalid job ID")
        path = jobs_root / job_id
        if not path.is_dir() or not (path / "job.json").is_file():
            raise HTTPException(404, "Job not found")
        return path

    def update(path, **changes):
        with mutex:
            state = read_json(path / "job.json")
            state.update(changes, updated_at=time.time())
            write_json(path / "job.json", state)
            return state

    def execute(path):
        update(path, status="running", message="正在执行预测任务", **worker_identity())
        try:
            if runner is not None:
                runner(path)
            else:
                command = [str(predictor_python), "-u", str(ROOT / "scripts/prediction_worker.py"),
                           "--job-dir", str(path), "--checkpoint", str(checkpoint)]
                # Fixed argument list, no shell or client-supplied executable/path.
                with (path / "worker.log").open("xb") as log:
                    worker_environment = dict(os.environ)
                    worker_environment.pop("SPT_API_TOKEN", None)
                    process = subprocess.Popen(command, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT, env=worker_environment)
                    update(path, worker_pid=process.pid)
                    code = process.wait()
                if code:
                    raise RuntimeError("实验室处理失败；请查看该任务的 worker.log 和 progress.json")
            manifest = verified_result(path)
            update(path, status="completed", message="预测和结果写入完成", result=manifest)
        except Exception as exc:
            update(path, status="failed", message=str(exc))

    @asynccontextmanager
    async def lifespan(app):
        lock = (jobs_root / ".service.lock").open("a+b")
        try:
            if os.name == "nt":
                import msvcrt
                if lock.seek(0, 2) == 0:
                    lock.write(b"0")
                    lock.flush()
                lock.seek(0)
                msvcrt.locking(lock.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            lock.close()
            raise RuntimeError("Another service already owns this jobs directory; run one service only")
        # Preserve old task directories. Never rerun a possibly active GPU job.
        for path in jobs_root.iterdir():
            if path.is_dir() and (path / "job.json").is_file():
                state = read_json(path / "job.json")
                if state.get("status") in ("queued", "running"):
                    update(path, status="interrupted", message="服务重启；旧任务和文件已保留，请检查实验室任务日志")
        try:
            yield
        finally:
            # Running workers are allowed to finish; no processes are killed.
            executor.shutdown(wait=True)
            lock.close()

    app = FastAPI(title="SPT building prediction", lifespan=lifespan)

    async def authenticate(request: Request):
        supplied = request.headers.get("Authorization", "")
        if not hmac.compare_digest(supplied.encode("utf-8"), ("Bearer " + token).encode("utf-8")):
            raise HTTPException(401, "Invalid access token")

    auth = [Depends(authenticate)]

    @app.get("/v1/health", dependencies=auth)
    def health():
        return {"api_version": API_VERSION, "service": "spt-buildings", "checkpoint_exists": checkpoint.is_file(),
                "service_revision": SERVICE_REVISION, **worker_identity()}

    @app.post("/v1/jobs", dependencies=auth, status_code=201)
    def create_job(options: JobOptions):
        if len(set(options.target_fields)) != len(options.target_fields):
            raise HTTPException(400, "Duplicate target field names")
        if options.height_field and options.height_field not in options.target_fields:
            raise HTTPException(400, "Height field is not in the target layer")
        identity = str(uuid4())
        path = jobs_root / identity
        path.mkdir(exist_ok=False)
        state = {"job_id": identity, "status": "uploading", "options": options.model_dump(),
                 "files": {}, "created_at": time.time(), "updated_at": time.time()}
        write_json(path / "job.json", state)
        return {"job_id": identity, "status": "uploading"}

    @app.put("/v1/jobs/{job_id}/{kind}", dependencies=auth)
    async def upload(job_id: str, kind: str, request: Request):
        if kind not in ("pointcloud", "target"):
            raise HTTPException(404, "Unknown upload kind")
        path = job_dir(job_id)
        with mutex:
            state = read_json(path / "job.json")
            if state["status"] != "uploading" or job_id in uploading or kind in state["files"]:
                raise HTTPException(409, "Job is busy or this file has already been uploaded")
            uploading.add(job_id)
        temporary = path / (kind + "." + uuid4().hex + ".partial")
        destination = path / ("input.copc.laz" if kind == "pointcloud" else "target.gpkg")
        digest, count = hashlib.sha256(), 0
        try:
            try:
                expected = int(request.headers.get("content-length", "-1"))
            except ValueError:
                raise HTTPException(400, "Invalid content length")
            if expected <= 0 or expected > max_upload_bytes:
                raise HTTPException(413, "Empty upload or upload exceeds server size limit")
            with temporary.open("xb") as stream:
                async for chunk in request.stream():
                    count += len(chunk)
                    if count > expected or count > max_upload_bytes:
                        raise HTTPException(413, "Upload size exceeded")
                    digest.update(chunk)
                    await asyncio.to_thread(stream.write, chunk)
            if count != expected:
                raise HTTPException(400, "Incomplete upload")
            # link() refuses to overwrite; partial files are intentionally kept.
            os.link(temporary, destination)
            with mutex:
                state = read_json(path / "job.json")
                state["files"][kind] = {"bytes": count, "sha256": digest.hexdigest()}
                write_json(path / "job.json", state)
            return state["files"][kind]
        finally:
            with mutex:
                uploading.discard(job_id)

    @app.post("/v1/jobs/{job_id}/start", dependencies=auth, status_code=202)
    def start_job(job_id: str):
        path = job_dir(job_id)
        with mutex:
            state = read_json(path / "job.json")
            if state["status"] != "uploading" or job_id in uploading:
                raise HTTPException(409, "Job has already started or is uploading")
            if set(state["files"]) != {"pointcloud", "target"}:
                raise HTTPException(400, "Upload both the point cloud and target layer first")
            if runner is None and not checkpoint.is_file():
                raise HTTPException(503, "Server checkpoint is missing")
            update(path, status="queued", message="排队等待实验室 GPU")
            executor.submit(execute, path)
        return {"job_id": job_id, "status": "queued"}

    @app.get("/v1/jobs/{job_id}/status", dependencies=auth)
    def status(job_id: str):
        path = job_dir(job_id)
        state = read_json(path / "job.json")
        # Older workers could fail during CopcReader cleanup AFTER publishing
        # the complete GPKG. Recover on the user's existing "resume" request.
        # Preserve old metadata/logs, and never accept partial or corrupt files.
        if state["status"] in ("failed", "interrupted"):
            with mutex:
                try:
                    manifest = verified_result(path)
                except (OSError, ValueError, KeyError, TypeError):
                    pass
                else:
                    write_json(path / ("job.before_recovery." + uuid4().hex + ".json"), state)
                    state = update(path, status="completed", message="已校验并恢复已有最终结果，无需重新预测",
                                   recovered=True, result=manifest)
        progress = path / "progress.json"
        return {"job_id": job_id, "status": state["status"], "message": state.get("message", ""),
                "progress": read_json(progress) if progress.is_file() else {}}

    def completed(job_id):
        path = job_dir(job_id)
        if read_json(path / "job.json")["status"] != "completed":
            raise HTTPException(409, "Result is not complete")
        return path

    @app.get("/v1/jobs/{job_id}/result-info", dependencies=auth)
    def result_info(job_id: str):
        return read_json(completed(job_id) / "result.json")

    @app.get("/v1/jobs/{job_id}/result", dependencies=auth)
    def download(job_id: str):
        return FileResponse(completed(job_id) / "result.gpkg", media_type="application/geopackage+sqlite3",
                            filename="buildings_" + job_id + ".gpkg")

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--version", action="version", version=SERVICE_REVISION)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--jobs-dir", type=Path, default=ROOT / "prediction_jobs")
    parser.add_argument("--checkpoint", type=Path, default=ROOT / "spt-2_dales.ckpt")
    parser.add_argument("--python", type=Path, help="Python of the existing working SPT/conda environment; service dependencies can live in a separate venv.")
    args = parser.parse_args()
    import uvicorn
    app = create_app(args.jobs_dir, os.environ.get("SPT_API_TOKEN", ""), args.checkpoint, predictor_python=args.python)
    print("Service: " + SERVICE_REVISION + "; worker: " + worker_identity()["worker_revision"], flush=True)
    uvicorn.run(app, host=args.host, port=args.port, workers=1, access_log=False)


if __name__ == "__main__":
    main()
