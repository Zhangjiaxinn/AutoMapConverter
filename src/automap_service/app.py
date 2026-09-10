"""Run one uvicorn worker per container; scale using separate volumes/instances."""
import hmac
import json
import os
from contextlib import asynccontextmanager
from pathlib import Path
from threading import Event, Thread
from urllib.parse import urlsplit

from fastapi import Depends, FastAPI, Header, HTTPException

from .models import TaskRequest
from .runner import supervise
from .store import Store

root = Path(os.environ.get("AUTOMAP_DATA_DIR", "/var/lib/automap")).resolve()
token = os.environ.get("AUTOMAP_SERVICE_TOKEN", "")
allowed_hosts = set(os.environ.get("AUTOMAP_STORAGE_HOSTS", "").split(",")) - {""}
store = None


def authorize(authorization: str = Header(default="")):
    if not token or not hmac.compare_digest(authorization, "Bearer " + token):
        raise HTTPException(401, "Invalid service identity")


@asynccontextmanager
async def lifespan(app):
    global store
    if len(token) < 32 or not allowed_hosts:
        raise RuntimeError("Set a 32+ character service token and AUTOMAP_STORAGE_HOSTS")
    store = Store(root)
    stop = Event()
    thread = Thread(target=supervise, args=(store, root, stop), daemon=True)
    thread.start()
    app.state.supervisor = thread
    yield
    stop.set()
    thread.join(timeout=10)


app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None, openapi_url=None)


@app.get("/health/ready")
def ready():
    if store is None or not app.state.supervisor.is_alive():
        raise HTTPException(503, "Supervisor unavailable")
    return {"ready": True}


@app.get("/internal/capabilities", dependencies=[Depends(authorize)])
def capabilities():
    import importlib.util
    import shutil
    return {"formats": ["lanelet2", "opendrive", "osm"], "version": "0.1.0",
            "checkers": {"lanelet2": importlib.util.find_spec("lanelet2") is not None,
                         "opendriveQuality": shutil.which("qc_opendrive") is not None,
                         "osmium": shutil.which("osmium") is not None}}


@app.post("/internal/tasks", status_code=202, dependencies=[Depends(authorize)])
def create_task(request: TaskRequest):
    for url in [request.sourceUrl, *request.uploadUrls.values()]:
        if urlsplit(url).hostname not in allowed_hosts:
            raise HTTPException(422, "Storage host is not allowed")
    try:
        store.create(request.taskId, request.model_dump())
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc
    except OverflowError as exc:
        raise HTTPException(429, str(exc)) from exc
    return get_task(request.taskId)


@app.get("/internal/tasks/{task_id}", dependencies=[Depends(authorize)])
def get_task(task_id: str):
    row = store.get(task_id)
    if not row:
        raise HTTPException(404, "Task not found")
    stage = row["state"]
    progress = root / row["id"] / "stage.json"
    if row["state"] == "RUNNING" and progress.is_file():
        try:
            stage = json.loads(progress.read_text())["stage"]
        except (ValueError, OSError):
            pass
    return {"taskId": task_id, "state": row["state"], "stage": stage,
            "updatedAt": row["updated"], **json.loads(row["result"])}


@app.post("/internal/tasks/{task_id}/cancel", dependencies=[Depends(authorize)])
def cancel_task(task_id: str):
    if not store.cancel(task_id):
        raise HTTPException(404, "Task not found")
    return get_task(task_id)
