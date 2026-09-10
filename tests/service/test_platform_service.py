"""Contract/lifecycle tests run without Lanelet2; engine regression is separate."""
import json
import sys
import threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from fastapi.testclient import TestClient

from automap_service import app as api
from automap_service.models import TaskRequest
from automap_service.store import Store
from automap_service.worker import execute, validate_xml

TASK_ID = "aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"


def request(**overrides):
    return TaskRequest(**{
        "taskId": TASK_ID, "sourceFormat": "osm", "targetFormat": "lanelet2",
        "sourceUrl": "http://storage/source?signature=secret",
        "uploadUrls": {kind: "http://storage/" + kind for kind in ["map", "report", "bundle"]},
        **overrides,
    }).model_dump()


def test_durable_idempotency_and_recovery(tmp_path):
    store = Store(tmp_path)
    assert store.create(TASK_ID, request())
    assert not store.create(TASK_ID, request(sourceUrl="http://storage/source?signature=new"))
    with pytest.raises(ValueError):
        store.create(TASK_ID, request(targetFormat="opendrive"))
    assert store.claim()["id"] == TASK_ID
    store = Store(tmp_path)
    store.recover()
    assert store.get(TASK_ID)["state"] == "FAILED"
    assert "sourceUrl" not in json.loads(store.get(TASK_ID)["request"])
    assert not store.create(TASK_ID, request())


def test_atomic_claim_across_consumers(tmp_path):
    store = Store(tmp_path)
    store.create(TASK_ID, request())
    claimed = []
    threads = [threading.Thread(target=lambda: claimed.append(store.claim())) for _ in range(8)]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert sum(item is not None for item in claimed) == 1


def test_cancellation_and_queue_limit(tmp_path):
    store = Store(tmp_path)
    store.create(TASK_ID, request(), capacity=1)
    with pytest.raises(OverflowError):
        store.create("bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb", request(), capacity=1)
    assert store.cancel(TASK_ID)
    assert store.get(TASK_ID)["state"] == "CANCELLED"
    assert store.claim() is None


def test_http_auth_host_restriction_and_credentials_not_exposed(tmp_path, monkeypatch):
    monkeypatch.setattr(api, "root", tmp_path)
    monkeypatch.setattr(api, "token", "a" * 32)
    monkeypatch.setattr(api, "allowed_hosts", {"storage"})
    monkeypatch.setattr(api, "supervise", lambda store, root, stop: stop.wait())
    headers = {"Authorization": "Bearer " + "a" * 32}
    with TestClient(api.app) as client:
        assert client.post("/internal/tasks", json=request()).status_code == 401
        assert client.post("/internal/tasks", json=request(sourceUrl="http://other/private"), headers=headers).status_code == 422
        response = client.post("/internal/tasks", json=request(), headers=headers)
        assert response.status_code == 202
        assert "sourceUrl" not in response.text and "secret" not in response.text
        assert client.post("/internal/tasks", json=request(), headers=headers).status_code == 202
        response = client.post(f"/internal/tasks/{TASK_ID}/cancel", headers=headers)
        assert response.json()["state"] == "CANCELLED"
        assert client.get("/internal/tasks/unknown", headers=headers).status_code == 404


@pytest.mark.parametrize("xml,kind", [
    ('<!DOCTYPE osm [<!ENTITY x SYSTEM "file:///etc/passwd">]><osm>&x;</osm>', "osm"),
    ('<osm/>', "lanelet2"), ('<OpenDRIVE/>', "osm"),
    ('<osm><relation><tag k="type" v="lanelet"/></relation></osm>', "osm"),
])
def test_rejects_external_entities_and_format_mismatch(tmp_path, xml, kind):
    path = tmp_path / "source.osm"
    path.write_text(xml)
    with pytest.raises(ValueError):
        validate_xml(path, kind)


def test_accepts_namespaced_opendrive(tmp_path):
    path = tmp_path / "source.xodr"
    path.write_text('<OpenDRIVE xmlns="http://www.opendrive.org"><header/></OpenDRIVE>')
    validate_xml(path, "opendrive")


def test_worker_contract_uploads_artifacts_and_keeps_quality_failure(tmp_path, monkeypatch):
    (tmp_path / "request.json").write_text(json.dumps(request()))
    from automap_service import worker
    monkeypatch.setattr(worker, "download", lambda url, path, max_bytes: path.write_text('<osm/>'))
    def fake_convert(source, target, *args, **kwargs):
        target.write_text('<osm><relation><tag k="type" v="lanelet"/></relation></osm>')
        report = tmp_path / "diagnostics" / "diagnostics.txt"
        report.parent.mkdir()
        report.write_text("FAIL " + str(tmp_path / "source.osm"))
        report.with_suffix('.json').write_text(json.dumps({"result": "FAIL", "source": str(source)}))
        return SimpleNamespace(diagnostics_report=report, conversion="osm_to_lanelet2")
    monkeypatch.setitem(sys.modules, "automap_converter", SimpleNamespace(convert=fake_convert, convert_and_diagnose=fake_convert))
    put = Mock(return_value=SimpleNamespace(status_code=200))
    monkeypatch.setattr(worker.requests, "put", put)
    execute(tmp_path)
    result = json.loads((tmp_path / "result.json").read_text())
    assert result["quality"] == "FAIL"  # Execution completed, quality did not pass.
    assert {a['kind'] for a in result['artifacts']} == {"map", "report", "bundle"}
    assert put.call_count == 3
    assert str(tmp_path) not in (tmp_path / "report.json").read_text()


def test_supervisor_honors_running_cancel(tmp_path, monkeypatch):
    from automap_service import runner
    store = Store(tmp_path)
    store.create(TASK_ID, request())
    stop = threading.Event()
    class Process:
        returncode = None
        def poll(self): return self.returncode
    process = Process()
    def start(*args, **kwargs):
        store.cancel(TASK_ID)
        return process
    def kill(p):
        p.returncode = -9
        stop.set()
    monkeypatch.setattr(runner.subprocess, "Popen", start)
    monkeypatch.setattr(runner, "terminate", kill)
    runner.supervise(store, tmp_path, stop)
    assert store.get(TASK_ID)["state"] == "CANCELLED"


def test_supervisor_enforces_timeout(tmp_path, monkeypatch):
    from automap_service import runner
    store = Store(tmp_path)
    store.create(TASK_ID, request(timeoutSeconds=30))
    stop = threading.Event()
    class Process:
        returncode = None
        def poll(self): return self.returncode
    process = Process()
    def kill(p):
        p.returncode = -9
        stop.set()
    monkeypatch.setattr(runner.subprocess, "Popen", lambda *args, **kwargs: process)
    monkeypatch.setattr(runner, "terminate", kill)
    times = iter([0, 31])
    monkeypatch.setattr(runner.time, "monotonic", lambda: next(times))
    runner.supervise(store, tmp_path, stop)
    assert store.get(TASK_ID)["state"] == "TIMED_OUT"
