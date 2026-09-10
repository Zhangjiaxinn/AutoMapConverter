"""Single supervisor, with one OS process group per map conversion."""
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from threading import Event

from .store import Store


def terminate(process):
    if process.poll() is None:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()
    process.wait()


def supervise(store: Store, root: Path, stop: Event):
    store.recover()
    next_cleanup = 0
    while not stop.is_set():
        if time.time() >= next_cleanup:
            # Keep scratch bounded even when the server receives no further jobs.
            for task_id in store.expired(time.time() - 86400):
                path = root / task_id
                if path.is_dir() and path.parent == root:
                    shutil.rmtree(path)
            next_cleanup = time.time() + 3600
        row = store.claim()
        if not row:
            stop.wait(1)
            continue
        directory = root / row["id"]
        directory.mkdir(exist_ok=True)
        (directory / "request.json").write_text(row["request"], encoding="utf-8")
        os.chmod(directory / "request.json", 0o600)
        process = None
        try:
            with (directory / "execution.log").open("wb") as log:
                process = subprocess.Popen(
                    [sys.executable, "-m", "automap_service.worker", str(directory)],
                    stdout=log, stderr=log, cwd=directory, start_new_session=(os.name == "posix"),
                )
                deadline = time.monotonic() + json.loads(row["request"])["timeoutSeconds"]
                forced = None
                while process.poll() is None:
                    if store.get(row["id"])["state"] == "CANCELLING":
                        forced = "CANCELLED"
                    elif stop.is_set():
                        forced = "FAILED"
                    elif time.monotonic() > deadline:
                        forced = "TIMED_OUT"
                    if forced:
                        terminate(process)
                        break
                    stop.wait(0.5)
                result_path = directory / "result.json"
                result = json.loads(result_path.read_text()) if result_path.exists() else {}
                # A cancellation racing with completion wins before publishing platform results.
                if store.get(row["id"])["state"] == "CANCELLING":
                    forced = "CANCELLED"
                state = forced or ("COMPLETED" if process.returncode == 0 else "FAILED")
                if state != "COMPLETED":
                    result = {"error": result.get("error", "任务已取消或执行中断，请查看执行状态并重试")}
                store.update(row["id"], state, result)
        except Exception:
            if process is not None:
                terminate(process)
            store.update(row["id"], "FAILED", {"error": "执行服务无法启动或读取转换结果"})
        finally:
            (directory / "request.json").unlink(missing_ok=True)
