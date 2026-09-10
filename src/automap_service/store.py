"""SQLite is an execution journal, not the platform's business database."""
import json
import sqlite3
import time
from contextlib import contextmanager
from pathlib import Path

from .models import TERMINAL


class Store:
    def __init__(self, root: Path):
        root.mkdir(parents=True, exist_ok=True)
        self.path = root / "tasks.sqlite3"
        with self.connect() as db:
            db.execute("PRAGMA journal_mode=WAL")
            db.execute("""CREATE TABLE IF NOT EXISTS tasks (
                id TEXT PRIMARY KEY, request TEXT NOT NULL, state TEXT NOT NULL,
                result TEXT NOT NULL DEFAULT '{}', created REAL NOT NULL, updated REAL NOT NULL)""")

    @contextmanager
    def connect(self):
        db = sqlite3.connect(self.path, timeout=10)
        db.row_factory = sqlite3.Row
        try:
            with db:
                yield db
        finally:
            db.close()

    def create(self, task_id, request, capacity=2):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            old = db.execute("SELECT request FROM tasks WHERE id=?", (task_id,)).fetchone()
            if old:
                previous = json.loads(old["request"])
                # Presigned credentials can change on an idempotent HTTP retry.
                for key in ("sourceUrl", "uploadUrls"):
                    previous.pop(key, None)
                comparable = {k: v for k, v in request.items() if k not in {"sourceUrl", "uploadUrls"}}
                if previous != comparable:
                    raise ValueError("Task id already exists with different parameters")
                return False
            count = db.execute("SELECT count(*) FROM tasks WHERE state IN ('QUEUED','RUNNING','CANCELLING')").fetchone()[0]
            if count >= capacity:
                raise OverflowError("Worker queue is full")
            now = time.time()
            db.execute("INSERT INTO tasks(id,request,state,created,updated) VALUES(?,?,'QUEUED',?,?)",
                       (task_id, json.dumps(request), now, now))
            return True

    def get(self, task_id):
        with self.connect() as db:
            row = db.execute("SELECT * FROM tasks WHERE id=?", (task_id,)).fetchone()
        return dict(row) if row else None

    def update(self, task_id, state, result=None):
        with self.connect() as db:
            db.execute("UPDATE tasks SET state=?,result=?,updated=? WHERE id=?",
                       (state, json.dumps(result or {}), time.time(), task_id))
            if state in TERMINAL:
                self._redact_request(db, task_id)

    @staticmethod
    def _redact_request(db, task_id):
        row = db.execute("SELECT request FROM tasks WHERE id=?", (task_id,)).fetchone()
        if row:
            request = json.loads(row[0])
            request.pop("sourceUrl", None)
            request.pop("uploadUrls", None)
            db.execute("UPDATE tasks SET request=? WHERE id=?", (json.dumps(request), task_id))

    def claim(self):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT * FROM tasks WHERE state='QUEUED' ORDER BY created LIMIT 1").fetchone()
            if row:
                db.execute("UPDATE tasks SET state='RUNNING',updated=? WHERE id=?", (time.time(), row["id"]))
            return dict(row) if row else None

    def cancel(self, task_id):
        with self.connect() as db:
            db.execute("BEGIN IMMEDIATE")
            row = db.execute("SELECT state FROM tasks WHERE id=?", (task_id,)).fetchone()
            if not row:
                return False
            if row[0] not in TERMINAL:
                state = "CANCELLED" if row[0] == "QUEUED" else "CANCELLING"
                db.execute("UPDATE tasks SET state=?,updated=? WHERE id=?", (state, time.time(), task_id))
                if state == "CANCELLED":
                    self._redact_request(db, task_id)
            return True

    def recover(self):
        # A container restart kills its children. Never silently rerun a partially completed map.
        with self.connect() as db:
            interrupted = [r[0] for r in db.execute("SELECT id FROM tasks WHERE state IN ('RUNNING','CANCELLING')")]
            db.execute("UPDATE tasks SET state='FAILED',result=?,updated=? WHERE state IN ('RUNNING','CANCELLING')",
                       (json.dumps({"error": "执行服务重启，原任务已中断，请重试"}), time.time()))
            for task_id in interrupted:
                self._redact_request(db, task_id)

    def expired(self, cutoff):
        with self.connect() as db:
            return [r[0] for r in db.execute("SELECT id FROM tasks WHERE updated<? AND state IN ('COMPLETED','FAILED','CANCELLED','TIMED_OUT')", (cutoff,))]
