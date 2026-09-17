"""Stable result-directory layout for runnable examples and batch workflows."""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path


@dataclass(frozen=True)
class ResultRun:
    """Allocated destination for one conversion attempt."""

    conversion: str
    source: Path
    directory: Path
    target: Path
    manifest: Path
    run_id: str


def allocate_result_run(
    results_root: Path,
    conversion: str,
    source_path: Path,
    target_suffix: str,
) -> ResultRun:
    """Allocate the next numbered result directory for one source map."""
    source = source_path.expanduser().resolve()
    map_directory = results_root.expanduser().resolve() / conversion / source.stem
    run_id = _next_numbered_name(map_directory, "run")
    directory = map_directory / run_id
    directory.mkdir(parents=True, exist_ok=False)
    return ResultRun(
        conversion=conversion,
        source=source,
        directory=directory,
        target=directory / f"{source.stem}{target_suffix}",
        manifest=directory / "manifest.json",
        run_id=run_id,
    )


def allocate_batch_run(results_root: Path, conversion: str, requested_name: str = "") -> Path:
    """Allocate a directory for one batch summary without duplicating map outputs."""
    batch_root = results_root.expanduser().resolve() / conversion / "_batches"
    if requested_name:
        candidate = batch_root / requested_name
        if not candidate.exists():
            candidate.mkdir(parents=True, exist_ok=False)
            return candidate
    run_id = _next_numbered_name(batch_root, "batch")
    directory = batch_root / run_id
    directory.mkdir(parents=True, exist_ok=False)
    return directory


def write_result_manifest(
    result_run: ResultRun,
    diagnostics_report: Path | None,
    *,
    elapsed_seconds: float | None = None,
    result: str = "UNKNOWN",
) -> Path:
    """Record the inputs and outcome needed to identify a conversion attempt."""
    payload_prepared_source = ""
    if diagnostics_report is not None:
        diagnostics_report = diagnostics_report.expanduser().resolve()
        diagnostic_json = diagnostics_report.with_suffix(".json")
        if diagnostic_json.is_file():
            try:
                diagnostic_payload = json.loads(diagnostic_json.read_text(encoding="utf-8"))
                result = str(diagnostic_payload.get("result", result))
                prepared_source = str(diagnostic_payload.get("prepared_source", ""))
            except (OSError, json.JSONDecodeError):
                pass
            else:
                payload_prepared_source = prepared_source
    payload = {
        "schema_version": "automap-converter/result-run/v1",
        "generated": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "conversion": result_run.conversion,
        "run": result_run.run_id,
        "source": str(result_run.source),
        "prepared_source": payload_prepared_source,
        "target": str(result_run.target),
        "diagnostics": str(diagnostics_report) if diagnostics_report is not None else "",
        "result": result,
        "elapsed_seconds": round(elapsed_seconds, 3) if elapsed_seconds is not None else None,
    }
    result_run.manifest.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return result_run.manifest


def _next_numbered_name(parent: Path, prefix: str) -> str:
    highest = 0
    if parent.is_dir():
        for child in parent.iterdir():
            if not child.is_dir():
                continue
            match = re.fullmatch(rf"{re.escape(prefix)}_(\d{{3,}})$", child.name)
            if match:
                highest = max(highest, int(match.group(1)))
    return f"{prefix}_{highest + 1:03d}"
