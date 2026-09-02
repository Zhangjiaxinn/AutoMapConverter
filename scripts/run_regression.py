"""Run every sample-map conversion regression in the reproducible runtime."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from collections.abc import Iterable
from dataclasses import dataclass
from pathlib import Path

from automap_converter import convert_and_diagnose

REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SUCCESS_RESULTS = {"PASS", "PASS_WITH_WARNINGS"}
REQUIRED_TOOLS = ("odrplot", "odrviewer", "osmium", "qc_opendrive")


@dataclass(frozen=True)
class RegressionCase:
    name: str
    source: str
    source_format: str
    target_format: str

    @property
    def target_suffix(self) -> str:
        return ".xodr" if self.target_format == "opendrive" else ".osm"


SAMPLE_WORKFLOWS: dict[str, tuple[str, tuple[str, ...]]] = {
    "lanelet2": (".osm", ("opendrive", "osm")),
    "opendrive": (".xodr", ("lanelet2", "osm")),
    "osm": (".osm", ("lanelet2", "opendrive")),
}


def _case_name(source_format: str, source: Path, target_format: str) -> str:
    """Create a stable output name while preserving nested sample directories."""
    source_root = REPOSITORY_ROOT / "data" / "samples" / source_format
    relative_stem = source.relative_to(source_root).with_suffix("")
    parts = [source_format, *relative_stem.parts, "to", target_format]
    return "__".join(parts)


def discover_cases() -> tuple[RegressionCase, ...]:
    """Return every supported conversion initiated by a map in data/samples."""
    cases: list[RegressionCase] = []
    samples_root = REPOSITORY_ROOT / "data" / "samples"
    for source_format, (suffix, target_formats) in SAMPLE_WORKFLOWS.items():
        source_root = samples_root / source_format
        for source in sorted(source_root.rglob(f"*{suffix}")):
            if not source.is_file():
                continue
            relative_source = source.relative_to(REPOSITORY_ROOT)
            for target_format in target_formats:
                cases.append(
                    RegressionCase(
                        _case_name(source_format, source, target_format),
                        str(relative_source),
                        source_format,
                        target_format,
                    )
                )
    return tuple(cases)


def _read_result(report_path: Path) -> str:
    payload = json.loads(report_path.with_suffix(".json").read_text(encoding="utf-8"))
    return str(payload.get("result", "UNKNOWN"))


def _find_tool(tool: str) -> str:
    if tool == "qc_opendrive":
        configured = os.environ.get("AUTOMAP_CONVERTER_ASAM_QC_CMD", "").strip()
        if configured:
            candidate = Path(configured).expanduser()
            if candidate.is_file() and os.access(candidate, os.X_OK):
                return str(candidate)
    return shutil.which(tool) or ""


def _tool_records() -> Iterable[dict[str, str]]:
    for tool in REQUIRED_TOOLS:
        found = _find_tool(tool)
        yield {
            "name": tool,
            "status": "PASS" if found else "FAIL",
            "path": found,
        }


def run(output_dir: Path) -> int:
    output_dir.mkdir(parents=True, exist_ok=True)
    tool_records = list(_tool_records())
    rows: list[dict[str, object]] = []
    cases = discover_cases()
    if not cases:
        rows.append(
            {
                "name": "sample-discovery",
                "result": "FAIL",
                "error": "No supported maps were found under data/samples.",
            }
        )
    elif any(record["status"] == "FAIL" for record in tool_records):
        rows.append(
            {
                "name": "runtime-tools",
                "result": "FAIL",
                "error": "Required external tools are unavailable.",
            }
        )
    else:
        for case in cases:
            source = REPOSITORY_ROOT / case.source
            target = output_dir / "maps" / f"{case.name}{case.target_suffix}"
            diagnostics = output_dir / "diagnostics" / case.name
            started = time.monotonic()
            try:
                conversion = convert_and_diagnose(
                    source,
                    target,
                    case.source_format,
                    case.target_format,
                    diagnostics_dir=diagnostics,
                )
                report_path = conversion.diagnostics_report
                if report_path is None:
                    raise RuntimeError("Conversion completed without a diagnostics report.")
                result = _read_result(report_path)
                rows.append(
                    {
                        "name": case.name,
                        "source": str(source),
                        "target": str(target),
                        "result": result,
                        "diagnostics": str(report_path),
                        "elapsed_seconds": round(time.monotonic() - started, 3),
                    }
                )
            except Exception as exc:  # noqa: BLE001 - preserve complete CI artifacts.
                rows.append(
                    {
                        "name": case.name,
                        "source": str(source),
                        "result": "FAIL",
                        "error": f"{type(exc).__name__}: {exc}",
                        "elapsed_seconds": round(time.monotonic() - started, 3),
                    }
                )

    passed = sum(row.get("result") in SUCCESS_RESULTS for row in rows)
    failed = len(rows) - passed
    summary = {
        "schema": "automap-converter/map-regression/v1",
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "tools": tool_records,
        "cases": rows,
        "summary": {
            "total": len(rows),
            "passed": passed,
            "failed": failed,
            "discovered_conversion_cases": len(cases),
        },
    }
    json_path = output_dir / "summary.json"
    txt_path = output_dir / "summary.txt"
    json_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = [
        "AutoMapConverter sample-map regression",
        "=" * 72,
        "Runtime tools:",
        *[
            f"  [{record['status']}] {record['name']}: {record['path'] or 'not found'}"
            for record in tool_records
        ],
        "",
        f"Cases={len(rows)} PASS={passed} FAIL={failed} Discovered={len(cases)}",
        "",
    ]
    for row in rows:
        lines.append(f"[{row.get('result', 'UNKNOWN')}] {row['name']}")
        if row.get("diagnostics"):
            lines.append(f"  diagnostics: {row['diagnostics']}")
        if row.get("error"):
            lines.append(f"  error: {row['error']}")
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(txt_path)
    return 0 if failed == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run every supported conversion for maps under data/samples."
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=REPOSITORY_ROOT / "data" / "results" / "regression",
    )
    args = parser.parse_args()
    return run(args.output_dir.expanduser().resolve())


if __name__ == "__main__":
    raise SystemExit(main())
