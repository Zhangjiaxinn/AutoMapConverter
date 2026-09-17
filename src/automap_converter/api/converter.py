"""Stable Python API for AutoMapConverter map-conversion workflows."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Dict, List, Sequence, Union

from ..validation.diagnostics._workflow_runtime import (
    ConversionSpec,
    convert_lanelet2_to_opendrive,
    convert_lanelet2_to_osm,
    convert_opendrive_to_lanelet2,
    convert_osm_to_lanelet2,
    run_single,
)
from ..validation.diagnostics.source_map_precheck import (
    detect_repairable_source_issues,
    prepare_source_if_needed,
)
from .result_layout import allocate_batch_run, allocate_result_run, write_result_manifest
from .settings import current_runtime_settings, load_runtime_settings, use_runtime_settings


class MapFormat(str, Enum):
    """Map formats supported by the public conversion API."""

    LANELET2 = "lanelet2"
    OPENDRIVE = "opendrive"
    OSM = "osm"
    RASTER = "raster"


PathLike = Union[str, Path]


@dataclass
class ConversionResult:
    """Files and conversion notes produced by one conversion."""

    conversion: str
    source: Path
    target: Path
    prepared_source: Path | None = None
    conversion_notes: List[Dict[str, str]] = field(default_factory=list)
    diagnostics_report: Path | None = None
    artifacts: Dict[str, Path] = field(default_factory=dict)


_CONVERSION_NAMES = {
    (MapFormat.LANELET2, MapFormat.OPENDRIVE): "lanelet2_to_opendrive",
    (MapFormat.OPENDRIVE, MapFormat.LANELET2): "opendrive_to_lanelet2",
    (MapFormat.OSM, MapFormat.LANELET2): "osm_to_lanelet2",
    (MapFormat.LANELET2, MapFormat.OSM): "lanelet2_to_osm",
    (MapFormat.OSM, MapFormat.OPENDRIVE): "osm_to_opendrive",
    (MapFormat.OPENDRIVE, MapFormat.OSM): "opendrive_to_osm",
    (MapFormat.LANELET2, MapFormat.RASTER): "lanelet2_to_raster",
}

_COMPOSED_CONVERSIONS = {
    "osm_to_opendrive": ("osm_to_lanelet2", "lanelet2_to_opendrive"),
    "opendrive_to_osm": ("opendrive_to_lanelet2", "lanelet2_to_osm"),
}


def _as_format(value: MapFormat | str) -> MapFormat:
    if isinstance(value, MapFormat):
        return value
    try:
        return MapFormat(str(value).lower())
    except ValueError as exc:
        allowed = ", ".join(member.value for member in MapFormat)
        raise ValueError(f"Unsupported map format {value!r}; choose one of: {allowed}.") from exc


def conversion_name(source_format: MapFormat | str, target_format: MapFormat | str) -> str:
    """Return the canonical name for a supported directed conversion."""

    source = _as_format(source_format)
    target = _as_format(target_format)
    try:
        return _CONVERSION_NAMES[(source, target)]
    except KeyError as exc:
        raise ValueError(f"Unsupported conversion: {source.value} -> {target.value}.") from exc


def _intermediate_lanelet_path(source_path: Path, target_path: Path) -> Path:
    return target_path.parent / "intermediate_lanelet2" / f"{source_path.stem}.osm"


def _prepared_source_path(source_path: Path, target_path: Path) -> Path:
    """Return the run-local source-copy path used only when repair changes it."""
    candidate = target_path.parent / f"{source_path.stem}_prepared{source_path.suffix}"
    if candidate.resolve() == source_path.resolve():
        return target_path.parent / f"{source_path.stem}_prepared_input{source_path.suffix}"
    return candidate


def _prepare_source_for_direct_conversion(
    conversion: str,
    source_path: Path,
    target_path: Path,
) -> tuple[Path, Path | None, List[Dict[str, str]]]:
    """Create a prepared copy only after a read-only repair scan finds an issue."""
    repair_candidates = detect_repairable_source_issues(conversion, source_path)
    if not repair_candidates:
        return source_path, None, []

    prepared_path, repair_notes = prepare_source_if_needed(
        conversion,
        source_path,
        _prepared_source_path(source_path, target_path),
    )
    prepared_source = prepared_path if prepared_path != source_path else None
    return prepared_source or source_path, prepared_source, repair_candidates + repair_notes


def _convert_atomic(
    conversion: str,
    source_path: Path,
    target_path: Path,
    *,
    extract_sublayer: bool,
) -> List[Dict[str, str]]:
    intermediate_dir = target_path.parent / "intermediate_commonroad"
    if conversion == "lanelet2_to_opendrive":
        convert_lanelet2_to_opendrive(source_path, target_path, intermediate_dir)
        return []
    if conversion == "opendrive_to_lanelet2":
        convert_opendrive_to_lanelet2(source_path, target_path, intermediate_dir)
        return []
    if conversion == "osm_to_lanelet2":
        return convert_osm_to_lanelet2(
            source_path,
            target_path,
            intermediate_dir,
            extract_sublayer=extract_sublayer,
        )
    if conversion == "lanelet2_to_osm":
        convert_lanelet2_to_osm(source_path, target_path, intermediate_dir)
        return []
    if conversion == "lanelet2_to_raster":
        from ..conversion.lanelet2_to_raster import convert_lanelet2_to_raster

        settings = current_runtime_settings()
        convert_lanelet2_to_raster(
            source_path,
            target_path,
            resolution_m=settings.raster_resolution_m,
            padding_m=settings.raster_padding_m,
            supersampling=settings.raster_supersampling,
            topology_tolerance_m=settings.raster_topology_tolerance_m,
            max_output_pixels=settings.raster_max_output_pixels,
        )
        return []
    raise ValueError(f"Unhandled atomic conversion {conversion!r}.")


def _convert_without_diagnostics(
    conversion: str,
    source_path: Path,
    target_path: Path,
    *,
    extract_sublayer: bool,
) -> ConversionResult:
    target_path.parent.mkdir(parents=True, exist_ok=True)

    notes: List[Dict[str, str]] = []
    if conversion in _COMPOSED_CONVERSIONS:
        first, second = _COMPOSED_CONVERSIONS[conversion]
        intermediate_path = _intermediate_lanelet_path(source_path, target_path)
        intermediate_path.parent.mkdir(parents=True, exist_ok=True)
        notes.extend(
            _convert_atomic(
                first,
                source_path,
                intermediate_path,
                extract_sublayer=extract_sublayer,
            )
        )
        notes.extend(
            _convert_atomic(
                second,
                intermediate_path,
                target_path,
                extract_sublayer=extract_sublayer,
            )
        )
    else:
        notes = _convert_atomic(
            conversion,
            source_path,
            target_path,
            extract_sublayer=extract_sublayer,
        )

    artifacts: Dict[str, Path] = {}
    if conversion == "lanelet2_to_raster":
        from ..conversion.lanelet2_to_raster import raster_artifact_paths

        artifacts = raster_artifact_paths(target_path).as_dict()
    return ConversionResult(
        conversion=conversion,
        source=source_path,
        target=target_path,
        conversion_notes=notes,
        artifacts=artifacts,
    )


def _convert(
    source: PathLike,
    target: PathLike,
    source_format: MapFormat | str,
    target_format: MapFormat | str,
    *,
    extract_sublayer: bool,
) -> ConversionResult:
    """Convert one vector map without writing a diagnostic report.

    Set ``extract_sublayer=False`` for OSM-origin workflows when pedestrian and
    cyclist sublayers should be omitted for a faster vehicle-focused import.
    """

    conversion = conversion_name(source_format, target_format)
    source_path = Path(source).expanduser().resolve()
    target_path = Path(target).expanduser().resolve()
    if not source_path.is_file():
        raise FileNotFoundError(f"Source map does not exist: {source_path}")
    effective_source, prepared_source, repair_notes = _prepare_source_for_direct_conversion(
        conversion,
        source_path,
        target_path,
    )
    result = _convert_without_diagnostics(
        conversion,
        effective_source,
        target_path,
        extract_sublayer=extract_sublayer,
    )
    result.source = source_path
    result.prepared_source = prepared_source
    result.conversion_notes = repair_notes + result.conversion_notes
    return result


def convert(
    source: PathLike,
    target: PathLike,
    source_format: MapFormat | str,
    target_format: MapFormat | str,
    *,
    extract_sublayer: bool | None = None,
    config_path: PathLike | None = None,
) -> ConversionResult:
    """Convert one map with settings loaded from the selected YAML profile."""

    settings = load_runtime_settings(config_path)
    effective_extract_sublayer = (
        settings.osm_extract_sublayer
        if extract_sublayer is None
        else bool(extract_sublayer)
    )
    with use_runtime_settings(settings):
        return _convert(
            source,
            target,
            source_format,
            target_format,
            extract_sublayer=effective_extract_sublayer,
        )


def _spec_for(conversion: str, source_path: Path, target_path: Path) -> ConversionSpec:
    if conversion == "lanelet2_to_opendrive":
        from ..validation.diagnostics._workflow_runtime import validate_opendrive_target

        return ConversionSpec(
            name=conversion,
            source_dir=source_path.parent,
            output_root=target_path.parent / "batch_results",
            source_suffix=".osm",
            target_suffix=".xodr",
            convert=convert_lanelet2_to_opendrive,
            validate=validate_opendrive_target,
        )
    if conversion == "opendrive_to_lanelet2":
        from ..validation.diagnostics.opendrive_lanelet2_diagnostics import create_spec
    elif conversion == "osm_to_lanelet2":
        from ..validation.diagnostics.osm_lanelet2_diagnostics import create_spec
    elif conversion == "lanelet2_to_osm":
        from ..validation.diagnostics.lanelet2_osm_diagnostics import create_spec
    elif conversion == "lanelet2_to_raster":
        from ..validation.diagnostics.lanelet2_raster_diagnostics import create_spec
    else:
        raise ValueError(f"No direct diagnostic profile exists for {conversion!r}.")
    return create_spec(source_path, target_path)


def _load_report_payload(report_path: Path) -> Dict[str, object]:
    return json.loads(report_path.with_suffix(".json").read_text(encoding="utf-8"))


def _write_composed_report(
    conversion: str,
    source_path: Path,
    target_path: Path,
    diagnostics_dir: Path,
    stage_reports: Sequence[Path],
) -> Path:
    """Write a concise index over reports from an explicit two-stage route."""

    diagnostics_dir.mkdir(parents=True, exist_ok=True)
    stages = []
    for report_path in stage_reports:
        payload = _load_report_payload(report_path)
        stages.append(
            {
                "conversion": payload.get("conversion", "unknown"),
                "result": payload.get("result", "UNKNOWN"),
                "summary": payload.get("summary", {}),
                "report": str(report_path),
                "prepared_source": payload.get("prepared_source", ""),
            }
        )
    result = (
        "FAIL"
        if any(stage["result"] in {"FAIL", "ERROR", "UNKNOWN"} for stage in stages)
        else "PASS"
    )
    if result == "PASS" and any(stage["result"] == "PASS_WITH_WARNINGS" for stage in stages):
        result = "PASS_WITH_WARNINGS"
    payload = {
        "schema": "automap-converter/composed-diagnostics/v1",
        "conversion": conversion,
        "source": str(source_path),
        "target": str(target_path),
        "prepared_source": next(
            (stage["prepared_source"] for stage in stages if stage["prepared_source"]),
            "",
        ),
        "result": result,
        "stages": stages,
    }
    stem = f"{target_path.stem}_{conversion}_diagnostics"
    json_path = diagnostics_dir / f"{stem}.json"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return json_path


def _diagnose_composed(
    conversion: str,
    source_path: Path,
    target_path: Path,
    diagnostics_dir: Path,
) -> Path:
    first, second = _COMPOSED_CONVERSIONS[conversion]
    intermediate_path = _intermediate_lanelet_path(source_path, target_path)
    first_report = run_single(
        _spec_for(first, source_path, intermediate_path),
        source_path,
        intermediate_path,
        diagnostics_dir / "stage_1",
    )
    if not intermediate_path.is_file():
        return _write_composed_report(
            conversion, source_path, target_path, diagnostics_dir, [first_report]
        )
    second_report = run_single(
        _spec_for(second, intermediate_path, target_path),
        intermediate_path,
        target_path,
        diagnostics_dir / "stage_2",
    )
    return _write_composed_report(
        conversion, source_path, target_path, diagnostics_dir, [first_report, second_report]
    )


def _convert_and_diagnose(
    source: PathLike,
    target: PathLike,
    source_format: MapFormat | str,
    target_format: MapFormat | str,
    *,
    diagnostics_dir: PathLike | None = None,
) -> ConversionResult:
    """Convert one map and write Stage 1 and Stage 2 diagnostic reports."""

    conversion = conversion_name(source_format, target_format)
    source_path = Path(source).expanduser().resolve()
    target_path = Path(target).expanduser().resolve()
    if not source_path.is_file():
        raise FileNotFoundError(f"Source map does not exist: {source_path}")
    report_dir = (
        Path(diagnostics_dir).expanduser().resolve()
        if diagnostics_dir is not None
        else target_path.parent / "diagnostics"
    )
    if conversion in _COMPOSED_CONVERSIONS:
        return ConversionResult(
            conversion=conversion,
            source=source_path,
            target=target_path,
            diagnostics_report=_diagnose_composed(
                conversion, source_path, target_path, report_dir
            ),
        )
    if conversion == "lanelet2_to_opendrive":
        from ..validation.diagnostics.lanelet2_opendrive_diagnostics import (
            diagnose_lanelet2_to_opendrive,
        )

        effective_source, prepared_source, repair_notes = _prepare_source_for_direct_conversion(
            conversion,
            source_path,
            target_path,
        )
        result = _convert_without_diagnostics(
            conversion,
            effective_source,
            target_path,
            extract_sublayer=True,
        )
        result.source = source_path
        result.prepared_source = prepared_source
        result.conversion_notes = repair_notes + result.conversion_notes
        result.diagnostics_report = diagnose_lanelet2_to_opendrive(
            effective_source,
            target_path,
            original_source_path=source_path,
            source_preparation=repair_notes,
        )
        return result
    report_path = run_single(
        _spec_for(conversion, source_path, target_path),
        source_path,
        target_path,
        report_dir,
    )
    artifacts: Dict[str, Path] = {}
    if conversion == "lanelet2_to_raster":
        from ..conversion.lanelet2_to_raster import raster_artifact_paths

        artifacts = raster_artifact_paths(target_path).as_dict()
    return ConversionResult(
        conversion=conversion,
        source=source_path,
        target=target_path,
        diagnostics_report=report_path,
        artifacts=artifacts,
    )


def convert_and_diagnose(
    source: PathLike,
    target: PathLike,
    source_format: MapFormat | str,
    target_format: MapFormat | str,
    *,
    diagnostics_dir: PathLike | None = None,
    config_path: PathLike | None = None,
) -> ConversionResult:
    """Convert and diagnose one map with settings loaded from YAML."""

    settings = load_runtime_settings(config_path)
    with use_runtime_settings(settings):
        return _convert_and_diagnose(
            source,
            target,
            source_format,
            target_format,
            diagnostics_dir=diagnostics_dir,
        )


def convert_batch_and_diagnose(
    source_dir: PathLike,
    output_dir: PathLike,
    source_format: MapFormat | str,
    target_format: MapFormat | str,
    *,
    run_name: str = "",
    limit: int | None = None,
    config_path: PathLike | None = None,
) -> Path:
    """Run one conversion route for every compatible map in a directory."""

    source_kind = _as_format(source_format)
    target_kind = _as_format(target_format)
    conversion = conversion_name(source_kind, target_kind)
    source_root = Path(source_dir).expanduser().resolve()
    destination_root = Path(output_dir).expanduser().resolve()
    suffix = ".xodr" if source_kind is MapFormat.OPENDRIVE else ".osm"
    target_suffix = (
        ".xodr"
        if target_kind is MapFormat.OPENDRIVE
        else ".tif" if target_kind is MapFormat.RASTER else ".osm"
    )
    sources = sorted(source_root.glob(f"*{suffix}"))
    if limit is not None:
        sources = sources[:limit]
    if not sources:
        raise FileNotFoundError(f"No {suffix} files found in {source_root}")

    batch_dir = allocate_batch_run(destination_root, conversion, run_name)
    rows = []
    for index, source_path in enumerate(sources, start=1):
        result_run = allocate_result_run(
            destination_root,
            conversion,
            source_path,
            target_suffix,
        )
        target_path = result_run.target
        print(f"[{index}/{len(sources)}] {conversion}: {source_path.name}")
        started = time.time()
        try:
            result = convert_and_diagnose(
                source_path,
                target_path,
                source_kind,
                target_kind,
                diagnostics_dir=result_run.directory,
                config_path=config_path,
            )
            report_payload = _load_report_payload(result.diagnostics_report)
            elapsed = time.time() - started
            write_result_manifest(
                result_run,
                result.diagnostics_report,
                elapsed_seconds=elapsed,
                result=str(report_payload.get("result", "UNKNOWN")),
            )
            row = {
                "map": source_path.name,
                "target": str(target_path),
                "result": report_payload.get("result", "UNKNOWN"),
                "diagnostics": str(result.diagnostics_report),
                "elapsed_seconds": round(elapsed, 3),
            }
        except Exception as exc:
            elapsed = time.time() - started
            write_result_manifest(
                result_run,
                None,
                elapsed_seconds=elapsed,
                result="FAIL",
            )
            row = {
                "map": source_path.name,
                "target": str(target_path),
                "result": "FAIL",
                "error": f"{type(exc).__name__}: {exc}",
                "elapsed_seconds": round(elapsed, 3),
            }
        rows.append(row)

    summary_path = batch_dir / "summary.txt"
    summary_json_path = batch_dir / "summary.json"
    counts = {
        status: sum(row["result"] == status for row in rows)
        for status in ("PASS", "PASS_WITH_WARNINGS", "FAIL")
    }
    lines = [
        f"{conversion} batch summary",
        "=" * 72,
        f"Maps={len(rows)} PASS={counts['PASS']} "
        f"PASS_WITH_WARNINGS={counts['PASS_WITH_WARNINGS']} FAIL={counts['FAIL']}",
        "",
    ]
    for row in rows:
        lines.append(
            f"{row['map']}: {row['result']} ({row['elapsed_seconds']:.1f}s)"
        )
        if row.get("diagnostics"):
            lines.append(f"  diagnostics: {row['diagnostics']}")
        if row.get("error"):
            lines.append(f"  error: {row['error']}")
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    summary_json_path.write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return summary_path
