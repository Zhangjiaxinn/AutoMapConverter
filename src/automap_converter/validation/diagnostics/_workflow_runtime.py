from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import time
import traceback
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from lxml import etree


PROJECT_ROOT = Path(__file__).resolve().parents[4]
os.environ.setdefault("MPLCONFIGDIR", str(PROJECT_ROOT / ".cache" / "matplotlib"))

from commonroad.planning.planning_problem import PlanningProblemSet  # noqa: E402
from commonroad.scenario.scenario import Tag  # noqa: E402

from automap_converter.core.config.general_config import general_config  # noqa: E402
from automap_converter.core.config.lanelet2_config import lanelet2_config  # noqa: E402
from automap_converter.core.config.opendrive_config import open_drive_config  # noqa: E402
from automap_converter.core.config.osm_config import osm_config  # noqa: E402
from automap_converter.core.file_writer import (  # noqa: E402
    CommonRoadScenarioWriter,
    OverwriteExistingFile,
)
from automap_converter.api.interface import (  # noqa: E402
    osm_commonroad_file_to_lanelet2 as commonroad_to_lanelet,
    lanelet2_to_opendrive as lanelet_to_opendrive,
    lanelet2_to_osm as lanelet_to_osm,
    opendrive_to_lanelet2 as opendrive_to_lanelet,
    osm_to_commonroad,
)
from automap_converter.api.settings import current_runtime_settings  # noqa: E402
from automap_converter.api.result_layout import (  # noqa: E402
    allocate_batch_run,
    allocate_result_run,
    write_result_manifest,
)
from .source_map_precheck import (  # noqa: E402
    detect_repairable_source_issues,
    inspect_source_for_conversion,
    prepare_source_if_needed,
)
from ._semantic_engine import diagnose_stage2  # noqa: E402


STATUS_ORDER = ("PASS", "WARN", "FAIL", "REVIEW", "SKIP")


@dataclass
class Stage1Check:
    category: str
    status: str
    summary: str
    method: str = ""
    target: str = ""
    hint: str = ""
    details: List[Dict[str, object]] = field(default_factory=list)


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def _count_statuses(checks: Sequence[Stage1Check]) -> Dict[str, int]:
    return {status.lower(): sum(1 for check in checks if check.status == status) for status in STATUS_ORDER}


def _result_from_checks(checks: Sequence[Stage1Check]) -> str:
    if any(check.status == "FAIL" for check in checks):
        return "FAIL"
    if any(check.status == "WARN" for check in checks):
        return "PASS_WITH_WARNINGS"
    if any(check.status == "PASS" for check in checks):
        return "PASS"
    if any(check.status == "SKIP" for check in checks):
        return "SKIP"
    return "UNKNOWN"


def _raise_on_source_validation_failure(records: Sequence[Dict[str, str]]) -> None:
    failures = [record for record in records if record.get("status") == "FAIL"]
    if not failures:
        return
    message = "; ".join(
        f"{record.get('category', 'source-validation')}: {record.get('summary', '')}"
        for record in failures[:3]
    )
    raise ValueError(f"Source map validation failed: {message}")


def _which(names: Sequence[str]) -> Optional[str]:
    for name in names:
        path = shutil.which(name)
        if path:
            return path
    return None


def _run_command(command: Sequence[str], timeout: int = 20) -> Tuple[int, str]:
    completed = subprocess.run(
        list(command),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=False,
    )
    return completed.returncode, completed.stdout.strip()


def _launch_viewer_enabled() -> bool:
    return current_runtime_settings().launch_viewer or os.environ.get(
        "AUTOMAP_CONVERTER_LAUNCH_VIEWER", ""
    ).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _try_launch_viewer(command: Sequence[str], timeout: float = 3.0) -> Tuple[str, str]:
    try:
        process = subprocess.Popen(
            list(command),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    except Exception as exc:
        return "FAIL", f"{type(exc).__name__}: {exc}"
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        return "REVIEW", "Viewer process stayed open; manual visual comparison is required."
    output = (output or "").strip()
    if process.returncode == 0:
        return "REVIEW", output[:1000]
    return "FAIL", output[:1000] or f"Viewer exited with code {process.returncode}."


def validate_lanelet2_target(target_path: Path) -> List[Stage1Check]:
    checks: List[Stage1Check] = []
    try:
        import lanelet2

        origin = lanelet2.io.Origin(
            float(lanelet2_config.routing_origin_lat),
            float(lanelet2_config.routing_origin_lon),
        )
        lanelet2.io.load(str(target_path), origin)
        checks.append(
            Stage1Check(
                category="lanelet2-loader",
                status="PASS",
                summary="Lanelet2 Python loader loaded the target without exception.",
                method="lanelet2.io.load",
                target=_relative(target_path),
            )
        )
    except Exception as exc:
        checks.append(
            Stage1Check(
                category="lanelet2-loader",
                status="FAIL",
                summary="Lanelet2 Python loader failed to load the target.",
                method="lanelet2.io.load",
                target=_relative(target_path),
                hint=f"{type(exc).__name__}: {exc}",
            )
        )

    rviz = _which(("rviz2", "rviz"))
    visual_status = "SKIP"
    visual_summary = "RViz is unavailable; manual Lanelet2 visualization was not started."
    visual_hint = ""
    if rviz and _launch_viewer_enabled():
        visual_status, visual_hint = _try_launch_viewer((rviz,))
        visual_summary = (
            "RViz was launched; load the Lanelet2 map display/plugin for manual comparison."
            if visual_status == "REVIEW"
            else "RViz launch failed."
        )
    elif rviz:
        visual_status = "REVIEW"
        visual_summary = "RViz is available for manual Lanelet2 visual comparison."
    checks.append(
        Stage1Check(
            category="visual-comparison",
            status=visual_status,
            summary=visual_summary,
            method=(
                "Set AUTOMAP_CONVERTER_LAUNCH_VIEWER=1 to launch RViz from the validator; "
                "otherwise open the target with an RViz Lanelet2 map display/plugin."
            ),
            target=f"{rviz} <lanelet2 map display>" if rviz else "",
            hint=visual_hint,
        )
    )
    return checks


def validate_opendrive_target(target_path: Path) -> List[Stage1Check]:
    """Check basic OpenDRIVE readability and record viewer availability."""
    checks: List[Stage1Check] = []
    try:
        root = etree.parse(str(target_path)).getroot()
        if root.tag != "OpenDRIVE":
            raise ValueError(f"Root tag is {root.tag!r}, expected 'OpenDRIVE'.")
        roads = root.findall("road")
        if not roads:
            raise ValueError("OpenDRIVE contains no road elements.")
        checks.append(
            Stage1Check(
                category="opendrive-xml",
                status="PASS",
                summary=f"Target is well-formed OpenDRIVE XML (roads={len(roads)}).",
                method="lxml XML parser",
                target=_relative(target_path),
            )
        )
    except Exception as exc:
        checks.append(
            Stage1Check(
                category="opendrive-xml",
                status="FAIL",
                summary="OpenDRIVE XML parsing failed.",
                method="lxml XML parser",
                target=_relative(target_path),
                hint=f"{type(exc).__name__}: {exc}",
            )
        )

    viewer = _which(("odrviewer",))
    checks.append(
        Stage1Check(
            category="visual-comparison",
            status="REVIEW" if viewer else "SKIP",
            summary=(
                "odrviewer is available for manual OpenDRIVE visual comparison."
                if viewer
                else "odrviewer is unavailable; manual OpenDRIVE visualization was not started."
            ),
            method="odrviewer" if viewer else "Install esmini odrviewer for manual visual comparison.",
            target=f"{viewer} {target_path}" if viewer else "",
        )
    )
    return checks


def validate_osm_target(target_path: Path) -> List[Stage1Check]:
    checks: List[Stage1Check] = []
    try:
        root = etree.parse(str(target_path)).getroot()
        if root.tag != "osm":
            raise ValueError(f"Root tag is {root.tag!r}, expected 'osm'.")
        node_count = len(root.findall("node"))
        way_count = len(root.findall("way"))
        relation_count = len(root.findall("relation"))
        checks.append(
            Stage1Check(
                category="osm-xml",
                status="PASS",
                summary=(
                    "Target is well-formed OSM XML "
                    f"(nodes={node_count}, ways={way_count}, relations={relation_count})."
                ),
                method="lxml XML parser and OSM root check",
                target=_relative(target_path),
            )
        )
    except Exception as exc:
        checks.append(
            Stage1Check(
                category="osm-xml",
                status="FAIL",
                summary="Target OSM XML cannot be parsed.",
                method="lxml XML parser and OSM root check",
                target=_relative(target_path),
                hint=f"{type(exc).__name__}: {exc}",
            )
        )

    osmium = _which(("osmium",))
    if osmium:
        try:
            code, output = _run_command((osmium, "check-refs", str(target_path)))
            checks.append(
                Stage1Check(
                    category="osmium-check-refs",
                    status="PASS" if code == 0 else "FAIL",
                    summary=(
                        "osmium check-refs completed successfully."
                        if code == 0
                        else f"osmium check-refs returned {code}."
                    ),
                    method="osmium check-refs",
                    target=f"{osmium} check-refs {_relative(target_path)}",
                    hint=output[:1000],
                )
            )
        except Exception as exc:
            checks.append(
                Stage1Check(
                    category="osmium-check-refs",
                    status="FAIL",
                    summary="osmium check-refs could not run.",
                    method="osmium check-refs",
                    hint=f"{type(exc).__name__}: {exc}",
                )
            )
    else:
        checks.append(
            Stage1Check(
                category="osmium-check-refs",
                status="SKIP",
                summary="osmium is unavailable; OSM reference validation was skipped.",
                method="Install osmium-tool to enable strict OSM reference checks.",
            )
        )

    viewer = _which(("josm", "qgis"))
    visual_status = "SKIP"
    visual_summary = "JOSM/QGIS is unavailable; manual OSM visualization was not started."
    visual_hint = ""
    if viewer and _launch_viewer_enabled():
        visual_status, visual_hint = _try_launch_viewer((viewer, str(target_path)))
        visual_summary = (
            "OSM visual tool was launched; manual source/target comparison is required."
            if visual_status == "REVIEW"
            else "OSM visual tool launch failed."
        )
    elif viewer:
        visual_status = "REVIEW"
        visual_summary = "OSM visual tool is available for manual comparison."
    checks.append(
        Stage1Check(
            category="visual-comparison",
            status=visual_status,
            summary=visual_summary,
            method=(
                "Set AUTOMAP_CONVERTER_LAUNCH_VIEWER=1 to launch JOSM/QGIS from the validator; "
                "otherwise open the target in JOSM or QGIS."
            ),
            target=f"{viewer} {_relative(target_path)}" if viewer else "",
            hint=visual_hint,
        )
    )
    return checks


def write_stage1_report(
    diagnostics_dir: Path,
    conversion: str,
    source_path: Path,
    target_path: Path,
    checks: Sequence[Stage1Check],
    elapsed: float,
    error: str = "",
    source_precheck: Optional[Sequence[Dict[str, str]]] = None,
    prepared_source: Optional[Path] = None,
    conversion_notes: Optional[Sequence[Dict[str, str]]] = None,
) -> Tuple[Path, Dict[str, object]]:
    diagnostics_dir.mkdir(parents=True, exist_ok=True)
    result = "ERROR" if error else _result_from_checks(checks)
    counts = _count_statuses(checks)
    payload = {
        "schema_version": "1.0",
        "stage": "stage1_acceptance",
        "conversion": conversion,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": str(source_path),
        "target": str(target_path),
        "elapsed_seconds": round(elapsed, 3),
        "result": result,
        "summary": counts,
        "error": error,
        "source_precheck": _non_pass_records(source_precheck),
        "prepared_source": str(prepared_source) if prepared_source else "",
        "conversion_notes": _non_pass_records(conversion_notes),
        "acceptance": [asdict(check) for check in checks if check.status != "PASS"],
    }
    stem = target_path.stem
    json_path = diagnostics_dir / f"{stem}_stage1_diagnostics.json"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return json_path, payload


def _final_diagnostics_path(diagnostics_dir: Path, target_path: Path) -> Path:
    """Return the single user-facing report path for one conversion run."""

    return diagnostics_dir / f"{target_path.stem}_diagnostics.json"


def _retain_stage1_as_final_report(
    stage1_path: Path,
    diagnostics_dir: Path,
    target_path: Path,
    *,
    stage2_error: str = "",
) -> Path:
    """Promote a Stage 1 report when no combined report can be produced."""

    final_path = _final_diagnostics_path(diagnostics_dir, target_path)
    if stage2_error:
        payload = json.loads(stage1_path.read_text(encoding="utf-8"))
        payload["result"] = "ERROR"
        payload["error"] = stage2_error
        payload["stage2"] = {
            "status": "FAIL",
            "summary": "Stage 2 diagnostics raised an exception.",
            "hint": stage2_error.splitlines()[-1] if stage2_error else "",
        }
        stage1_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
    if stage1_path != final_path:
        stage1_path.replace(final_path)
    return final_path


def _non_pass_records(
    records: Optional[Sequence[Dict[str, str]]],
) -> List[Dict[str, str]]:
    """Keep run reports focused on actionable source and conversion records."""
    return [record for record in records or [] if record.get("status") != "PASS"]


def convert_opendrive_to_lanelet2(source_path: Path, target_path: Path, _: Path) -> None:
    target_path.parent.mkdir(parents=True, exist_ok=True)
    lanelet2_config.autoware = False
    lanelet2_config.use_local_coordinates = True
    opendrive_to_lanelet(
        input_file=str(source_path),
        output_file=str(target_path),
        opendrive_conf=open_drive_config,
        general_conf=general_config,
        lanelet_conf=lanelet2_config,
    )


def convert_lanelet2_to_opendrive(source_path: Path, target_path: Path, _: Path) -> None:
    """Convert one Lanelet2 map with the packaged OpenDRIVE converter."""
    target_path.parent.mkdir(parents=True, exist_ok=True)
    lanelet2_config.adjacencies = True
    lanelet_to_opendrive(
        str(source_path),
        str(target_path),
        lanelet_conf=lanelet2_config,
        opendrive_conf=open_drive_config,
        enrich_topology=current_runtime_settings().lanelet2_routing_enrichment,
    )


def convert_osm_to_lanelet2(
    source_path: Path,
    target_path: Path,
    intermediate_dir: Path,
    extract_sublayer: bool = True,
) -> List[Dict[str, str]]:
    target_path.parent.mkdir(parents=True, exist_ok=True)
    intermediate_dir.mkdir(parents=True, exist_ok=True)
    old_extract_sublayer = osm_config.EXTRACT_SUBLAYER
    old_main_highways = dict(osm_config.ACCEPTED_HIGHWAYS_MAINLAYER)
    configured_sublayer = {
        highway
        for highway, enabled in osm_config.ACCEPTED_HIGHWAYS_SUBLAYER.items()
        if enabled
    }
    source_highways = {
        str(tag.get("v", ""))
        for tag in etree.parse(str(source_path)).xpath(
            ".//*[local-name()='way']/*[local-name()='tag'][@k='highway']"
        )
    }
    effective_extract_sublayer = bool(
        extract_sublayer
        and current_runtime_settings().osm_extract_sublayer
        and configured_sublayer.intersection(source_highways)
    )
    accepted_main_highways = dict(old_main_highways)
    # `unclassified` is the standard OSM fallback for a motor road whose more
    # specific highway class is unknown. The Lanelet2 -> OSM converter emits it,
    # so rejecting it here makes the two public conversion directions asymmetric.
    accepted_main_highways["unclassified"] = True
    osm_config.EXTRACT_SUBLAYER = effective_extract_sublayer
    osm_config.ACCEPTED_HIGHWAYS_MAINLAYER = accepted_main_highways
    try:
        scenario = osm_to_commonroad(str(source_path))
    finally:
        osm_config.EXTRACT_SUBLAYER = old_extract_sublayer
        osm_config.ACCEPTED_HIGHWAYS_MAINLAYER = old_main_highways
    commonroad_path = intermediate_dir / f"{source_path.stem}.xml"
    writer = CommonRoadScenarioWriter(
        scenario=scenario,
        planning_problem_set=PlanningProblemSet(),
        author="AutoMapConverter",
        affiliation="CommonRoad",
        source="OSM batch conversion",
        tags={Tag.URBAN},
    )
    writer.write_to_file(str(commonroad_path), OverwriteExistingFile.ALWAYS)
    lanelet2_config.autoware = False
    lanelet2_config.use_local_coordinates = True
    commonroad_to_lanelet(str(commonroad_path), str(target_path), lanelet_conf=lanelet2_config)
    records = [
        {
            "category": "osm-sublayer",
            "status": "WARN" if effective_extract_sublayer else "PASS",
            "summary": (
                "Enabled OSM service/footway/cycleway/path sublayer during OSM -> Lanelet2 conversion."
                if effective_extract_sublayer
                else (
                    "OSM sublayer extraction was not needed because the source contains no enabled sublayer highways."
                    if extract_sublayer
                    else "OSM service/footway/cycleway/path sublayer was disabled for this conversion."
                )
            ),
            "hint": (
                "This improves pedestrian/cyclist semantic coverage; Stage2 will show whether extra lanelets affect target quality."
                if effective_extract_sublayer
                else "Use --extract-sublayer when pedestrian/cyclist semantics are required."
            ),
        }
    ]
    return records


def convert_lanelet2_to_osm(source_path: Path, target_path: Path, _: Path) -> None:
    target_path.parent.mkdir(parents=True, exist_ok=True)
    lanelet2_config.adjacencies = True
    osm_out = lanelet_to_osm(
        str(source_path),
        lanelet_conf=lanelet2_config,
        enrich_topology=current_runtime_settings().lanelet2_routing_enrichment,
    )
    osm_out.write_xml(target_path)


@dataclass
class ConversionSpec:
    name: str
    source_dir: Path
    output_root: Path
    source_suffix: str
    target_suffix: str
    convert: Callable[[Path, Path, Path], Optional[List[Dict[str, str]]]]
    validate: Callable[[Path], List[Stage1Check]]
    validate_source: Callable[[str, Path], List[Dict[str, str]]] = inspect_source_for_conversion


def _prepare_source_input(
    spec: ConversionSpec,
    source_path: Path,
    prepared_output_path: Path,
) -> Tuple[Path, Optional[Path], List[Dict[str, str]]]:
    """Validate first, then prepare a run-local copy only for detected issues."""
    initial_validation = spec.validate_source(spec.name, source_path)
    repair_candidates = detect_repairable_source_issues(spec.name, source_path)
    if not repair_candidates:
        _raise_on_source_validation_failure(initial_validation)
        return source_path, None, initial_validation

    prepared_path, repair_records = prepare_source_if_needed(
        spec.name,
        source_path,
        prepared_output_path,
    )
    prepared_source = prepared_path if prepared_path != source_path else None
    effective_source = prepared_source or source_path
    effective_validation = spec.validate_source(spec.name, effective_source)
    repair_categories = {record.get("category", "") for record in repair_records}
    compact_validation = [
        record
        for record in effective_validation
        if record.get("category", "") not in repair_categories
    ]
    _raise_on_source_validation_failure(effective_validation)
    return (
        effective_source,
        prepared_source,
        repair_candidates + repair_records + compact_validation,
    )


def run_single(
    spec: ConversionSpec,
    source_path: Path,
    target_path: Path,
    diagnostics_dir: Path,
    intermediate_dir: Optional[Path] = None,
) -> Path:
    started = time.time()
    checks: List[Stage1Check] = []
    error = ""
    source_precheck: List[Dict[str, str]] = []
    prepared_source: Optional[Path] = None
    conversion_notes: List[Dict[str, str]] = []
    if intermediate_dir is None:
        intermediate_dir = target_path.parent / "intermediate_commonroad"
    try:
        effective_source, prepared_source, source_precheck = _prepare_source_input(
            spec,
            source_path,
            diagnostics_dir / f"{source_path.stem}_prepared{source_path.suffix}",
        )
        conversion_notes = spec.convert(effective_source, target_path, intermediate_dir) or []
        checks = spec.validate(target_path)
    except Exception:
        error = traceback.format_exc()
        checks = [
            Stage1Check(
                category="conversion",
                status="FAIL",
                summary="Conversion raised an exception before target validation.",
                hint=error.splitlines()[-1] if error else "",
            )
        ]
        print(f"ERROR: {error.splitlines()[-1] if error else 'unknown error'}")
    stage1_path, _ = write_stage1_report(
        diagnostics_dir,
        spec.name,
        source_path,
        target_path,
        checks,
        time.time() - started,
        error=error,
        source_precheck=source_precheck,
        prepared_source=prepared_source,
        conversion_notes=conversion_notes,
    )
    if not error:
        try:
            stage1_payload = json.loads(stage1_path.read_text(encoding="utf-8"))
            combined_path, _ = diagnose_stage2(
                spec.name,
                prepared_source or source_path,
                target_path,
                diagnostics_dir,
                stage1_payload=stage1_payload,
            )
        except Exception:
            stage2_error = traceback.format_exc()
            final_path = _retain_stage1_as_final_report(
                stage1_path,
                diagnostics_dir,
                target_path,
                stage2_error=stage2_error,
            )
            print(f"Diagnostics written to: {final_path}")
            return final_path
        stage1_path.unlink(missing_ok=True)
        print(f"Diagnostics written to: {combined_path}")
        return combined_path
    final_path = _retain_stage1_as_final_report(stage1_path, diagnostics_dir, target_path)
    print(f"Diagnostics written to: {final_path}")
    return final_path


def run_batch(spec: ConversionSpec, args: argparse.Namespace) -> Path:
    source_dir_arg = getattr(args, "batch_source_dir", "") or getattr(args, "source_dir", "")
    source_dir = Path(source_dir_arg) if source_dir_arg else spec.source_dir
    if not source_dir.is_absolute():
        source_dir = PROJECT_ROOT / source_dir
    sources = sorted(source_dir.glob(f"*{spec.source_suffix}"))
    if args.limit is not None:
        sources = sources[: args.limit]
    if not sources:
        raise FileNotFoundError(f"No {spec.source_suffix} files found in {source_dir}")

    batch_dir = allocate_batch_run(spec.output_root, spec.name, args.run_name)
    work_dir = batch_dir / "_work"

    rows: List[Dict[str, object]] = []
    for index, source_path in enumerate(sources, start=1):
        started = time.time()
        result_run = allocate_result_run(
            spec.output_root,
            spec.name,
            source_path,
            spec.target_suffix,
        )
        target_path = result_run.target
        print(f"[{index}/{len(sources)}] {spec.name}: {_relative(source_path)}")
        checks: List[Stage1Check] = []
        error = ""
        source_precheck: List[Dict[str, str]] = []
        prepared_source: Optional[Path] = None
        conversion_notes: List[Dict[str, str]] = []
        try:
            effective_source, prepared_source, source_precheck = _prepare_source_input(
                spec,
                source_path,
                result_run.directory / f"{source_path.stem}_prepared{source_path.suffix}",
            )
            conversion_notes = spec.convert(
                effective_source,
                target_path,
                work_dir / source_path.stem,
            ) or []
            checks = spec.validate(target_path)
        except Exception:
            error = traceback.format_exc()
            checks = [
                Stage1Check(
                    category="conversion",
                    status="FAIL",
                    summary="Conversion raised an exception before target validation.",
                    hint=error.splitlines()[-1] if error else "",
                )
            ]
            print(f"  ERROR: {error.splitlines()[-1] if error else 'unknown error'}")
        elapsed = time.time() - started
        stage1_path, payload = write_stage1_report(
            result_run.directory,
            spec.name,
            source_path,
            target_path,
            checks,
            elapsed,
            error=error,
            source_precheck=source_precheck,
            prepared_source=prepared_source,
            conversion_notes=conversion_notes,
        )
        row = {
            "map": source_path.name,
            "source": _relative(source_path),
            "target": _relative(target_path),
            "diagnostics": _relative(stage1_path),
            "result": payload["result"],
            "stage1_result": payload["result"],
            "stage1_pass": payload["summary"]["pass"],
            "stage1_warn": payload["summary"]["warn"],
            "stage1_fail": payload["summary"]["fail"],
            "stage1_review": payload["summary"]["review"],
            "stage1_skip": payload["summary"]["skip"],
            "stage2_result": "SKIP" if error else "UNKNOWN",
            "stage2_pass": 0,
            "stage2_warn": 0,
            "stage2_fail": 0,
            "stage2_review": 0,
            "stage2_skip": 1 if error else 0,
            "elapsed_seconds": round(elapsed, 3),
            **payload["summary"],
        }
        if error:
            row["error"] = error.splitlines()[-1] if error.splitlines() else "conversion error"
            final_path = _retain_stage1_as_final_report(
                stage1_path, result_run.directory, target_path
            )
            row["diagnostics"] = _relative(final_path)
            write_result_manifest(
                result_run,
                final_path,
                elapsed_seconds=elapsed,
                result="FAIL",
            )
        else:
            try:
                combined_path, combined_payload = diagnose_stage2(
                    spec.name,
                    prepared_source or source_path,
                    target_path,
                    result_run.directory,
                    stage1_payload=payload,
                )
            except Exception:
                stage2_error = traceback.format_exc()
                final_path = _retain_stage1_as_final_report(
                    stage1_path,
                    result_run.directory,
                    target_path,
                    stage2_error=stage2_error,
                )
                row["diagnostics"] = _relative(final_path)
                row["result"] = "FAIL"
                row["stage2_result"] = "FAIL"
                row["stage2_fail"] = 1
                row["error"] = stage2_error.splitlines()[-1]
                write_result_manifest(
                    result_run,
                    final_path,
                    elapsed_seconds=elapsed,
                    result="FAIL",
                )
                rows.append(row)
                continue
            stage1_path.unlink(missing_ok=True)
            row["diagnostics"] = _relative(combined_path)
            row["result"] = combined_payload["result"]
            row["stage2_result"] = combined_payload["result"]
            row["stage2_pass"] = combined_payload["summary"]["pass"]
            row["stage2_warn"] = combined_payload["summary"]["warn"]
            row["stage2_fail"] = combined_payload["summary"]["fail"]
            row["stage2_review"] = combined_payload["summary"]["review"]
            row["stage2_skip"] = combined_payload["summary"]["skip"]
            write_result_manifest(
                result_run,
                combined_path,
                elapsed_seconds=elapsed,
                result=str(combined_payload["result"]),
            )
        rows.append(row)

    summary_path = batch_dir / "summary.txt"
    summary_json_path = batch_dir / "summary.json"
    write_batch_summary(summary_path, spec.name, rows, batch_dir)
    summary_json_path.write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    shutil.rmtree(work_dir, ignore_errors=True)
    print(f"Batch summary written to: {summary_path}")
    return summary_path


def write_batch_summary(
    summary_path: Path,
    conversion: str,
    rows: Sequence[Dict[str, object]],
    batch_dir: Path,
) -> None:
    totals = {
        "maps": len(rows),
        "stage1_pass": sum(1 for row in rows if row.get("stage1_result") == "PASS"),
        "stage1_warn": sum(1 for row in rows if row.get("stage1_result") == "PASS_WITH_WARNINGS"),
        "stage1_fail": sum(1 for row in rows if row.get("stage1_result") in {"FAIL", "ERROR"}),
        "stage1_skip": sum(1 for row in rows if row.get("stage1_result") == "SKIP"),
        "stage2_pass": sum(1 for row in rows if row.get("stage2_result") == "PASS"),
        "stage2_warn": sum(1 for row in rows if row.get("stage2_result") == "PASS_WITH_WARNINGS"),
        "stage2_fail": sum(1 for row in rows if row.get("stage2_result") in {"FAIL", "ERROR"}),
        "stage2_skip": sum(1 for row in rows if row.get("stage2_result") == "SKIP"),
    }
    lines = [
        f"{conversion} Stage1+Stage2 Batch Summary",
        "=" * 112,
        f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        f"Batch directory: {_relative(batch_dir)}",
        (
            f"Stage1: Maps={totals['maps']} PASS={totals['stage1_pass']} "
            f"PASS_WITH_WARNINGS={totals['stage1_warn']} "
            f"FAIL/ERROR={totals['stage1_fail']} SKIP={totals['stage1_skip']}"
        ),
        (
            f"Stage2: PASS={totals['stage2_pass']} "
            f"PASS_WITH_WARNINGS={totals['stage2_warn']} "
            f"FAIL/ERROR={totals['stage2_fail']} SKIP={totals['stage2_skip']}"
        ),
        "",
        (
            f"{'#':>2}  {'map':<52} {'S1':<18} {'S2':<18} "
            f"{'P2':>3} {'W2':>3} {'F2':>3} {'R2':>3} {'S2k':>3} {'sec':>8}"
        ),
        "-" * 112,
    ]
    for index, row in enumerate(rows, start=1):
        lines.append(
            f"{index:>2}  "
            f"{str(row['map'])[:52]:<52} "
            f"{str(row.get('stage1_result', row.get('result', 'UNKNOWN')))[:18]:<18} "
            f"{str(row.get('stage2_result', 'UNKNOWN'))[:18]:<18} "
            f"{int(row.get('stage2_pass', 0)):>3} "
            f"{int(row.get('stage2_warn', 0)):>3} "
            f"{int(row.get('stage2_fail', 0)):>3} "
            f"{int(row.get('stage2_review', 0)):>3} "
            f"{int(row.get('stage2_skip', 0)):>3} "
            f"{float(row.get('elapsed_seconds', 0.0)):>8.1f}"
        )
        if row.get("error"):
            lines.append(f"    error: {row['error']}")
    lines.extend(
        [
            "",
            "Legend:",
            "  Stage1 checks target loadability/readability and visual-tool availability.",
            "  Stage2 checks target structure, reference/topology validity and source-element preservation.",
        "  Lanelet2 target: lanelet2.io.load + RViz availability.",
        "  OSM target: XML/OSM parse + optional osmium check-refs + JOSM/QGIS availability.",
        "  Raster target: GDAL/rasterio schema checks + generated PNG previews.",
            "  REVIEW means the visualization tool is available but manual visual comparison is still required.",
            "  SKIP means an optional external tool is not installed; it is not counted as conversion failure.",
            "  P2/W2/F2/R2/S2k = PASS/WARN/FAIL/REVIEW/SKIP counts for Stage2 only.",
        ]
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
