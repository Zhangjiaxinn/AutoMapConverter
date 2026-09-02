from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
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
from .source_map_precheck import (  # noqa: E402
    prepare_source_for_conversion,
    repair_target_after_conversion,
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


def _raise_on_source_precheck_failure(records: Sequence[Dict[str, str]]) -> None:
    failures = [record for record in records if record.get("status") == "FAIL"]
    if not failures:
        return
    message = "; ".join(
        f"{record.get('category', 'source-precheck')}: {record.get('summary', '')}"
        for record in failures[:3]
    )
    raise ValueError(f"Source map precheck failed: {message}")


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
    conversion_notes: Optional[Sequence[Dict[str, str]]] = None,
    target_repair: Optional[Sequence[Dict[str, str]]] = None,
) -> Tuple[Path, Path, Dict[str, object]]:
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
        "source_precheck": list(source_precheck or []),
        "conversion_notes": list(conversion_notes or []),
        "target_repair": list(target_repair or []),
        "acceptance": [asdict(check) for check in checks],
    }
    stem = target_path.stem
    json_path = diagnostics_dir / f"{stem}_stage1_diagnostics.json"
    txt_path = diagnostics_dir / f"{stem}_stage1_diagnostics.txt"
    json_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    lines = [
        f"{conversion} Stage1 Acceptance Report",
        "=" * 72,
        f"Source: {_relative(source_path)}",
        f"Target: {_relative(target_path)}",
        f"Result: {result}",
        (
            f"PASS={counts['pass']} WARN={counts['warn']} FAIL={counts['fail']} "
            f"REVIEW={counts['review']} SKIP={counts['skip']}"
        ),
    ]
    if error:
        lines.extend(["", "Conversion error:", error])
    if source_precheck:
        lines.append("")
        lines.append("Source precheck / repair:")
        for record in source_precheck:
            lines.append(
                f"[{record.get('status', 'UNKNOWN')}] {record.get('category', 'source-precheck')}: "
                f"{record.get('summary', '')}"
            )
            if record.get("hint"):
                lines.append(f"  hint: {record['hint']}")
    if conversion_notes:
        lines.append("")
        lines.append("Conversion notes:")
        for record in conversion_notes:
            lines.append(
                f"[{record.get('status', 'UNKNOWN')}] {record.get('category', 'conversion-note')}: "
                f"{record.get('summary', '')}"
            )
            if record.get("hint"):
                lines.append(f"  hint: {record['hint']}")
    if target_repair:
        lines.append("")
        lines.append("Target postprocess / repair:")
        for record in target_repair:
            lines.append(
                f"[{record.get('status', 'UNKNOWN')}] {record.get('category', 'target-repair')}: "
                f"{record.get('summary', '')}"
            )
            if record.get("hint"):
                lines.append(f"  hint: {record['hint']}")
    lines.append("")
    for check in checks:
        lines.append(f"[{check.status}] {check.category}: {check.summary}")
        if check.method:
            lines.append(f"  method: {check.method}")
        if check.target:
            lines.append(f"  target: {check.target}")
        if check.hint:
            lines.append(f"  hint: {check.hint}")
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return txt_path, json_path, payload


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
    preprocess_source: Callable[[str, Path, Path], Tuple[Path, List[Dict[str, str]]]] = (
        prepare_source_for_conversion
    )
    postprocess_target: Callable[[str, Path, Path], List[Dict[str, str]]] = (
        repair_target_after_conversion
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
    conversion_notes: List[Dict[str, str]] = []
    target_repair: List[Dict[str, str]] = []
    if intermediate_dir is None:
        intermediate_dir = target_path.parent / "intermediate_commonroad"
    try:
        prepared_source, source_precheck = spec.preprocess_source(
            spec.name,
            source_path,
            target_path.parent / "preprocessed_sources",
        )
        _raise_on_source_precheck_failure(source_precheck)
        conversion_notes = spec.convert(prepared_source, target_path, intermediate_dir) or []
        target_repair = spec.postprocess_target(
            spec.name,
            target_path,
            target_path.parent / "target_repairs",
        )
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
    txt_path, json_path, _ = write_stage1_report(
        diagnostics_dir,
        spec.name,
        source_path,
        target_path,
        checks,
        time.time() - started,
        error=error,
        source_precheck=source_precheck,
        conversion_notes=conversion_notes,
        target_repair=target_repair,
    )
    print(f"Stage1 report written to: {txt_path}")
    print(f"Stage1 JSON written to: {json_path}")
    if not error:
        stage1_payload = json.loads(json_path.read_text(encoding="utf-8"))
        combined_txt, combined_json, _ = diagnose_stage2(
            spec.name,
            source_path,
            target_path,
            diagnostics_dir,
            stage1_payload=stage1_payload,
        )
        print(f"Stage1+Stage2 report written to: {combined_txt}")
        print(f"Stage1+Stage2 JSON written to: {combined_json}")
        return combined_txt
    return txt_path


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

    run_id = args.run_name or time.strftime("%Y%m%d_%H%M%S")
    batch_dir = spec.output_root / run_id
    output_dir = batch_dir / "output"
    diagnostics_dir = batch_dir / "diagnostics"
    intermediate_dir = batch_dir / "intermediate_commonroad"
    output_dir.mkdir(parents=True, exist_ok=True)
    diagnostics_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, object]] = []
    for index, source_path in enumerate(sources, start=1):
        started = time.time()
        target_path = output_dir / f"{source_path.stem}{spec.target_suffix}"
        print(f"[{index}/{len(sources)}] {spec.name}: {_relative(source_path)}")
        checks: List[Stage1Check] = []
        error = ""
        source_precheck: List[Dict[str, str]] = []
        conversion_notes: List[Dict[str, str]] = []
        target_repair: List[Dict[str, str]] = []
        try:
            prepared_source, source_precheck = spec.preprocess_source(
                spec.name,
                source_path,
                batch_dir / "preprocessed_sources",
            )
            _raise_on_source_precheck_failure(source_precheck)
            conversion_notes = spec.convert(prepared_source, target_path, intermediate_dir) or []
            target_repair = spec.postprocess_target(
                spec.name,
                target_path,
                batch_dir / "target_repairs",
            )
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
        txt_path, json_path, payload = write_stage1_report(
            diagnostics_dir,
            spec.name,
            source_path,
            target_path,
            checks,
            elapsed,
            error=error,
            source_precheck=source_precheck,
            conversion_notes=conversion_notes,
            target_repair=target_repair,
        )
        row = {
            "map": source_path.name,
            "source": _relative(source_path),
            "target": _relative(target_path),
            "diagnostics_txt": _relative(txt_path),
            "diagnostics_json": _relative(json_path),
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
        else:
            combined_txt, combined_json, combined_payload = diagnose_stage2(
                spec.name,
                source_path,
                target_path,
                diagnostics_dir,
                stage1_payload=payload,
            )
            row["diagnostics_txt"] = _relative(combined_txt)
            row["diagnostics_json"] = _relative(combined_json)
            row["result"] = combined_payload["result"]
            row["stage2_result"] = combined_payload["result"]
            row["stage2_pass"] = combined_payload["summary"]["pass"]
            row["stage2_warn"] = combined_payload["summary"]["warn"]
            row["stage2_fail"] = combined_payload["summary"]["fail"]
            row["stage2_review"] = combined_payload["summary"]["review"]
            row["stage2_skip"] = combined_payload["summary"]["skip"]
        rows.append(row)

    summary_path = batch_dir / "summary.txt"
    summary_json_path = batch_dir / "summary.json"
    write_batch_summary(summary_path, spec.name, rows, batch_dir)
    summary_json_path.write_text(
        json.dumps(rows, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
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
            "  REVIEW means the visualization tool is available but manual visual comparison is still required.",
            "  SKIP means an optional external tool is not installed; it is not counted as conversion failure.",
            "  P2/W2/F2/R2/S2k = PASS/WARN/FAIL/REVIEW/SKIP counts for Stage2 only.",
        ]
    )
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
