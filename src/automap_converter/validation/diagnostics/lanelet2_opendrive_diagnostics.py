from __future__ import annotations

import json
import math
import os
import shlex
import shutil
import subprocess
import time
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

from lxml import etree


Point2D = Tuple[float, float]
PROJECT_ROOT = Path(__file__).resolve().parents[4]


def _conversion_artifact_path(target_path: Path, subdir: str, suffix: str) -> Path:
    """Place generated reports beside the organized conversion output tree."""
    if (
        target_path.parent.name == "output"
        and target_path.parent.parent.name == "lanelet2_to_opendrive"
    ):
        artifact_dir = target_path.parent.parent / subdir
        artifact_dir.mkdir(parents=True, exist_ok=True)
        return artifact_dir / f"{target_path.stem}{suffix}"
    return target_path.with_name(f"{target_path.stem}{suffix}")


@dataclass
class CheckResult:
    stage: str
    category: str
    status: str
    summary: str
    source: str = ""
    target: str = ""
    method: str = ""
    location: str = ""
    hint: str = ""
    details: List[Dict[str, object]] = field(default_factory=list)


@dataclass
class SourceStats:
    nodes: int = 0
    ways: int = 0
    lanelets: int = 0
    successor_edges: int = 0
    predecessor_edges: int = 0
    branch_lanelets: int = 0
    approximate_length: float = 0.0
    curved_lanelets: int = 0
    regulatory_elements: int = 0
    traffic_light_relations: int = 0
    traffic_light_ways: int = 0
    traffic_sign_ways: int = 0
    multipolygons: int = 0
    convertible_multipolygons: int = 0
    lanelet_subtypes: Counter = field(default_factory=Counter)
    regulatory_subtypes: Counter = field(default_factory=Counter)
    multipolygon_subtypes: Counter = field(default_factory=Counter)
    bounds: Optional[Tuple[float, float, float, float]] = None
    node_ids: set = field(default_factory=set)
    way_nodes: Dict[str, List[str]] = field(default_factory=dict)
    expected_way_ids: set = field(default_factory=set)
    lanelet_ids: set = field(default_factory=set)
    lanelet_boundaries: Dict[str, Tuple[str, str]] = field(default_factory=dict)
    lanelet_endpoints: Dict[
        str, Tuple[Tuple[str, str], Tuple[str, str]]
    ] = field(default_factory=dict)
    topology_edges: set = field(default_factory=set)
    topology_directed: bool = False
    topology_source: str = ""
    area_ids: set = field(default_factory=set)
    area_way_ids: Dict[str, set] = field(default_factory=dict)
    regulatory_ids: set = field(default_factory=set)
    convertible_regulatory_ids: set = field(default_factory=set)
    regulatory_way_ids: Dict[str, set] = field(default_factory=dict)


@dataclass
class TargetStats:
    roads: int = 0
    junctions: int = 0
    driving_lanes: int = 0
    road_links: int = 0
    lane_links: int = 0
    road_length: float = 0.0
    geometry_length: float = 0.0
    curved_geometries: int = 0
    signals: int = 0
    traffic_lights: int = 0
    stop_lines: int = 0
    controllers: int = 0
    objects: int = 0
    source_area_objects: int = 0
    object_references: int = 0
    outlines: int = 0
    signal_types: Counter = field(default_factory=Counter)
    object_types: Counter = field(default_factory=Counter)
    bounds: Optional[Tuple[float, float, float, float]] = None
    lanelet_locations: Dict[str, set] = field(default_factory=dict)
    mapped_way_ids: set = field(default_factory=set)
    mapped_area_ids: set = field(default_factory=set)
    mapped_regulatory_ids: set = field(default_factory=set)
    unresolved_regulatory_signal_ids: set = field(default_factory=set)


class Lanelet2OpenDriveDiagnostics:
    """Two-stage Lanelet2 -> OpenDRIVE conversion quality evaluation."""

    def __init__(
        self,
        source_lanelet2_path: str | Path,
        target_xodr_path: str | Path,
        external_validator_cmd: Optional[str] = None,
        external_timeout_sec: int = 20,
        original_source_path: str | Path | None = None,
        source_preparation: Optional[Sequence[Dict[str, str]]] = None,
    ) -> None:
        self.source_path = Path(source_lanelet2_path)
        self.original_source_path = Path(original_source_path or source_lanelet2_path)
        self.source_preparation = list(source_preparation or [])
        self.target_path = Path(target_xodr_path)
        self.json_report_path = _conversion_artifact_path(
            self.target_path, "diagnostics", "_diagnostics.json"
        )
        self.report_path = self.json_report_path
        self.external_validator_cmd = external_validator_cmd or os.environ.get(
            "OPENDRIVE_VALIDATOR_CMD"
        )
        self.external_timeout_sec = int(external_timeout_sec)
        self.results: List[CheckResult] = []
        self.source_stats = SourceStats()
        self.target_stats = TargetStats()

    def run(self) -> Path:
        started = time.time()
        source_root = self._parse_xml(self.source_path, "source")
        target_root = self._parse_xml(self.target_path, "target")

        if source_root is not None:
            self.source_stats = self._collect_source_stats(source_root)
            self._apply_official_source_topology(source_root)
        if target_root is not None:
            self.target_stats = self._collect_target_stats(target_root)
        if source_root is not None and target_root is not None:
            for area_id in self.target_stats.mapped_area_ids:
                self.target_stats.mapped_way_ids.update(
                    self.source_stats.area_way_ids.get(area_id, set())
                )

        self._run_syntax_validation(target_root)
        if source_root is not None and target_root is not None:
            self._run_semantic_comparison(source_root, target_root)
        else:
            self._add(
                "semantic",
                "overall",
                "SKIP",
                "Source/target comparison skipped because one XML file could not be parsed.",
            )

        viewer = find_opendrive_viewer()
        viewer_status = "SKIP"
        viewer_summary = "odrviewer is unavailable; manual visualization was not started."
        viewer_hint = ""
        if viewer and _launch_viewer_enabled():
            viewer_status, viewer_hint = _try_launch_viewer([viewer, "--odr", str(self.target_path)])
            viewer_summary = (
                "odrviewer was launched; manual source/target comparison is required."
                if viewer_status == "REVIEW"
                else "odrviewer launch failed."
            )
        elif viewer:
            viewer_status = "REVIEW"
            viewer_summary = "OpenDRIVE visualization is ready for manual source/target comparison."
        self._add(
            "acceptance",
            "visual-comparison",
            viewer_status,
            viewer_summary,
            target=(
                f"{viewer} --odr {self.target_path}" if viewer else ""
            ),
            method=(
                "Set AUTOMAP_CONVERTER_LAUNCH_VIEWER=1 to launch odrviewer from diagnostics; "
                "inspect overall road layout, junction shape and visible deformation."
            ),
            hint=viewer_hint,
        )
        self._write_report(time.time() - started)
        return self.report_path

    def _parse_xml(
        self, path: Path, label: str
    ) -> Optional[etree._Element]:
        if not path.exists():
            self._add(
                "diagnosis",
                "xml",
                "FAIL",
                f"{label.capitalize()} file does not exist.",
                location=str(path),
            )
            return None
        try:
            return etree.parse(str(path)).getroot()
        except (etree.XMLSyntaxError, OSError) as exc:
            self._add(
                "diagnosis",
                "xml",
                "FAIL",
                f"{label.capitalize()} XML cannot be parsed: {exc}",
                location=str(path),
            )
            return None

    # ------------------------------------------------------------------
    # Stage 1: target syntax/loadability
    # ------------------------------------------------------------------

    def _run_syntax_validation(
        self, target_root: Optional[etree._Element]
    ) -> None:
        if target_root is None:
            self._add(
                "diagnosis",
                "opendrive-structure",
                "SKIP",
                "OpenDRIVE structure validation skipped.",
            )
            self._add(
                "acceptance",
                "external-loader",
                "SKIP",
                "External OpenDRIVE load test skipped.",
            )
            return

        self._add(
            "diagnosis",
            "xml",
            "PASS",
            "Target file is well-formed XML.",
            target=str(self.target_path),
            method="lxml XML parser",
        )

        root_ok = target_root.tag == "OpenDRIVE"
        roads = target_root.findall("road")
        road_ids = [road.get("id", "") for road in roads]
        duplicate_ids = sorted(
            road_id
            for road_id, count in Counter(road_ids).items()
            if road_id and count > 1
        )
        incomplete_roads = [
            road.get("id", "<missing>")
            for road in roads
            if road.find("planView") is None or road.find("lanes") is None
        ]
        invalid_lengths = [
            road.get("id", "<missing>")
            for road in roads
            if self._float(road.get("length"), -1.0) <= 0.0
        ]

        if not root_ok or not roads or duplicate_ids or incomplete_roads or invalid_lengths:
            details = []
            if not root_ok:
                details.append(f"root={target_root.tag!r}, expected 'OpenDRIVE'")
            if not roads:
                details.append("no road elements")
            if duplicate_ids:
                details.append(f"duplicate road ids={duplicate_ids[:10]}")
            if incomplete_roads:
                details.append(f"roads missing planView/lanes={incomplete_roads[:10]}")
            if invalid_lengths:
                details.append(f"roads with invalid length={invalid_lengths[:10]}")
            self._add(
                "diagnosis",
                "opendrive-structure",
                "FAIL",
                "; ".join(details),
                method="Minimal OpenDRIVE root and road checks required before loading.",
                hint="Repair the listed root/road elements, then rerun the external loader.",
            )
        else:
            self._add(
                "diagnosis",
                "opendrive-structure",
                "PASS",
                f"OpenDRIVE root and {len(roads)} road elements have the required basic structure.",
                method="Check root, unique road ids, positive length, planView and lanes.",
            )

        missing_road_refs, missing_lane_refs = self._target_dangling_links(
            target_root
        )
        if missing_road_refs or missing_lane_refs:
            self._add(
                "diagnosis",
                "link-integrity",
                "FAIL",
                (
                    f"Dangling references found: road/junction={missing_road_refs}, "
                    f"invalid lane ids={missing_lane_refs}."
                ),
                method="Resolve every road/junction link and reject malformed zero lane-link IDs.",
            )
        else:
            self._add(
                "diagnosis",
                "link-integrity",
                "PASS",
                "Road, junction and basic lane-link references resolve.",
            )

        self._check_junction_role_constraints(target_root)
        self._run_asam_quality_checker()
        self._run_external_loader()

    def _check_junction_role_constraints(
        self, root: etree._Element
    ) -> None:
        roads = {road.get("id", ""): road for road in root.findall("road")}
        connecting_owners: Dict[str, set] = {}
        incoming_memberships: Dict[str, set] = {}
        violations: List[str] = []

        for junction in root.findall("junction"):
            junction_id = junction.get("id", "")
            junction_type = junction.get("type", "default")
            connection_ids = [
                connection.get("id", "")
                for connection in junction.findall("connection")
            ]
            duplicates = [
                connection_id
                for connection_id, count in Counter(connection_ids).items()
                if connection_id and count > 1
            ]
            if duplicates:
                violations.append(
                    f"junction {junction_id} has duplicate connection IDs {duplicates}"
                )
            if junction_type != "virtual":
                forbidden = [
                    name
                    for name in ("mainRoad", "sStart", "sEnd", "orientation")
                    if junction.get(name) is not None
                ]
                if forbidden:
                    violations.append(
                        f"non-virtual junction {junction_id} uses virtual attributes {forbidden}"
                    )

            connection_pairs = Counter()
            for connection in junction.findall("connection"):
                connection_id = connection.get("id", "<missing>")
                incoming = connection.get("incomingRoad", "")
                connecting = connection.get("connectingRoad", "")
                contact_point = connection.get("contactPoint")
                if junction_type != "virtual" and (
                    not incoming or not connecting or contact_point not in {"start", "end"}
                ):
                    violations.append(
                        f"junction {junction_id} connection {connection_id} has incomplete "
                        f"incomingRoad/connectingRoad/contactPoint"
                    )
                    continue
                if incoming == connecting and incoming:
                    violations.append(
                        f"junction {junction_id} connection {connection_id} uses road "
                        f"{incoming} as both incomingRoad and connectingRoad"
                    )
                connecting_owners.setdefault(connecting, set()).add(junction_id)
                incoming_memberships.setdefault(incoming, set()).add(junction_id)
                connection_pairs[(incoming, connecting)] += 1

                incoming_road = roads.get(incoming)
                connecting_road = roads.get(connecting)
                if incoming_road is None:
                    violations.append(
                        f"junction {junction_id} connection {connection_id} incomingRoad "
                        f"{incoming} does not exist"
                    )
                    continue
                if connecting_road is None:
                    violations.append(
                        f"junction {junction_id} connection {connection_id} connectingRoad "
                        f"{connecting} does not exist"
                    )
                    continue

                lane_links = connection.findall("laneLink")
                if not lane_links:
                    continue

                incoming_contact = self._incoming_junction_contact(
                    incoming_road, junction_id
                )
                if incoming_contact is None:
                    violations.append(
                        f"incomingRoad {incoming} does not link to junction {junction_id}"
                    )
                    continue
                incoming_lanes = self._road_lane_ids_at_contact(
                    incoming_road, incoming_contact
                )
                connecting_lanes = self._road_lane_ids_at_contact(
                    connecting_road, contact_point
                )
                seen_lane_links = set()
                for lane_link in lane_links:
                    from_lane = self._int(lane_link.get("from"))
                    to_lane = self._int(lane_link.get("to"))
                    pair = (from_lane, to_lane)
                    if pair in seen_lane_links:
                        violations.append(
                            f"junction {junction_id} connection {connection_id} "
                            f"duplicates laneLink {from_lane}->{to_lane}"
                        )
                    seen_lane_links.add(pair)
                    if from_lane == 0 or from_lane not in incoming_lanes:
                        violations.append(
                            f"junction {junction_id} connection {connection_id} laneLink "
                            f"from={from_lane} is absent on incomingRoad {incoming}"
                        )
                    if to_lane == 0 or to_lane not in connecting_lanes:
                        violations.append(
                            f"junction {junction_id} connection {connection_id} laneLink "
                            f"to={to_lane} is absent at {contact_point} of connectingRoad {connecting}"
                        )
                    if not self._connecting_lane_link_matches(
                        connecting_road,
                        contact_point,
                        to_lane,
                        from_lane,
                    ):
                        violations.append(
                            f"junction {junction_id} connection {connection_id} laneLink "
                            f"{from_lane}->{to_lane} disagrees with connectingRoad lane link"
                        )

            for (incoming, connecting), count in connection_pairs.items():
                if count > 1:
                    violations.append(
                        f"junction {junction_id} has {count} connections for "
                        f"incomingRoad {incoming} and connectingRoad {connecting}"
                    )

        for road_id, owners in connecting_owners.items():
            road = roads.get(road_id)
            if road is None:
                continue
            if len(owners) != 1:
                violations.append(
                    f"connectingRoad {road_id} belongs to junctions {sorted(owners)}"
                )
            owner = next(iter(owners), "")
            if road.get("junction", "-1") != owner:
                violations.append(
                    f"connectingRoad {road_id} has road@junction={road.get('junction')} but owner={owner}"
                )
            if road_id in incoming_memberships:
                violations.append(
                    f"connectingRoad {road_id} is also incomingRoad of junctions "
                    f"{sorted(incoming_memberships[road_id])}"
                )

        for road_id, road in roads.items():
            junction_id = road.get("junction", "-1")
            has_drivable_lane = any(
                lane.get("type") in {
                    "driving",
                    "entry",
                    "exit",
                    "onRamp",
                    "offRamp",
                    "connectingRamp",
                }
                for lane in road.findall("./lanes/laneSection/*/lane")
            )
            if (
                junction_id != "-1"
                and road_id not in connecting_owners
                and has_drivable_lane
            ):
                violations.append(
                    f"drivable road {road_id} has junction={junction_id} but is not "
                    "referenced as a connectingRoad"
                )

        if violations:
            self._add(
                "diagnosis",
                "junction-role-constraints",
                "FAIL",
                f"{len(violations)} OpenDRIVE junction-role violations found.",
                target="; ".join(violations[:20]),
                method=(
                    "Apply ASAM junction connection invariants: unique connection IDs, "
                    "valid incoming/connecting roads, contact points, laneLink endpoints, "
                    "unique incoming/connecting pairs, road@junction consistency, and "
                    "connectingRoad not used as incomingRoad."
                ),
                hint=(
                    "Repair the listed connection, road role or lane-link records."
                ),
            )
        else:
            self._add(
                "diagnosis",
                "junction-role-constraints",
                "PASS",
                f"All {len(connecting_owners)} connectingRoad assignments satisfy the role constraints.",
                method=(
                    "Check ASAM common-junction road and lane connection invariants."
                ),
            )

    @staticmethod
    def _incoming_junction_contact(
        road: etree._Element, junction_id: str
    ) -> Optional[str]:
        predecessor = road.find("./link/predecessor")
        if (
            predecessor is not None
            and predecessor.get("elementType") == "junction"
            and predecessor.get("elementId") == junction_id
        ):
            return "start"
        successor = road.find("./link/successor")
        if (
            successor is not None
            and successor.get("elementType") == "junction"
            and successor.get("elementId") == junction_id
        ):
            return "end"
        return None

    @classmethod
    def _road_lane_ids_at_contact(
        cls, road: etree._Element, contact_point: str
    ) -> set:
        sections = road.findall("./lanes/laneSection")
        if not sections:
            return set()
        section = sections[0] if contact_point == "start" else sections[-1]
        return {
            cls._int(lane.get("id"))
            for lane in section.findall("./*/lane")
            if cls._int(lane.get("id")) != 0
        }

    @classmethod
    def _connecting_lane_link_matches(
        cls,
        road: etree._Element,
        contact_point: str,
        connecting_lane_id: int,
        incoming_lane_id: int,
    ) -> bool:
        sections = road.findall("./lanes/laneSection")
        if not sections:
            return False
        section = sections[0] if contact_point == "start" else sections[-1]
        lane = next(
            (
                item
                for item in section.findall("./*/lane")
                if cls._int(item.get("id")) == connecting_lane_id
            ),
            None,
        )
        if lane is None:
            return False
        link_name = "predecessor" if contact_point == "start" else "successor"
        link = lane.find(f"./link/{link_name}")
        return link is not None and cls._int(link.get("id")) == incoming_lane_id

    def _run_external_loader(self) -> None:
        if not self.external_validator_cmd:
            local_odrplot = (
                PROJECT_ROOT
                / "tools"
                / "esmini"
                / "bin"
                / "odrplot"
            )
            odrplot = (
                str(local_odrplot)
                if local_odrplot.is_file() and os.access(local_odrplot, os.X_OK)
                else shutil.which("odrplot")
            )
            if odrplot:
                output_path = Path("/tmp/map_quality_odrplot.csv")
                self.external_validator_cmd = (
                    f"{shlex.quote(odrplot)} {{xodr}} "
                    f"{shlex.quote(str(output_path))}"
                )
            else:
                self._add(
                    "acceptance",
                    "external-loader",
                    "SKIP",
                    "No external OpenDRIVE parser is installed or configured.",
                    target="No project-local or PATH odrplot executable was found.",
                    method=(
                        "Install esmini or set OPENDRIVE_VALIDATOR_CMD; use {xodr} "
                        "as the target-file placeholder."
                    ),
                    hint=(
                        "SKIP means the professional parser was not executed; it is "
                        "not a PASS and not evidence that the map is invalid."
                    ),
                )
                return

        command = self.external_validator_cmd.format(xodr=str(self.target_path))
        try:
            completed = subprocess.run(
                shlex.split(command),
                capture_output=True,
                text=True,
                timeout=self.external_timeout_sec,
            )
        except subprocess.TimeoutExpired:
            self._add(
                "acceptance",
                "external-loader",
                "FAIL",
                f"External loader timed out after {self.external_timeout_sec}s.",
                target=command,
                hint="Use a non-interactive validation command or increase the timeout.",
            )
            return
        except (OSError, ValueError) as exc:
            self._add(
                "acceptance",
                "external-loader",
                "FAIL",
                f"External loader could not be executed: {exc}",
                target=command,
            )
            return

        output = "\n".join(
            part.strip()
            for part in (completed.stdout, completed.stderr)
            if part.strip()
        )
        error_lines = [
            line.strip()
            for line in output.splitlines()
            if "[error]" in line.lower() or "parse error" in line.lower()
        ]
        warning_lines = list(
            dict.fromkeys(
                line.strip()
                for line in output.splitlines()
                if "[warn]" in line.lower()
            )
        )
        if completed.returncode == 0 and not error_lines:
            self._add(
                "acceptance",
                "external-loader",
                "PASS",
                "External OpenDRIVE tool loaded the target without a process error.",
                target=command,
                method="Exit code is zero and loader output contains no error-level message.",
                hint=(
                    f"Loader warnings={len(warning_lines)}: "
                    + " | ".join(warning_lines[:8])
                    if warning_lines
                    else ""
                ),
            )
        elif completed.returncode == 0:
            self._add(
                "acceptance",
                "external-loader",
                "WARN",
                "External OpenDRIVE tool loaded the target but reported non-fatal messages.",
                target=command,
                method=(
                    "Exit code is zero; loader emitted messages containing error/warn text "
                    "that should be reviewed but did not prevent loading."
                ),
                hint=self._truncate(
                    "\n".join(error_lines + warning_lines) if error_lines or warning_lines else output,
                    2000,
                ),
            )
        else:
            self._add(
                "acceptance",
                "external-loader",
                "FAIL",
                (
                    f"External OpenDRIVE loader returned {completed.returncode}."
                    if completed.returncode != 0
                    else f"External loader reported {len(error_lines)} error-level message(s)."
                ),
                target=command,
                hint=self._truncate(
                    "\n".join(error_lines) if error_lines else output, 2000
                ),
            )

    def _run_asam_quality_checker(self) -> None:
        checker = self._find_asam_quality_checker()
        if checker is None:
            self._add(
                "diagnosis",
                "asam-quality-checker",
                "SKIP",
                "ASAM OpenDRIVE Quality Checker is not installed.",
                hint=(
                    "Install asam-qc-opendrive in an isolated environment and set "
                    "AUTOMAP_CONVERTER_ASAM_QC_CMD, or use the provided Docker image."
                ),
            )
            return

        stem = "".join(
            character if character.isalnum() else "_"
            for character in self.target_path.stem
        )
        config_path = Path("/tmp") / f"{stem}_asam_qc_config.xml"
        result_path = _conversion_artifact_path(
            self.target_path, "diagnostics", "_asam_qc_result.xqar"
        ).resolve()
        config_root = etree.Element("Config")
        etree.SubElement(
            config_root,
            "Param",
            name="InputFile",
            value=str(self.target_path.resolve()),
        )
        bundle = etree.SubElement(
            config_root, "CheckerBundle", application="xodrBundle"
        )
        etree.SubElement(
            bundle,
            "Param",
            name="resultFile",
            value=str(result_path),
        )
        etree.ElementTree(config_root).write(
            str(config_path),
            encoding="UTF-8",
            xml_declaration=True,
            pretty_print=True,
        )

        try:
            completed = subprocess.run(
                [checker, "-c", str(config_path)],
                capture_output=True,
                text=True,
                timeout=max(60, self.external_timeout_sec),
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._add(
                "diagnosis",
                "asam-quality-checker",
                "FAIL",
                f"ASAM checker could not complete: {exc}",
            )
            return

        if completed.returncode != 0 or not result_path.exists():
            self._add(
                "diagnosis",
                "asam-quality-checker",
                "FAIL",
                f"ASAM checker returned {completed.returncode} without a usable result.",
                hint=self._truncate(completed.stderr or completed.stdout, 1500),
            )
            return

        result_root = etree.parse(str(result_path)).getroot()
        checkers = result_root.findall(".//Checker")
        issues = result_root.findall(".//Issue")
        completed_checks = sum(
            checker_node.get("status") == "completed"
            for checker_node in checkers
        )
        skipped_checks = sum(
            checker_node.get("status") == "skipped"
            for checker_node in checkers
        )
        issue_groups = Counter(
            issue.find("Locations").get("description", "unspecified issue")
            if issue.find("Locations") is not None
            else issue.get("description", "unspecified issue")
            for issue in issues
        )
        issue_summary = " | ".join(
            f"{count}x {description}"
            for description, count in issue_groups.most_common(8)
        )
        issue_details = []
        target_tree = etree.parse(str(self.target_path))
        for issue in issues:
            checker_node = issue.getparent()
            locations = issue.find("Locations")
            location_items = []
            if locations is not None:
                for location_node in locations:
                    location_data = {
                        "type": location_node.tag,
                        **dict(location_node.attrib),
                    }
                    xpath = location_node.get("xpath")
                    if xpath:
                        try:
                            matched = target_tree.xpath(xpath)
                        except etree.XPathError:
                            matched = []
                        if matched and isinstance(matched[0], etree._Element):
                            location_data["resolved_target"] = (
                                self._describe_target_element(matched[0])
                            )
                    location_items.append(location_data)
            issue_details.append(
                {
                    "issue_id": issue.get("issueId", ""),
                    "level": self._int(issue.get("level")),
                    "rule_uid": issue.get("ruleUID", ""),
                    "description": issue.get("description", ""),
                    "checker_id": (
                        checker_node.get("checkerId", "")
                        if checker_node is not None
                        else ""
                    ),
                    "checker_description": (
                        checker_node.get("description", "")
                        if checker_node is not None
                        else ""
                    ),
                    "locations_description": (
                        locations.get("description", "")
                        if locations is not None
                        else ""
                    ),
                    "locations": location_items,
                }
            )
        self._add(
            "diagnosis",
            "asam-quality-checker",
            "FAIL" if issues else "PASS",
            (
                f"Official ASAM checker found {len(issues)} issue(s); "
                f"completed={completed_checks}, skipped={skipped_checks}."
            ),
            target=str(result_path),
            method=(
                "Run the official asam-qc-opendrive 1.0.0 schema, semantic, "
                "geometry, performance and smoothness checker bundle."
            ),
            hint=self._truncate(issue_summary, 2500),
            details=issue_details,
        )

    @staticmethod
    def _find_asam_quality_checker() -> Optional[str]:
        """Locate the checker without mixing its dependencies into the main environment."""

        configured = os.environ.get("AUTOMAP_CONVERTER_ASAM_QC_CMD", "").strip()
        candidates = [configured] if configured else []
        candidates.append(
            str(PROJECT_ROOT / "tools" / "asam_qc_env" / "bin" / "qc_opendrive")
        )
        candidates.append(shutil.which("qc_opendrive") or "")
        for candidate in candidates:
            path = Path(candidate).expanduser()
            if path.is_file() and os.access(path, os.X_OK):
                return str(path)
        return None

    # ------------------------------------------------------------------
    # Stage 2: source/target semantic consistency
    # ------------------------------------------------------------------

    def _run_semantic_comparison(
        self, source_root: etree._Element, target_root: etree._Element
    ) -> None:
        self._compare_source_nodes()
        self._compare_source_ways()
        self._compare_source_lanelets(target_root)
        self._compare_source_topology(target_root)
        self._compare_source_areas()
        self._compare_source_regulatory_elements()

    def _compare_source_nodes(self) -> None:
        src = self.source_stats
        dst = self.target_stats
        expected_nodes = {
            node_id
            for way_id in src.expected_way_ids
            for node_id in src.way_nodes.get(way_id, [])
        }
        mapped_nodes = {
            node_id
            for way_id in dst.mapped_way_ids
            for node_id in src.way_nodes.get(way_id, [])
        }
        missing = sorted(expected_nodes - mapped_nodes)
        self._add_source_coverage_result(
            "1-nodes",
            expected_nodes,
            mapped_nodes,
            missing,
            source=(
                f"all_nodes={len(src.node_ids)}, nodes_used_by_convertible_ways={len(expected_nodes)}"
            ),
            target=f"nodes_represented_through_mapped_ways={len(expected_nodes & mapped_nodes)}",
            method=(
                "A Lanelet2 node is considered preserved when it belongs to a source way "
                "that is explicitly mapped to an OpenDRIVE lane, signal, stop line or area. "
                "OpenDRIVE does not store Lanelet2 nodes as independent elements."
            ),
        )

    def _compare_source_ways(self) -> None:
        src = self.source_stats
        dst = self.target_stats
        missing = sorted(src.expected_way_ids - dst.mapped_way_ids)
        self._add_source_coverage_result(
            "2-ways-linestrings",
            src.expected_way_ids,
            dst.mapped_way_ids,
            missing,
            source=f"convertible_or_referenced_ways={len(src.expected_way_ids)}",
            target=f"source_way_ids_traced_in_xodr={len(src.expected_way_ids & dst.mapped_way_ids)}",
            method=(
                "Expected ways are lanelet boundaries, valid area rings, regulatory "
                "refers/ref_line ways and standalone traffic ways. Boundary ways are read "
                "from lanelet2:lanelet_mapping; physical traffic ways from source metadata."
            ),
        )

    def _compare_source_lanelets(self, target_root: etree._Element) -> None:
        src = self.source_stats
        dst = self.target_stats
        mapped = set(dst.lanelet_locations)
        missing = sorted(src.lanelet_ids - mapped)
        self._add_source_coverage_result(
            "3-lanelets",
            src.lanelet_ids,
            mapped,
            missing,
            source=(
                f"lanelets={len(src.lanelet_ids)}, "
                f"subtypes={self._counter(src.lanelet_subtypes)}, "
                f"approximate_length={src.approximate_length:.3f}m"
            ),
            target=(
                f"lanelets_with_explicit_target_lane_mapping="
                f"{len(src.lanelet_ids & mapped)}, roads={self.target_stats.roads}, "
                f"driving_lanes={self.target_stats.driving_lanes}"
            ),
            method=(
                "Each source lanelet must appear in a lanelet2:lanelet_mapping record "
                "on an OpenDRIVE lane. Multiple source lanelets may legitimately map to "
                "one longitudinal OpenDRIVE lane."
            ),
        )

    def _compare_source_topology(self, target_root: etree._Element) -> None:
        src = self.source_stats
        dst = self.target_stats
        road_adjacency = self._target_road_adjacency(target_root)
        missing_edges = []
        missing_details = []
        roads = {
            road.get("id", ""): road for road in target_root.findall("road")
        }
        for source_id, target_id in sorted(src.topology_edges):
            source_locations = dst.lanelet_locations.get(source_id, set())
            target_locations = dst.lanelet_locations.get(target_id, set())
            preserved = any(
                self._roads_reachable(
                    source_road, target_road, road_adjacency, max_depth=3
                )
                or (
                    not src.topology_directed
                    and self._roads_reachable(
                        target_road, source_road, road_adjacency, max_depth=3
                    )
                )
                for source_road, _ in source_locations
                for target_road, _ in target_locations
            )
            if not preserved:
                source_label = self._format_target_locations(source_locations)
                target_label = self._format_target_locations(target_locations)
                missing_edges.append(
                    f"{source_id}[{source_label}]->{target_id}[{target_label}]"
                )
                source_endpoints = src.lanelet_endpoints.get(source_id)
                target_endpoints = src.lanelet_endpoints.get(target_id)
                missing_details.append(
                    {
                        "source_lanelet_id": source_id,
                        "target_lanelet_id": target_id,
                        "relation_type": (
                            src.topology_source
                            or (
                                "explicit_predecessor_successor"
                                if src.topology_directed
                                else "inferred_shared_cross_section"
                            )
                        ),
                        "shared_endpoint_node_ids": (
                            list(source_endpoints[1])
                            if source_endpoints
                            and target_endpoints
                            and source_endpoints[1] == target_endpoints[0]
                            else []
                        ),
                        "source_target_locations": self._topology_locations(
                            source_locations, roads
                        ),
                        "target_target_locations": self._topology_locations(
                            target_locations, roads
                        ),
                        "search_max_road_depth": 3,
                    }
                )

        expected = len(src.topology_edges)
        preserved = expected - len(missing_edges)
        status = (
            "PASS"
            if not missing_edges
            else "FAIL"
            if expected and preserved / expected < 0.95
            else "WARN"
        )
        self._add(
            "semantic",
            "3b-lanelet-topology",
            status,
            (
                f"All {expected} source lanelet topology edges are preserved."
                if not missing_edges
                else f"{len(missing_edges)} of {expected} source topology edges are not represented."
            ),
            source=(
                f"lanelet_edges={expected}, "
                f"direction={'directed' if src.topology_directed else 'inferred-undirected'}, "
                f"source={src.topology_source or 'unknown'}"
            ),
            target=f"preserved_source_edges={preserved}",
            method=(
                "Check every source lanelet topology edge individually. Explicit source "
                "successor/predecessor edges require a directed target path; topology inferred "
                "only from shared endpoints checks connectivity in either direction. "
                "The target path may contain at most three roads."
            ),
            location=", ".join(missing_edges[:30]),
            hint=(
                "Inspect the listed source lanelet pairs and their target road/lane mappings."
                if missing_edges
                else ""
            ),
            details=missing_details,
        )

    def _compare_source_areas(self) -> None:
        src = self.source_stats
        dst = self.target_stats
        missing = sorted(src.area_ids - dst.mapped_area_ids)
        self._add_source_coverage_result(
            "4-areas",
            src.area_ids,
            dst.mapped_area_ids,
            missing,
            source=(
                f"convertible_areas={len(src.area_ids)}, "
                f"all_multipolygon_relations={src.multipolygons}, "
                f"subtypes={self._counter(src.multipolygon_subtypes)}"
            ),
            target=(
                f"source_area_ids_traced_in_objects="
                f"{len(src.area_ids & dst.mapped_area_ids)}, "
                f"outlines={dst.outlines}, objectReferences={dst.object_references}"
            ),
            method=(
                "Each source multipolygon with usable outer geometry must have an "
                "OpenDRIVE object carrying the same lanelet2 source ID."
            ),
        )

    def _compare_source_regulatory_elements(self) -> None:
        src = self.source_stats
        dst = self.target_stats
        missing = sorted(
            src.convertible_regulatory_ids - dst.mapped_regulatory_ids
        )
        self._add_source_coverage_result(
            "5-regulatory-elements",
            src.convertible_regulatory_ids,
            dst.mapped_regulatory_ids,
            missing,
            source=(
                f"all_regulatory_elements={len(src.regulatory_ids)}, "
                f"convertible_regulatory_elements={len(src.convertible_regulatory_ids)}, "
                f"subtypes={self._counter(src.regulatory_subtypes)}"
            ),
            target=(
                f"source_regulatory_ids_traced="
                f"{len(src.regulatory_ids & dst.mapped_regulatory_ids)}, "
                f"signals={dst.signals}, controllers={dst.controllers}, "
                f"signal_types={self._counter(dst.signal_types)}"
            ),
            method=(
                "Every source regulatory relation attached to a lanelet or having usable "
                "refers/ref_line geometry must be referenced by target signal/controller "
                "userData. Its physical ways are checked separately in the source-way audit."
            ),
        )
        unresolved = sorted(
            src.convertible_regulatory_ids & dst.unresolved_regulatory_signal_ids
        )
        if unresolved:
            self._add(
                "semantic",
                "5b-regulatory-signal-type",
                "WARN",
                f"{len(unresolved)} source regulatory relations have only an unknown OpenDRIVE signal type.",
                source=f"regulatory_relations={len(src.convertible_regulatory_ids)}",
                target=f"unknown_signal_type_relations={len(unresolved)}",
                method=(
                    "Source ID metadata preserves traceability, but OpenDRIVE type=-1 "
                    "does not encode the original traffic or area semantics."
                ),
                location=", ".join(unresolved[:30]),
            )

    def _add_source_coverage_result(
        self,
        category: str,
        expected: set,
        mapped: set,
        missing: List[str],
        source: str,
        target: str,
        method: str,
    ) -> None:
        expected_count = len(expected)
        covered_count = len(expected & mapped)
        ratio = covered_count / expected_count if expected_count else 1.0
        status = "PASS" if not missing else "FAIL" if ratio < 0.95 else "WARN"
        self._add(
            "semantic",
            category,
            status,
            (
                f"All {expected_count} expected source elements are traced in OpenDRIVE."
                if not missing
                else f"{len(missing)} of {expected_count} expected source elements are missing."
            ),
            source=source,
            target=target,
            method=method,
            location=", ".join(missing[:30]),
            hint=(
                "Inspect the listed source IDs and the corresponding conversion step."
                if missing
                else ""
            ),
        )

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def _collect_source_stats(self, root: etree._Element) -> SourceStats:
        stats = SourceStats()
        origin_lat = None
        origin_lon = None
        for node in root.findall("node"):
            if node.get("lat") is not None and node.get("lon") is not None:
                origin_lat = self._float(node.get("lat"))
                origin_lon = self._float(node.get("lon"))
                break
        nodes = {
            node.get("id", ""): self._node_point(
                node, origin_lat=origin_lat, origin_lon=origin_lon
            )
            for node in root.findall("node")
            if node.get("id")
        }
        nodes = {node_id: point for node_id, point in nodes.items() if point is not None}
        ways = {
            way.get("id", ""): [
                nd.get("ref", "") for nd in way.findall("nd") if nd.get("ref")
            ]
            for way in root.findall("way")
            if way.get("id")
        }
        stats.node_ids = set(nodes)
        stats.way_nodes = ways
        way_tags = {
            way.get("id", ""): self._tags(way)
            for way in root.findall("way")
            if way.get("id")
        }
        stats.nodes = len(root.findall("node"))
        stats.ways = len(root.findall("way"))
        all_points = list(nodes.values())
        stats.bounds = self._bounds(all_points)
        lanelet_endpoints: List[
            Tuple[str, Tuple[str, str], Tuple[str, str]]
        ] = []
        explicit_topology_edges = set()
        explicit_successors = 0
        explicit_predecessors = 0
        lanelet_referenced_regulatory_ids = set()

        for way_id, tags in way_tags.items():
            typ = tags.get("type", "").lower()
            subtype = tags.get("subtype", "").lower()
            if typ == "traffic_light" or subtype == "traffic_light":
                stats.traffic_light_ways += 1
                if ways.get(way_id):
                    stats.expected_way_ids.add(way_id)
            if typ == "traffic_sign" or subtype == "traffic_sign":
                stats.traffic_sign_ways += 1
                if ways.get(way_id):
                    stats.expected_way_ids.add(way_id)
            if typ == "stop_line" and ways.get(way_id):
                stats.expected_way_ids.add(way_id)

        for relation in root.findall("relation"):
            tags = self._tags(relation)
            relation_type = tags.get("type", "").lower()
            subtype = tags.get("subtype", "").lower()
            members = relation.findall("member")
            roles = {member.get("role", "") for member in members}

            if relation_type == "lanelet" or {"left", "right"}.issubset(roles):
                lanelet_id = relation.get("id", "")
                stats.lanelets += 1
                stats.lanelet_ids.add(lanelet_id)
                stats.lanelet_subtypes[subtype or "unknown"] += 1
                lanelet_referenced_regulatory_ids.update(
                    member.get("ref", "")
                    for member in members
                    if member.get("role") == "regulatory_element"
                    and member.get("ref")
                )
                successors = [
                    member for member in members if member.get("role") == "successor"
                ]
                predecessors = [
                    member for member in members if member.get("role") == "predecessor"
                ]
                explicit_successors += len(successors)
                explicit_predecessors += len(predecessors)
                explicit_topology_edges.update(
                    (lanelet_id, member.get("ref", ""))
                    for member in successors
                    if member.get("ref")
                )
                explicit_topology_edges.update(
                    (member.get("ref", ""), lanelet_id)
                    for member in predecessors
                    if member.get("ref")
                )

                left_id = next(
                    (
                        member.get("ref", "")
                        for member in members
                        if member.get("role") == "left"
                    ),
                    "",
                )
                right_id = next(
                    (
                        member.get("ref", "")
                        for member in members
                        if member.get("role") == "right"
                    ),
                    "",
                )
                left_nodes = ways.get(left_id, [])
                right_nodes = ways.get(right_id, [])
                stats.lanelet_boundaries[lanelet_id] = (left_id, right_id)
                stats.expected_way_ids.update(
                    way_id for way_id in (left_id, right_id) if way_id
                )
                if left_nodes and right_nodes:
                    start_endpoint = tuple(
                        sorted((left_nodes[0], right_nodes[0]))
                    )
                    end_endpoint = tuple(
                        sorted((left_nodes[-1], right_nodes[-1]))
                    )
                    stats.lanelet_endpoints[lanelet_id] = (
                        start_endpoint,
                        end_endpoint,
                    )
                    lanelet_endpoints.append(
                        (
                            lanelet_id,
                            start_endpoint,
                            end_endpoint,
                        )
                    )
                left = self._way_points(left_id, ways, nodes)
                right = self._way_points(right_id, ways, nodes)
                centerline = self._approximate_centerline(left, right)
                stats.approximate_length += self._polyline_length(centerline)
                if self._heading_change(centerline) > math.radians(5.0):
                    stats.curved_lanelets += 1

            elif relation_type == "regulatory_element":
                regulatory_id = relation.get("id", "")
                stats.regulatory_elements += 1
                stats.regulatory_ids.add(regulatory_id)
                stats.regulatory_subtypes[subtype or "unknown"] += 1
                regulatory_ways = {
                    member.get("ref", "")
                    for member in members
                    if member.get("type") == "way"
                    and member.get("role") in {"refers", "ref_line"}
                    and member.get("ref")
                    and len(ways.get(member.get("ref", ""), [])) >= 1
                }
                stats.regulatory_way_ids[regulatory_id] = regulatory_ways
                stats.expected_way_ids.update(regulatory_ways)
                if subtype == "traffic_light":
                    stats.traffic_light_relations += 1

            elif relation_type == "multipolygon":
                area_id = relation.get("id", "")
                stats.multipolygons += 1
                stats.multipolygon_subtypes[subtype or "unknown"] += 1
                outer_way_ids = [
                    member.get("ref", "")
                    for member in members
                    if member.get("role") == "outer"
                ]
                if any(len(ways.get(way_id, [])) >= 3 for way_id in outer_way_ids):
                    stats.convertible_multipolygons += 1
                    area_way_ids = {
                        member.get("ref", "")
                        for member in members
                        if member.get("role") in {"outer", "inner"}
                        and member.get("ref")
                        and len(ways.get(member.get("ref", ""), [])) >= 3
                    }
                    stats.area_ids.add(area_id)
                    stats.area_way_ids[area_id] = area_way_ids
                    stats.expected_way_ids.update(area_way_ids)

        stats.convertible_regulatory_ids = {
            regulatory_id
            for regulatory_id in stats.regulatory_ids
            if regulatory_id in lanelet_referenced_regulatory_ids
            or stats.regulatory_way_ids.get(regulatory_id)
        }
        if explicit_successors or explicit_predecessors:
            stats.successor_edges = explicit_successors
            stats.predecessor_edges = explicit_predecessors
            stats.topology_edges = explicit_topology_edges
            stats.topology_directed = True
            stats.topology_source = "osm_explicit_predecessor_successor"
        else:
            starts = Counter(start for _, start, _ in lanelet_endpoints)
            ends = Counter(end for _, _, end in lanelet_endpoints)
            stats.successor_edges = sum(
                starts.get(end, 0) for _, _, end in lanelet_endpoints
            )
            stats.predecessor_edges = sum(
                ends.get(start, 0) for _, start, _ in lanelet_endpoints
            )
            stats.branch_lanelets = sum(
                1
                for _, start, end in lanelet_endpoints
                if starts.get(end, 0) > 1 or ends.get(start, 0) > 1
            )
            starts_to_lanelets: Dict[Tuple[str, str], List[str]] = {}
            for lanelet_id, start, _ in lanelet_endpoints:
                starts_to_lanelets.setdefault(start, []).append(lanelet_id)
            stats.topology_edges = {
                (lanelet_id, successor_id)
                for lanelet_id, _, end in lanelet_endpoints
                for successor_id in starts_to_lanelets.get(end, [])
                if lanelet_id != successor_id
            }
            stats.topology_directed = False
            stats.topology_source = "boundary_endpoint_fallback"
        return stats

    def _apply_official_source_topology(
        self, source_root: etree._Element
    ) -> None:
        """Replace fallback topology with the official Lanelet2 RoutingGraph."""
        try:
            from automap_converter.core.config.lanelet2_config import lanelet2_config
            from automap_converter.formats.lanelet2.map import (
                enrich_topology_from_official_routing_graph,
            )
            from automap_converter.formats.lanelet2.parser import (
                Lanelet2Parser,
            )

            lanelet_map = Lanelet2Parser(source_root, lanelet2_config).parse()
            topology = enrich_topology_from_official_routing_graph(
                osm_lanelet=lanelet_map,
                osm_file_path=str(self.source_path),
                origin_lat=float(lanelet2_config.routing_origin_lat),
                origin_lon=float(lanelet2_config.routing_origin_lon),
                location=str(lanelet2_config.routing_location),
                participant=str(lanelet2_config.routing_participant),
                overwrite=True,
            )
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            self.source_stats.topology_source += (
                f"; official_routing_graph_unavailable={exc}"
            )
            return

        valid_ids = self.source_stats.lanelet_ids
        edges = {
            (lanelet_id, successor_id)
            for lanelet_id, relationships in topology.items()
            for successor_id in relationships.get("successors", [])
            if lanelet_id in valid_ids and successor_id in valid_ids
        }
        self.source_stats.topology_edges = edges
        self.source_stats.successor_edges = len(edges)
        self.source_stats.predecessor_edges = len(edges)
        self.source_stats.topology_directed = True
        self.source_stats.topology_source = "official_lanelet2_routing_graph"

    def _collect_target_stats(self, root: etree._Element) -> TargetStats:
        stats = TargetStats()
        points: List[Point2D] = []
        roads = root.findall("road")
        stats.roads = len(roads)
        stats.junctions = len(root.findall("junction"))
        stats.controllers = len(root.findall("controller"))

        for road in roads:
            stats.road_length += self._float(road.get("length"))
            stats.road_links += len(road.findall("./link/predecessor"))
            stats.road_links += len(road.findall("./link/successor"))
            for geometry in road.findall("./planView/geometry"):
                length = self._float(geometry.get("length"))
                stats.geometry_length += length
                if geometry.find("line") is None:
                    stats.curved_geometries += 1
                points.extend(self._geometry_sample_points(geometry))

            for lane in road.findall("./lanes/laneSection/*/lane"):
                lane_id = self._int(lane.get("id"))
                lane_type = lane.get("type", "")
                if lane_id != 0 and lane_type in {
                    "driving",
                    "entry",
                    "exit",
                    "onRamp",
                    "offRamp",
                    "connectingRamp",
                    "bidirectional",
                }:
                    stats.driving_lanes += 1
                stats.lane_links += len(lane.findall("./link/predecessor"))
                stats.lane_links += len(lane.findall("./link/successor"))
                for item in lane.findall(
                    "userData[@code='lanelet2:lanelet_mapping']"
                ):
                    parts = item.get("value", "").split("|")
                    if len(parts) != 3:
                        continue
                    lanelet_id, left_way, right_way = parts
                    stats.lanelet_locations.setdefault(
                        lanelet_id, set()
                    ).add((road.get("id", ""), lane_id))
                    stats.mapped_way_ids.update((left_way, right_way))

            for signal in road.findall("./signals/signal"):
                stats.signals += 1
                key = (
                    signal.get("country", ""),
                    signal.get("type", ""),
                    signal.get("subtype", ""),
                )
                stats.signal_types[":".join(key)] += 1
                if signal.get("type") == "1000001":
                    stats.traffic_lights += 1
                if signal.get("type") == "1100001":
                    stats.stop_lines += 1
                self._collect_target_source_metadata(signal, stats)
                if signal.get("type") in {None, "", "-1"}:
                    user_data = {
                        item.get("code", ""): item.get("value", "")
                        for item in signal.findall("userData")
                    }
                    if user_data.get("lanelet2:source_kind") == "regulatory_element":
                        source_id = user_data.get("lanelet2:source_id", "")
                        if source_id:
                            stats.unresolved_regulatory_signal_ids.add(source_id)

            for obj in road.findall("./objects/object"):
                stats.objects += 1
                stats.object_types[obj.get("type", "unknown")] += 1
                stats.outlines += len(obj.findall("./outlines/outline"))
                user_data = {
                    item.get("code", ""): item.get("value", "")
                    for item in obj.findall("userData")
                }
                if user_data.get("lanelet2:source_kind") == "multipolygon":
                    stats.source_area_objects += 1
                    source_id = user_data.get("lanelet2:source_id", "")
                    if source_id:
                        stats.mapped_area_ids.add(source_id)
                self._collect_target_source_metadata(obj, stats)
            stats.object_references += len(
                road.findall("./objects/objectReference")
            )

        for controller in root.findall("controller"):
            self._collect_target_source_metadata(controller, stats)

        stats.bounds = self._bounds(points)
        return stats

    @staticmethod
    def _collect_target_source_metadata(
        element: etree._Element, stats: TargetStats
    ) -> None:
        values: Dict[str, List[str]] = {}
        for item in element.findall("userData"):
            values.setdefault(item.get("code", ""), []).append(
                item.get("value", "")
            )
        kinds = values.get("lanelet2:source_kind", [])
        source_ids = values.get("lanelet2:source_id", [])
        if "way" in kinds:
            stats.mapped_way_ids.update(value for value in source_ids if value)
        stats.mapped_way_ids.update(
            value for value in values.get("lanelet2:source_way_id", []) if value
        )
        if "regulatory_element" in kinds:
            stats.mapped_regulatory_ids.update(
                value for value in source_ids if value
            )
        stats.mapped_regulatory_ids.update(
            value
            for value in values.get("lanelet2:regulatory_element", [])
            if value
        )

    @staticmethod
    def _target_road_adjacency(root: etree._Element) -> set:
        adjacency = set()
        for road in root.findall("road"):
            road_id = road.get("id", "")
            predecessor = road.find("./link/predecessor")
            if (
                predecessor is not None
                and predecessor.get("elementType", "road") == "road"
            ):
                adjacency.add((predecessor.get("elementId", ""), road_id))
            successor = road.find("./link/successor")
            if (
                successor is not None
                and successor.get("elementType", "road") == "road"
            ):
                adjacency.add((road_id, successor.get("elementId", "")))
        for junction in root.findall("junction"):
            for connection in junction.findall("connection"):
                adjacency.add(
                    (
                        connection.get("incomingRoad", ""),
                        connection.get("connectingRoad", ""),
                    )
                )
        return adjacency

    @staticmethod
    def _roads_reachable(
        source_road: str,
        target_road: str,
        adjacency: set,
        max_depth: int,
    ) -> bool:
        if source_road == target_road:
            return True
        frontier = {source_road}
        visited = {source_road}
        for _ in range(max_depth):
            frontier = {
                target
                for source, target in adjacency
                if source in frontier and target not in visited
            }
            if target_road in frontier:
                return True
            if not frontier:
                return False
            visited.update(frontier)
        return False

    def _target_dangling_links(
        self, root: etree._Element
    ) -> Tuple[int, int]:
        road_ids = {road.get("id", "") for road in root.findall("road")}
        junction_ids = {
            junction.get("id", "") for junction in root.findall("junction")
        }
        missing_road_refs = 0
        missing_lane_refs = 0

        for road in root.findall("road"):
            for link in road.findall("./link/predecessor") + road.findall(
                "./link/successor"
            ):
                element_id = link.get("elementId", "")
                element_type = link.get("elementType", "road")
                if element_type == "junction":
                    missing_road_refs += int(element_id not in junction_ids)
                else:
                    missing_road_refs += int(element_id not in road_ids)

            for section in road.findall("./lanes/laneSection"):
                lane_ids = {
                    self._int(lane.get("id"))
                    for lane in section.findall("./*/lane")
                }
                for lane in section.findall("./*/lane"):
                    for link in lane.findall("./link/predecessor") + lane.findall(
                        "./link/successor"
                    ):
                        linked_id = self._int(link.get("id"))
                        if linked_id == 0 or linked_id not in lane_ids:
                            # Cross-road lane IDs cannot always be resolved inside this section.
                            # Only count zero/malformed IDs as definitively dangling here.
                            missing_lane_refs += int(linked_id == 0)
        return missing_road_refs, missing_lane_refs

    @staticmethod
    def _describe_target_element(element: etree._Element) -> Dict[str, object]:
        roads = element.xpath("ancestor-or-self::road[1]")
        lanes = element.xpath("ancestor-or-self::lane[1]")
        sections = element.xpath("ancestor-or-self::laneSection[1]")
        mappings = []
        if lanes:
            mappings = [
                item.get("value", "")
                for item in lanes[0].xpath(
                    "./userData[@code='lanelet2:lanelet_mapping']"
                )
            ]
        road_links = (
            [
                {"role": link.tag, **dict(link.attrib)}
                for link in roads[0].findall("./link/*")
            ]
            if roads
            else []
        )
        lane_links = (
            [
                {"role": link.tag, **dict(link.attrib)}
                for link in lanes[0].findall("./link/*")
            ]
            if lanes
            else []
        )
        return {
            "element": element.tag,
            "attributes": dict(element.attrib),
            "road_id": roads[0].get("id", "") if roads else "",
            "lane_id": lanes[0].get("id", "") if lanes else "",
            "lane_section_s": sections[0].get("s", "") if sections else "",
            "source_lanelet_mappings": mappings,
            "source_lanelet_ids": [
                value.split("|", 1)[0] for value in mappings if value
            ],
            "road_links": road_links,
            "lane_links": lane_links,
        }

    @staticmethod
    def _format_target_locations(locations: set) -> str:
        if not locations:
            return "unmapped"
        return ",".join(
            f"road={road_id}/lane={lane_id}"
            for road_id, lane_id in sorted(locations)
        )

    @staticmethod
    def _topology_locations(
        locations: set, roads: Dict[str, etree._Element]
    ) -> List[Dict[str, object]]:
        result = []
        for road_id, lane_id in sorted(locations):
            road = roads.get(road_id)
            road_links = []
            lane_links = []
            if road is not None:
                road_links = [
                    {"role": link.tag, **dict(link.attrib)}
                    for link in road.findall("./link/*")
                ]
                lane = road.find(
                    f"./lanes/laneSection/*/lane[@id='{lane_id}']"
                )
                if lane is not None:
                    lane_links = [
                        {"role": link.tag, **dict(link.attrib)}
                        for link in lane.findall("./link/*")
                    ]
            result.append(
                {
                    "road_id": road_id,
                    "lane_id": lane_id,
                    "road_links": road_links,
                    "lane_links": lane_links,
                }
            )
        return result

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------

    def _write_report(self, elapsed: float) -> None:
        counts = Counter(result.status for result in self.results)
        lines = [
            "Lanelet2 -> OpenDRIVE Conversion Quality Report",
            "=" * 64,
            f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Source: {self.original_source_path}",
            f"Target: {self.target_path}",
            f"Elapsed: {elapsed:.2f}s",
            "",
            "Overall",
            "-" * 64,
            f"PASS={counts['PASS']}  WARN={counts['WARN']}  "
            f"FAIL={counts['FAIL']}  REVIEW={counts['REVIEW']}  "
            f"SKIP={counts['SKIP']}",
            (
                "Result: FAIL"
                if counts["FAIL"]
                else "Result: PASS WITH WARNINGS"
                if counts["WARN"]
                else "Result: PASS"
            ),
        ]
        if self.source_path != self.original_source_path:
            lines.extend(
                [
                    "",
                    "Source preparation:",
                    f"Prepared source used for conversion: {self.source_path}",
                ]
            )
            for record in self.source_preparation:
                lines.append(
                    f"[{record.get('status', 'UNKNOWN')}] "
                    f"{record.get('category', 'source-repair')}: "
                    f"{record.get('summary', '')}"
                )
                if record.get("hint"):
                    lines.append(f"  hint: {record['hint']}")

        report_groups = (
            (
                "acceptance",
                "1. Tool acceptance",
                lambda result: result.stage == "acceptance",
            ),
            (
                "target_conformance",
                "2.1 Target conformance and geometry",
                lambda result: result.stage == "diagnosis"
                and result.category
                not in {"link-integrity", "junction-role-constraints"},
            ),
            (
                "topology",
                "2.2 Topology",
                lambda result: result.category
                in {
                    "link-integrity",
                    "junction-role-constraints",
                    "3b-lanelet-topology",
                },
            ),
            (
                "element_mapping",
                "2.3 Source element mapping",
                lambda result: result.stage == "semantic"
                and result.category != "3b-lanelet-topology",
            ),
        )
        structured_groups = {}
        for group_name, title, predicate in report_groups:
            group_results = [item for item in self.results if predicate(item)]
            structured_groups[group_name] = [
                asdict(result) for result in group_results if result.status != "PASS"
            ]
            lines.extend(["", title, "-" * 64])
            for result in group_results:
                lines.append(
                    f"[{result.status}] {result.category}: {result.summary}"
                )
                if result.source:
                    lines.append(f"  source: {result.source}")
                if result.target:
                    lines.append(f"  target: {result.target}")
                if result.method:
                    lines.append(f"  comparison: {result.method}")
                if result.location:
                    lines.append(f"  location: {result.location}")
                if result.hint:
                    lines.append(f"  hint: {result.hint}")
                if result.details:
                    lines.append("  issues:")
                    for detail in result.details[:50]:
                        if result.category == "asam-quality-checker":
                            locations = ", ".join(
                                location.get("xpath", "")
                                or location.get("description", "")
                                or str(location)
                                for location in detail.get("locations", [])
                            )
                            resolved = [
                                location.get("resolved_target", {})
                                for location in detail.get("locations", [])
                                if location.get("resolved_target")
                            ]
                            resolved_text = ", ".join(
                                f"road={item.get('road_id')}/lane={item.get('lane_id')}"
                                for item in resolved
                            )
                            lines.append(
                                "    - "
                                f"rule={detail.get('rule_uid', '')}; "
                                f"level={detail.get('level', '')}; "
                                f"description={detail.get('description', '')}; "
                                f"rule_description={detail.get('checker_description', '')}; "
                                f"location={locations or '<not provided>'}; "
                                f"resolved={resolved_text or '<not resolved>'}"
                            )
                        else:
                            source_text = ", ".join(
                                f"road={item.get('road_id')}/lane={item.get('lane_id')}"
                                for item in detail.get(
                                    "source_target_locations", []
                                )
                            )
                            target_text = ", ".join(
                                f"road={item.get('road_id')}/lane={item.get('lane_id')}"
                                for item in detail.get(
                                    "target_target_locations", []
                                )
                            )
                            lines.append(
                                "    - "
                                f"source_lanelet={detail.get('source_lanelet_id')}"
                                f"[{source_text or 'unmapped'}] -> "
                                f"target_lanelet={detail.get('target_lanelet_id')}"
                                f"[{target_text or 'unmapped'}]; "
                                f"relation={detail.get('relation_type')}; "
                                f"shared_nodes={detail.get('shared_endpoint_node_ids')}"
                            )
                    if len(result.details) > 50:
                        lines.append(
                            f"    - ... {len(result.details) - 50} more issue(s); "
                            "see diagnostics JSON or the original XQAR."
                        )
                lines.append("")

        payload = {
            "schema_version": "1.0",
            "conversion": "lanelet2_to_opendrive",
            "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "source": str(self.original_source_path),
            "prepared_source": (
                str(self.source_path)
                if self.source_path != self.original_source_path
                else ""
            ),
            "source_preparation": [
                record
                for record in self.source_preparation
                if record.get("status") != "PASS"
            ],
            "target": str(self.target_path),
            "elapsed_seconds": round(elapsed, 3),
            "summary": {
                status.lower(): counts[status]
                for status in ("PASS", "WARN", "FAIL", "REVIEW", "SKIP")
            },
            "result": (
                "FAIL"
                if counts["FAIL"]
                else "PASS_WITH_WARNINGS"
                if counts["WARN"]
                else "PASS"
            ),
            "acceptance": structured_groups["acceptance"],
            "diagnostics": {
                "target_conformance": structured_groups["target_conformance"],
                "topology": structured_groups["topology"],
                "element_mapping": structured_groups["element_mapping"],
            },
        }
        self.json_report_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def _add(
        self,
        stage: str,
        category: str,
        status: str,
        summary: str,
        source: str = "",
        target: str = "",
        method: str = "",
        location: str = "",
        hint: str = "",
        details: Optional[List[Dict[str, object]]] = None,
    ) -> None:
        self.results.append(
            CheckResult(
                stage=stage,
                category=category,
                status=status,
                summary=summary,
                source=source,
                target=target,
                method=method,
                location=location,
                hint=hint,
                details=details or [],
            )
        )

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _tags(element: etree._Element) -> Dict[str, str]:
        return {
            tag.get("k", ""): tag.get("v", "")
            for tag in element.findall("tag")
            if tag.get("k")
        }

    @staticmethod
    def _node_point(
        node: etree._Element,
        origin_lat: Optional[float] = None,
        origin_lon: Optional[float] = None,
    ) -> Optional[Point2D]:
        for x_key, y_key in (("local_x", "local_y"), ("x", "y")):
            if node.get(x_key) is None or node.get(y_key) is None:
                continue
            try:
                return float(node.get(x_key)), float(node.get(y_key))
            except ValueError:
                return None
        if node.get("lon") is not None and node.get("lat") is not None:
            try:
                latitude = float(node.get("lat"))
                longitude = float(node.get("lon"))
            except ValueError:
                return None
            if origin_lat is None or origin_lon is None:
                return longitude, latitude
            radius = 6371000.0
            x = (
                radius
                * math.radians(longitude - origin_lon)
                * math.cos(math.radians(origin_lat))
            )
            y = radius * math.radians(latitude - origin_lat)
            return x, y
        return None

    @staticmethod
    def _way_points(
        way_id: str,
        ways: Dict[str, List[str]],
        nodes: Dict[str, Point2D],
    ) -> List[Point2D]:
        return [
            nodes[node_id]
            for node_id in ways.get(way_id, [])
            if node_id in nodes
        ]

    @classmethod
    def _approximate_centerline(
        cls, left: Sequence[Point2D], right: Sequence[Point2D]
    ) -> List[Point2D]:
        if not left:
            return list(right)
        if not right:
            return list(left)
        count = max(2, min(30, max(len(left), len(right))))
        left_length = cls._polyline_length(left)
        right_length = cls._polyline_length(right)
        result = []
        for index in range(count):
            ratio = index / (count - 1)
            lp = cls._point_at_distance(left, ratio * left_length)
            rp = cls._point_at_distance(right, ratio * right_length)
            result.append(((lp[0] + rp[0]) * 0.5, (lp[1] + rp[1]) * 0.5))
        return result

    @staticmethod
    def _point_at_distance(
        points: Sequence[Point2D], distance: float
    ) -> Point2D:
        if not points:
            return 0.0, 0.0
        remaining = max(0.0, distance)
        for start, end in zip(points, points[1:]):
            length = math.hypot(end[0] - start[0], end[1] - start[1])
            if length <= 1e-12:
                continue
            if remaining <= length:
                ratio = remaining / length
                return (
                    start[0] + ratio * (end[0] - start[0]),
                    start[1] + ratio * (end[1] - start[1]),
                )
            remaining -= length
        return points[-1]

    @staticmethod
    def _polyline_length(points: Sequence[Point2D]) -> float:
        return sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(points, points[1:])
        )

    @staticmethod
    def _heading_change(points: Sequence[Point2D]) -> float:
        headings = []
        for start, end in zip(points, points[1:]):
            if math.hypot(end[0] - start[0], end[1] - start[1]) > 1e-9:
                headings.append(
                    math.atan2(end[1] - start[1], end[0] - start[0])
                )
        return sum(
            abs(
                math.atan2(
                    math.sin(second - first), math.cos(second - first)
                )
            )
            for first, second in zip(headings, headings[1:])
        )

    @classmethod
    def _geometry_sample_points(
        cls, geometry: etree._Element
    ) -> List[Point2D]:
        x = cls._float(geometry.get("x"))
        y = cls._float(geometry.get("y"))
        heading = cls._float(geometry.get("hdg"))
        length = cls._float(geometry.get("length"))
        points = [(x, y)]
        arc = geometry.find("arc")
        if arc is not None:
            curvature = cls._float(arc.get("curvature"))
            if abs(curvature) > 1e-12:
                end_heading = heading + curvature * length
                points.append(
                    (
                        x + (math.sin(end_heading) - math.sin(heading)) / curvature,
                        y - (math.cos(end_heading) - math.cos(heading)) / curvature,
                    )
                )
                return points
        points.append(
            (
                x + length * math.cos(heading),
                y + length * math.sin(heading),
            )
        )
        return points

    @staticmethod
    def _bounds(
        points: Sequence[Point2D],
    ) -> Optional[Tuple[float, float, float, float]]:
        if not points:
            return None
        return (
            min(point[0] for point in points),
            min(point[1] for point in points),
            max(point[0] for point in points),
            max(point[1] for point in points),
        )

    @staticmethod
    def _float(value: Optional[str], default: float = 0.0) -> float:
        try:
            return float(value) if value is not None else default
        except ValueError:
            return default

    @staticmethod
    def _int(value: Optional[str], default: int = 0) -> int:
        try:
            return int(value) if value is not None else default
        except ValueError:
            return default

    @staticmethod
    def _counter(counter: Counter) -> str:
        if not counter:
            return "{}"
        return "{" + ", ".join(
            f"{key}:{value}" for key, value in sorted(counter.items())
        ) + "}"

    @staticmethod
    def _truncate(text: str, limit: int) -> str:
        if not text:
            return ""
        return text if len(text) <= limit else text[:limit] + "...[truncated]"


def diagnose_lanelet2_to_opendrive(
    source_lanelet2_path: str | Path,
    target_xodr_path: str | Path,
    external_validator_cmd: Optional[str] = None,
    external_timeout_sec: int = 20,
    original_source_path: str | Path | None = None,
    source_preparation: Optional[Sequence[Dict[str, str]]] = None,
) -> Path:
    """Diagnose one Lanelet2 -> OpenDRIVE conversion and return the report path."""
    return Lanelet2OpenDriveDiagnostics(
        source_lanelet2_path=source_lanelet2_path,
        target_xodr_path=target_xodr_path,
        external_validator_cmd=external_validator_cmd,
        external_timeout_sec=external_timeout_sec,
        original_source_path=original_source_path,
        source_preparation=source_preparation,
    ).run()


def find_opendrive_viewer() -> Optional[str]:
    local_viewer = (
        PROJECT_ROOT
        / "tools"
        / "esmini"
        / "bin"
        / "odrviewer"
    )
    if local_viewer.is_file() and os.access(local_viewer, os.X_OK):
        return str(local_viewer)
    return shutil.which("odrviewer")


def _launch_viewer_enabled() -> bool:
    return os.environ.get("AUTOMAP_CONVERTER_LAUNCH_VIEWER", "").strip().lower() in {
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


def launch_opendrive_viewer(target_xodr_path: str | Path) -> Optional[subprocess.Popen]:
    """Launch odrviewer without blocking the conversion process."""
    viewer = find_opendrive_viewer()
    if not viewer:
        return None
    return subprocess.Popen(
        [viewer, "--odr", str(Path(target_xodr_path).resolve())],
        start_new_session=True,
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Evaluate Lanelet2 -> OpenDRIVE conversion quality."
    )
    parser.add_argument("source_lanelet2")
    parser.add_argument("target_xodr")
    parser.add_argument(
        "--validator-command",
        default=None,
        help="External loader command containing {xodr}.",
    )
    parser.add_argument("--timeout", type=int, default=20)
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Do not open odrviewer after writing the reports.",
    )
    arguments = parser.parse_args()

    report = diagnose_lanelet2_to_opendrive(
        arguments.source_lanelet2,
        arguments.target_xodr,
        external_validator_cmd=arguments.validator_command,
        external_timeout_sec=arguments.timeout,
    )
    print(report)
    if not arguments.no_viewer:
        launch_opendrive_viewer(arguments.target_xodr)
