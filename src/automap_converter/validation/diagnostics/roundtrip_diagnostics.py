from __future__ import annotations

import argparse
import json
import math
import re
import time
import traceback
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

from lxml import etree
from shapely.geometry import LineString
from shapely.strtree import STRtree

from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion.network import (
    Network,
)
from automap_converter.core.config.general_config import general_config
from automap_converter.core.config.opendrive_config import open_drive_config
from automap_converter.core.traceability import (
    LINEAGE,
    coordinate_controls,
    document_id,
    extension,
    metric_frame,
    native_elements,
    parse_xml,
    read_frame,
    transform_points,
)
from automap_converter.formats.opendrive.parser.parser import parse_opendrive
from automap_converter.formats.lanelet2.geometry import orient_boundaries

from ._workflow_runtime import (
    convert_lanelet2_to_osm,
    convert_opendrive_to_lanelet2,
    convert_osm_to_lanelet2,
)

PROJECT_ROOT = Path(__file__).resolve().parents[4]
SAMPLES_ROOT = PROJECT_ROOT / "data" / "samples"
ROUNDTRIP_ROOT = PROJECT_ROOT / "data" / "results" / "roundtrip"
PROVENANCE_PREFIXES = ("lanelet2:", "source:", "conversion:")


@dataclass
class Feature:
    key: str
    points: List[Tuple[float, float]]
    width: Optional[float]
    kind: str
    direction: str
    left: List[Tuple[float, float]] = field(default_factory=list)
    right: List[Tuple[float, float]] = field(default_factory=list)
    attributes: dict = field(default_factory=dict)
    ancestors: List[dict] = field(default_factory=list)
    native_keys: List[str] = field(default_factory=list)
    relations: dict = field(default_factory=dict)
    rules: List[dict] = field(default_factory=list)
    measurements: dict = field(default_factory=dict)


@dataclass
class FeatureSet:
    format: str
    features: List[Feature]
    topology_edges: set[Tuple[str, str]] = field(default_factory=set)
    semantics: Counter = field(default_factory=Counter)
    geographic: bool = False
    frame: dict = field(default_factory=dict)
    document: str = ""
    coordinate_checks: dict = field(default_factory=dict)


@dataclass
class Match:
    source_index: int
    target_index: int
    mean_error_m: float
    reverse_error_m: float
    orientation_preserved: bool


@dataclass(frozen=True)
class RoundtripSpec:
    name: str
    pair: str
    source_format: str
    intermediate_format: str
    source_dir: Path
    source_suffix: str
    first: Callable[[Path, Path, Path], object]
    second: Callable[[Path, Path, Path], object]


def _lanelet2_to_opendrive(source: Path, target: Path, _: Path) -> None:
    from ._workflow_runtime import convert_lanelet2_to_opendrive
    convert_lanelet2_to_opendrive(source, target, _)


def _opendrive_to_osm(source: Path, target: Path, intermediate_dir: Path) -> None:
    """Compose OpenDRIVE -> Lanelet2 -> OSM for a roundtrip middle map."""

    lanelet2_path = intermediate_dir / "opendrive_to_lanelet2.osm"
    convert_opendrive_to_lanelet2(
        source, lanelet2_path, intermediate_dir / "opendrive_to_lanelet2"
    )
    convert_lanelet2_to_osm(
        lanelet2_path, target, intermediate_dir / "lanelet2_to_osm"
    )


def _osm_to_opendrive(source: Path, target: Path, intermediate_dir: Path) -> None:
    """Compose OSM -> Lanelet2 -> OpenDRIVE for a roundtrip middle map."""

    lanelet2_path = intermediate_dir / "osm_to_lanelet2.osm"
    convert_osm_to_lanelet2(
        source, lanelet2_path, intermediate_dir / "osm_to_lanelet2"
    )
    _lanelet2_to_opendrive(
        lanelet2_path, target, intermediate_dir / "lanelet2_to_opendrive"
    )


SPECS: Dict[str, RoundtripSpec] = {
    "lanelet2-opendrive-lanelet2": RoundtripSpec(
        name="lanelet2-opendrive-lanelet2",
        pair="lanelet2-opendrive",
        source_format="lanelet2",
        intermediate_format="opendrive",
        source_dir=SAMPLES_ROOT / "lanelet2",
        source_suffix=".osm",
        first=_lanelet2_to_opendrive,
        second=convert_opendrive_to_lanelet2,
    ),
    "opendrive-lanelet2-opendrive": RoundtripSpec(
        name="opendrive-lanelet2-opendrive",
        pair="lanelet2-opendrive",
        source_format="opendrive",
        intermediate_format="lanelet2",
        source_dir=SAMPLES_ROOT / "opendrive",
        source_suffix=".xodr",
        first=convert_opendrive_to_lanelet2,
        second=_lanelet2_to_opendrive,
    ),
    "lanelet2-osm-lanelet2": RoundtripSpec(
        name="lanelet2-osm-lanelet2",
        pair="lanelet2-osm",
        source_format="lanelet2",
        intermediate_format="osm",
        source_dir=SAMPLES_ROOT / "lanelet2",
        source_suffix=".osm",
        first=convert_lanelet2_to_osm,
        second=convert_osm_to_lanelet2,
    ),
    "osm-lanelet2-osm": RoundtripSpec(
        name="osm-lanelet2-osm",
        pair="lanelet2-osm",
        source_format="osm",
        intermediate_format="lanelet2",
        source_dir=SAMPLES_ROOT / "osm",
        source_suffix=".osm",
        first=convert_osm_to_lanelet2,
        second=convert_lanelet2_to_osm,
    ),
    "opendrive-osm-opendrive": RoundtripSpec(
        name="opendrive-osm-opendrive",
        pair="opendrive-osm",
        source_format="opendrive",
        intermediate_format="osm",
        source_dir=SAMPLES_ROOT / "opendrive",
        source_suffix=".xodr",
        first=_opendrive_to_osm,
        second=_osm_to_opendrive,
    ),
    "osm-opendrive-osm": RoundtripSpec(
        name="osm-opendrive-osm",
        pair="opendrive-osm",
        source_format="osm",
        intermediate_format="opendrive",
        source_dir=SAMPLES_ROOT / "osm",
        source_suffix=".osm",
        first=_osm_to_opendrive,
        second=_opendrive_to_osm,
    ),
}

DEFAULT_SCENARIOS = {
    "lanelet2": [
        "traffic_speed_limit_utm.osm",
        "merging_lanelets_utm.osm",
        "basic_intersection_area.osm",
    ],
    "opendrive": [
        "commonroad_straight_road.xodr",
        "highway_merge.xodr",
        "highway_intersection_test0.xodr",
    ],
    "osm": [
        "ped_crossing.osm",
    ],
}


def _conversion_contract():
    from automap_converter.api.settings import current_runtime_settings
    if current_runtime_settings().osm_profile == "lanelet2-v1":
        return "osm-lanelet2-profile/v1; public roads + declared native Lanelet2 detail; provenance identity-only"
    return "native-format-semantics/v1; provenance identity-only; no reconstruction-profile fallback"


def run_roundtrip(
    spec: RoundtripSpec,
    source: Path,
    output_dir: Path,
) -> Dict[str, object]:
    started = time.time()
    case_dir = output_dir / spec.name / source.stem
    intermediate_dir = case_dir / "intermediate"
    case_dir.mkdir(parents=True, exist_ok=True)
    intermediate_dir.mkdir(parents=True, exist_ok=True)
    middle_suffix = ".xodr" if spec.intermediate_format == "opendrive" else ".osm"
    middle = case_dir / f"01_{spec.intermediate_format}{middle_suffix}"
    result_suffix = ".xodr" if spec.source_format == "opendrive" else ".osm"
    result = case_dir / f"03_roundtrip_{spec.source_format}{result_suffix}"

    payload: Dict[str, object] = {
        "schema": "vector-map-roundtrip-diagnostics/v6",
        "case": spec.name,
        "pair": spec.pair,
        "source": _relative(source),
        "intermediate": _relative(middle),
        "roundtrip_target": _relative(result),
        "pipeline": [],
        "metrics": [],
        "issues": [],
        "visual_review": {
            "status": "REVIEW",
            "method": "Open the source and roundtrip target in their format viewers and compare them manually.",
        },
        "result": "FAIL",
    }
    current_stage = "source_feature_extraction"
    try:
        source_features = extract_features(source, spec.source_format)
        payload["source_inventory"] = inventory(source_features)
        _record_stage(payload, current_stage, "PASS")
        current_stage = "first_conversion"
        spec.first(source, middle, intermediate_dir / "first")
        _record_stage(payload, current_stage, "PASS", _relative(middle))
        payload["lineage_policy"] = {
            "matching": "Native conversion lineage, retained across both legs; no nearest-distance matching.",
            "measurement": "Actual generated coordinates, boundaries, connectivity and attributes only.",
        }
        payload["conversion_contract"] = _conversion_contract()
        payload["sanitized_intermediate"] = None
        current_stage = "reverse_conversion"
        spec.second(middle, result, intermediate_dir / "second")
        _record_stage(payload, current_stage, "PASS", _relative(result))
        current_stage = "roundtrip_feature_extraction"
        result_features = extract_features(
            result, spec.source_format
        )
        _record_stage(payload, current_stage, "PASS")
        current_stage = "metric_comparison"
        comparison = compare_feature_sets(source_features, result_features)
        payload.update(comparison)
        payload["target_inventory"] = inventory(result_features)
        payload["result"] = comparison["result"]
        from .native_semantics import opendrive_speed_consistency
        payload["native_semantic_checks"] = [
            {"file": _relative(path), **opendrive_speed_consistency(path)}
            for path, fmt in ((middle, spec.intermediate_format), (result, spec.source_format))
            if fmt == "opendrive"
        ]
        if any(check["status"] == "FAIL" for check in payload["native_semantic_checks"]):
            payload["result"] = "FAIL"
            payload["issues"].append({
                "category": "native-speed-consistency", "status": "FAIL",
                "summary": "Generated native lane speed and applicable speed signal disagree.",
            })
        _record_stage(payload, "native_semantic_validation",
                      "FAIL" if any(c["status"] == "FAIL" for c in payload["native_semantic_checks"]) else "PASS")
        _record_stage(payload, current_stage, str(comparison["result"]))
    except Exception as exc:
        from ._roundtrip_contract import METRIC_NAMES
        payload["metrics"] = [
            {"name": name, "value": None, "pass_rate": None, "pass_rate_percent": None,
             "passed": 0, "total": 0, "status": "FAIL",
             "method": "Not measured: conversion/extraction failed; see failed_stage and error."}
            for name in METRIC_NAMES
        ]
        _record_stage(payload, current_stage, "FAIL", f"{type(exc).__name__}: {exc}")
        payload["issues"] = [
            {
                "category": "roundtrip-execution",
                "status": "FAIL",
                "summary": f"{type(exc).__name__}: {exc}",
                "hint": "Inspect the generated intermediate files and converter traceback.",
            }
        ]
        payload["error_type"] = type(exc).__name__
        payload["error"] = str(exc)
        payload["failed_stage"] = current_stage
        payload["traceback"] = traceback.format_exc()
        payload["failed_metrics"] = ["roundtrip_execution"]
    payload["elapsed_seconds"] = round(time.time() - started, 3)
    json_path = case_dir / "roundtrip_diagnostics.json"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    payload["diagnostics_json"] = _relative(json_path)
    return payload


def _record_stage(
    payload: Dict[str, object], stage: str, status: str, detail: str = ""
) -> None:
    pipeline = payload.setdefault("pipeline", [])
    if isinstance(pipeline, list):
        pipeline.append({"stage": stage, "status": status, "detail": detail})


def sanitize_intermediate(source: Path, target: Path, map_format: str) -> Dict[str, int]:
    tree = etree.parse(str(source))
    root = tree.getroot()
    removed = Counter()
    if map_format == "opendrive":
        for element in list(root.xpath(".//*[local-name()='userData']")):
            code = str(element.get("code", ""))
            if code.startswith(PROVENANCE_PREFIXES):
                removed[code.split(":", 1)[0] + ":*"] += 1
                parent = element.getparent()
                if parent is not None:
                    parent.remove(element)
    else:
        for tag in list(root.xpath(".//*[local-name()='tag']")):
            key = str(tag.get("k", ""))
            if key.startswith(PROVENANCE_PREFIXES):
                removed[key.split(":", 1)[0] + ":*"] += 1
                parent = tag.getparent()
                if parent is not None:
                    parent.remove(tag)
    target.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    return dict(removed)


def extract_features(
    path: Path, map_format: str, prefer_geographic: Optional[bool] = None
) -> FeatureSet:
    if map_format == "opendrive":
        return _extract_opendrive(path)
    if map_format == "lanelet2":
        return _extract_lanelet2(path, prefer_geographic)
    if map_format == "osm":
        return _extract_osm(path, prefer_geographic)
    raise ValueError(f"Unsupported roundtrip format: {map_format}")


def _extract_lanelet2(path: Path, prefer_geographic: Optional[bool] = None) -> FeatureSet:
    root = etree.parse(str(path)).getroot()
    frame = read_frame(root, "lanelet2", fallback_id=document_id(path))
    if prefer_geographic is None:
        prefer_geographic = frame["kind"] == "geographic"
    nodes, geographic = _read_osm_nodes(root, prefer_geographic)
    native_frame = frame
    if geographic and nodes:
        frame = metric_frame(native_frame, next(iter(nodes.values())))
        nodes = dict(zip(nodes, transform_points(list(nodes.values()), native_frame, frame)))
        geographic = False
    ways = {
        str(way.get("id", "")): [
            nodes[ref]
            for ref in (str(nd.get("ref", "")) for nd in way.findall("nd"))
            if ref in nodes
        ]
        for way in root.findall("way")
    }
    features: List[Feature] = []
    semantics = Counter()
    for relation in root.findall("relation"):
        tags = _tags(relation)
        relation_type = tags.get("type", "")
        if relation_type == "regulatory_element":
            semantics[f"regulatory:{tags.get('subtype', '<none>')}"] += 1
        elif relation_type == "multipolygon":
            semantics[f"area:{tags.get('subtype', '<none>')}"] += 1
        if relation_type != "lanelet":
            continue
        members = relation.findall("member")
        left_id = next((str(m.get("ref", "")) for m in members if m.get("role") == "left"), "")
        right_id = next((str(m.get("ref", "")) for m in members if m.get("role") == "right"), "")
        left = ways.get(left_id, [])
        right = ways.get(right_id, [])
        if len(left) < 2 or len(right) < 2:
            continue
        lp, rp = orient_boundaries(LineString(left), LineString(right))
        left, right = list(lp.coords), list(rp.coords)
        sample_count = max(8, min(40, max(len(left), len(right))))
        left_sample = _resample(left, sample_count)
        right_sample = _resample(right, sample_count)
        center = [
            ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            for a, b in zip(left_sample, right_sample)
        ]
        center_id = next((m.get("ref") for m in members if m.get("role") == "centerline"), "")
        if len(ways.get(center_id, [])) >= 2:
            center = ways[center_id]
        widths = [
            _geographic_distance(a, b) if geographic else _distance(a, b)
            for a, b in zip(left_sample, right_sample)
        ]
        features.append(
            Feature(
                key=str(relation.get("id", "")),
                points=center,
                width=sum(widths) / len(widths),
                kind=tags.get("subtype", tags.get("location", "road")),
                direction=_direction_value(tags.get("one_way:vehicle", tags.get("one_way", tags.get("oneway", "yes")))),
                left=left,
                right=right,
                attributes=_lanelet_attributes(root, relation),
                rules=_lanelet_rules(root, relation, nodes),
            )
        )
    for way in root.findall("way"):
        tags = _tags(way)
        element_type = tags.get("type", "")
        if element_type in {"stop_line", "traffic_light", "traffic_sign"}:
            semantics[f"primitive:{element_type}"] += 1
    feature_set = FeatureSet("lanelet2", features, semantics=semantics, geographic=geographic)
    _decorate_features(feature_set, root, path, native_frame)
    feature_set.frame = frame
    _lanelet_relations(feature_set, root)
    return feature_set


def _extract_osm(path: Path, prefer_geographic: Optional[bool] = None) -> FeatureSet:
    from automap_converter.formats.osm.road_semantics import road_profile
    root = etree.parse(str(path)).getroot()
    frame = read_frame(root, "osm", fallback_id=document_id(path))
    if prefer_geographic is None:
        prefer_geographic = frame["kind"] == "geographic"
    nodes, geographic = _read_osm_nodes(root, prefer_geographic)
    features: List[Feature] = []
    semantics = Counter()
    for node in root.findall("node"):
        categories = _osm_semantic_categories(_tags(node))
        semantics.update(categories)
    for way in root.findall("way"):
        tags = _tags(way)
        refs = [str(nd.get("ref", "")) for nd in way.findall("nd")]
        points = [nodes[ref] for ref in refs if ref in nodes]
        highway = tags.get("highway", "")
        if highway and len(points) >= 2:
            features.append(
                Feature(
                    key=str(way.get("id", "")),
                    points=points,
                    width=road_profile(tags)["width_m"],
                    kind=highway,
                    direction=_direction_value(tags.get("oneway", "no")),
                    attributes=_osm_attributes(root, way),
                    rules=_osm_rules(root, way, nodes),
                    measurements={k: road_profile(tags)[k] for k in ("width_basis", "estimated", "assumptions")},
                )
            )
        semantics.update(_osm_semantic_categories(tags))
        if _closed_area(refs, tags):
            semantics[f"area:{_osm_area_kind(tags)}"] += 1
    for relation in root.findall("relation"):
        tags = _tags(relation)
        if tags.get("type") == "restriction":
            semantics[f"restriction:{tags.get('restriction', '<none>')}"] += 1
    feature_set = FeatureSet("osm", features, semantics=semantics, geographic=geographic)
    _decorate_features(feature_set, root, path, frame)
    endpoints = {f.key: [n.get("ref") for n in root.find(f"way[@id='{f.key}']").findall("nd")]
                 for f in features}
    departures = {}
    for feature in features:
        refs = endpoints[feature.key]
        outgoing = refs[:-1] if feature.direction in {"yes", "1", "true"} else refs[1:] if feature.direction == "-1" else refs
        for node in outgoing:
            departures.setdefault(node, set()).add(feature.key)
    for feature in features:
        refs = endpoints[feature.key]
        incoming = refs[1:] if feature.direction in {"yes", "1", "true"} else refs[:-1] if feature.direction == "-1" else refs
        for node in incoming:
            for successor in departures.get(node, set()) - {feature.key}:
                feature_set.topology_edges.add((feature.key, successor))
    for relation in root.findall("relation"):
        data = _tags(relation)
        if data.get("type") != "restriction":
            continue
        members = relation.findall("member")
        incoming = {m.get("ref") for m in members if m.get("role") == "from" and m.get("type") == "way"}
        outgoing = {m.get("ref") for m in members if m.get("role") == "to" and m.get("type") == "way"}
        if data.get("restriction", "").startswith("no_"):
            feature_set.topology_edges -= {(a, b) for a in incoming for b in outgoing}
        elif data.get("restriction", "").startswith("only_"):
            feature_set.topology_edges = {(a, b) for a, b in feature_set.topology_edges if a not in incoming or b in outgoing}
    return feature_set


def _extract_opendrive(path: Path) -> FeatureSet:
    root = parse_xml(path).getroot()
    elements = native_elements(root, "opendrive")
    opendrive = parse_opendrive(path)
    # Roundtrip geometry is compared in the native OpenDRIVE local frame.
    # geoReference is valid conversion metadata, but applying it here would
    # compare projected coordinates on one side with local coordinates on the other.
    opendrive.header.geo_reference = None
    network = Network()
    network.load_opendrive(opendrive)
    scenario = network.export_commonroad_scenario(general_config, open_drive_config)
    features: List[Feature] = []
    id_to_key: Dict[object, str] = {}
    lanelets = list(scenario.lanelet_network.lanelets)
    for lanelet in lanelets:
        key = str(lanelet.lanelet_id)
        id_to_key[lanelet.lanelet_id] = key
        points = [tuple(map(float, point[:2])) for point in lanelet.center_vertices]
        left = [tuple(map(float, point[:2])) for point in lanelet.left_vertices]
        right = [tuple(map(float, point[:2])) for point in lanelet.right_vertices]
        widths = [_distance(a, b) for a, b in zip(_resample(left, 20), _resample(right, 20))]
        lanelet_type = getattr(lanelet, "lanelet_type", set()) or set()
        kind = ",".join(sorted(str(item).split(".")[-1].lower() for item in lanelet_type)) or "driving"
        native_keys = ["|".join(map(str, item)) for item in
                       getattr(lanelet, "opendrive_lane_locations_detailed", set())]
        if not native_keys and getattr(lanelet, "opendrive_road_id", None) is not None:
            native_keys = ["|".join(str(getattr(lanelet, attr)) for attr in (
                "opendrive_road_id", "opendrive_lane_section_id", "opendrive_lane_id"))]
        if not native_keys and "crosswalk" in kind:
            # The importer models crosswalk objects as lanelets; they are road
            # objects, not additional native driving lanes for these measures.
            continue
        native = [elements[k] for k in native_keys if k in elements]
        native_kinds = sorted({e.get("type", "driving") for e in native})
        if native_kinds:
            kind = ",".join(native_kinds)
        ancestors = [ref for e in native for ref in (extension(e, LINEAGE) or {}).get("ancestors", [])]
        features.append(
            Feature(
                key=key,
                points=points,
                width=sum(widths) / len(widths) if widths else None,
                kind=kind,
                direction="forward",
                left=left,
                right=right,
                native_keys=native_keys,
                ancestors=ancestors,
                attributes=_opendrive_attributes(root, native_keys),
                rules=_opendrive_rules(root, native_keys, opendrive),
                relations={
                    "left": [str(lanelet.adj_left)] if lanelet.adj_left is not None else [],
                    "right": [str(lanelet.adj_right)] if lanelet.adj_right is not None else [],
                },
            )
        )
    topology = set()
    for lanelet in lanelets:
        source_key = id_to_key.get(lanelet.lanelet_id)
        if source_key is None:
            continue
        for successor in getattr(lanelet, "successor", []) or []:
            if successor in id_to_key:
                topology.add((source_key, id_to_key[successor]))
    semantics = Counter()
    for signal in root.xpath(".//*[local-name()='signal']"):
        semantics[f"signal:{_opendrive_signal_kind(signal)}"] += 1
    for obj in root.xpath(".//*[local-name()='object']"):
        semantics[f"object:{obj.get('type', '<none>')}"] += 1
    result = FeatureSet("opendrive", features, topology, semantics, False)
    result.frame = read_frame(root, "opendrive", fallback_id=document_id(path))
    result.document = document_id(path)
    result.coordinate_checks = coordinate_controls(root, "opendrive", result.frame)
    return result


def _decorate_features(feature_set, root, path, frame):
    feature_set.frame = frame
    feature_set.document = document_id(path)
    feature_set.coordinate_checks = coordinate_controls(root, feature_set.format, frame)
    elements = native_elements(root, feature_set.format)
    for feature in feature_set.features:
        feature.native_keys = [feature.key]
        element = elements.get(feature.key)
        if element is not None:
            feature.ancestors = (extension(element, LINEAGE) or {}).get("ancestors", [])


def _lanelet_relations(feature_set, root):
    relations = {r.get("id"): r for r in root.findall("relation")}
    ways = {w.get("id"): [n.get("ref") for n in w.findall("nd")] for w in root.findall("way")}
    bounds, endpoints, starts, ends = {}, {}, {}, {}
    left_users, right_users = {}, {}
    native_frame = read_frame(root, "lanelet2", fallback_id=feature_set.document)
    node_points, _ = _read_osm_nodes(root, native_frame["kind"] == "geographic")
    node_points = dict(zip(node_points, transform_points(list(node_points.values()), native_frame, feature_set.frame)))
    for feature in feature_set.features:
        relation = relations[feature.key]
        left = next((m.get("ref") for m in relation.findall("member") if m.get("role") == "left"), "")
        right = next((m.get("ref") for m in relation.findall("member") if m.get("role") == "right"), "")
        bounds[feature.key] = (left, right)
        left_nodes, right_nodes = ways.get(left, []), ways.get(right, [])
        if left_nodes and right_nodes:
            if feature.left and _distance(feature.left[0], node_points[left_nodes[0]]) > 1e-10:
                left_nodes = list(reversed(left_nodes))
            if feature.right and _distance(feature.right[0], node_points[right_nodes[0]]) > 1e-10:
                right_nodes = list(reversed(right_nodes))
            start, end = (left_nodes[0], right_nodes[0]), (left_nodes[-1], right_nodes[-1])
            endpoints[feature.key] = (start, end)
            starts.setdefault(start, []).append(feature.key)
            ends.setdefault(end, []).append(feature.key)
        left_users.setdefault(left, []).append(feature.key)
        right_users.setdefault(right, []).append(feature.key)
    # For native Lanelet2 files, topology is built from actual shared boundary
    # objects/node IDs, never proximity of unrelated centerline endpoints.
    for a in feature_set.features:
        la, ra = bounds[a.key]
        a.measurements["native_boundary_ids"] = {"left": la, "right": ra}
        start, end = endpoints.get(a.key, (None, None))
        a.relations = {
            "predecessor": [k for k in ends.get(start, []) if k != a.key],
            "successor": [k for k in starts.get(end, []) if k != a.key],
            "left": sorted((set(left_users.get(la, []) + right_users.get(la, []))) - {a.key}),
            "right": sorted((set(left_users.get(ra, []) + right_users.get(ra, []))) - {a.key}),
        }
    feature_set.topology_edges = {(f.key, k) for f in feature_set.features for k in f.relations.get("successor", [])}


def _lanelet_attributes(root, relation):
    rules = {}
    for member in relation.findall("member"):
        if member.get("role") != "regulatory_element":
            continue
        rule = root.find(f"relation[@id='{member.get('ref')}']")
        if rule is None:
            continue
        rt = _tags(rule)
        kind = rt.get("subtype", "")
        if kind == "speed_limit":
            value = _speed_value(rt.get("sign_type", rt.get("maxspeed", "")), "km/h")
        elif kind == "traffic_light":
            value = "traffic_light"
        elif kind in {"right_of_way", "all_way_stop"}:
            roles = sorted({m.get("role") for m in rule.findall("member") if m.get("ref") == relation.get("id")})
            value = ",".join(roles) or kind
        else:
            continue
        rules.setdefault("rule:" + kind, []).append(value)
        if any(m.get("role") == "ref_line" for m in rule.findall("member")):
            rules.setdefault("rule:stop_line", []).append("stop_line")
    return {k: sorted(v) for k, v in rules.items()}


def _osm_attributes(root, way):
    from automap_converter.formats.osm.road_semantics import road_profile
    data = _tags(way)
    result = {k: data[k] for k in ("maxspeed", "turn:lanes") if k in data}
    model = road_profile(data)
    result["lanes"] = model["lanes"]
    if model["highway"] not in {"footway", "path", "cycleway", "pedestrian", "steps"}:
        result["lanes_forward"] = model["forward"]
        result["lanes_backward"] = model["backward"]
    if "maxspeed" in result:
        result["maxspeed"] = _speed_value(result["maxspeed"], "km/h")
    controls = []
    for ref in way.findall("nd"):
        node = root.find(f"node[@id='{ref.get('ref')}']")
        if node is not None:
            nt = _tags(node)
            if nt.get("highway") in {"traffic_signals", "stop", "give_way", "crossing"}:
                controls.append(nt["highway"])
    if controls:
        result["traffic_controls"] = sorted(controls)
    restrictions = []
    for relation in root.findall("relation"):
        rt = _tags(relation)
        if rt.get("type") == "restriction" and any(m.get("type") == "way" and m.get("ref") == way.get("id")
                for m in relation.findall("member")):
            restrictions.append(rt.get("restriction", ""))
    if restrictions:
        result["turn_restrictions"] = sorted(restrictions)
    return result


def _opendrive_attributes(root, keys):
    attrs = {}
    elements = native_elements(root, "opendrive")
    for key in keys:
        element = elements.get(key)
        if element is None:
            continue
        road_id, _section, lane_id = key.split("|")
        road = root.find(f"road[@id='{road_id}']")
        speeds = element.findall("speed") or road.findall("type/speed")
        if speeds:
            attrs["speeds"] = sorted(_speed_value(s.get("max", ""), s.get("unit", "m/s")) for s in speeds)
        signals = []
        for signal in road.findall("signals/signal"):
            validity = signal.findall("validity")
            if validity and not any(int(v.get("fromLane")) <= int(lane_id) <= int(v.get("toLane")) for v in validity):
                continue
            signals.append((signal.get("type", ""), signal.get("subtype", ""),
                            signal.get("value", ""), signal.get("unit", "")))
        if signals:
            attrs["signals"] = sorted(signals)
        crossings = [o for o in road.findall("objects/object") if o.get("type") == "crosswalk"]
        if crossings:
            attrs["crosswalk_objects"] = sorted(o.get("type") for o in crossings)
    return attrs


def _object_points(root, element, nodes):
    if element is None:
        return []
    if element.tag == "node":
        return [nodes[element.get("id")]] if element.get("id") in nodes else []
    return [nodes[n.get("ref")] for n in element.findall("nd") if n.get("ref") in nodes]


def _direction_value(value):
    value = str(value).strip().lower()
    return {"1": "yes", "true": "yes", "0": "no", "false": "no"}.get(value, value)


def _speed_value(value, default_unit):
    text = str(value).strip().lower()
    match = re.fullmatch(r"([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:e[+-]?\d+)?)\s*(km/?h|kmh|kph|m/s|mps|mph)?", text)
    if not match:
        return text
    unit = match.group(2) or default_unit
    multiplier = 1 / 3.6 if unit in {"km/h", "kmh", "kph"} else 0.44704 if unit == "mph" else 1.0
    return f"{float(match.group(1)) * multiplier:.4f} m/s"


def _lanelet_rules(root, relation, nodes):
    from ._roundtrip_contract import SUPPORTED_RULES
    result = []
    for member in relation.findall("member"):
        if member.get("role") != "regulatory_element":
            continue
        rule = root.find(f"relation[@id='{member.get('ref')}']")
        if rule is None:
            continue
        data = _tags(rule)
        kind = data.get("subtype", "")
        if kind not in SUPPORTED_RULES and kind != "all_way_stop":
            continue
        for ref in rule.findall("member"):
            role = ref.get("role")
            if role not in {"refers", "ref_line"}:
                continue
            element = root.find(f"{ref.get('type')}[@id='{ref.get('ref')}']")
            points = _object_points(root, element, nodes)
            if points:
                result.append({"kind": "stop_line" if role == "ref_line" else kind,
                               "points": points, "value": (_speed_value(data.get("sign_type", ""), "m/s")
                                                            if kind == "speed_limit" else data.get("sign_type", ""))})
    return result


def _osm_rules(root, way, nodes):
    result = []
    for ref in way.findall("nd"):
        node = root.find(f"node[@id='{ref.get('ref')}']")
        if node is None:
            continue
        data = _tags(node)
        kind = {"traffic_signals": "traffic_light", "stop": "stop", "give_way": "yield",
                "crossing": "crosswalk"}.get(data.get("highway"))
        if kind and ref.get("ref") in nodes:
            result.append({"kind": kind, "value": "", "points": [nodes[ref.get("ref")]]})
    return result


def _opendrive_rules(root, keys, parsed):
    result = []
    for key in keys:
        road_id, section_index, lane_id = key.split("|")
        road = root.find(f"road[@id='{road_id}']")
        if road is None:
            continue
        sections = road.findall("lanes/laneSection")
        start = float(sections[int(section_index)].get("s"))
        end = float(sections[int(section_index) + 1].get("s")) if int(section_index) + 1 < len(sections) else float(road.get("length"))
        parsed_road = next((r for r in parsed.roads if str(r.id) == road_id), None)
        if parsed_road is None:
            continue
        for signal in road.findall("signals/signal"):
            s, t = float(signal.get("s", "0")), float(signal.get("t", "0"))
            if not start <= s <= end:
                continue
            validity = signal.findall("validity")
            if validity and not any(int(v.get("fromLane")) <= int(lane_id) <= int(v.get("toLane")) for v in validity):
                continue
            kind = _opendrive_signal_kind(signal)
            if kind not in {"traffic_light", "right_of_way", "speed_limit", "stop_line"}:
                continue
            p, heading, _, _ = parsed_road.plan_view.calc(s, compute_curvature=False)
            point = (float(p[0] - t * math.sin(heading)), float(p[1] + t * math.cos(heading)))
            value = signal.get("value", "")
            if kind == "speed_limit":
                value = _speed_value(value, signal.get("unit", "m/s"))
            result.append({"kind": kind, "points": [point], "value": value})
        for obj in road.findall("objects/object"):
            if obj.get("type") != "crosswalk":
                continue
            s, t = float(obj.get("s", "0")), float(obj.get("t", "0"))
            if not start <= s <= end:
                continue
            p, heading, _, _ = parsed_road.plan_view.calc(s, compute_curvature=False)
            result.append({"kind": "crosswalk", "value": "",
                           "points": [(float(p[0] - t * math.sin(heading)), float(p[1] + t * math.cos(heading)))]})
    return result


def compare_feature_sets(source: FeatureSet, target: FeatureSet) -> Dict[str, object]:
    from ._roundtrip_contract import compare
    return compare(source, target)


def _metric(
    name: str,
    value: Optional[float],
    threshold: float,
    operator: str,
    method: str,
) -> Dict[str, object]:
    if value is None:
        status = "SKIP"
        rendered: object = None
    else:
        status = "PASS" if (value <= threshold if operator == "<=" else value >= threshold) else "FAIL"
        rendered = round(value, 6) if math.isfinite(value) else "infinity"
    return {
        "name": name,
        "value": rendered,
        "threshold": threshold,
        "operator": operator,
        "status": status,
        "method": method,
    }


def _match_features(
    source: FeatureSet,
    target: FeatureSet,
    source_points: List[List[Tuple[float, float]]],
    target_points: List[List[Tuple[float, float]]],
) -> List[Match]:
    indexed_targets = [
        (index, LineString(points))
        for index, points in enumerate(target_points)
        if len(points) >= 2
    ]
    target_tree = (
        STRtree([geometry for _, geometry in indexed_targets])
        if indexed_targets
        else None
    )
    selected: List[Match] = []
    for source_index, points in enumerate(source_points):
        sampled = _resample(points, 20)
        candidates: List[Match] = []
        candidate_indices: List[int] = []
        if target_tree is not None and len(points) >= 2:
            source_geometry = LineString(points)
            tree_indices = list(target_tree.query(source_geometry.envelope.buffer(10.0)))
            if not tree_indices:
                nearest = target_tree.nearest(source_geometry)
                if nearest is not None:
                    tree_indices = [nearest]
            candidate_indices = [indexed_targets[int(index)][0] for index in tree_indices]
        for target_index in candidate_indices:
            target_line = target_points[target_index]
            distances_and_positions = [
                _point_to_polyline_distance_and_position(point, target_line)
                for point in sampled
            ]
            mean_error = (
                sum(item[0] for item in distances_and_positions)
                / len(distances_and_positions)
                if distances_and_positions
                else math.inf
            )
            start_position = distances_and_positions[0][1]
            end_position = distances_and_positions[-1][1]
            candidates.append(
                Match(
                    source_index,
                    target_index,
                    mean_error,
                    mean_error,
                    end_position + 1e-9 >= start_position,
                )
            )
        selected.append(
            min(candidates, key=lambda item: item.mean_error_m)
            if candidates
            else Match(source_index, -1, math.inf, math.inf, False)
        )
    return selected


def _metric_feature_points(
    source: FeatureSet, target: FeatureSet
) -> Tuple[List[List[Tuple[float, float]]], List[List[Tuple[float, float]]]]:
    source_lines = [feature.points for feature in source.features]
    target_lines = [feature.points for feature in target.features]
    if not source.geographic:
        return source_lines, target_lines
    all_points = [point for line in source_lines + target_lines for point in line]
    if not all_points:
        return source_lines, target_lines
    lon0 = sum(point[0] for point in all_points) / len(all_points)
    lat0 = sum(point[1] for point in all_points) / len(all_points)
    cos_lat = math.cos(math.radians(lat0))

    def project(lines: Sequence[Sequence[Tuple[float, float]]]) -> List[List[Tuple[float, float]]]:
        return [
            [
                (
                    (point[0] - lon0) * 111320.0 * cos_lat,
                    (point[1] - lat0) * 110540.0,
                )
                for point in line
            ]
            for line in lines
        ]

    return project(source_lines), project(target_lines)


def _retained_topology_edges(
    source: FeatureSet,
    target: FeatureSet,
    mapping: Dict[int, int],
    max_hops: int,
) -> Tuple[int, List[Dict[str, str]]]:
    source_index = {feature.key: index for index, feature in enumerate(source.features)}
    target_index = {feature.key: index for index, feature in enumerate(target.features)}
    adjacency: Dict[int, set[int]] = {}
    for start, end in target.topology_edges:
        if start in target_index and end in target_index:
            adjacency.setdefault(target_index[start], set()).add(target_index[end])
    retained = 0
    missing: List[Dict[str, str]] = []
    for start, end in source.topology_edges:
        source_start = source_index.get(start)
        source_end = source_index.get(end)
        target_start = mapping.get(source_start) if source_start is not None else None
        target_end = mapping.get(source_end) if source_end is not None else None
        if target_start is not None and target_end is not None and (
            target_start == target_end or _reachable(adjacency, target_start, target_end, max_hops)
        ):
            retained += 1
        else:
            missing.append({"source_from": start, "source_to": end})
    return retained, missing


def _reachable(adjacency: Dict[int, set[int]], start: int, end: int, max_hops: int) -> bool:
    queue = deque([(start, 0)])
    visited = {start}
    while queue:
        node, depth = queue.popleft()
        if depth >= max_hops:
            continue
        for neighbor in adjacency.get(node, set()):
            if neighbor == end:
                return True
            if neighbor not in visited:
                visited.add(neighbor)
                queue.append((neighbor, depth + 1))
    return False


def _infer_topology(features: Sequence[Feature], geographic: bool) -> set[Tuple[str, str]]:
    if not features:
        return set()
    scale_x = 111320.0 * math.cos(
        math.radians(sum(feature.points[0][1] for feature in features) / len(features))
    ) if geographic else 1.0
    scale_y = 110540.0 if geographic else 1.0

    def endpoint_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot((a[0] - b[0]) * scale_x, (a[1] - b[1]) * scale_y)

    edges = set()
    for source in features:
        for target in features:
            if source.key != target.key and endpoint_distance(source.points[-1], target.points[0]) <= 1.0:
                edges.add((source.key, target.key))
    return edges


def _read_osm_nodes(
    root: etree._Element, prefer_geographic: Optional[bool] = None
) -> Tuple[Dict[str, Tuple[float, float]], bool]:
    nodes: Dict[str, Tuple[float, float]] = {}
    geographic_count = 0
    for node in root.findall("node"):
        tags = _tags(node)
        lon = _as_float(node.get("lon"))
        lat = _as_float(node.get("lat"))
        local_x = _as_float(tags.get("local_x"))
        local_y = _as_float(tags.get("local_y"))
        if prefer_geographic is False:
            if local_x is None or local_y is None:
                continue
            point = (local_x, local_y)
        elif lon is not None and lat is not None:
            point = (lon, lat)
            geographic_count += 1
        else:
            if local_x is None or local_y is None:
                continue
            point = (local_x, local_y)
        node_id = str(node.get("id", ""))
        if node_id:
            nodes[node_id] = point
    return nodes, geographic_count > 0


def _tags(element: etree._Element) -> Dict[str, str]:
    return {
        str(tag.get("k", "")): str(tag.get("v", ""))
        for tag in element.findall("tag")
        if tag.get("k")
    }


def _resample(points: Sequence[Tuple[float, float]], count: int) -> List[Tuple[float, float]]:
    if not points:
        return []
    if len(points) == 1 or count <= 1:
        return [points[0]] * max(1, count)
    lengths = [0.0]
    for index in range(1, len(points)):
        lengths.append(lengths[-1] + _distance(points[index - 1], points[index]))
    total = lengths[-1]
    if total <= 1e-12:
        return [points[0]] * count
    result = []
    segment = 1
    for sample_index in range(count):
        position = total * sample_index / (count - 1)
        while segment < len(lengths) - 1 and lengths[segment] < position:
            segment += 1
        previous = segment - 1
        span = lengths[segment] - lengths[previous]
        ratio = 0.0 if span <= 1e-12 else (position - lengths[previous]) / span
        a, b = points[previous], points[segment]
        result.append((a[0] + ratio * (b[0] - a[0]), a[1] + ratio * (b[1] - a[1])))
    return result


def _paired_mean_distance(
    source: Sequence[Tuple[float, float]], target: Sequence[Tuple[float, float]]
) -> float:
    if not source or not target:
        return math.inf
    return sum(_distance(a, b) for a, b in zip(source, target)) / min(len(source), len(target))


def _point_to_polyline_distance_and_position(
    point: Tuple[float, float], polyline: Sequence[Tuple[float, float]]
) -> Tuple[float, float]:
    if not polyline:
        return math.inf, 0.0
    if len(polyline) == 1:
        return _distance(point, polyline[0]), 0.0

    best_distance = math.inf
    best_position = 0.0
    traversed = 0.0
    for start, end in zip(polyline, polyline[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        segment_length = math.sqrt(length_squared)
        if length_squared <= 1e-18:
            distance = _distance(point, start)
            ratio = 0.0
        else:
            ratio = max(
                0.0,
                min(
                    1.0,
                    ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
                    / length_squared,
                ),
            )
            projection = (start[0] + ratio * dx, start[1] + ratio * dy)
            distance = _distance(point, projection)
        if distance < best_distance:
            best_distance = distance
            best_position = traversed + ratio * segment_length
        traversed += segment_length
    return best_distance, best_position


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _geographic_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    latitude = math.radians((a[1] + b[1]) / 2.0)
    return math.hypot(
        (a[0] - b[0]) * 111320.0 * math.cos(latitude),
        (a[1] - b[1]) * 110540.0,
    )


def _as_float(value: object) -> Optional[float]:
    try:
        return float(value) if value not in (None, "") else None
    except (TypeError, ValueError):
        return None


def _osm_width(tags: Dict[str, str]) -> Optional[float]:
    width_text = tags.get("width", "").split(";", 1)[0].strip().lower().replace("m", "")
    width = _as_float(width_text)
    if width is not None and width > 0.0:
        return width
    lanes = _as_float(tags.get("lanes"))
    return lanes * 3.5 if lanes is not None and lanes > 0.0 else None


def _osm_semantic_categories(tags: Dict[str, str]) -> Counter:
    result = Counter()
    highway = tags.get("highway", "")
    traffic_sign = tags.get("traffic_sign", "").lower()
    if highway == "traffic_signals" or tags.get("traffic_signals"):
        result["traffic:traffic_light"] += 1
    if highway == "stop" or "stop" in traffic_sign or "206" in traffic_sign:
        result["traffic:stop"] += 1
    if highway in {"give_way", "yield"} or "give_way" in traffic_sign or "205" in traffic_sign:
        result["traffic:give_way"] += 1
    if tags.get("maxspeed") or "274" in traffic_sign:
        result["traffic:speed_limit"] += 1
    if tags.get("type") == "stop_line":
        result["traffic:stop_line"] += 1
    if highway == "crossing" or tags.get("crossing"):
        result["traffic:crossing"] += 1
    return result


def _closed_area(refs: Sequence[str], tags: Dict[str, str]) -> bool:
    return len(refs) >= 4 and refs[0] == refs[-1] and (
        tags.get("area") == "yes" or bool(tags.get("building")) or bool(tags.get("landuse"))
    )


def _osm_area_kind(tags: Dict[str, str]) -> str:
    for key in ("building", "landuse", "amenity", "highway", "subtype"):
        if tags.get(key):
            return f"{key}={tags[key]}"
    return "unknown"


def _opendrive_signal_kind(signal: etree._Element) -> str:
    dynamic = str(signal.get("dynamic", "")).lower()
    country = str(signal.get("country", "")).upper()
    signal_type = str(signal.get("type", ""))
    subtype = str(signal.get("subtype", "")).lower()
    if dynamic in {"yes", "true", "1"}:
        return "traffic_light"
    if signal_type in {"205", "206"} or subtype in {"yield", "give_way", "stop"}:
        return "right_of_way"
    if signal_type in {"274", "274.1"} or "speed" in subtype:
        return "speed_limit"
    if country == "OPENDRIVE" and signal_type in {"294", "1100001"}:
        return "stop_line"
    return f"{country or '<none>'}:{signal_type or '<none>'}:{subtype or '<none>'}"


def _kind_equal(source: str, target: str) -> bool:
    aliases = {
        "road": "driving",
        "urban": "driving",
        "motorway": "driving",
        "primary": "driving",
        "secondary": "driving",
        "tertiary": "driving",
        "residential": "driving",
        "unclassified": "driving",
    }
    return aliases.get(source, source) == aliases.get(target, target)


def inventory(feature_set: FeatureSet) -> Dict[str, object]:
    return {
        "format": feature_set.format,
        "features": len(feature_set.features),
        "topology_edges": len(feature_set.topology_edges),
        "feature_types": dict(Counter(feature.kind for feature in feature_set.features)),
        "semantics": dict(feature_set.semantics),
        "coordinate_system": "geographic" if feature_set.geographic else "local_metric",
    }


def write_summary(output_dir: Path, rows: Sequence[Dict[str, object]]) -> Tuple[Path, Path]:
    passed = sum(row.get("result") == "PASS" for row in rows)
    total = len(rows)
    pair_totals: Dict[str, Dict[str, object]] = {}
    for pair in sorted({str(row.get("pair", "")) for row in rows if row.get("pair")}):
        pair_rows = [row for row in rows if row.get("pair") == pair]
        pair_passed = sum(row.get("result") == "PASS" for row in pair_rows)
        pair_totals[pair] = {
            "total": len(pair_rows),
            "passed": pair_passed,
            "failed": len(pair_rows) - pair_passed,
            "pass_rate": round(pair_passed / len(pair_rows), 6) if pair_rows else 0.0,
        }
    payload = {
        "schema": "vector-map-roundtrip-summary/v6",
        "conversion_contract": _conversion_contract(),
        "total": total,
        "passed": passed,
        "failed": total - passed,
        "pass_rate": round(passed / total, 6) if total else 0.0,
        "pairs": pair_totals,
        "cases": list(rows),
    }
    json_path = output_dir / "summary.json"
    txt_path = output_dir / "summary.txt"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    lines = [
        "Vector map bidirectional conversion summary",
        "=" * 100,
        f"Total={total} PASS={passed} FAIL={total - passed} PassRate={payload['pass_rate']:.2%}",
        "",
        "Result  Roundtrip                              Map                              LineMeans   Width       Topology    Attributes  Failed metrics",
        "-" * 100,
    ]
    for pair, values in pair_totals.items():
        lines.insert(
            4,
            f"Pair {pair}: PASS={values['passed']}/{values['total']} "
            f"PassRate={values['pass_rate']:.2%}",
        )
    for row in rows:
        from ._roundtrip_contract import METRIC_NAMES
        by_name = {m["name"]: m for m in row.get("metrics", [])}
        percentages = [
            (f"{by_name[name]['pass_rate']:.2%}" if by_name.get(name, {}).get("pass_rate") is not None
             else "N/A") for name in METRIC_NAMES
        ]
        failed_metrics = ",".join(
            str(metric.get("name"))
            for metric in row.get("metrics", [])
            if metric.get("status") == "FAIL"
        )
        if not failed_metrics:
            failed_metrics = ",".join(str(item) for item in row.get("failed_metrics", []))
        lines.append(
            f"{str(row.get('result', 'FAIL')):<7} "
            f"{str(row.get('case', '')):<38} "
            f"{Path(str(row.get('source', ''))).name:<32} "
            + " ".join(f"{p:<11}" for p in percentages) + f" {failed_metrics or '-'}"
        )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    markdown = ["# Roundtrip Test-Plan Results", "",
                "Contract: " + _conversion_contract(), "",
                "| Route | Map | Line means <=1m | Width <=10% | Topology | Attributes | Result |",
                "| --- | --- | ---: | ---: | ---: | ---: | --- |"]
    for row in rows:
        by_name = {m["name"]: m for m in row.get("metrics", [])}
        values = [(f"{by_name[name]['pass_rate']:.2%}" if by_name.get(name, {}).get("pass_rate") is not None
                   else "N/A") for name in METRIC_NAMES]
        markdown.append("| " + " | ".join([
            str(row.get("case", "")), Path(str(row.get("source", ""))).name,
            *values, str(row.get("result", "FAIL"))]) + " |")
    markdown.extend(["", "N/A means no applicable source samples, or an execution error (see JSON).",
                     "Coordinate pass rate counts source lines whose mean paired-point error is <=1m; acceptance requires 100% of source-line means to pass. Individual point errors remain evidence only.",
                     "Width acceptance requires every sampled applicable section <=10%; topology/attributes >=85%.",
                     "Lanelet2 lateral matches additionally require actual native shared-boundary intervals at all five fixed source stations. Raw graph matches and full-interval coverage are retained as evidence.",
                     "Functional equivalence requires separate viewer/routing/simulation review."])
    (output_dir / "summary.md").write_text("\n".join(markdown) + "\n", encoding="utf-8")
    return txt_path, json_path


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run lineage-based A -> B -> A' tests using four test-plan measures."
    )
    parser.add_argument(
        "--case",
        choices=["all", *SPECS.keys()],
        default="all",
        help="Roundtrip direction to test. Default: all six vector directions.",
    )
    parser.add_argument("--source", default="", help="Optional single source map for one case.")
    parser.add_argument("--source-dir", default="", help="Optional source directory override.")
    parser.add_argument("--limit", type=int, default=3, help="Maps per case; default 3.")
    parser.add_argument("--run-name", default="", help="Output folder name.")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> Path:
    args = parse_args(argv)
    selected = list(SPECS.values()) if args.case == "all" else [SPECS[args.case]]
    run_name = args.run_name or time.strftime("roundtrip_%Y%m%d_%H%M%S")
    output_dir = ROUNDTRIP_ROOT / run_name
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for spec in selected:
        if args.source:
            sources = [Path(args.source)]
        else:
            source_dir = Path(args.source_dir) if args.source_dir else spec.source_dir
            if not args.source_dir and spec.source_format in DEFAULT_SCENARIOS:
                sources = [
                    source_dir / name
                    for name in DEFAULT_SCENARIOS[spec.source_format]
                    if (source_dir / name).is_file()
                ][: args.limit]
            else:
                sources = sorted(
                    source_dir.glob(f"*{spec.source_suffix}"),
                    key=lambda path: (path.stat().st_size, path.name.lower()),
                )[: args.limit]
        for source in sources:
            print(f"[roundtrip] {spec.name}: {_relative(source)}")
            row = run_roundtrip(spec, source, output_dir)
            rows.append(row)
            failed = [
                metric["name"]
                for metric in row.get("metrics", [])
                if metric.get("status") == "FAIL"
            ]
            print(f"  result={row['result']} failed_metrics={','.join(failed) or '-'}")
    txt_path, _ = write_summary(output_dir, rows)
    print(f"Roundtrip summary: {txt_path}")
    return txt_path


if __name__ == "__main__":
    main()
