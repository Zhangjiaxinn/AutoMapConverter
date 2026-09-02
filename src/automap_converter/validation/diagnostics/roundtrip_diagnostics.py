from __future__ import annotations

import argparse
import json
import math
import time
import traceback
from collections import Counter, deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Tuple

from lxml import etree

from automap_converter.core.config.lanelet2_config import lanelet2_config
from automap_converter.core.config.opendrive_config import open_drive_config
from automap_converter.core.config.general_config import general_config
from automap_converter.api.interface import (
    lanelet_to_opendrive,
)
from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion.network import Network
from automap_converter.formats.opendrive.parser.parser import parse_opendrive

from ._workflow_runtime import (
    convert_lanelet2_to_osm,
    convert_opendrive_to_lanelet2,
    convert_osm_to_lanelet2,
)


PROJECT_ROOT = Path(__file__).resolve().parents[4]
ROUNDTRIP_ROOT = PROJECT_ROOT / "roundtrip_results"
PROVENANCE_PREFIXES = ("lanelet2:", "source:", "conversion:")


@dataclass
class Feature:
    key: str
    points: List[Tuple[float, float]]
    width: Optional[float]
    kind: str
    direction: str


@dataclass
class FeatureSet:
    format: str
    features: List[Feature]
    topology_edges: set[Tuple[str, str]] = field(default_factory=set)
    semantics: Counter = field(default_factory=Counter)
    geographic: bool = False


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
    target.parent.mkdir(parents=True, exist_ok=True)
    lanelet2_config.adjacencies = True
    lanelet_to_opendrive(
        str(source),
        str(target),
        lanelet2_conf=lanelet2_config,
        odr_conf=open_drive_config,
    )


SPECS: Dict[str, RoundtripSpec] = {
    "lanelet2-opendrive-lanelet2": RoundtripSpec(
        name="lanelet2-opendrive-lanelet2",
        pair="lanelet2-opendrive",
        source_format="lanelet2",
        intermediate_format="opendrive",
        source_dir=PROJECT_ROOT / "maps/lanelet2",
        source_suffix=".osm",
        first=_lanelet2_to_opendrive,
        second=convert_opendrive_to_lanelet2,
    ),
    "opendrive-lanelet2-opendrive": RoundtripSpec(
        name="opendrive-lanelet2-opendrive",
        pair="lanelet2-opendrive",
        source_format="opendrive",
        intermediate_format="lanelet2",
        source_dir=PROJECT_ROOT / "maps/opendrive",
        source_suffix=".xodr",
        first=convert_opendrive_to_lanelet2,
        second=_lanelet2_to_opendrive,
    ),
    "lanelet2-osm-lanelet2": RoundtripSpec(
        name="lanelet2-osm-lanelet2",
        pair="lanelet2-osm",
        source_format="lanelet2",
        intermediate_format="osm",
        source_dir=PROJECT_ROOT / "maps/lanelet2",
        source_suffix=".osm",
        first=convert_lanelet2_to_osm,
        second=convert_osm_to_lanelet2,
    ),
    "osm-lanelet2-osm": RoundtripSpec(
        name="osm-lanelet2-osm",
        pair="lanelet2-osm",
        source_format="osm",
        intermediate_format="lanelet2",
        source_dir=PROJECT_ROOT / "maps/osm",
        source_suffix=".osm",
        first=convert_osm_to_lanelet2,
        second=convert_lanelet2_to_osm,
    ),
}

DEFAULT_SCENARIOS = {
    "lanelet2": [
        "traffic_speed_limit_utm.osm",
        "merging_lanelets_utm.osm",
        "basic_intersection_area.osm",
    ],
    "opendrive": [
        "commonroad__straight_road.xodr",
        "highway_merge.xodr",
        "highway_intersection_test0.xodr",
    ],
    "osm": [
        "garching_intersection.osm",
        "munich_garching_small.osm",
        "map_without_crossing_nodes.osm",
    ],
}


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
    clean_middle = case_dir / f"02_{spec.intermediate_format}_standard_only{middle_suffix}"
    result_suffix = ".xodr" if spec.source_format == "opendrive" else ".osm"
    result = case_dir / f"03_roundtrip_{spec.source_format}{result_suffix}"

    payload: Dict[str, object] = {
        "schema": "vector-map-roundtrip-diagnostics/v1",
        "case": spec.name,
        "pair": spec.pair,
        "source": _relative(source),
        "intermediate": _relative(middle),
        "sanitized_intermediate": _relative(clean_middle),
        "roundtrip_target": _relative(result),
        "anti_cheat": {},
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
        current_stage = "provenance_sanitization"
        removed = sanitize_intermediate(middle, clean_middle, spec.intermediate_format)
        payload["anti_cheat"] = {
            "status": "PASS",
            "policy": (
                "The reverse conversion receives a copy with lanelet2:/source:/conversion: "
                "provenance removed. Matching never reads source IDs."
            ),
            "removed_fields": removed,
        }
        _record_stage(payload, current_stage, "PASS", _relative(clean_middle))
        current_stage = "reverse_conversion"
        spec.second(clean_middle, result, intermediate_dir / "second")
        _record_stage(payload, current_stage, "PASS", _relative(result))
        current_stage = "roundtrip_feature_extraction"
        result_features = extract_features(result, spec.source_format)
        _record_stage(payload, current_stage, "PASS")
        current_stage = "metric_comparison"
        comparison = compare_feature_sets(source_features, result_features)
        payload.update(comparison)
        payload["target_inventory"] = inventory(result_features)
        payload["result"] = comparison["result"]
        _record_stage(payload, current_stage, str(comparison["result"]))
    except Exception as exc:
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
    txt_path = case_dir / "roundtrip_diagnostics.txt"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    _write_text_report(txt_path, payload)
    payload["diagnostics_json"] = _relative(json_path)
    payload["diagnostics_txt"] = _relative(txt_path)
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


def extract_features(path: Path, map_format: str) -> FeatureSet:
    if map_format == "opendrive":
        return _extract_opendrive(path)
    if map_format == "lanelet2":
        return _extract_lanelet2(path)
    if map_format == "osm":
        return _extract_osm(path)
    raise ValueError(f"Unsupported roundtrip format: {map_format}")


def _extract_lanelet2(path: Path) -> FeatureSet:
    root = etree.parse(str(path)).getroot()
    nodes, geographic = _read_osm_nodes(root)
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
        if _distance(left[0], right[-1]) + _distance(left[-1], right[0]) < _distance(
            left[0], right[0]
        ) + _distance(left[-1], right[-1]):
            right = list(reversed(right))
        sample_count = max(8, min(40, max(len(left), len(right))))
        left_sample = _resample(left, sample_count)
        right_sample = _resample(right, sample_count)
        center = [
            ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)
            for a, b in zip(left_sample, right_sample)
        ]
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
                direction=tags.get("one_way", "yes"),
            )
        )
    for way in root.findall("way"):
        tags = _tags(way)
        element_type = tags.get("type", "")
        if element_type in {"stop_line", "traffic_light", "traffic_sign"}:
            semantics[f"primitive:{element_type}"] += 1
    feature_set = FeatureSet("lanelet2", features, semantics=semantics, geographic=geographic)
    feature_set.topology_edges = _infer_topology(features, geographic)
    return feature_set


def _extract_osm(path: Path) -> FeatureSet:
    root = etree.parse(str(path)).getroot()
    nodes, geographic = _read_osm_nodes(root)
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
                    width=_osm_width(tags),
                    kind=highway,
                    direction=tags.get("oneway", "no"),
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
    feature_set.topology_edges = _infer_topology(features, geographic)
    return feature_set


def _extract_opendrive(path: Path) -> FeatureSet:
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
        features.append(
            Feature(
                key=key,
                points=points,
                width=sum(widths) / len(widths) if widths else None,
                kind=kind,
                direction="forward",
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
    if not topology:
        topology = _infer_topology(features, False)
    root = etree.parse(str(path)).getroot()
    semantics = Counter()
    for signal in root.xpath(".//*[local-name()='signal']"):
        semantics[f"signal:{_opendrive_signal_kind(signal)}"] += 1
    for obj in root.xpath(".//*[local-name()='object']"):
        semantics[f"object:{obj.get('type', '<none>')}"] += 1
    return FeatureSet("opendrive", features, topology, semantics, False)


def compare_feature_sets(source: FeatureSet, target: FeatureSet) -> Dict[str, object]:
    if source.geographic != target.geographic:
        raise ValueError(
            "Source and roundtrip target use incompatible local/geographic coordinate representations."
        )
    source_points, target_points = _metric_feature_points(source, target)
    matches = _match_features(source, target, source_points, target_points)
    accepted = [match for match in matches if match.mean_error_m <= 5.0]
    mapping = {match.source_index: match.target_index for match in accepted}
    geometry_errors = [match.mean_error_m for match in accepted]
    width_errors = []
    attribute_matches = 0
    attribute_total = 0
    for match in accepted:
        src = source.features[match.source_index]
        dst = target.features[match.target_index]
        if src.width is not None and src.width > 1e-6 and dst.width is not None:
            width_errors.append(abs(src.width - dst.width) / src.width)
        attribute_total += 2
        attribute_matches += int(_kind_equal(src.kind, dst.kind))
        direction_equal = src.direction == dst.direction
        if src.direction in {"yes", "1", "true", "forward"}:
            direction_equal = direction_equal and match.orientation_preserved
        attribute_matches += int(direction_equal)

    retained_edges, missing_edges = _retained_topology_edges(
        source, target, mapping, max_hops=3
    )
    semantic_total = sum(source.semantics.values())
    semantic_retained = sum(
        min(count, target.semantics.get(category, 0))
        for category, count in source.semantics.items()
    )
    metrics = [
        _metric(
            "feature_coverage",
            len(accepted) / len(source.features) if source.features else 1.0,
            0.85,
            ">=",
            "Ratio of source lanes/roads with a geometry match within 5 m.",
        ),
        _metric(
            "centerline_mean_error_m",
            sum(geometry_errors) / len(geometry_errors) if geometry_errors else math.inf,
            1.0,
            "<=",
            "Mean point distance after equal-distance resampling; no ID/provenance matching.",
        ),
        _metric(
            "lane_width_mean_relative_error",
            sum(width_errors) / len(width_errors) if width_errors else None,
            0.10,
            "<=",
            "Mean relative error of matched lane/road widths.",
        ),
        _metric(
            "topology_edge_match_rate",
            retained_edges / len(source.topology_edges) if source.topology_edges else None,
            0.85,
            ">=",
            "Source predecessor/successor edges retained directly, by merge, or within 3 target hops.",
        ),
        _metric(
            "type_direction_attribute_match_rate",
            attribute_matches / attribute_total if attribute_total else None,
            0.85,
            ">=",
            "Matched lane/road type and travel direction agreement.",
        ),
        _metric(
            "semantic_element_retention_rate",
            semantic_retained / semantic_total if semantic_total else None,
            0.85,
            ">=",
            "Traffic-rule and area/object category count retained without source-only metadata.",
        ),
    ]
    failed = [metric for metric in metrics if metric["status"] == "FAIL"]
    unmatched = [
        {
            "source_key": source.features[index].key,
            "nearest_error_m": round(matches[index].mean_error_m, 6) if index < len(matches) else None,
        }
        for index in range(len(source.features))
        if index not in mapping
    ]
    worst = sorted(accepted, key=lambda item: item.mean_error_m, reverse=True)[:25]
    issues = [
        {
            "category": metric["name"],
            "status": "FAIL",
            "summary": (
                f"actual={metric['value']} does not satisfy "
                f"{metric['operator']} {metric['threshold']}"
            ),
        }
        for metric in failed
    ]
    return {
        "result": "PASS" if not failed else "FAIL",
        "metrics": metrics,
        "issues": issues,
        "evidence": {
            "matched_features": len(accepted),
            "unmatched_source_features": unmatched[:500],
            "worst_geometry_matches": [
                {
                    "source_key": source.features[item.source_index].key,
                    "target_key": target.features[item.target_index].key,
                    "mean_error_m": round(item.mean_error_m, 6),
                    "orientation_preserved": item.orientation_preserved,
                }
                for item in worst
            ],
            "missing_topology_edges": missing_edges[:500],
            "source_semantics": dict(source.semantics),
            "target_semantics": dict(target.semantics),
        },
    }


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
    candidates: List[Match] = []
    for source_index, points in enumerate(source_points):
        sampled = _resample(points, 20)
        for target_index, target_line in enumerate(target_points):
            target_sampled = _resample(target_line, 20)
            direct = _paired_mean_distance(sampled, target_sampled)
            reverse = _paired_mean_distance(sampled, list(reversed(target_sampled)))
            candidates.append(
                Match(
                    source_index,
                    target_index,
                    min(direct, reverse),
                    reverse,
                    direct <= reverse,
                )
            )
    selected: Dict[int, Match] = {}
    used_targets = set()
    for candidate in sorted(candidates, key=lambda item: item.mean_error_m):
        if candidate.source_index in selected or candidate.target_index in used_targets:
            continue
        selected[candidate.source_index] = candidate
        used_targets.add(candidate.target_index)
    return [
        selected.get(index, Match(index, -1, math.inf, math.inf, False))
        for index in range(len(source_points))
    ]


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


def _read_osm_nodes(root: etree._Element) -> Tuple[Dict[str, Tuple[float, float]], bool]:
    nodes: Dict[str, Tuple[float, float]] = {}
    geographic_count = 0
    for node in root.findall("node"):
        tags = _tags(node)
        lon = _as_float(node.get("lon"))
        lat = _as_float(node.get("lat"))
        if lon is not None and lat is not None:
            point = (lon, lat)
            geographic_count += 1
        else:
            local_x = _as_float(tags.get("local_x"))
            local_y = _as_float(tags.get("local_y"))
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


def _write_text_report(path: Path, payload: Dict[str, object]) -> None:
    lines = [
        "Vector map roundtrip diagnostic",
        "=" * 72,
        f"Case: {payload.get('case', '')}",
        f"Source: {payload.get('source', '')}",
        f"Roundtrip target: {payload.get('roundtrip_target', '')}",
        f"Result: {payload.get('result', 'FAIL')}",
        "",
        "Anti-cheat:",
        f"  {json.dumps(payload.get('anti_cheat', {}), ensure_ascii=False)}",
        "",
        "Metrics:",
    ]
    for metric in payload.get("metrics", []):
        lines.append(
            f"  [{metric.get('status')}] {metric.get('name')}: "
            f"value={metric.get('value')} required {metric.get('operator')} {metric.get('threshold')}"
        )
    lines.extend(["", "Pipeline:"])
    for stage in payload.get("pipeline", []):
        lines.append(
            f"  [{stage.get('status')}] {stage.get('stage')}: {stage.get('detail', '')}"
        )
    issues = payload.get("issues", [])
    lines.extend(["", f"Issues: {len(issues)}"])
    for issue in issues:
        lines.append(f"  [{issue.get('status')}] {issue.get('category')}: {issue.get('summary')}")
    lines.extend(
        [
            "",
            "Visual review:",
            f"  {json.dumps(payload.get('visual_review', {}), ensure_ascii=False)}",
        ]
    )
    if payload.get("traceback"):
        lines.extend(["", "Traceback:", str(payload["traceback"]).rstrip()])
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
        "schema": "vector-map-roundtrip-summary/v1",
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
        "Result  Roundtrip                              Map                              Failed metrics",
        "-" * 100,
    ]
    for pair, values in pair_totals.items():
        lines.insert(
            4,
            f"Pair {pair}: PASS={values['passed']}/{values['total']} "
            f"PassRate={values['pass_rate']:.2%}",
        )
    for row in rows:
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
            f"{Path(str(row.get('source', ''))).name:<32} {failed_metrics or '-'}"
        )
    txt_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return txt_path, json_path


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run provenance-free A -> B -> A vector-map consistency tests."
    )
    parser.add_argument(
        "--case",
        choices=["all", *SPECS.keys()],
        default="all",
        help="Roundtrip direction to test. Default: all four directions.",
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
