from __future__ import annotations

import json
import math
import time
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from lxml import etree

from automap_converter.api.settings import current_runtime_settings


PROJECT_ROOT = Path(__file__).resolve().parents[4]
STATUS_ORDER = ("PASS", "WARN", "FAIL", "REVIEW", "SKIP")


@dataclass
class DiagnosticCheck:
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
class OsmStats:
    nodes: int = 0
    ways: int = 0
    relations: int = 0
    highway_ways: int = 0
    area_ways: int = 0
    lanelets: int = 0
    regulatory_elements: int = 0
    multipolygons: int = 0
    traffic_signals: int = 0
    traffic_signs: int = 0
    speed_limits: int = 0
    turn_restrictions: int = 0
    highway_intersections: int = 0
    stop_controls: int = 0
    give_way_controls: int = 0
    crosswalk_areas: int = 0
    missing_refs: int = 0
    missing_primitive_versions: int = 0
    zero_node_ways: int = 0
    lanelet_missing_boundaries: int = 0
    duplicate_ids: int = 0
    bounds: Optional[Tuple[float, float, float, float]] = None
    highway_types: Counter = field(default_factory=Counter)
    relation_types: Counter = field(default_factory=Counter)
    regulatory_subtypes: Counter = field(default_factory=Counter)
    way_ids: set = field(default_factory=set)
    node_ids: set = field(default_factory=set)
    relation_ids: set = field(default_factory=set)
    lanelet_ids: set = field(default_factory=set)
    node_xy: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    node_lonlat: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    way_nodes: Dict[str, List[str]] = field(default_factory=dict)
    way_tags: Dict[str, Dict[str, str]] = field(default_factory=dict)
    highway_details: List[Dict[str, object]] = field(default_factory=list)
    area_details: List[Dict[str, object]] = field(default_factory=list)
    traffic_control_details: List[Dict[str, object]] = field(default_factory=list)
    lanelet_details: List[Dict[str, object]] = field(default_factory=list)
    invalid_lanelet_details: List[Dict[str, object]] = field(default_factory=list)
    lanelet_adjacency_edges: int = 0
    lanelet_topology_edges: set = field(default_factory=set)
    lanelet_topology_edge_details: List[Dict[str, object]] = field(default_factory=list)
    highway_topology_edges: set = field(default_factory=set)
    highway_topology_edge_details: List[Dict[str, object]] = field(default_factory=list)
    opendrive_mapped_lanelets: int = 0
    opendrive_lane_mapping_keys: set = field(default_factory=set)
    source_lanelet_mapped_highway_ways: int = 0
    source_lanelet_mapping_keys: set = field(default_factory=set)
    source_lanelet_regulatory_keys: set = field(default_factory=set)
    source_multipolygon_mapping_keys: set = field(default_factory=set)
    source_osm_semantic_keys: set = field(default_factory=set)
    stop_line_ways: int = 0
    traffic_light_relations: int = 0
    right_of_way_relations: int = 0
    speed_limit_relations: int = 0
    turn_restriction_relations: int = 0
    mapped_turn_restrictions: int = 0
    partial_turn_restrictions: int = 0
    regulatory_details: List[Dict[str, object]] = field(default_factory=list)
    turn_restriction_details: List[Dict[str, object]] = field(default_factory=list)
    stop_line_details: List[Dict[str, object]] = field(default_factory=list)
    highway_intersection_details: List[Dict[str, object]] = field(default_factory=list)
    crosswalk_details: List[Dict[str, object]] = field(default_factory=list)
    virtual_boundary_lanelets: List[Dict[str, object]] = field(default_factory=list)


@dataclass
class OpenDriveStats:
    roads: int = 0
    junctions: int = 0
    driving_lanes: int = 0
    road_links: int = 0
    lane_links: int = 0
    signals: int = 0
    objects: int = 0
    controllers: int = 0
    duplicate_road_ids: int = 0
    duplicate_junction_ids: int = 0
    missing_road_refs: int = 0
    missing_lane_links: int = 0
    invalid_road_lengths: int = 0
    zero_length_geometries: int = 0
    total_road_length: float = 0.0
    signal_types: Counter = field(default_factory=Counter)
    object_types: Counter = field(default_factory=Counter)
    driving_lane_details: List[Dict[str, object]] = field(default_factory=list)
    road_link_details: List[Dict[str, object]] = field(default_factory=list)
    lane_link_details: List[Dict[str, object]] = field(default_factory=list)
    invalid_lane_link_details: List[Dict[str, object]] = field(default_factory=list)
    driving_lane_keys: set = field(default_factory=set)
    road_lane_section_ranges: Dict[str, List[Tuple[str, float, float]]] = field(default_factory=dict)
    topology_edges: set = field(default_factory=set)
    topology_edge_details: List[Dict[str, object]] = field(default_factory=list)
    signal_details: List[Dict[str, object]] = field(default_factory=list)
    object_details: List[Dict[str, object]] = field(default_factory=list)
    signal_categories: Counter = field(default_factory=Counter)


def diagnose_stage2(
    conversion: str,
    source_path: str | Path,
    target_path: str | Path,
    diagnostics_dir: str | Path,
    stage1_payload: Optional[Dict[str, object]] = None,
) -> Tuple[Path, Path, Dict[str, object]]:
    source_path = Path(source_path)
    target_path = Path(target_path)
    diagnostics_dir = Path(diagnostics_dir)
    diagnostics_dir.mkdir(parents=True, exist_ok=True)
    started = time.time()

    checks: List[DiagnosticCheck] = []
    source_root = _parse_xml(source_path, "source", checks)
    target_root = _parse_xml(target_path, "target", checks)

    if source_root is not None and target_root is not None:
        if conversion == "opendrive_to_lanelet2":
            checks.extend(_diagnose_opendrive_to_lanelet2(source_root, target_root))
        elif conversion == "osm_to_lanelet2":
            checks.extend(_diagnose_osm_to_lanelet2(source_root, target_root))
        elif conversion == "lanelet2_to_osm":
            checks.extend(_diagnose_lanelet2_to_osm(source_root, target_root))
        else:
            checks.append(
                DiagnosticCheck(
                    stage="diagnosis",
                    category="unsupported-conversion",
                    status="SKIP",
                    summary=f"No Stage2 diagnostic profile is registered for {conversion}.",
                )
            )
    else:
        checks.append(
            DiagnosticCheck(
                stage="semantic",
                category="overall",
                status="SKIP",
                summary="Stage2 skipped because source or target XML could not be parsed.",
            )
        )

    checks = _filter_enabled_diagnostics(checks)
    counts = _count_statuses(checks)
    result = _result_from_counts(counts)
    payload = {
        "schema_version": "1.0",
        "stage": "stage1_and_stage2",
        "conversion": conversion,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": str(source_path),
        "target": str(target_path),
        "elapsed_seconds": round(time.time() - started, 3),
        "result": result,
        "summary": counts,
        "acceptance": (stage1_payload or {}).get("acceptance", []),
        "acceptance_summary": (stage1_payload or {}).get("summary", {}),
        "diagnostics": {
            "target_conformance": [
                asdict(check)
                for check in checks
                if check.stage == "diagnosis"
            ],
            "topology": [
                asdict(check)
                for check in checks
                if check.stage == "topology"
            ],
            "element_mapping": [
                asdict(check)
                for check in checks
                if check.stage == "semantic"
            ],
        },
    }

    txt_path = diagnostics_dir / f"{target_path.stem}_diagnostics.txt"
    json_path = diagnostics_dir / f"{target_path.stem}_diagnostics.json"
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    _write_text_report(txt_path, payload)
    return txt_path, json_path, payload


def _filter_enabled_diagnostics(checks: Sequence[DiagnosticCheck]) -> List[DiagnosticCheck]:
    """Keep only diagnostic groups enabled by the active public YAML profile."""

    settings = current_runtime_settings()
    enabled_by_stage = {
        "diagnosis": settings.diagnose_target_conformance,
        "topology": settings.diagnose_topology,
        "semantic": settings.diagnose_source_element_mapping,
    }
    filtered: List[DiagnosticCheck] = []
    for check in checks:
        enabled = enabled_by_stage.get(check.stage, True)
        if enabled:
            filtered.append(check)
        else:
            filtered.append(
                DiagnosticCheck(
                    stage=check.stage,
                    category=check.category,
                    status="SKIP",
                    summary="Disabled by the active diagnostics YAML profile.",
                    method=check.method,
                )
            )
    return filtered


def _diagnose_opendrive_to_lanelet2(
    source_root: etree._Element, target_root: etree._Element
) -> List[DiagnosticCheck]:
    checks: List[DiagnosticCheck] = []
    source = _collect_opendrive_stats(source_root)
    target = _collect_osm_stats(target_root)

    checks.append(_opendrive_structure_check(source))
    checks.append(_lanelet2_structure_check(target))
    checks.append(_opendrive_lane_inventory_check(source, target))
    checks.append(
        _coverage_check(
            stage="semantic",
            category="road-network",
            expected=source.driving_lanes,
            actual=target.lanelets,
            summary=(
                "OpenDRIVE driving lanes should produce a non-empty Lanelet2 lanelet network."
            ),
            source=f"roads={source.roads}, driving_lanes={source.driving_lanes}, junctions={source.junctions}",
            target=f"lanelets={target.lanelets}, ways={target.ways}, nodes={target.nodes}",
            strict_zero=True,
            warn_ratio=0.35,
            method="Compare OpenDRIVE driving lane count with generated Lanelet2 lanelet count.",
        )
    )
    checks.append(_opendrive_lane_link_integrity_check(source))
    checks.append(_lanelet2_lanelet_geometry_check(target))
    checks.append(_lanelet2_adjacency_proxy_check(source, target))
    checks.append(_opendrive_topology_preservation_check(source, target))
    checks.append(_opendrive_signal_object_mapping_check(source, target))
    checks.append(_opendrive_unknown_signal_check(source))
    checks.append(_opendrive_unmapped_object_check(source))
    return checks


def _diagnose_osm_to_lanelet2(
    source_root: etree._Element, target_root: etree._Element
) -> List[DiagnosticCheck]:
    checks: List[DiagnosticCheck] = []
    source = _collect_osm_stats(source_root)
    target = _collect_osm_stats(target_root)

    checks.append(_osm_structure_check(source, label="source-osm", strict_refs=False))
    checks.append(_lanelet2_structure_check(target))
    checks.append(
        _coverage_check(
            stage="semantic",
            category="highway-to-lanelet",
            expected=source.highway_ways,
            actual=target.lanelets,
            summary="OSM highway ways should produce a non-empty Lanelet2 lanelet network.",
            source=f"highway_ways={source.highway_ways}, highway_types={_counter(source.highway_types)}",
            target=f"lanelets={target.lanelets}, ways={target.ways}, nodes={target.nodes}",
            strict_zero=True,
            warn_ratio=0.08,
            method="Compare source OSM highway ways with generated Lanelet2 lanelets.",
        )
    )
    checks.append(_osm_highway_to_lanelet_mapping_check(source, target))
    checks.append(_osm_highway_layer_coverage_check(source, target))
    checks.append(_osm_to_lanelet2_topology_preservation_check(source, target))
    checks.append(_osm_semantic_elements_mapping_check(source, target))
    checks.append(_osm_turn_restriction_mapping_check(source, target))
    checks.append(
        _topology_reference_check(
            target,
            "lanelet2-target-references",
            "Generated Lanelet2 relation member references should all resolve.",
        )
    )
    return checks


def _diagnose_lanelet2_to_osm(
    source_root: etree._Element, target_root: etree._Element
) -> List[DiagnosticCheck]:
    checks: List[DiagnosticCheck] = []
    source = _collect_osm_stats(source_root)
    target = _collect_osm_stats(target_root)

    checks.append(_lanelet2_structure_check(source, label="source-lanelet2"))
    checks.append(_osm_structure_check(target, label="target-osm"))
    checks.append(
        _coverage_check(
            stage="semantic",
            category="lanelet-to-osm-road",
            expected=source.lanelets,
            actual=target.highway_ways,
            summary="Source Lanelet2 lanelets should produce a non-empty OSM road network.",
            source=f"lanelets={source.lanelets}, ways={source.ways}, nodes={source.nodes}",
            target=f"highway_ways={target.highway_ways}, highway_types={_counter(target.highway_types)}",
            strict_zero=True,
            warn_ratio=0.05,
            method="Compare source lanelet count with target OSM highway way count.",
        )
    )
    checks.append(_lanelet2_to_osm_lanelet_mapping_check(source, target))
    checks.append(_lanelet2_to_osm_topology_preservation_check(source, target))
    checks.append(_lanelet2_to_osm_semantic_mapping_check(source, target))
    checks.append(_lanelet2_virtual_boundary_filter_check(source, target))
    checks.append(
        _topology_reference_check(
            target,
            "osm-target-references",
            "Generated OSM way/relation references should all resolve.",
        )
    )
    return checks


def _parse_xml(
    path: Path, label: str, checks: List[DiagnosticCheck]
) -> Optional[etree._Element]:
    if not path.exists():
        checks.append(
            DiagnosticCheck(
                stage="diagnosis",
                category="xml",
                status="FAIL",
                summary=f"{label.capitalize()} file does not exist.",
                location=str(path),
            )
        )
        return None
    try:
        root = etree.parse(str(path)).getroot()
    except (OSError, etree.XMLSyntaxError) as exc:
        checks.append(
            DiagnosticCheck(
                stage="diagnosis",
                category="xml",
                status="FAIL",
                summary=f"{label.capitalize()} XML cannot be parsed.",
                location=str(path),
                hint=f"{type(exc).__name__}: {exc}",
            )
        )
        return None
    checks.append(
        DiagnosticCheck(
            stage="diagnosis",
            category=f"{label}-xml",
            status="PASS",
            summary=f"{label.capitalize()} XML is well formed.",
            location=str(path),
            method="lxml XML parser",
        )
    )
    return root


def _collect_osm_stats(root: etree._Element) -> OsmStats:
    stats = OsmStats()
    nodes = root.findall("node")
    ways = root.findall("way")
    relations = root.findall("relation")
    highway_node_usage: Counter = Counter()
    stats.nodes = len(nodes)
    stats.ways = len(ways)
    stats.relations = len(relations)
    stats.node_ids = {node.get("id", "") for node in nodes if node.get("id", "")}
    stats.way_ids = {way.get("id", "") for way in ways if way.get("id", "")}
    stats.relation_ids = {
        relation.get("id", "") for relation in relations if relation.get("id", "")
    }

    all_ids: List[str] = []
    min_x = min_y = math.inf
    max_x = max_y = -math.inf
    have_bounds = False
    for node in nodes:
        node_id = node.get("id", "")
        if node_id:
            stats.node_ids.add(node_id)
            all_ids.append(f"node:{node_id}")
        if "version" not in node.attrib:
            stats.missing_primitive_versions += 1
        x, y = _node_xy(node)
        if x is not None and y is not None:
            if node_id:
                stats.node_xy[node_id] = (x, y)
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)
            have_bounds = True
        lonlat = _node_lonlat(node)
        if node_id and lonlat is not None:
            stats.node_lonlat[node_id] = lonlat
        tags = _tags(node)
        _collect_osm_semantic_provenance(stats, tags)
        traffic_tags = _traffic_control_tags(tags)
        if traffic_tags:
            control_categories = _osm_control_categories(tags)
            if "traffic_light" in control_categories:
                stats.traffic_signals += 1
            if "speed_limit" in control_categories:
                stats.speed_limits += 1
            if "stop" in control_categories:
                stats.stop_controls += 1
            if "give_way" in control_categories:
                stats.give_way_controls += 1
            if tags.get("type") == "traffic_sign":
                stats.traffic_signs += 1
            stats.traffic_control_details.append(
                {
                    "element": "node",
                    "node_id": node_id,
                    "xy": _round_point((x, y)) if x is not None and y is not None else None,
                    "tags": traffic_tags,
                    "categories": sorted(control_categories),
                    "location": f"/osm/node[@id='{node_id}']",
                }
            )
    if have_bounds:
        stats.bounds = (min_x, min_y, max_x, max_y)

    for way in ways:
        way_id = way.get("id", "")
        if way_id:
            stats.way_ids.add(way_id)
            all_ids.append(f"way:{way_id}")
        if "version" not in way.attrib:
            stats.missing_primitive_versions += 1
        tags = _tags(way)
        _collect_osm_semantic_provenance(stats, tags)
        nd_refs = [nd.get("ref", "") for nd in way.findall("nd")]
        if way_id:
            stats.way_tags[way_id] = tags
        highway = tags.get("highway", "")
        if highway:
            stats.highway_ways += 1
            stats.highway_types[highway] += 1
            highway_node_usage.update(ref for ref in set(nd_refs) if ref)
        if tags.get("area") == "yes" or tags.get("building") or tags.get("landuse") or tags.get("amenity") == "parking":
            stats.area_ways += 1
            stats.area_details.append(
                {
                    "element": "way",
                    "way_id": way_id,
                    "nodes": [ref for ref in nd_refs if ref],
                    "tags": tags,
                    "location": f"/osm/way[@id='{way_id}']",
                }
            )
        control_categories = _osm_control_categories(tags)
        if "traffic_light" in control_categories:
            stats.traffic_signals += 1
        if "stop" in control_categories:
            stats.stop_controls += 1
        if "give_way" in control_categories:
            stats.give_way_controls += 1
        if tags.get("type") == "traffic_sign":
            stats.traffic_signs += 1
        if tags.get("type") == "stop_line":
            stats.stop_line_ways += 1
            stats.stop_line_details.append(
                {
                    "way_id": way_id,
                    "nodes": [ref for ref in nd_refs if ref],
                    "location": f"/osm/way[@id='{way_id}']",
                    "tags": tags,
                }
            )
        if "speed_limit" in control_categories:
            stats.speed_limits += 1
        if _is_osm_crosswalk_area(tags, nd_refs):
            stats.crosswalk_areas += 1
            stats.crosswalk_details.append(
                {
                    "element": "way",
                    "way_id": way_id,
                    "tags": tags,
                    "location": f"/osm/way[@id='{way_id}']",
                }
            )
        if way_id:
            stats.way_nodes[way_id] = [ref for ref in nd_refs if ref]
        points = [stats.node_xy[node_id] for node_id in nd_refs if node_id in stats.node_xy]
        lonlat_points = [
            stats.node_lonlat[node_id]
            for node_id in nd_refs
            if node_id in stats.node_lonlat
        ]
        if highway:
            source_lanelet_keys = _parse_csv_tag(tags.get("lanelet2:lanelet_ids", ""))
            source_regulatory_keys = _source_regulatory_keys_from_tags(tags)
            if source_lanelet_keys:
                stats.source_lanelet_mapped_highway_ways += 1
                stats.source_lanelet_mapping_keys.update(source_lanelet_keys)
            if source_regulatory_keys:
                stats.source_lanelet_regulatory_keys.update(source_regulatory_keys)
            stats.highway_details.append(
                {
                    "way_id": way_id,
                    "highway": highway,
                    "nodes": [ref for ref in nd_refs if ref],
                    "node_count": len(nd_refs),
                    "length": round(_polyline_length(points), 6),
                    "start_node": nd_refs[0] if nd_refs else "",
                    "end_node": nd_refs[-1] if nd_refs else "",
                    "start_xy": _round_point(points[0]) if points else None,
                    "end_xy": _round_point(points[-1]) if points else None,
                    "mid_xy": _round_point(_points_midpoint(points)) if points else None,
                    "mid_lonlat": (
                        _round_point(_points_midpoint(lonlat_points))
                        if lonlat_points
                        else None
                    ),
                    "source_lanelet_keys": source_lanelet_keys,
                    "source_regulatory_keys": source_regulatory_keys,
                    "source_multipolygon": tags.get("lanelet2:source_multipolygon", ""),
                    "tags": tags,
                    "location": f"/osm/way[@id='{way_id}']",
                }
            )
            if tags.get("lanelet2:source_multipolygon"):
                stats.source_multipolygon_mapping_keys.add(str(tags["lanelet2:source_multipolygon"]))
        traffic_tags = _traffic_control_tags(tags)
        if traffic_tags:
            stats.traffic_control_details.append(
                {
                    "element": "way",
                    "way_id": way_id,
                    "mid_xy": _round_point(_points_midpoint(points)) if points else None,
                    "tags": traffic_tags,
                    "categories": sorted(control_categories),
                    "source_lanelet_keys": _parse_csv_tag(tags.get("lanelet2:lanelet_ids", "")),
                    "source_regulatory_keys": _source_regulatory_keys_from_tags(tags),
                    "location": f"/osm/way[@id='{way_id}']",
                }
            )
        if not nd_refs:
            stats.zero_node_ways += 1
        for ref in nd_refs:
            if ref and ref not in stats.node_ids:
                stats.missing_refs += 1

    for relation in relations:
        relation_id = relation.get("id", "")
        if relation_id:
            stats.relation_ids.add(relation_id)
            all_ids.append(f"relation:{relation_id}")
        if "version" not in relation.attrib:
            stats.missing_primitive_versions += 1
        tags = _tags(relation)
        _collect_osm_semantic_provenance(stats, tags)
        relation_type = tags.get("type", "")
        subtype = tags.get("subtype", "")
        stats.relation_types[relation_type or "<none>"] += 1
        if relation_type == "lanelet":
            stats.lanelets += 1
            if relation_id:
                stats.lanelet_ids.add(relation_id)
            roles = Counter(member.get("role", "") for member in relation.findall("member"))
            if not roles.get("left") or not roles.get("right"):
                stats.lanelet_missing_boundaries += 1
            lanelet_detail = _lanelet_detail(relation, stats)
            stats.lanelet_details.append(lanelet_detail)
            if subtype in {"crosswalk", "ped_crossing", "pedestrian_crossing"}:
                stats.crosswalk_areas += 1
                stats.crosswalk_details.append(
                    {
                        "element": "relation",
                        "relation_id": relation_id,
                        "subtype": subtype,
                        "tags": tags,
                        "location": f"/osm/relation[@id='{relation_id}']",
                    }
                )
            if lanelet_detail.get("virtual_boundaries"):
                stats.virtual_boundary_lanelets.append(lanelet_detail)
            if lanelet_detail.get("issues"):
                stats.invalid_lanelet_details.append(lanelet_detail)
            source_key = lanelet_detail.get("source_opendrive_key")
            if source_key:
                stats.opendrive_mapped_lanelets += 1
                stats.opendrive_lane_mapping_keys.add(str(source_key))
            for source_key in lanelet_detail.get("source_opendrive_keys", []):
                stats.opendrive_lane_mapping_keys.add(str(source_key))
        elif relation_type == "regulatory_element":
            stats.regulatory_elements += 1
            stats.regulatory_subtypes[subtype or "<none>"] += 1
            if subtype == "traffic_light":
                stats.traffic_light_relations += 1
            elif subtype == "right_of_way":
                stats.right_of_way_relations += 1
            elif subtype == "speed_limit":
                stats.speed_limit_relations += 1
            detail = {
                "relation_id": relation_id,
                "subtype": subtype,
                "members": [
                    {
                        "type": member.get("type", ""),
                        "ref": member.get("ref", ""),
                        "role": member.get("role", ""),
                    }
                    for member in relation.findall("member")
                ],
                "tags": tags,
                "location": f"/osm/relation[@id='{relation_id}']",
            }
            stats.regulatory_details.append(detail)
            if tags.get("source:osm:semantic") == "turn_restriction":
                stats.turn_restriction_relations += 1
                if tags.get("source:osm:mapping_status") == "mapped":
                    stats.mapped_turn_restrictions += 1
                else:
                    stats.partial_turn_restrictions += 1
                stats.turn_restriction_details.append(detail)
        elif relation_type == "multipolygon":
            stats.multipolygons += 1
            if relation_id:
                stats.source_multipolygon_mapping_keys.add(relation_id)
            stats.area_details.append(
                {
                    "element": "relation",
                    "relation_id": relation_id,
                    "subtype": subtype,
                    "members": [
                        {
                            "type": member.get("type", ""),
                            "ref": member.get("ref", ""),
                            "role": member.get("role", ""),
                        }
                        for member in relation.findall("member")
                    ],
                    "tags": tags,
                    "location": f"/osm/relation[@id='{relation_id}']",
                }
            )
            if subtype in {"crosswalk", "ped_crossing", "pedestrian_crossing"}:
                stats.crosswalk_areas += 1
                stats.crosswalk_details.append(stats.area_details[-1])
        elif relation_type == "restriction":
            stats.turn_restrictions += 1
            stats.traffic_control_details.append(
                {
                    "element": "relation",
                    "relation_id": relation_id,
                    "subtype": subtype,
                    "members": [
                        {
                            "type": member.get("type", ""),
                            "ref": member.get("ref", ""),
                            "role": member.get("role", ""),
                        }
                        for member in relation.findall("member")
                    ],
                    "tags": tags,
                    "location": f"/osm/relation[@id='{relation_id}']",
                }
            )
        for member in relation.findall("member"):
            member_type = member.get("type", "")
            ref = member.get("ref", "")
            if not ref:
                continue
            if member_type == "node" and ref not in stats.node_ids:
                stats.missing_refs += 1
            elif member_type == "way" and ref not in stats.way_ids:
                stats.missing_refs += 1
            elif member_type == "relation" and ref not in stats.relation_ids:
                stats.missing_refs += 1

    stats.duplicate_ids = len(all_ids) - len(set(all_ids))
    for node_id, way_count in sorted(highway_node_usage.items(), key=lambda item: _safe_text_sort_key(item[0])):
        if way_count < 2:
            continue
        stats.highway_intersections += 1
        stats.highway_intersection_details.append(
            {
                "node_id": node_id,
                "highway_way_count": way_count,
                "xy": _round_point(stats.node_xy[node_id]) if node_id in stats.node_xy else None,
                "location": f"/osm/node[@id='{node_id}']",
            }
        )
    stats.lanelet_adjacency_edges = _count_lanelet_adjacency_edges(stats.lanelet_details)
    stats.lanelet_topology_edges, stats.lanelet_topology_edge_details = _lanelet_source_edges(
        stats.lanelet_details
    )
    stats.highway_topology_edges, stats.highway_topology_edge_details = _highway_source_edges(
        stats.highway_details
    )
    return stats


def _collect_opendrive_stats(root: etree._Element) -> OpenDriveStats:
    stats = OpenDriveStats()
    roads = _children_by_local_name(root, "road")
    junctions = _children_by_local_name(root, "junction")
    controllers = _children_by_local_name(root, "controller")
    stats.roads = len(roads)
    stats.junctions = len(junctions)
    stats.controllers = len(controllers)
    road_ids = [road.get("id", "") for road in roads]
    junction_ids = [junction.get("id", "") for junction in junctions]
    stats.duplicate_road_ids = len(road_ids) - len(set(road_ids))
    stats.duplicate_junction_ids = len(junction_ids) - len(set(junction_ids))
    road_id_set = set(road_ids)
    road_lane_ids: Dict[str, set] = {}
    road_lane_keys: Dict[Tuple[str, str], List[str]] = {}

    for road in roads:
        road_id = road.get("id", "")
        road_length = _float(road.get("length"))
        if road_length is None or road_length <= 0.0:
            stats.invalid_road_lengths += 1
        else:
            stats.total_road_length += road_length
        for link in _xpath_local(road, "./*[local-name()='link']/*"):
            if link.get("elementType") == "road":
                stats.road_links += 1
                stats.road_link_details.append(
                    {
                        "road_id": road_id,
                        "kind": _local_name(link),
                        "element_type": link.get("elementType", ""),
                        "element_id": link.get("elementId", ""),
                        "contact_point": link.get("contactPoint", ""),
                        "location": f"/OpenDRIVE/road[@id='{road_id}']/link/{_local_name(link)}",
                    }
                )
                if link.get("elementId", "") not in road_id_set:
                    stats.missing_road_refs += 1
        for geometry in _xpath_local(road, "./*[local-name()='planView']/*[local-name()='geometry']"):
            geometry_length = _float(geometry.get("length"))
            if geometry_length is None or geometry_length <= 0.0:
                stats.zero_length_geometries += 1
        lane_sections = _xpath_local(road, "./*[local-name()='lanes']/*[local-name()='laneSection']")
        section_starts = [_float(section.get("s")) or 0.0 for section in lane_sections]
        for section_index, lane_section in enumerate(lane_sections):
            section_s = lane_section.get("s", "")
            section_start = section_starts[section_index]
            if section_index + 1 < len(section_starts):
                section_end = section_starts[section_index + 1]
            else:
                section_end = road_length if road_length is not None else section_start
            source_lane_section = str(section_index)
            if road_id:
                stats.road_lane_section_ranges.setdefault(road_id, []).append(
                    (source_lane_section, section_start, section_end)
                )
            for side in ("left", "center", "right"):
                for lane in _xpath_local(lane_section, f"./*[local-name()='{side}']/*[local-name()='lane']"):
                    lane_id = lane.get("id", "")
                    if road_id:
                        road_lane_ids.setdefault(road_id, set()).add(lane_id)
                    if lane.get("type") != "driving":
                        continue
                    stats.driving_lanes += 1
                    source_key = f"{road_id}|{source_lane_section}|{lane_id}"
                    stats.driving_lane_keys.add(source_key)
                    road_lane_keys.setdefault((road_id, lane_id), []).append(source_key)
                    lane_link_children = _xpath_local(lane, "./*[local-name()='link']/*")
                    stats.lane_links += len(lane_link_children)
                    stats.driving_lane_details.append(
                        {
                            "road_id": road_id,
                            "lane_id": lane_id,
                            "source_key": source_key,
                            "side": side,
                            "source_lane_section": source_lane_section,
                            "section_s": section_s,
                            "road_junction": road.get("junction", "-1"),
                            "road_length": road.get("length", ""),
                            "lane_link_count": len(lane_link_children),
                            "location": (
                                f"/OpenDRIVE/road[@id='{road_id}']/lanes/"
                                f"laneSection[@s='{section_s}']/{side}/lane[@id='{lane_id}']"
                            ),
                        }
                    )
                    if not _xpath_local(lane, "./*[local-name()='link']") and road.get("junction", "-1") != "-1":
                        stats.missing_lane_links += 1
        for lane in _xpath_local(road, ".//*[local-name()='lane']"):
            if lane.get("type") == "driving":
                continue
        for signal in _xpath_local(road, "./*[local-name()='signals']/*[local-name()='signal']"):
            stats.signals += 1
            signal_type = signal.get("type", "<none>")
            category = _opendrive_signal_category(signal)
            user_data = _opendrive_user_data(signal)
            stats.signal_types[signal_type] += 1
            stats.signal_categories[category] += 1
            stats.signal_details.append(
                {
                    "road_id": road_id,
                    "signal_id": signal.get("id", ""),
                    "name": signal.get("name", ""),
                    "dynamic": signal.get("dynamic", ""),
                    "type": signal.get("type", ""),
                    "subtype": signal.get("subtype", ""),
                    "country": signal.get("country", ""),
                    "value": signal.get("value", ""),
                    "unit": signal.get("unit", ""),
                    "s": signal.get("s", ""),
                    "t": signal.get("t", ""),
                    "category": category,
                    "mapping_status": user_data.get("lanelet2:mapping_status", ""),
                    "user_data": user_data,
                    "validity": [
                        {
                            "fromLane": validity.get("fromLane", ""),
                            "toLane": validity.get("toLane", ""),
                        }
                        for validity in signal.findall("validity")
                    ],
                    "location": (
                        f"/OpenDRIVE/road[@id='{road_id}']/signals/"
                        f"signal[@id='{signal.get('id', '')}']"
                    ),
                }
            )
        for obj in _xpath_local(road, "./*[local-name()='objects']/*[local-name()='object']"):
            stats.objects += 1
            object_type = obj.get("type", "<none>")
            user_data = _opendrive_user_data(obj)
            stats.object_types[object_type] += 1
            stats.object_details.append(
                {
                    "road_id": road_id,
                    "object_id": obj.get("id", ""),
                    "name": obj.get("name", ""),
                    "type": object_type,
                    "s": obj.get("s", ""),
                    "t": obj.get("t", ""),
                    "length": obj.get("length", ""),
                    "width": obj.get("width", ""),
                    "category": _opendrive_object_category(obj, user_data),
                    "user_data": user_data,
                    "location": (
                        f"/OpenDRIVE/road[@id='{road_id}']/objects/"
                        f"object[@id='{obj.get('id', '')}']"
                    ),
                }
            )

    for junction in junctions:
        for connection in _xpath_local(junction, "./*[local-name()='connection']"):
            incoming = connection.get("incomingRoad", "")
            connecting = connection.get("connectingRoad", "")
            if incoming and incoming not in road_id_set:
                stats.missing_road_refs += 1
            if connecting and connecting not in road_id_set:
                stats.missing_road_refs += 1
            lane_links = _xpath_local(connection, "./*[local-name()='laneLink']")
            stats.lane_links += len(lane_links)
            if not lane_links:
                stats.missing_lane_links += 1
                stats.invalid_lane_link_details.append(
                    {
                        "junction_id": junction.get("id", ""),
                        "connection_id": connection.get("id", ""),
                        "incoming_road": incoming,
                        "connecting_road": connecting,
                        "issues": ["connection has no laneLink"],
                        "location": (
                            f"/OpenDRIVE/junction[@id='{junction.get('id', '')}']/"
                            f"connection[@id='{connection.get('id', '')}']"
                        ),
                    }
                )
            for lane_link in lane_links:
                from_lane = lane_link.get("from", "")
                to_lane = lane_link.get("to", "")
                issues: List[str] = []
                if incoming and from_lane not in road_lane_ids.get(incoming, set()):
                    issues.append(f"from lane {from_lane} not found on incomingRoad {incoming}")
                if connecting and to_lane not in road_lane_ids.get(connecting, set()):
                    issues.append(f"to lane {to_lane} not found on connectingRoad {connecting}")
                detail = {
                    "junction_id": junction.get("id", ""),
                    "connection_id": connection.get("id", ""),
                    "incoming_road": incoming,
                    "connecting_road": connecting,
                    "from_lane": from_lane,
                    "to_lane": to_lane,
                    "source_key": _last_lane_key(road_lane_keys.get((incoming, from_lane), [])),
                    "target_key": _first_lane_key(road_lane_keys.get((connecting, to_lane), [])),
                    "edge_type": "junction_laneLink",
                    "issues": issues,
                    "location": (
                        f"/OpenDRIVE/junction[@id='{junction.get('id', '')}']/"
                        f"connection[@id='{connection.get('id', '')}']/laneLink"
                        f"[@from='{from_lane}'][@to='{to_lane}']"
                    ),
                }
                if _lane_drives_against_reference(from_lane):
                    detail["source_key"], detail["target_key"] = (
                        detail["target_key"],
                        detail["source_key"],
                    )
                stats.lane_link_details.append(detail)
                if detail["source_key"] and detail["target_key"]:
                    _add_topology_edge(
                        stats,
                        str(detail["source_key"]),
                        str(detail["target_key"]),
                        "junction_laneLink",
                        detail,
                    )
                if issues:
                    stats.invalid_lane_link_details.append(detail)
    _add_explicit_lane_link_topology_edges(stats, roads)
    return stats


def _children_by_local_name(root: etree._Element, name: str) -> List[etree._Element]:
    return list(root.xpath(f"./*[local-name()='{name}']"))


def _xpath_local(root: etree._Element, expression: str) -> List[etree._Element]:
    return list(root.xpath(expression))


def _local_name(element: etree._Element) -> str:
    return etree.QName(element).localname


def _opendrive_structure_check(stats: OpenDriveStats) -> DiagnosticCheck:
    failures = []
    if stats.roads <= 0:
        failures.append("no roads")
    if stats.duplicate_road_ids:
        failures.append(f"duplicate road ids={stats.duplicate_road_ids}")
    if stats.duplicate_junction_ids:
        failures.append(f"duplicate junction ids={stats.duplicate_junction_ids}")
    if stats.missing_road_refs:
        failures.append(f"missing road refs={stats.missing_road_refs}")
    if stats.invalid_road_lengths:
        failures.append(f"invalid road lengths={stats.invalid_road_lengths}")
    if stats.zero_length_geometries:
        failures.append(f"zero geometry lengths={stats.zero_length_geometries}")
    return DiagnosticCheck(
        stage="diagnosis",
        category="opendrive-structure",
        status="FAIL" if failures else "PASS",
        summary=(
            "OpenDRIVE target/source structure has blocking issues."
            if failures
            else "OpenDRIVE structure has roads, unique ids and resolvable road references."
        ),
        source=(
            f"roads={stats.roads}, junctions={stats.junctions}, driving_lanes={stats.driving_lanes}, "
            f"road_links={stats.road_links}, lane_links={stats.lane_links}"
        ),
        hint="; ".join(failures),
        details=[_opendrive_detail(stats)],
    )


def _osm_structure_check(
    stats: OsmStats, label: str = "osm", strict_refs: bool = True
) -> DiagnosticCheck:
    failures = []
    warnings = []
    if stats.nodes <= 0:
        failures.append("no nodes")
    if stats.ways <= 0:
        failures.append("no ways")
    if stats.missing_refs and strict_refs:
        failures.append(f"missing refs={stats.missing_refs}")
    elif stats.missing_refs:
        warnings.append(f"missing refs={stats.missing_refs}")
    if stats.duplicate_ids:
        failures.append(f"duplicate ids={stats.duplicate_ids}")
    if stats.missing_primitive_versions:
        warnings.append(f"missing primitive version={stats.missing_primitive_versions}")
    if stats.zero_node_ways:
        warnings.append(f"zero-node ways={stats.zero_node_ways}")
    status = "FAIL" if failures else ("WARN" if warnings else "PASS")
    return DiagnosticCheck(
        stage="diagnosis",
        category=label,
        status=status,
        summary=(
            "OSM/Lanelet2 XML structure has reference/id issues."
            if failures
            else "OSM/Lanelet2 XML structure has valid primitive references."
        ),
        source=(
            f"nodes={stats.nodes}, ways={stats.ways}, relations={stats.relations}, "
            f"highway_ways={stats.highway_ways}, lanelets={stats.lanelets}"
        ),
        hint="; ".join(failures + warnings),
        details=[_osm_detail(stats)],
    )


def _lanelet2_structure_check(stats: OsmStats, label: str = "lanelet2-target") -> DiagnosticCheck:
    failures = []
    warnings = []
    if stats.nodes <= 0 or stats.ways <= 0:
        failures.append("empty node/way layer")
    if stats.lanelets <= 0:
        failures.append("no lanelet relations")
    if stats.missing_refs:
        failures.append(f"missing refs={stats.missing_refs}")
    if stats.lanelet_missing_boundaries:
        warnings.append(f"lanelets missing left/right boundaries={stats.lanelet_missing_boundaries}")
    if stats.missing_primitive_versions:
        warnings.append(f"missing primitive version={stats.missing_primitive_versions}")
    status = "FAIL" if failures else ("WARN" if warnings else "PASS")
    return DiagnosticCheck(
        stage="diagnosis",
        category=label,
        status=status,
        summary=(
            "Lanelet2 target/source has blocking structure issues."
            if failures
            else "Lanelet2 structure has lanelet relations and valid references."
        ),
        source=(
            f"nodes={stats.nodes}, ways={stats.ways}, lanelets={stats.lanelets}, "
            f"regulatory_elements={stats.regulatory_elements}, multipolygons={stats.multipolygons}"
        ),
        hint="; ".join(failures + warnings),
        details=[_osm_detail(stats)],
    )


def _topology_reference_check(stats: OsmStats, category: str, summary: str) -> DiagnosticCheck:
    return DiagnosticCheck(
        stage="topology",
        category=category,
        status="FAIL" if stats.missing_refs else "PASS",
        summary=summary if not stats.missing_refs else "Generated target has unresolved OSM/Lanelet2 references.",
        target=f"missing_refs={stats.missing_refs}, lanelets={stats.lanelets}, ways={stats.ways}",
        method="Resolve every way nd ref and relation member ref against generated node/way/relation ids.",
    )


def _opendrive_lane_inventory_check(source: OpenDriveStats, target: OsmStats) -> DiagnosticCheck:
    expected = source.driving_lanes
    actual = target.lanelets
    source_keys = set(source.driving_lane_keys)
    target_keys = set(target.opendrive_lane_mapping_keys)
    missing_keys = sorted(source_keys - target_keys)
    extra_keys = sorted(target_keys - source_keys)
    if expected > 0 and actual <= 0:
        status = "FAIL"
        summary = "No target Lanelet2 lanelet was generated from OpenDRIVE driving lanes."
        hint = "Every source driving lane should be inspected; target lanelet layer is empty."
    elif source_keys and not target_keys:
        status = "WARN"
        summary = "Generated Lanelet2 lanelets do not carry OpenDRIVE source provenance tags."
        hint = (
            "Without source:opendrive:* tags, Stage2 can only compare counts and cannot "
            "pin mapping loss to concrete road/lane ids."
        )
    elif missing_keys:
        status = "FAIL"
        summary = "Some source OpenDRIVE driving lanes are not confirmed in target Lanelet2 provenance."
        hint = "Inspect missing_source_lane_keys and the corresponding OpenDRIVE lane locations."
    elif extra_keys:
        status = "WARN"
        summary = "Target Lanelet2 contains OpenDRIVE provenance keys that were not found in source lanes."
        hint = "This usually means stale tags or a lane-section id mismatch in provenance metadata."
    elif expected > 0 and actual < max(1, int(expected * 0.35)):
        status = "WARN"
        summary = "Generated Lanelet2 lanelet count is much lower than source OpenDRIVE driving lane count."
        hint = "OpenDRIVE roads can split/merge during conversion, but this ratio is low enough for review."
    else:
        status = "PASS"
        summary = "OpenDRIVE driving lanes produced a target Lanelet2 lanelet inventory."
        hint = ""
    return DiagnosticCheck(
        stage="semantic",
        category="road-lane-to-lanelet-inventory",
        status=status,
        summary=summary,
        source=f"driving_lanes={expected}, roads={source.roads}, source_keys={len(source_keys)}",
        target=(
            f"lanelets={actual}, mapped_lanelets={target.opendrive_mapped_lanelets}, "
            f"mapped_keys={len(target_keys)}, nodes={target.nodes}, ways={target.ways}"
        ),
        method=(
            "Compare each source OpenDRIVE driving lane key road|laneSectionIndex|laneId "
            "with source:opendrive:* tags written on generated Lanelet2 lanelet relations. "
            "One target lanelet may carry several detailed source lane locations after merges."
        ),
        hint=hint,
        details=[
            {
                "source_driving_lanes": source.driving_lane_details[:300],
                "target_lanelets": target.lanelet_details[:300],
                "missing_source_lane_keys": missing_keys[:300],
                "extra_target_lane_keys": extra_keys[:300],
                "expected": expected,
                "actual": actual,
                "mapped_lanelets": target.opendrive_mapped_lanelets,
                "mapped_keys": len(target_keys),
            }
        ],
    )


def _opendrive_lane_link_integrity_check(source: OpenDriveStats) -> DiagnosticCheck:
    invalid = source.invalid_lane_link_details
    status = "FAIL" if invalid else "PASS"
    return DiagnosticCheck(
        stage="topology",
        category="opendrive-junction-laneLink-integrity",
        status=status,
        summary=(
            "OpenDRIVE junction laneLink entries contain unresolved road/lane references."
            if invalid
            else "Every OpenDRIVE junction laneLink references existing incoming/connecting road lanes."
        ),
        source=(
            f"junctions={source.junctions}, lane_links={source.lane_links}, "
            f"invalid_lane_links={len(invalid)}, missing_road_refs={source.missing_road_refs}"
        ),
        method=(
            "For every junction/connection/laneLink, resolve incomingRoad/from lane and "
            "connectingRoad/to lane against concrete OpenDRIVE road/lane ids."
        ),
        hint="; ".join(
            f"{item.get('location')}: {', '.join(item.get('issues', []))}"
            for item in invalid[:8]
        ),
        details=invalid[:500],
    )


def _lanelet2_lanelet_geometry_check(target: OsmStats) -> DiagnosticCheck:
    invalid = target.invalid_lanelet_details
    status = "FAIL" if invalid else "PASS"
    return DiagnosticCheck(
        stage="diagnosis",
        category="target-lanelet-geometry",
        status=status,
        summary=(
            "Generated Lanelet2 contains lanelets with invalid boundary geometry."
            if invalid
            else "Every generated Lanelet2 lanelet has usable left/right boundary geometry."
        ),
        target=f"lanelets={target.lanelets}, invalid_lanelets={len(invalid)}",
        method=(
            "Inspect each Lanelet2 relation with type=lanelet: left/right member existence, "
            "boundary way node count, node coordinate availability and approximate polygon area."
        ),
        hint="; ".join(
            f"lanelet {item.get('lanelet_id')}: {', '.join(item.get('issues', []))}"
            for item in invalid[:8]
        ),
        details=invalid[:500],
    )


def _lanelet2_adjacency_proxy_check(source: OpenDriveStats, target: OsmStats) -> DiagnosticCheck:
    expected_connections = source.road_links + len(source.lane_link_details)
    actual_edges = target.lanelet_adjacency_edges
    if target.lanelets <= 1 or expected_connections <= 0:
        status = "PASS"
        hint = ""
    elif actual_edges <= 0:
        status = "WARN"
        hint = "Target lanelets do not share start/end boundary endpoints; possible disconnected output."
    else:
        status = "PASS"
        hint = ""
    return DiagnosticCheck(
        stage="topology",
        category="target-lanelet-adjacency-proxy",
        status=status,
        summary=(
            "Generated Lanelet2 lanelets expose endpoint adjacency compatible with a connected network."
            if status == "PASS"
            else "Generated Lanelet2 lanelets do not expose endpoint adjacency; topology should be reviewed."
        ),
        source=(
            f"opendrive_road_links={source.road_links}, "
            f"opendrive_junction_laneLinks={len(source.lane_link_details)}"
        ),
        target=f"lanelets={target.lanelets}, endpoint_adjacency_edges={actual_edges}",
        method=(
            "Build a target-side lanelet adjacency proxy by matching each lanelet end boundary "
            "node pair to another lanelet start boundary node pair. This locates target lanelet "
            "connectivity issues without assuming a one-to-one road/lane mapping."
        ),
        hint=hint,
        details=[
            {
                "expected_source_connections": expected_connections,
                "target_endpoint_adjacency_edges": actual_edges,
                "sample_target_lanelets": target.lanelet_details[:80],
            }
        ],
    )


def _add_explicit_lane_link_topology_edges(
    stats: OpenDriveStats,
    roads: Sequence[etree._Element],
) -> None:
    """Build directed lane edges without guessing same-id road connections."""
    road_by_id = {road.get("id", ""): road for road in roads}
    lane_keys: Dict[Tuple[str, int, str], str] = {}
    road_sections: Dict[str, List[etree._Element]] = {}

    for road in roads:
        road_id = road.get("id", "")
        sections = _xpath_local(
            road, "./*[local-name()='lanes']/*[local-name()='laneSection']"
        )
        road_sections[road_id] = sections
        for section_index, section in enumerate(sections):
            for lane in _xpath_local(section, ".//*[local-name()='lane']"):
                if lane.get("type") == "driving":
                    lane_id = lane.get("id", "")
                    lane_keys[(road_id, section_index, lane_id)] = (
                        f"{road_id}|{section_index}|{lane_id}"
                    )

    for road in roads:
        road_id = road.get("id", "")
        sections = road_sections.get(road_id, [])
        road_links = {
            _local_name(link): link
            for link in _xpath_local(road, "./*[local-name()='link']/*")
        }
        for section_index, section in enumerate(sections):
            for lane in _xpath_local(section, ".//*[local-name()='lane']"):
                if lane.get("type") != "driving":
                    continue
                lane_id = lane.get("id", "")
                current_key = lane_keys.get((road_id, section_index, lane_id), "")
                for lane_link in _xpath_local(
                    lane, "./*[local-name()='link']/*"
                ):
                    kind = _local_name(lane_link)
                    linked_lane_id = lane_link.get("id", "")
                    linked_key = ""
                    linked_road_id = road_id
                    linked_section_index = section_index

                    if kind == "successor" and section_index + 1 < len(sections):
                        linked_section_index = section_index + 1
                        linked_key = lane_keys.get(
                            (road_id, linked_section_index, linked_lane_id), ""
                        )
                    elif kind == "predecessor" and section_index > 0:
                        linked_section_index = section_index - 1
                        linked_key = lane_keys.get(
                            (road_id, linked_section_index, linked_lane_id), ""
                        )
                    else:
                        road_link = road_links.get(kind)
                        if road_link is None or road_link.get("elementType") != "road":
                            continue
                        linked_road_id = road_link.get("elementId", "")
                        linked_sections = road_sections.get(linked_road_id, [])
                        if not linked_sections or linked_road_id not in road_by_id:
                            continue
                        linked_section_index = (
                            len(linked_sections) - 1
                            if road_link.get("contactPoint") == "end"
                            else 0
                        )
                        linked_key = lane_keys.get(
                            (
                                linked_road_id,
                                linked_section_index,
                                linked_lane_id,
                            ),
                            "",
                        )

                    if not current_key or not linked_key:
                        continue
                    if kind == "successor":
                        source_key, target_key = current_key, linked_key
                    elif kind == "predecessor":
                        source_key, target_key = linked_key, current_key
                    else:
                        continue
                    if _lane_drives_against_reference(lane_id):
                        source_key, target_key = target_key, source_key
                    _add_topology_edge(
                        stats,
                        source_key,
                        target_key,
                        "lane_link",
                        {
                            "road_id": road_id,
                            "linked_road": linked_road_id,
                            "kind": kind,
                            "lane_id": lane_id,
                            "linked_lane_id": linked_lane_id,
                            "source_key": source_key,
                            "target_key": target_key,
                            "location": (
                                f"/OpenDRIVE/road[@id='{road_id}']/lanes/"
                                f"laneSection[{section_index + 1}]//lane[@id='{lane_id}']/"
                                f"link/{kind}"
                            ),
                        },
                    )


def _add_topology_edge(
    stats: OpenDriveStats,
    source_key: str,
    target_key: str,
    edge_type: str,
    detail: Dict[str, object],
) -> None:
    edge_key = f"{source_key}->{target_key}"
    if edge_key in stats.topology_edges:
        return
    stats.topology_edges.add(edge_key)
    edge_detail = dict(detail)
    edge_detail["edge"] = edge_key
    edge_detail["edge_type"] = edge_type
    stats.topology_edge_details.append(edge_detail)


def _target_source_key_adjacency_edges(
    lanelet_details: Sequence[Dict[str, object]], tolerance_m: float = 0.5
) -> set:
    starts: Dict[str, List[Dict[str, object]]] = {}
    spatial_starts: Dict[Tuple[int, int], List[Dict[str, object]]] = {}
    for detail in lanelet_details:
        start_key = str(detail.get("start_key", ""))
        if start_key:
            starts.setdefault(start_key, []).append(detail)
        start_point = _tuple_point(detail.get("center_start_xy"))
        if start_point is not None:
            bucket = _spatial_bucket(start_point, tolerance_m)
            spatial_starts.setdefault(bucket, []).append(detail)

    edges = set()
    for detail in lanelet_details:
        from_source_keys = [
            str(key) for key in detail.get("source_opendrive_keys", []) if key
        ]
        # A generated Lanelet2 lanelet may merge several successive OpenDRIVE
        # lane sections. Their source edges are preserved inside this one target
        # primitive even though there is no target relation boundary between them.
        for from_key in from_source_keys:
            for to_key in from_source_keys:
                if from_key != to_key:
                    edges.add(f"{from_key}->{to_key}")

        end_key = str(detail.get("end_key", ""))
        end_point = _tuple_point(detail.get("center_end_xy"))
        if not end_key and end_point is None:
            continue
        if not from_source_keys:
            continue

        candidates: Dict[str, Dict[str, object]] = {}
        for next_detail in starts.get(end_key, []):
            candidates[str(next_detail.get("lanelet_id", id(next_detail)))] = next_detail
        if end_point is not None:
            end_bucket = _spatial_bucket(end_point, tolerance_m)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for next_detail in spatial_starts.get(
                        (end_bucket[0] + dx, end_bucket[1] + dy), []
                    ):
                        start_point = _tuple_point(
                            next_detail.get("center_start_xy")
                        )
                        if (
                            start_point is not None
                            and math.hypot(
                                end_point[0] - start_point[0],
                                end_point[1] - start_point[1],
                            )
                            <= tolerance_m
                        ):
                            candidates[
                                str(next_detail.get("lanelet_id", id(next_detail)))
                            ] = next_detail

        for next_detail in candidates.values():
            to_source_keys = [
                str(key)
                for key in next_detail.get("source_opendrive_keys", [])
                if key
            ]
            for from_key in from_source_keys:
                for to_key in to_source_keys:
                    if from_key != to_key:
                        edges.add(f"{from_key}->{to_key}")
    return edges


def _spatial_bucket(
    point: Tuple[float, float], tolerance: float
) -> Tuple[int, int]:
    return (
        math.floor(point[0] / tolerance),
        math.floor(point[1] / tolerance),
    )


def _lane_drives_against_reference(lane_id: str) -> bool:
    try:
        return int(lane_id) > 0
    except (TypeError, ValueError):
        return False


def _match_highways_to_lanelets(
    source: OsmStats, target: OsmStats, threshold_m: float = 25.0
) -> Tuple[List[Dict[str, object]], List[Dict[str, object]]]:
    matches: List[Dict[str, object]] = []
    unmatched: List[Dict[str, object]] = []
    target_lanelets = [
        detail
        for detail in target.lanelet_details
        if detail.get("center_mid_xy") and not detail.get("issues")
    ]
    for highway in source.highway_details:
        source_mid_lonlat = _tuple_point(highway.get("mid_lonlat"))
        source_mid = source_mid_lonlat or _tuple_point(highway.get("mid_xy"))
        if source_mid is None:
            unmatched.append({"source_highway": highway, "reason": "missing source geometry"})
            continue
        best: Optional[Tuple[float, Dict[str, object]]] = None
        for lanelet in target_lanelets:
            target_mid = (
                _tuple_point(lanelet.get("center_mid_lonlat"))
                if source_mid_lonlat is not None
                else None
            ) or _tuple_point(lanelet.get("center_mid_xy"))
            if target_mid is None:
                continue
            score = _point_distance_m(source_mid, target_mid)
            if best is None or score < best[0]:
                best = (score, lanelet)
        if best is not None and best[0] <= threshold_m:
            matches.append(
                {
                    "source_way_id": highway.get("way_id", ""),
                    "source_highway": highway.get("highway", ""),
                    "target_lanelet_id": best[1].get("lanelet_id", ""),
                    "distance_m": round(best[0], 3),
                    "mapping_status": "geometry_match",
                    "source_location": highway.get("location", ""),
                    "target_location": best[1].get("location", ""),
                }
            )
        else:
            unmatched.append(
                {
                    "source_highway": highway,
                    "nearest_distance_m": round(best[0], 3) if best is not None else None,
                    "nearest_lanelet": best[1].get("lanelet_id", "") if best is not None else "",
                    "reason": "nearest lanelet exceeds threshold",
                }
            )
    return matches, unmatched


def _lanelet_source_edges(
    lanelet_details: Sequence[Dict[str, object]]
) -> Tuple[set, List[Dict[str, object]]]:
    starts: Dict[str, List[Dict[str, object]]] = {}
    for detail in lanelet_details:
        start_key = str(detail.get("start_key", ""))
        if start_key:
            starts.setdefault(start_key, []).append(detail)
    edges = set()
    details: List[Dict[str, object]] = []
    for detail in lanelet_details:
        end_key = str(detail.get("end_key", ""))
        from_id = str(detail.get("lanelet_id", ""))
        if not end_key or not from_id:
            continue
        for next_detail in starts.get(end_key, []):
            to_id = str(next_detail.get("lanelet_id", ""))
            if not to_id or from_id == to_id:
                continue
            edge = f"{from_id}->{to_id}"
            if edge in edges:
                continue
            edges.add(edge)
            details.append(
                {
                    "edge": edge,
                    "from_lanelet": from_id,
                    "to_lanelet": to_id,
                    "from_location": detail.get("location", ""),
                    "to_location": next_detail.get("location", ""),
                }
            )
    return edges, details


def _highway_source_edges(
    highway_details: Sequence[Dict[str, object]]
) -> Tuple[set, List[Dict[str, object]]]:
    starts: Dict[str, List[Dict[str, object]]] = {}
    for detail in highway_details:
        start_node = str(detail.get("start_node", ""))
        if start_node:
            starts.setdefault(start_node, []).append(detail)
    edges = set()
    details: List[Dict[str, object]] = []
    for detail in highway_details:
        end_node = str(detail.get("end_node", ""))
        from_way = str(detail.get("way_id", ""))
        if not end_node or not from_way:
            continue
        for next_detail in starts.get(end_node, []):
            to_way = str(next_detail.get("way_id", ""))
            if not to_way or from_way == to_way:
                continue
            source_from_keys = [str(key) for key in detail.get("source_lanelet_keys", []) if key]
            source_to_keys = [str(key) for key in next_detail.get("source_lanelet_keys", []) if key]
            if source_from_keys and source_to_keys:
                for from_key in source_from_keys:
                    for to_key in source_to_keys:
                        if from_key == to_key:
                            continue
                        edge = f"{from_key}->{to_key}"
                        if edge not in edges:
                            edges.add(edge)
                            details.append(
                                {
                                    "edge": edge,
                                    "from_way_id": from_way,
                                    "to_way_id": to_way,
                                    "from_source_lanelet": from_key,
                                    "to_source_lanelet": to_key,
                                    "mapping_status": "source_lanelet_tags",
                                }
                            )
            else:
                edge = f"{from_way}->{to_way}"
                if edge not in edges:
                    edges.add(edge)
                    details.append(
                        {
                            "edge": edge,
                            "from_way_id": from_way,
                            "to_way_id": to_way,
                            "mapping_status": "way_endpoint_adjacency",
                        }
                    )
    return edges, details


def _first_lane_key(keys: Sequence[str]) -> str:
    return keys[0] if keys else ""


def _last_lane_key(keys: Sequence[str]) -> str:
    return keys[-1] if keys else ""


def _lane_id_sort_key(value: str) -> Tuple[int, str]:
    try:
        return (int(value), value)
    except ValueError:
        return (0, value)


def _opendrive_signal_category(signal: etree._Element) -> str:
    dynamic = signal.get("dynamic", "")
    signal_type = signal.get("type", "")
    signal_value = signal.get("value", "")
    semantic_values = {
        child.get("value", "").lower()
        for child in signal.findall("userData")
        if child.get("code") in {"lanelet2:semantic", "lanelet2:tag:subtype"}
    }
    user_data = _opendrive_user_data(signal)

    def has_meaningful_numeric_value() -> bool:
        try:
            return float(signal_value) >= 0.0
        except (TypeError, ValueError):
            return False

    if dynamic == "yes":
        return "traffic_light"
    if signal_type in {"294", "1100001"} or "stop_line" in semantic_values:
        return "stop_line"
    if signal_type in {"205", "206", "301", "306"} or "right_of_way" in semantic_values:
        return "right_of_way_or_yield"
    if signal_type == "274" or "speed_limit" in semantic_values or has_meaningful_numeric_value():
        return "speed_limit"
    if signal_type.strip().lower() in {"", "-1", "none", "unknown", "<none>"}:
        return "unknown"
    if user_data.get("lanelet2:mapping_status", "").lower() in {"unresolved", "unknown"}:
        return "unknown"
    return "traffic_sign"


def _opendrive_user_data(element: etree._Element) -> Dict[str, str]:
    return {
        child.get("code", ""): child.get("value", "")
        for child in _xpath_local(element, "./*[local-name()='userData']")
        if child.get("code")
    }


def _opendrive_object_category(
    obj: etree._Element, user_data: Optional[Dict[str, str]] = None
) -> str:
    user_data = user_data or _opendrive_user_data(obj)
    object_type = str(obj.get("type", "")).strip().lower()
    subtype = str(obj.get("subtype", "")).strip().lower()
    name = str(obj.get("name", "")).strip().lower()
    if object_type == "crosswalk":
        return "crosswalk"
    if (
        object_type in {"stopline", "stop_line"}
        or subtype in {"stopline", "stop_line"}
        or name == "stopline"
    ):
        return "stop_line"
    if user_data.get("lanelet2:source_kind") == "multipolygon":
        return "lanelet2_multipolygon"
    if object_type in {"", "-1", "none", "unknown", "<none>"}:
        return "ignored_or_unknown"
    return "generic_object"


def _opendrive_topology_preservation_check(source: OpenDriveStats, target: OsmStats) -> DiagnosticCheck:
    source_edges = set(source.topology_edges)
    target_edges = _target_source_key_adjacency_edges(target.lanelet_details)
    missing_edges = sorted(source_edges - target_edges)
    extra_edges = sorted(target_edges - source_edges)
    if source_edges and not target.opendrive_lane_mapping_keys:
        status = "WARN"
        summary = "Target Lanelet2 lacks OpenDRIVE provenance tags for lane-level topology comparison."
        hint = "Add source:opendrive:* tags to target lanelets before using edge-level topology preservation."
    elif missing_edges:
        status = "FAIL"
        summary = "Some source OpenDRIVE lane-level topology edges are not preserved in target Lanelet2."
        hint = "Inspect missing_source_edges and mapped target lanelets near each road/lane key."
    else:
        status = "PASS"
        summary = "Source OpenDRIVE lane-level topology edges are confirmed in target Lanelet2 adjacency."
        hint = ""
    return DiagnosticCheck(
        stage="topology",
        category="opendrive-to-lanelet2-topology-preservation",
        status=status,
        summary=summary,
        source=f"source_edges={len(source_edges)}, road_links={source.road_links}, laneLinks={len(source.lane_link_details)}",
        target=f"target_source_key_edges={len(target_edges)}, lanelets={target.lanelets}",
        method=(
            "Build source topology edges from OpenDRIVE junction/laneLink and direct road links, "
            "then map target Lanelet2 endpoint adjacency back through source:opendrive:* tags. "
            "Each edge is represented as road|laneSectionIndex|laneId -> road|laneSectionIndex|laneId."
        ),
        hint=hint,
        details=[
            {
                "missing_source_edges": missing_edges[:500],
                "extra_target_edges": extra_edges[:300],
                "source_edge_details": source.topology_edge_details[:500],
                "sample_target_edges": sorted(target_edges)[:300],
            }
        ],
    )


def _opendrive_signal_object_mapping_check(source: OpenDriveStats, target: OsmStats) -> DiagnosticCheck:
    expected_signals = source.signal_categories
    object_categories = Counter(
        str(item.get("category", "ignored_or_unknown"))
        for item in source.object_details
    )
    target_subtypes = target.regulatory_subtypes
    checks = [
        {
            "source_category": "traffic_light",
            "expected": expected_signals.get("traffic_light", 0),
            "target": target_subtypes.get("traffic_light", 0) + target.traffic_signals,
            "target_carrier": "Lanelet2 traffic_light regulatory elements / ways",
        },
        {
            "source_category": "speed_limit",
            "expected": expected_signals.get("speed_limit", 0),
            "target": target_subtypes.get("speed_limit", 0) + target.speed_limits,
            "target_carrier": "Lanelet2 speed_limit regulatory elements",
        },
        {
            "source_category": "right_of_way_or_yield",
            "expected": expected_signals.get("right_of_way_or_yield", 0),
            "target": target.right_of_way_relations,
            "target_carrier": "Lanelet2 right_of_way regulatory elements",
        },
        {
            "source_category": "stop_line",
            "expected": expected_signals.get("stop_line", 0),
            "target": target.stop_line_ways,
            "target_carrier": "Lanelet2 stop_line ways",
        },
        {
            "source_category": "traffic_sign",
            "expected": expected_signals.get("traffic_sign", 0),
            "target": target.traffic_signs,
            "target_carrier": "Lanelet2 traffic_sign ways",
        },
        {
            "source_category": "crosswalk_object",
            "expected": object_categories.get("crosswalk", 0),
            "target": target.crosswalk_areas,
            "target_carrier": "Lanelet2 crosswalk lanelet/multipolygon",
        },
        {
            "source_category": "stop_line_object",
            "expected": object_categories.get("stop_line", 0),
            "target": target.stop_line_ways,
            "target_carrier": "Lanelet2 stop_line ways",
        },
        {
            "source_category": "lanelet2_multipolygon_object",
            "expected": object_categories.get("lanelet2_multipolygon", 0),
            "target": target.multipolygons,
            "target_carrier": "Lanelet2 multipolygon relations",
        },
    ]
    missing = [
        item
        for item in checks
        if int(item["expected"]) > 0 and int(item["target"]) <= 0
    ]
    status = "WARN" if missing else "PASS"
    return DiagnosticCheck(
        stage="semantic",
        category="traffic-rules-and-objects",
        status=status,
        summary=(
            "Some OpenDRIVE traffic rule/object classes have no Lanelet2 semantic carrier."
            if missing
            else "OpenDRIVE traffic rule/object classes have matching Lanelet2 semantic carriers where expected."
        ),
        source=(
            f"signals={source.signals}, signal_categories={_counter(source.signal_categories)}, "
            f"objects={source.objects}, object_categories={_counter(object_categories)}"
        ),
        target=(
            f"regulatory_elements={target.regulatory_elements}, "
            f"traffic_light_relations={target.traffic_light_relations}, "
            f"right_of_way_relations={target.right_of_way_relations}, "
            f"speed_limit_relations={target.speed_limit_relations}, "
            f"stop_line_ways={target.stop_line_ways}, crosswalk_areas={target.crosswalk_areas}, "
            f"multipolygons={target.multipolygons}"
        ),
        method=(
            "Classify OpenDRIVE signal/object elements into traffic_light, speed_limit, "
            "right_of_way_or_yield, stop_line and traffic_sign; classify objects as "
            "crosswalk, stop_line, provenance multipolygon or generic object; compare convertible classes with "
            "Lanelet2 regulatory/way/multipolygon carriers."
        ),
        hint="; ".join(
            f"{item['source_category']} expected={item['expected']} target={item['target']}"
            for item in missing
        ),
        details=[
            {
                "class_mapping": checks,
                "source_signals": source.signal_details[:500],
                "source_objects": source.object_details[:300],
                "non_lanelet2_object_classes": [
                    item
                    for item in source.object_details[:500]
                    if item.get("category") in {"generic_object", "ignored_or_unknown"}
                ],
                "target_regulatory": target.regulatory_details[:500],
                "target_stop_lines": target.stop_line_details[:300],
            }
        ],
    )


def _opendrive_unknown_signal_check(source: OpenDriveStats) -> DiagnosticCheck:
    unknown = [
        item
        for item in source.signal_details
        if item.get("category") == "unknown"
        or str(item.get("mapping_status", "")).lower() in {"unresolved", "unknown"}
    ]
    return DiagnosticCheck(
        stage="semantic",
        category="unknown-opendrive-signals",
        status="WARN" if unknown else "PASS",
        summary=(
            "Some OpenDRIVE signals have no reliable type/subtype semantic mapping."
            if unknown
            else "No unresolved OpenDRIVE signal type/subtype entries were found."
        ),
        source=f"signals={source.signals}, unknown_or_unresolved={len(unknown)}",
        method=(
            "Report each signal whose type is empty/-1/unknown or whose userData "
            "lanelet2:mapping_status is unresolved. Country-specific numeric sign types "
            "remain generic traffic_sign entries instead of being guessed."
        ),
        hint="Inspect signal id, road id, country/type/subtype and userData in details." if unknown else "",
        details=unknown[:500],
    )


def _opendrive_unmapped_object_check(source: OpenDriveStats) -> DiagnosticCheck:
    unsupported = [
        item for item in source.object_details if item.get("category") == "generic_object"
    ]
    ignored = [
        item for item in source.object_details if item.get("category") == "ignored_or_unknown"
    ]
    return DiagnosticCheck(
        stage="semantic",
        category="unmapped-opendrive-objects",
        status="WARN" if unsupported else "PASS",
        summary=(
            "Some OpenDRIVE objects have no stable Lanelet2 area/line/regulatory mapping."
            if unsupported
            else "All semantically convertible OpenDRIVE object classes are covered by the current mapping."
        ),
        source=f"generic_unmapped={len(unsupported)}, ignored_none_or_unknown={len(ignored)}",
        method=(
            "Treat crosswalk, StopLine and lanelet2:source_kind=multipolygon as convertible. "
            "List remaining concrete object ids/types separately; none/-1 helper objects are listed "
            "as ignored evidence and do not cause a warning."
        ),
        hint="Review generic_unmapped_objects before deciding whether to extend the object mapping table." if unsupported else "",
        details=[
            {
                "generic_unmapped_objects": unsupported[:500],
                "ignored_none_or_unknown_objects": ignored[:500],
            }
        ],
    )


def _osm_highway_to_lanelet_mapping_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    matches, unmatched = _match_highways_to_lanelets(source, target)
    expected = len(source.highway_details)
    matched = len(matches)
    if expected and matched == 0:
        status = "FAIL"
        summary = "No source OSM highway way could be matched to a target Lanelet2 lanelet."
    elif unmatched:
        status = "WARN"
        summary = "Some source OSM highway ways are not confirmed in target Lanelet2 geometry."
    else:
        status = "PASS"
        summary = "Source OSM highway ways are confirmed in target Lanelet2 geometry."
    return DiagnosticCheck(
        stage="semantic",
        category="osm-highway-to-lanelet-mapping",
        status=status,
        summary=summary,
        source=f"source_highways={expected}, highway_types={_counter(source.highway_types)}",
        target=f"target_lanelets={target.lanelets}, matched_highways={matched}",
        method=(
            "Match every source OSM highway way to the nearest generated Lanelet2 lanelet by "
            "centerline midpoint distance. This is geometry_match provenance because "
            "the OSM->CommonRoad->Lanelet2 chain does not preserve source way ids."
        ),
        hint=(
            f"unmatched_source_highways={len(unmatched)}"
            if unmatched
            else ""
        ),
        details=[
            {
                "matches": matches[:500],
                "unmatched_source_highways": unmatched[:500],
                "match_threshold_m": 25.0,
            }
        ],
    )


def _osm_highway_layer_coverage_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    main_types = {
        "motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link",
        "secondary", "secondary_link", "tertiary", "tertiary_link", "residential",
        "living_street", "unclassified", "road", "busway",
    }
    sublayer_types = {"service", "footway", "cycleway", "path", "pedestrian", "steps"}
    matches, _ = _match_highways_to_lanelets(source, target)
    matched_ids = {str(item.get("source_way_id", "")) for item in matches}
    groups = {
        "main_layer": main_types,
        "sublayer": sublayer_types,
    }
    class_rows: List[Dict[str, object]] = []
    completely_missing: List[str] = []
    for group_name, highway_types in groups.items():
        source_items = [
            item for item in source.highway_details if item.get("highway") in highway_types
        ]
        matched_items = [
            item for item in source_items if str(item.get("way_id", "")) in matched_ids
        ]
        if source_items and not matched_items:
            completely_missing.append(group_name)
        class_rows.append(
            {
                "group": group_name,
                "source_count": len(source_items),
                "matched_count": len(matched_items),
                "coverage_ratio": round(len(matched_items) / max(len(source_items), 1), 6),
                "highway_types": sorted(highway_types),
                "missing_source_way_ids": [
                    item.get("way_id", "")
                    for item in source_items
                    if str(item.get("way_id", "")) not in matched_ids
                ][:500],
            }
        )
    unknown_items = [
        item
        for item in source.highway_details
        if item.get("highway") not in main_types | sublayer_types
    ]
    class_rows.append(
        {
            "group": "unclassified_highway_types",
            "source_count": len(unknown_items),
            "matched_count": sum(
                1 for item in unknown_items if str(item.get("way_id", "")) in matched_ids
            ),
            "source_types": dict(Counter(str(item.get("highway", "")) for item in unknown_items)),
            "source_way_ids": [item.get("way_id", "") for item in unknown_items[:500]],
        }
    )
    return DiagnosticCheck(
        stage="semantic",
        category="osm-main-sublayer-coverage",
        status="WARN" if completely_missing or unknown_items else "PASS",
        summary=(
            "Some OSM main/sublayer highway classes are absent from the generated Lanelet2 network."
            if completely_missing
            else (
                "OSM main/sublayer classes have coverage, but some highway types are not classified by the current policy."
                if unknown_items
                else "OSM main-layer and enabled sublayer highway classes have target Lanelet2 coverage."
            )
        ),
        source=(
            f"highway_ways={source.highway_ways}, intersections={source.highway_intersections}, "
            f"types={_counter(source.highway_types)}"
        ),
        target=f"lanelets={target.lanelets}, geometrically_matched_highways={len(matched_ids)}",
        method=(
            "Classify source highway ways into motor-road main layer, service/pedestrian/cycle "
            "sublayer and unclassified types; reuse per-way geometry matches to report coverage."
        ),
        hint="; ".join(
            part
            for part in [
                f"completely_missing_groups={','.join(completely_missing)}" if completely_missing else "",
                f"unclassified_source_highways={len(unknown_items)}" if unknown_items else "",
            ]
            if part
        ),
        details=[
            {
                "layer_coverage": class_rows,
                "source_intersections": source.highway_intersection_details[:500],
            }
        ],
    )


def _osm_to_lanelet2_topology_preservation_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    matches, _ = _match_highways_to_lanelets(source, target)
    matched_lanelets_by_way = {
        str(item["source_way_id"]): [str(item["target_lanelet_id"])]
        for item in matches
        if item.get("target_lanelet_id")
    }
    target_edges = set(target.lanelet_topology_edges)
    checked_edges: List[Dict[str, object]] = []
    missing: List[Dict[str, object]] = []
    for detail in source.highway_topology_edge_details:
        from_way = str(detail.get("from_way_id", ""))
        to_way = str(detail.get("to_way_id", ""))
        from_lanelets = matched_lanelets_by_way.get(from_way, [])
        to_lanelets = matched_lanelets_by_way.get(to_way, [])
        if not from_lanelets or not to_lanelets:
            continue
        candidate_edges = [f"{src}->{dst}" for src in from_lanelets for dst in to_lanelets]
        preserved = any(edge in target_edges for edge in candidate_edges)
        record = {
            "source_edge": detail.get("edge", ""),
            "candidate_target_edges": candidate_edges,
            "preserved": preserved,
            "source_detail": detail,
        }
        checked_edges.append(record)
        if not preserved:
            missing.append(record)
    if source.highway_topology_edges and not checked_edges:
        status = "WARN"
        summary = "Source OSM highway topology exists, but too few highway ways were matched to target lanelets."
        hint = "Review osm-highway-to-lanelet-mapping first."
    elif missing:
        status = "WARN"
        summary = "Some matched OSM highway adjacency edges are not confirmed in target Lanelet2 adjacency."
        hint = f"missing_matched_edges={len(missing)}"
    else:
        status = "PASS"
        summary = "Matched OSM highway adjacency edges are confirmed in target Lanelet2 adjacency."
        hint = ""
    return DiagnosticCheck(
        stage="topology",
        category="osm-to-lanelet2-topology-preservation",
        status=status,
        summary=summary,
        source=f"source_highway_edges={len(source.highway_topology_edges)}",
        target=f"target_lanelet_edges={len(target.lanelet_topology_edges)}, checked_edges={len(checked_edges)}",
        method=(
            "Build source OSM way adjacency from shared end/start nodes; after geometry matching "
            "source ways to target lanelets, verify that corresponding target lanelets are adjacent."
        ),
        hint=hint,
        details=[
            {
                "checked_edges": checked_edges[:500],
                "missing_matched_edges": missing[:500],
                "source_edge_details": source.highway_topology_edge_details[:500],
                "target_lanelet_edges": sorted(target_edges)[:500],
            }
        ],
    )


def _osm_semantic_elements_mapping_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    source_by_category = {
        category: [
            item
            for item in source.traffic_control_details
            if category in item.get("categories", [])
        ]
        for category in ("traffic_light", "speed_limit", "stop", "give_way")
    }
    source_restrictions = [
        item for item in source.traffic_control_details
        if item.get("tags", {}).get("restriction")
        or item.get("tags", {}).get("type") == "restriction"
    ]
    source_stop_lines = source.stop_line_details
    source_crosswalks = source.crosswalk_details
    expected_source_keys = {
        f"{item.get('element')}:{item.get('node_id') or item.get('way_id')}:{category}"
        for category, items in source_by_category.items()
        for item in items
        if item.get("element") and (item.get("node_id") or item.get("way_id"))
    }
    expected_source_keys.update(
        f"way:{item.get('way_id')}:stop_line"
        for item in source_stop_lines
        if item.get("way_id")
    )


    expected_source_keys.update(
        f"way:{item.get('way_id')}:crosswalk_area"
        for item in source_crosswalks
        if item.get("element") == "way" and item.get("way_id")
    )
    expected_source_keys.update(
        f"relation:{item.get('relation_id')}:turn_restriction"
        for item in source_restrictions
        if item.get("relation_id")
    )
    mapped_source_keys = set(target.source_osm_semantic_keys)
    missing_source_keys = sorted(expected_source_keys - mapped_source_keys)
    checks = [
        {
            "source_category": "traffic_signals",
            "expected": source.traffic_signals,
            "target": target.traffic_light_relations + target.traffic_signals,
            "target_carrier": "Lanelet2 traffic_light relation/way",
            "source_elements": source_by_category["traffic_light"][:300],
        },
        {
            "source_category": "speed_limits",
            "expected": source.speed_limits,
            "target": target.speed_limit_relations,
            "target_carrier": "Lanelet2 speed_limit regulatory relation",
            "source_elements": source_by_category["speed_limit"][:300],
        },
        {
            "source_category": "stop_controls",
            "expected": source.stop_controls,
            "target": target.stop_controls,
            "target_carrier": "Lanelet2 de206/stop traffic-sign way and regulatory relation",
            "source_elements": source_by_category["stop"][:300],
        },
        {
            "source_category": "give_way_controls",
            "expected": source.give_way_controls,
            "target": target.give_way_controls,
            "target_carrier": "Lanelet2 de205/give_way traffic-sign way and regulatory relation",
            "source_elements": source_by_category["give_way"][:300],
        },
        {
            "source_category": "turn_restrictions",
            "expected": source.turn_restrictions,
            "target": target.turn_restriction_relations,
            "target_carrier": "Lanelet2 traffic_sign relation with mapped from/via/to lanelet metadata",
            "source_elements": source_restrictions[:300],
        },
        {
            "source_category": "stop_lines",
            "expected": source.stop_line_ways,
            "target": target.stop_line_ways,
            "target_carrier": "Lanelet2 type=stop_line way",
            "source_elements": source_stop_lines[:300],
        },
        {
            "source_category": "crosswalk_areas",
            "expected": source.crosswalk_areas,
            "target": target.crosswalk_areas,
            "target_carrier": "Lanelet2 crosswalk multipolygon/lanelet",
            "source_elements": source_crosswalks[:300],
        },
    ]
    missing = [
        item for item in checks if int(item["expected"]) > 0 and int(item["target"]) <= 0
    ]
    underrepresented = [
        item
        for item in checks
        if int(item["expected"]) > 0
        and int(item["target"]) > 0
        and int(item["target"]) < int(item["expected"])
    ]
    status = "WARN" if missing or underrepresented or missing_source_keys else "PASS"
    return DiagnosticCheck(
        stage="semantic",
        category="osm-semantic-elements",
        status=status,
        summary=(
            "Some source OSM semantic element classes are missing or underrepresented in target Lanelet2."
            if missing or underrepresented or missing_source_keys
            else "Source OSM semantic element classes have target Lanelet2 carriers where expected."
        ),
        source=(
            f"traffic_signals={source.traffic_signals}, speed_limits={source.speed_limits}, "
            f"stop={source.stop_controls}, give_way={source.give_way_controls}, "
            f"turn_restrictions={source.turn_restrictions}, stop_lines={source.stop_line_ways}, "
            f"crosswalk_areas={source.crosswalk_areas}"
        ),
        target=(
            f"regulatory_elements={target.regulatory_elements}, traffic_light_relations={target.traffic_light_relations}, "
            f"speed_limit_relations={target.speed_limit_relations}, right_of_way_relations={target.right_of_way_relations}, "
            f"stop_controls={target.stop_controls}, give_way_controls={target.give_way_controls}, "
            f"turn_restrictions={target.turn_restriction_relations}, "
            f"mapped_turn_restrictions={target.mapped_turn_restrictions}, "
            f"partial_turn_restrictions={target.partial_turn_restrictions}, "
            f"stop_lines={target.stop_line_ways}, crosswalk_areas={target.crosswalk_areas}"
        ),
        method=(
            "Classify every source OSM traffic light, speed limit, stop, give_way, restriction, "
            "stop_line and explicit crosswalk area; compare each class with its Lanelet2 carrier. "
            "Source element ids/tags/locations are retained in class_mapping for repair."
        ),
        hint="; ".join(
            [
                *(
                    f"{item['source_category']} expected={item['expected']} target={item['target']}"
                    for item in missing + underrepresented
                ),
                *(
                    [f"missing_source_semantic_ids={len(missing_source_keys)}"]
                    if missing_source_keys
                    else []
                ),
            ]
        ),
        details=[
            {
                "class_mapping": checks,
                "missing_or_underrepresented_classes": missing + underrepresented,
                "missing_source_semantic_keys": missing_source_keys[:500],
                "mapped_source_semantic_keys": sorted(mapped_source_keys)[:500],
                "source_traffic_controls": source.traffic_control_details[:500],
                "source_areas": source.area_details[:500],
                "target_regulatory": target.regulatory_details[:500],
                "target_areas": target.area_details[:500],
            }
        ],
    )


def _osm_turn_restriction_mapping_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    source_details = [
        item
        for item in source.traffic_control_details
        if item.get("element") == "relation"
        and item.get("tags", {}).get("type") == "restriction"
    ]
    source_ids = {
        str(item.get("relation_id", ""))
        for item in source_details
        if item.get("relation_id")
    }
    target_by_source = {
        str(item.get("tags", {}).get("source:osm:id", "")): item
        for item in target.turn_restriction_details
        if item.get("tags", {}).get("source:osm:id")
    }
    missing_ids = sorted(source_ids - set(target_by_source), key=_safe_text_sort_key)
    invalid: List[Dict[str, object]] = []
    target_lanelet_ids = set(str(item) for item in target.lanelet_ids)
    target_lanelet_details = {
        str(item.get("lanelet_id", "")): item for item in target.lanelet_details
    }
    for source_id, detail in target_by_source.items():
        tags = detail.get("tags", {})
        from_ids = _parse_csv_tag(str(tags.get("source:osm:from_lanelet_ids", "")))
        via_ids = _parse_csv_tag(str(tags.get("source:osm:via_lanelet_ids", "")))
        to_ids = _parse_csv_tag(str(tags.get("source:osm:to_lanelet_ids", "")))
        unknown_ids = sorted(
            (set(from_ids) | set(via_ids) | set(to_ids)) - target_lanelet_ids,
            key=_safe_text_sort_key,
        )
        restriction = str(tags.get("source:osm:restriction", ""))
        route_tag = (
            "source_osm_only_successor_lanelets"
            if restriction.startswith("only_")
            else "source_osm_forbidden_successor_lanelets"
        )
        missing_lanelet_annotations = [
            lanelet_id
            for lanelet_id in from_ids
            if not set(to_ids).issubset(
                set(
                    _parse_csv_tag(
                        str(target_lanelet_details.get(lanelet_id, {}).get(route_tag, ""))
                    )
                )
            )
        ]
        issues = []
        if not from_ids:
            issues.append("no mapped from lanelet")
        if not to_ids:
            issues.append("no mapped to lanelet")
        if unknown_ids:
            issues.append("mapped lanelet id does not exist")
        if missing_lanelet_annotations:
            issues.append("from lanelet routing annotation is incomplete")
        if tags.get("source:osm:routing_enforcement") != "custom_consumer_required":
            issues.append("routing enforcement mode is not declared")
        if tags.get("source:osm:mapping_status") != "mapped":
            issues.append("mapping is partial")
        if issues:
            invalid.append(
                {
                    "source_relation_id": source_id,
                    "restriction": restriction,
                    "from_lanelet_ids": from_ids,
                    "via_lanelet_ids": via_ids,
                    "to_lanelet_ids": to_ids,
                    "unknown_lanelet_ids": unknown_ids,
                    "missing_lanelet_annotations": missing_lanelet_annotations,
                    "issues": issues,
                    "location": detail.get("location", ""),
                }
            )

    if not source_ids:
        status = "PASS"
        summary = "Source OSM contains no explicit from/via/to turn restrictions."
        hint = ""
    elif missing_ids or invalid:
        status = "WARN"
        summary = "Some OSM turn restrictions are missing or only partially mapped to target lanelets."
        hint = f"missing={len(missing_ids)}, partial_or_invalid={len(invalid)}"
    else:
        status = "PASS"
        summary = "OSM turn restrictions have traceable target from/via/to lanelet mappings."
        hint = "Lanelet2 RoutingGraph needs a custom consumer to enforce the stored successor constraints."
    return DiagnosticCheck(
        stage="topology",
        category="osm-turn-restriction-path-mapping",
        status=status,
        summary=summary,
        source=f"restriction_relations={len(source_ids)}",
        target=(
            f"restriction_carriers={target.turn_restriction_relations}, "
            f"mapped={target.mapped_turn_restrictions}, partial={target.partial_turn_restrictions}"
        ),
        method=(
            "Map OSM from/to ways and via node/ways to directed target lanelets using geometric "
            "overlap, endpoint direction and via proximity; verify provenance, mapped lanelet ids "
            "and forbidden/only-successor annotations on every affected from lanelet."
        ),
        hint=hint,
        details=[
            {
                "missing_source_relation_ids": missing_ids[:500],
                "partial_or_invalid_mappings": invalid[:500],
                "source_restrictions": source_details[:500],
                "target_restrictions": target.turn_restriction_details[:500],
            }
        ],
    )


def _lanelet2_to_osm_lanelet_mapping_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    source_keys = set(str(key) for key in source.lanelet_ids)
    target_keys = set(str(key) for key in target.source_lanelet_mapping_keys)
    missing = sorted(source_keys - target_keys, key=_safe_text_sort_key)
    extra = sorted(target_keys - source_keys, key=_safe_text_sort_key)
    coverage_ratio = len(target_keys & source_keys) / max(len(source_keys), 1)
    if source_keys and not target_keys:
        status = "FAIL"
        summary = "Target OSM highway ways do not carry lanelet2:lanelet_ids provenance."
        hint = "Without lanelet2:lanelet_ids, conversion preservation cannot be checked at lanelet level."
    elif missing:
        status = "WARN"
        summary = "Some source Lanelet2 lanelets are not confirmed in target OSM highway provenance."
        hint = (
            f"missing_source_lanelets={len(missing)}, "
            f"coverage_ratio={coverage_ratio:.3f}. "
            "Lanelet2->OSM is lossy, so this is a quality warning rather than a hard failure."
        )
    elif extra:
        status = "WARN"
        summary = "Target OSM contains lanelet provenance ids not found in source Lanelet2."
        hint = f"extra_target_lanelet_ids={len(extra)}"
    else:
        status = "PASS"
        summary = "Source Lanelet2 lanelets are confirmed in target OSM highway provenance."
        hint = ""
    return DiagnosticCheck(
        stage="semantic",
        category="lanelet-to-osm-highway-mapping",
        status=status,
        summary=summary,
        source=f"source_lanelets={len(source_keys)}",
        target=(
            f"target_highways={target.highway_ways}, mapped_highways={target.source_lanelet_mapped_highway_ways}, "
            f"mapped_lanelet_keys={len(target_keys)}"
        ),
        method="Compare source Lanelet2 relation ids with lanelet2:lanelet_ids written on target OSM highway ways.",
        hint=hint,
        details=[
            {
                "missing_source_lanelets": missing[:500],
                "extra_target_lanelet_ids": extra[:300],
                "coverage_ratio": round(coverage_ratio, 6),
                "interpretation": (
                    "OSM is a lower-fidelity target for lane-level maps. Missing source lanelet ids "
                    "indicate reduced traceability or lane aggregation, not necessarily an invalid OSM file."
                ),
                "target_highways": target.highway_details[:500],
            }
        ],
    )


def _lanelet2_to_osm_topology_preservation_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    source_edges = set(source.lanelet_topology_edges)
    target_edges = set(target.highway_topology_edges)
    missing = sorted(source_edges - target_edges)
    if source_edges and not target.source_lanelet_mapping_keys:
        status = "WARN"
        summary = "Target OSM lacks lanelet2:lanelet_ids tags for lanelet-level topology comparison."
        hint = "Review lanelet-to-osm-highway-mapping first."
    elif missing:
        status = "WARN"
        summary = "Some source Lanelet2 lane-level adjacency edges are not confirmed in target OSM highway topology."
        hint = (
            f"missing_source_edges={len(missing)}. "
            "This is expected when lane-level topology is collapsed to lower-fidelity OSM ways."
        )
    else:
        status = "PASS"
        summary = "Source Lanelet2 adjacency edges are confirmed in target OSM highway topology."
        hint = ""
    return DiagnosticCheck(
        stage="topology",
        category="lanelet2-to-osm-topology-preservation",
        status=status,
        summary=summary,
        source=f"source_lanelet_edges={len(source_edges)}",
        target=f"target_source_lanelet_edges={len(target_edges)}, target_highways={target.highway_ways}",
        method=(
            "Build source Lanelet2 adjacency from lanelet boundary endpoints. Build target OSM "
            "topology from shared highway end/start nodes and translate it through lanelet2:lanelet_ids."
        ),
        hint=hint,
        details=[
            {
                "missing_source_edges": missing[:500],
                "source_edge_details": source.lanelet_topology_edge_details[:500],
                "target_edge_details": target.highway_topology_edge_details[:500],
            }
        ],
    )


def _lanelet2_to_osm_semantic_mapping_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    target_reg_keys = set(target.source_lanelet_regulatory_keys)
    expected_regulatory = set(
        str(item.get("relation_id", ""))
        for item in source.regulatory_details
        if item.get("relation_id")
    )
    missing_regulatory = sorted(expected_regulatory - target_reg_keys, key=_safe_text_sort_key)
    expected_multipolygons = set(
        str(item.get("relation_id", ""))
        for item in source.area_details
        if item.get("element") == "relation" and item.get("relation_id")
    )
    target_multipolygon_keys = set(target.source_multipolygon_mapping_keys)
    missing_multipolygons = sorted(expected_multipolygons - target_multipolygon_keys, key=_safe_text_sort_key)
    if missing_regulatory or missing_multipolygons:
        status = "WARN"
        summary = "Some source Lanelet2 semantic elements are not confirmed in target OSM provenance."
    else:
        status = "PASS"
        summary = "Source Lanelet2 regulatory and area elements are represented by target OSM carriers where expected."
    return DiagnosticCheck(
        stage="semantic",
        category="lanelet2-semantic-elements-to-osm",
        status=status,
        summary=summary,
        source=(
            f"regulatory_elements={source.regulatory_elements}, multipolygons={source.multipolygons}, "
            f"regulatory_subtypes={_counter(source.regulatory_subtypes)}"
        ),
        target=(
            f"traffic_signals={target.traffic_signals}, speed_limits={target.speed_limits}, "
            f"turn_restrictions={target.turn_restrictions}, area_carriers={target.multipolygons + target.area_ways}"
        ),
        method=(
            "Compare source Lanelet2 regulatory/multipolygon ids with target OSM tags such as "
            "lanelet2:regulatory_elements, lanelet2:traffic_light_relations, lanelet2:source_multipolygon."
        ),
        hint="; ".join(
            part
            for part in [
                f"missing_regulatory={len(missing_regulatory)}" if missing_regulatory else "",
                f"missing_multipolygons={len(missing_multipolygons)}" if missing_multipolygons else "",
            ]
            if part
        ),
        details=[
            {
                "missing_regulatory": missing_regulatory[:500],
                "missing_multipolygons": missing_multipolygons[:500],
                "source_regulatory": source.regulatory_details[:500],
                "source_areas": source.area_details[:500],
                "target_highways": target.highway_details[:500],
                "target_traffic_controls": target.traffic_control_details[:500],
                "target_areas": target.area_details[:500],
            }
        ],
    )


def _lanelet2_virtual_boundary_filter_check(source: OsmStats, target: OsmStats) -> DiagnosticCheck:
    virtual_lanelets = {
        str(item.get("lanelet_id", "")): item
        for item in source.virtual_boundary_lanelets
        if item.get("lanelet_id")
    }
    mapped = set(str(item) for item in target.source_lanelet_mapping_keys)
    missing_ids = sorted(set(virtual_lanelets) - mapped, key=_safe_text_sort_key)
    if not virtual_lanelets:
        status = "PASS"
        summary = "No source lanelet uses a virtual left/right boundary."
    elif missing_ids:
        status = "WARN"
        summary = "Some source lanelets with virtual boundaries are absent from target OSM provenance."
    else:
        status = "PASS"
        summary = "Lanelets with virtual boundaries remain represented in target OSM provenance."
    return DiagnosticCheck(
        stage="semantic",
        category="virtual-boundary-filter-impact",
        status=status,
        summary=summary,
        source=f"virtual_boundary_lanelets={len(virtual_lanelets)}",
        target=f"confirmed_virtual_lanelets={len(set(virtual_lanelets) & mapped)}",
        method=(
            "Identify source lanelets whose left or right boundary way is tagged type/subtype=virtual, "
            "then compare their ids with lanelet2:lanelet_ids provenance on target OSM highways. "
            "This isolates possible virtual-boundary filtering from normal lane aggregation."
        ),
        hint=f"missing_virtual_lanelets={len(missing_ids)}" if missing_ids else "",
        details=[
            {
                "missing_virtual_lanelet_ids": missing_ids[:500],
                "source_virtual_lanelets": [virtual_lanelets[item] for item in sorted(virtual_lanelets, key=_safe_text_sort_key)][:500],
            }
        ],
    )


def _coverage_check(
    *,
    stage: str,
    category: str,
    expected: int,
    actual: int,
    summary: str,
    source: str,
    target: str,
    strict_zero: bool,
    warn_ratio: float,
    method: str,
) -> DiagnosticCheck:
    if strict_zero and expected > 0 and actual <= 0:
        status = "FAIL"
        hint = "Expected convertible source elements, but target contains no corresponding network elements."
    elif expected > 0 and actual / max(expected, 1) < warn_ratio:
        status = "WARN"
        hint = f"Target/source ratio {actual}/{expected} is below review threshold {warn_ratio:.2f}."
    else:
        status = "PASS"
        hint = ""
    return DiagnosticCheck(
        stage=stage,
        category=category,
        status=status,
        summary=summary,
        source=source,
        target=target,
        method=method,
        hint=hint,
        details=[{"expected": expected, "actual": actual}],
    )


def _preservation_warning(
    stage: str,
    category: str,
    expected: int,
    actual: int,
    *,
    source: str,
    target: str,
    method: str,
) -> DiagnosticCheck:
    if expected <= 0:
        status = "PASS"
        summary = "No source semantic elements of this class were found."
        hint = ""
    elif actual <= 0:
        status = "WARN"
        summary = "Source semantic elements may not be preserved in target format."
        hint = "This may be expected for lossy target formats, but should be reviewed."
    else:
        status = "PASS"
        summary = "Target contains semantic carriers for this source element class."
        hint = ""
    return DiagnosticCheck(
        stage=stage,
        category=category,
        status=status,
        summary=summary,
        source=source,
        target=target,
        method=method,
        hint=hint,
        details=[{"expected": expected, "actual": actual}],
    )


def _write_text_report(path: Path, payload: Dict[str, object]) -> None:
    summary = payload["summary"]
    lines = [
        f"{payload['conversion']} Conversion Diagnostics",
        "=" * 80,
        f"Source: {_relative(Path(str(payload['source'])))}",
        f"Target: {_relative(Path(str(payload['target'])))}",
        f"Result: {payload['result']}",
        (
            f"PASS={summary['pass']} WARN={summary['warn']} FAIL={summary['fail']} "
            f"REVIEW={summary['review']} SKIP={summary['skip']}"
        ),
        "",
        "Stage1 Acceptance",
        "-" * 80,
    ]
    for item in payload.get("acceptance", []):
        lines.append(f"[{item.get('status', '')}] {item.get('category', '')}: {item.get('summary', '')}")
        if item.get("hint"):
            lines.append(f"  hint: {item['hint']}")
    lines.extend(["", "Stage2 Diagnostics", "-" * 80])
    diagnostics = payload.get("diagnostics", {})
    for section in ("target_conformance", "topology", "element_mapping"):
        lines.append(f"{section}:")
        for item in diagnostics.get(section, []):
            lines.append(f"  [{item.get('status', '')}] {item.get('category', '')}: {item.get('summary', '')}")
            if item.get("source"):
                lines.append(f"    source: {item['source']}")
            if item.get("target"):
                lines.append(f"    target: {item['target']}")
            if item.get("method"):
                lines.append(f"    method: {item['method']}")
            if item.get("hint"):
                lines.append(f"    hint: {item['hint']}")
            details = item.get("details") or []
            if details:
                lines.append(f"    details: {len(details)} item(s) in JSON")
                first_detail = details[0]
                if isinstance(first_detail, dict):
                    preview_keys = list(first_detail.keys())[:6]
                    preview = {
                        key: _text_preview_value(first_detail.get(key))
                        for key in preview_keys
                    }
                    lines.append(f"    first_detail: {json.dumps(preview, ensure_ascii=False)[:800]}")
        lines.append("")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _count_statuses(checks: Sequence[DiagnosticCheck]) -> Dict[str, int]:
    return {status.lower(): sum(1 for check in checks if check.status == status) for status in STATUS_ORDER}


def _result_from_counts(counts: Dict[str, int]) -> str:
    if counts["fail"]:
        return "FAIL"
    if counts["warn"]:
        return "PASS_WITH_WARNINGS"
    if counts["review"]:
        return "PASS_REVIEW"
    if counts["pass"]:
        return "PASS"
    if counts["skip"]:
        return "SKIP"
    return "UNKNOWN"


def _osm_detail(stats: OsmStats) -> Dict[str, object]:
    data = asdict(stats)
    data["highway_types"] = dict(stats.highway_types)
    data["relation_types"] = dict(stats.relation_types)
    data["regulatory_subtypes"] = dict(stats.regulatory_subtypes)
    data["opendrive_lane_mapping_keys"] = sorted(stats.opendrive_lane_mapping_keys)
    data["lanelet_topology_edges"] = sorted(stats.lanelet_topology_edges)
    data["highway_topology_edges"] = sorted(stats.highway_topology_edges)
    data["source_lanelet_mapping_keys"] = sorted(stats.source_lanelet_mapping_keys, key=_safe_text_sort_key)
    data["source_lanelet_regulatory_keys"] = sorted(stats.source_lanelet_regulatory_keys, key=_safe_text_sort_key)
    data["source_multipolygon_mapping_keys"] = sorted(stats.source_multipolygon_mapping_keys, key=_safe_text_sort_key)
    data["source_osm_semantic_keys"] = sorted(stats.source_osm_semantic_keys)
    for key in (
        "way_ids",
        "node_ids",
        "relation_ids",
        "lanelet_ids",
        "node_xy",
        "way_nodes",
        "way_tags",
        "lanelet_details",
        "highway_details",
        "area_details",
        "traffic_control_details",
        "invalid_lanelet_details",
        "lanelet_topology_edge_details",
        "highway_topology_edge_details",
        "regulatory_details",
        "turn_restriction_details",
        "stop_line_details",
        "highway_intersection_details",
        "crosswalk_details",
        "virtual_boundary_lanelets",
    ):
        data.pop(key, None)
    return data


def _text_preview_value(value: object) -> object:
    """Keep the readable report compact while full evidence remains in JSON."""
    if isinstance(value, list):
        return f"<{len(value)} items>"
    if isinstance(value, dict):
        return f"<{len(value)} fields>"
    text = str(value)
    return text if len(text) <= 160 else text[:157] + "..."


def _opendrive_detail(stats: OpenDriveStats) -> Dict[str, object]:
    data = asdict(stats)
    data["signal_types"] = dict(stats.signal_types)
    data["object_types"] = dict(stats.object_types)
    data["signal_categories"] = dict(stats.signal_categories)
    data["driving_lane_keys"] = sorted(stats.driving_lane_keys)
    data["topology_edges"] = sorted(stats.topology_edges)
    return data


def _tags(element: etree._Element) -> Dict[str, str]:
    return {
        tag.get("k", ""): tag.get("v", "")
        for tag in element.findall("tag")
        if tag.get("k")
    }


def _lanelet_detail(relation: etree._Element, stats: OsmStats) -> Dict[str, object]:
    lanelet_id = relation.get("id", "")
    tags = _tags(relation)
    members = relation.findall("member")
    left_way = next((member.get("ref", "") for member in members if member.get("role") == "left"), "")
    right_way = next((member.get("ref", "") for member in members if member.get("role") == "right"), "")
    issues: List[str] = []
    if not left_way:
        issues.append("missing left boundary")
    if not right_way:
        issues.append("missing right boundary")
    left_nodes = stats.way_nodes.get(left_way, [])
    right_nodes = stats.way_nodes.get(right_way, [])
    virtual_boundaries = [
        way_id
        for way_id in (left_way, right_way)
        if _is_virtual_boundary(stats.way_tags.get(way_id, {}))
    ]
    if left_way and left_way not in stats.way_nodes:
        issues.append(f"left way {left_way} missing")
    if right_way and right_way not in stats.way_nodes:
        issues.append(f"right way {right_way} missing")
    if left_way and len(left_nodes) < 2:
        issues.append(f"left way {left_way} has fewer than 2 nodes")
    if right_way and len(right_nodes) < 2:
        issues.append(f"right way {right_way} has fewer than 2 nodes")
    left_points = [stats.node_xy[node_id] for node_id in left_nodes if node_id in stats.node_xy]
    right_points = [stats.node_xy[node_id] for node_id in right_nodes if node_id in stats.node_xy]
    left_lonlat = [
        stats.node_lonlat[node_id]
        for node_id in left_nodes
        if node_id in stats.node_lonlat
    ]
    right_lonlat = [
        stats.node_lonlat[node_id]
        for node_id in right_nodes
        if node_id in stats.node_lonlat
    ]
    if left_nodes and len(left_points) != len(left_nodes):
        issues.append(f"left way {left_way} has missing node coordinates")
    if right_nodes and len(right_points) != len(right_nodes):
        issues.append(f"right way {right_way} has missing node coordinates")
    left_length = _polyline_length(left_points)
    right_length = _polyline_length(right_points)
    area = _lanelet_area(left_points, right_points)
    boundary_gap = _average_boundary_separation(left_points, right_points)
    if left_points and right_points and area <= 1e-18 and boundary_gap <= 1e-12:
        issues.append("left/right boundaries are nearly identical")
    start_key = _lanelet_endpoint_key(left_nodes, right_nodes, start=True)
    end_key = _lanelet_endpoint_key(left_nodes, right_nodes, start=False)
    source_road = tags.get("source:opendrive:road", "")
    source_lane = tags.get("source:opendrive:lane", "")
    source_section = tags.get("source:opendrive:lane_section", "")
    source_key = (
        f"{source_road}|{source_section}|{source_lane}"
        if source_road and source_section and source_lane
        else ""
    )
    detailed_source_keys = _parse_opendrive_source_lane_locations(
        tags.get("source:opendrive:lane_locations_detailed", "")
    )
    source_keys = sorted(set(([source_key] if source_key else []) + detailed_source_keys))
    center_start = _average_points([left_points[0], right_points[0]]) if left_points and right_points else None
    center_end = _average_points([left_points[-1], right_points[-1]]) if left_points and right_points else None
    center_mid = _average_points([_points_midpoint(left_points), _points_midpoint(right_points)]) if left_points and right_points else None
    center_mid_lonlat = (
        _average_points(
            [_points_midpoint(left_lonlat), _points_midpoint(right_lonlat)]
        )
        if left_lonlat and right_lonlat
        else None
    )
    return {
        "lanelet_id": lanelet_id,
        "left_way": left_way,
        "right_way": right_way,
        "left_nodes": len(left_nodes),
        "right_nodes": len(right_nodes),
        "approx_length": round((left_length + right_length) / 2.0, 3),
        "approx_area": round(area, 6),
        "avg_boundary_gap": round(boundary_gap, 12),
        "start_key": start_key,
        "end_key": end_key,
        "center_start_xy": _round_point(center_start) if center_start else None,
        "center_end_xy": _round_point(center_end) if center_end else None,
        "center_mid_xy": _round_point(center_mid) if center_mid else None,
        "center_mid_lonlat": (
            _round_point(center_mid_lonlat) if center_mid_lonlat else None
        ),
        "source_opendrive_road": source_road,
        "source_opendrive_lane": source_lane,
        "source_opendrive_lane_section": source_section,
        "source_opendrive_parametric_lane": tags.get("source:opendrive:parametric_lane", ""),
        "source_opendrive_lane_locations": tags.get("source:opendrive:lane_locations", ""),
        "source_opendrive_lane_locations_detailed": tags.get(
            "source:opendrive:lane_locations_detailed", ""
        ),
        "source_opendrive_mapping_status": tags.get("source:opendrive:mapping_status", ""),
        "source_opendrive_key": source_key,
        "source_opendrive_keys": source_keys,
        "source_osm_turn_restriction_ids": tags.get("source:osm:turn_restriction_ids", ""),
        "source_osm_restriction_paths": tags.get("source:osm:restriction_paths", ""),
        "source_osm_forbidden_successor_lanelets": tags.get(
            "source:osm:forbidden_successor_lanelets", ""
        ),
        "source_osm_only_successor_lanelets": tags.get(
            "source:osm:only_successor_lanelets", ""
        ),
        "virtual_boundaries": virtual_boundaries,
        "issues": issues,
        "location": f"/osm/relation[@id='{lanelet_id}']",
    }


def _parse_opendrive_source_lane_locations(value: str) -> List[str]:
    keys: List[str] = []
    for item in value.split(";"):
        parts = [part.strip() for part in item.split(",")]
        if len(parts) == 3 and all(parts):
            keys.append("|".join(parts))
    return keys


def _lanelet_endpoint_key(left_nodes: List[str], right_nodes: List[str], *, start: bool) -> str:
    if len(left_nodes) < 1 or len(right_nodes) < 1:
        return ""
    pair = (left_nodes[0], right_nodes[0]) if start else (left_nodes[-1], right_nodes[-1])
    return "|".join(sorted(pair))


def _count_lanelet_adjacency_edges(details: Sequence[Dict[str, object]]) -> int:
    starts = Counter(str(detail.get("start_key", "")) for detail in details if detail.get("start_key"))
    edges = 0
    for detail in details:
        end_key = str(detail.get("end_key", ""))
        if end_key:
            edges += starts.get(end_key, 0)
    return edges


def _parse_csv_tag(value: str) -> List[str]:
    if not value:
        return []
    return [item.strip() for item in str(value).split(",") if item.strip()]


def _source_regulatory_keys_from_tags(tags: Dict[str, str]) -> List[str]:
    keys: List[str] = []
    for tag_key in (
        "lanelet2:regulatory_elements",
        "lanelet2:speed_limit_relations",
        "lanelet2:traffic_light_relations",
        "lanelet2:right_of_way_relations",
        "lanelet2:yield_relations",
    ):
        keys.extend(_parse_csv_tag(tags.get(tag_key, "")))
    return sorted(set(keys), key=_safe_text_sort_key)


def _traffic_control_tags(tags: Dict[str, str]) -> Dict[str, str]:
    highway = str(tags.get("highway", ""))
    has_control = bool(
        tags.get("traffic_signals")
        or tags.get("traffic_sign")
        or tags.get("maxspeed")
        or tags.get("restriction")
        or tags.get("lanelet2:regulatory_elements")
        or tags.get("lanelet2:speed_limit_relations")
        or tags.get("lanelet2:traffic_light_relations")
        or tags.get("lanelet2:right_of_way_relations")
        or tags.get("lanelet2:yield_relations")
        or highway in {"traffic_signals", "stop", "give_way", "crossing"}
        or tags.get("type") in {"traffic_sign", "traffic_light", "stop_line", "regulatory_element", "restriction"}
    )
    if not has_control:
        return {}
    keys = {
        "highway",
        "traffic_signals",
        "traffic_sign",
        "crossing",
        "maxspeed",
        "restriction",
        "lanelet2:regulatory_elements",
        "lanelet2:speed_limit_relations",
        "lanelet2:traffic_light_relations",
        "lanelet2:right_of_way_relations",
        "lanelet2:yield_relations",
        "lanelet2:has_right_of_way",
        "lanelet2:must_yield",
    }
    result = {key: value for key, value in tags.items() if key in keys}
    if tags.get("type") in {"traffic_sign", "traffic_light", "stop_line", "regulatory_element", "restriction"}:
        result["type"] = tags.get("type", "")
        if tags.get("subtype"):
            result["subtype"] = tags.get("subtype", "")
    return result


def _collect_osm_semantic_provenance(stats: OsmStats, tags: Dict[str, str]) -> None:
    source_element = str(tags.get("source:osm:element", ""))
    source_id = str(tags.get("source:osm:id", ""))
    semantic = str(tags.get("source:osm:semantic", ""))
    if source_element and source_id and semantic:
        stats.source_osm_semantic_keys.add(f"{source_element}:{source_id}:{semantic}")
    source_way = str(tags.get("source:osm:way", ""))
    if source_way and semantic:
        stats.source_osm_semantic_keys.add(f"way:{source_way}:{semantic}")


def _osm_control_categories(tags: Dict[str, str]) -> set:
    categories = set()
    highway = str(tags.get("highway", "")).strip().lower()
    element_type = str(tags.get("type", "")).strip().lower()
    subtype = str(tags.get("subtype", "")).strip().lower()
    traffic_sign = str(tags.get("traffic_sign", "")).strip().lower()
    crossing = str(tags.get("crossing", "")).strip().lower()
    if (
        highway == "traffic_signals"
        or element_type == "traffic_light"
        or tags.get("traffic_signals")
        or "traffic_signals" in crossing
    ):
        categories.add("traffic_light")
    if tags.get("maxspeed") or element_type == "speed_limit" or "274" in traffic_sign or "de274" in subtype:
        categories.add("speed_limit")
    if highway == "stop" or "206" in traffic_sign or subtype in {"de206", "stop"}:
        categories.add("stop")
    if highway in {"give_way", "yield"} or "205" in traffic_sign or subtype in {"de205", "give_way", "yield"}:
        categories.add("give_way")
    if element_type == "stop_line" or tags.get("stop_line") == "yes" or tags.get("road_marking") == "stop_line":
        categories.add("stop_line")
    return categories


def _is_osm_crosswalk_area(tags: Dict[str, str], node_refs: Sequence[str]) -> bool:
    if len(node_refs) < 4 or node_refs[0] != node_refs[-1]:
        return False
    if tags.get("area") != "yes":
        return False
    values = {
        str(tags.get("type", "")).lower(),
        str(tags.get("subtype", "")).lower(),
        str(tags.get("highway", "")).lower(),
        str(tags.get("footway", "")).lower(),
        str(tags.get("crossing", "")).lower(),
    }
    return bool(values & {"crosswalk", "ped_crossing", "pedestrian_crossing", "crossing"})


def _is_virtual_boundary(tags: Dict[str, str]) -> bool:
    return str(tags.get("type", "")).lower() == "virtual" or str(tags.get("subtype", "")).lower() == "virtual"


def _polyline_length(points: Sequence[Tuple[float, float]]) -> float:
    return sum(
        math.hypot(points[index][0] - points[index - 1][0], points[index][1] - points[index - 1][1])
        for index in range(1, len(points))
    )


def _lanelet_area(left_points: Sequence[Tuple[float, float]], right_points: Sequence[Tuple[float, float]]) -> float:
    if len(left_points) < 2 or len(right_points) < 2:
        return 0.0
    polygon = list(left_points) + list(reversed(right_points))
    area = 0.0
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % len(polygon)]
        area += point[0] * next_point[1] - next_point[0] * point[1]
    return abs(area) / 2.0


def _average_boundary_separation(
    left_points: Sequence[Tuple[float, float]], right_points: Sequence[Tuple[float, float]]
) -> float:
    if not left_points or not right_points:
        return 0.0
    pairs = zip(left_points, right_points)
    distances = [math.hypot(left[0] - right[0], left[1] - right[1]) for left, right in pairs]
    if not distances:
        return 0.0
    return sum(distances) / len(distances)


def _points_midpoint(points: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    mid_index = len(points) // 2
    if len(points) % 2 == 1:
        return points[mid_index]
    a = points[mid_index - 1]
    b = points[mid_index]
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _average_points(points: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
    valid = [point for point in points if point is not None]
    if not valid:
        return (0.0, 0.0)
    return (
        sum(point[0] for point in valid) / len(valid),
        sum(point[1] for point in valid) / len(valid),
    )


def _round_point(point: Tuple[float, float]) -> Tuple[float, float]:
    return (round(point[0], 8), round(point[1], 8))


def _tuple_point(value: object) -> Optional[Tuple[float, float]]:
    if value is None:
        return None
    if isinstance(value, (list, tuple)) and len(value) >= 2:
        x = _float(str(value[0]))
        y = _float(str(value[1]))
        if x is not None and y is not None:
            return (x, y)
    return None


def _point_distance_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    if _looks_like_lon_lat(a) and _looks_like_lon_lat(b):
        lat = math.radians((a[1] + b[1]) / 2.0)
        dx = (a[0] - b[0]) * 111320.0 * math.cos(lat)
        dy = (a[1] - b[1]) * 110540.0
        return math.hypot(dx, dy)
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _looks_like_lon_lat(point: Tuple[float, float]) -> bool:
    return -180.0 <= point[0] <= 180.0 and -90.0 <= point[1] <= 90.0


def _node_xy(node: etree._Element) -> Tuple[Optional[float], Optional[float]]:
    tags = _tags(node)
    x = _float(tags.get("local_x"))
    y = _float(tags.get("local_y"))
    if x is None:
        x = _float(node.get("lon"))
    if y is None:
        y = _float(node.get("lat"))
    return x, y


def _node_lonlat(node: etree._Element) -> Optional[Tuple[float, float]]:
    lon = _float(node.get("lon"))
    lat = _float(node.get("lat"))
    if lon is None or lat is None:
        return None
    return lon, lat


def _float(value: Optional[str]) -> Optional[float]:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _counter(counter: Counter) -> str:
    if not counter:
        return "{}"
    return ", ".join(f"{key}:{value}" for key, value in counter.most_common(8))


def _safe_text_sort_key(value: object) -> Tuple[int, str]:
    text = str(value)
    try:
        return (int(text), text)
    except ValueError:
        return (0, text)


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return str(path)
