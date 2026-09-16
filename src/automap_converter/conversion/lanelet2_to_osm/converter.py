import logging
import math
import re
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

from automap_converter.formats.lanelet2.map import (
    Multipolygon,
    Node as Lanelet2Node,
    OSMLanelet as Lanelet2Map,
    Way as Lanelet2Way,
    WayRelation,
)
from automap_converter.formats.lanelet2.osm_model import (
    Node as OSMNode,
    OSM,
    Relation as OSMRelation,
    RelationMember,
    Way as OSMWay,
)

LOGGER = logging.getLogger(__name__)


class _IdGenerator:
    """Generate fresh negative ids without colliding with existing ids of the same type."""

    def __init__(self, existing_ids: Iterable[str]):
        numeric_ids: List[int] = []
        for raw_id in existing_ids:
            try:
                numeric_ids.append(int(raw_id))
            except (TypeError, ValueError):
                continue
        negative_ids = [value for value in numeric_ids if value < 0]
        self._next_id = (min(negative_ids) - 1) if negative_ids else -1

    def next(self) -> str:
        value = self._next_id
        self._next_id -= 1
        return str(value)


@dataclass
class _CenterlineSegment:
    segment_id: str
    lanelet_ids: List[str]
    outer_pair: Tuple[str, str]
    left_start_node: str
    left_end_node: str
    right_start_node: str
    right_end_node: str
    xy: List[Tuple[float, float]]
    ll: List[Tuple[float, float]]
    ele: List[float]
    tags: Dict[str, str]
    representative_lanelet_ids: List[str] = field(default_factory=list)
    representative_lanelet_count: int = 0
    representative_policy: str = ""
    ordered_lanelet_ids: List[str] = field(default_factory=list)

    @property
    def start_signature(self) -> Tuple[str, str]:
        return self.left_start_node, self.right_start_node

    @property
    def end_signature(self) -> Tuple[str, str]:
        return self.left_end_node, self.right_end_node


@dataclass
class _DraftCenterlineWay:
    """A not-yet-serialized centerline way.

    Draft ways let us keep separate OSM ways while still resolving topology and
    geometry at way boundaries before node ids are written.  This is used for
    the road-chain <-> junction-connector smoothing pass.
    """

    draft_id: str
    kind: str  # "road" or "connector"
    xy: List[Tuple[float, float]]
    ll: List[Tuple[float, float]]
    ele: List[float]
    tags: Dict[str, str]
    chain_index: Optional[int] = None
    from_chain: Optional[int] = None
    to_chain: Optional[int] = None
    from_lanelet: Optional[str] = None
    to_lanelet: Optional[str] = None
    start_lanelet_ids: List[str] = field(default_factory=list)
    end_lanelet_ids: List[str] = field(default_factory=list)
    # Full lanelet groups represented by this draft way.
    # start/end_group_lanelet_ids are the complete lateral lanelet group at the
    # endpoint segment, while all_lanelet_ids covers the whole road chain or
    # connector path.  Endpoint-extension topology checks use these group-level
    # sets rather than only representative lanelets.
    start_group_lanelet_ids: List[str] = field(default_factory=list)
    end_group_lanelet_ids: List[str] = field(default_factory=list)
    all_lanelet_ids: List[str] = field(default_factory=list)
    start_shared_node_id: Optional[str] = None
    end_shared_node_id: Optional[str] = None
    # Shared node ids inserted into the middle of this draft way.
    # Used when an endpoint is extended to hit the middle of another centerline way.
    shared_node_ids_by_index: Dict[int, str] = field(default_factory=dict)


class Lanelet2OSMConverter:
    """
    Convert Lanelet2 map into generic OSM centerline ways.

    This version adds:
    1. Representative-lanelet inheritance across 3->4 / 4->3 lane count transitions.
    2. Controlled bridge generation for short discontinuities after a valid longitudinal match.
    """

    def __init__(
        self,
        config: Optional[object] = None,
        min_centerline_points: int = 20,
        copy_way_tags_for_multipolygon_boundaries: bool = True,
        add_default_highway_tag: bool = True,
        longitudinal_join_distance: float = 8.0,
        bridge_join_distance: float = 20.0,
        bridge_lateral_shift_tolerance: float = 12.0,
        bridge_insert_points: int = 6,
        direction_cosine_threshold: float = 0.3,
        lateral_direction_cosine_threshold: float = 0.8,
        lateral_offset_tolerance: float = 15.0,
        topology_priority_bonus: float = 1000.0,
        prefer_existing_topology: bool = True,
        fallback_fill_missing_topology_only: bool = True,
        straight_connector_angle_threshold_deg: float = 45.0,
        connector_path_max_hops: int = 16,
        joint_smoothing_min_points: int = 10,
        joint_smoothing_max_points: int = 24,
        joint_tangent_span: int = 10,
        joint_post_smooth_passes: int = 4,
        joint_control_ratio: float = 0.65,
        joint_smoothing_angle_threshold_deg: float = 45.0,
        topology_endpoint_snap_tolerance: float = 0.05,
        # Geometry endpoint-extension pass for dangling endpoints.
        # This pass only extends an unshared source endpoint to an existing target-way segment.
        # It does NOT extend target ways and it does NOT use projection snapping.
        # Distances are in meters because lat/lon is projected to local metric XY if needed.
        endpoint_extension_enabled: bool = True,
        endpoint_extension_search_radius: float = 45.0,
        endpoint_extension_max_distance: float = 40.0,
        endpoint_extension_min_distance: float = 1.0,
        endpoint_extension_direction_cosine_threshold: float = 0.05,
        endpoint_extension_endpoint_margin: float = 0.0,
        endpoint_extension_crossing_max_parallel_cosine: float = 0.95,
        endpoint_extension_second_hit_enabled: bool = True,
        # Topology guard for endpoint extension.  Geometry candidates are still
        # generated by the original v13 ray/segment intersection logic; this
        # only decides which Q1/Q2/Q3... hits are allowed to create shared nodes.
        endpoint_extension_require_topology: bool = True,
        endpoint_extension_candidate_hit_limit: int = 4,
        endpoint_extension_max_selected_hits: int = 2,
        endpoint_extension_topology_max_hops: int = 16,
        endpoint_extension_target_neighbor_hops: int = 4,
        endpoint_extension_fallback_to_nearest_geometry: bool = False,
        # Endpoint tangent adaptation.  If the local tangent changes by more
        # than 15 degrees near an endpoint, use a shorter 1/2-segment tangent
        # instead of the long joint_tangent_span window.
        endpoint_extension_tangent_variation_threshold_deg: float = 15.0,
        endpoint_extension_tangent_high_variation_threshold_deg: float = 30.0,
        endpoint_extension_tangent_short_span: int = 2,
        endpoint_extension_tangent_min_span: int = 1,
        endpoint_extension_second_hit_min_gap: float = 2.0,
        endpoint_extension_second_hit_max_gap: float = 18.0,
        endpoint_extension_second_hit_parallel_cosine: float = 0.85,
        endpoint_extension_insert_points_enabled: bool = True,
        endpoint_extension_insert_spacing_factor: float = 1.0,
        endpoint_extension_insert_spacing_sample_points: int = 6,
        endpoint_extension_max_insert_points: int = 20,
        endpoint_extension_debug_examples: int = 0,
        endpoint_extension_max_draft_ways: int = 1200,
        debug_output_path: Optional[str] = None,
    ):
        self._config = config
        self.min_centerline_points = max(2, int(min_centerline_points))
        self.copy_way_tags_for_multipolygon_boundaries = copy_way_tags_for_multipolygon_boundaries
        self.add_default_highway_tag = add_default_highway_tag
        self.longitudinal_join_distance = float(longitudinal_join_distance)
        self.bridge_join_distance = float(bridge_join_distance)
        self.bridge_lateral_shift_tolerance = float(bridge_lateral_shift_tolerance)
        self.bridge_insert_points = max(2, int(bridge_insert_points))
        self.direction_cosine_threshold = float(direction_cosine_threshold)
        self.lateral_direction_cosine_threshold = float(lateral_direction_cosine_threshold)
        self.lateral_offset_tolerance = float(lateral_offset_tolerance)
        self.topology_priority_bonus = float(topology_priority_bonus)

        # topology strategy
        self.prefer_existing_topology = bool(prefer_existing_topology)
        self.fallback_fill_missing_topology_only = bool(fallback_fill_missing_topology_only)
        self.straight_connector_angle_threshold_deg = float(straight_connector_angle_threshold_deg)
        self.connector_path_max_hops = max(1, int(connector_path_max_hops))
        self.joint_smoothing_min_points = max(3, int(joint_smoothing_min_points))
        self.joint_smoothing_max_points = max(self.joint_smoothing_min_points, int(joint_smoothing_max_points))
        self.joint_tangent_span = max(2, int(joint_tangent_span))
        self.joint_post_smooth_passes = max(0, int(joint_post_smooth_passes))
        self.joint_control_ratio = max(0.1, float(joint_control_ratio))
        self.joint_smoothing_angle_threshold_deg = float(joint_smoothing_angle_threshold_deg)
        self.topology_endpoint_snap_tolerance = max(0.0, float(topology_endpoint_snap_tolerance))
        self.endpoint_extension_enabled = bool(endpoint_extension_enabled)
        self.endpoint_extension_search_radius = max(0.0, float(endpoint_extension_search_radius))
        self.endpoint_extension_max_distance = max(0.0, float(endpoint_extension_max_distance))
        self.endpoint_extension_min_distance = max(0.0, float(endpoint_extension_min_distance))
        self.endpoint_extension_direction_cosine_threshold = float(endpoint_extension_direction_cosine_threshold)
        self.endpoint_extension_endpoint_margin = max(0.0, float(endpoint_extension_endpoint_margin))
        self.endpoint_extension_crossing_max_parallel_cosine = max(0.0, min(1.0, float(endpoint_extension_crossing_max_parallel_cosine)))
        self.endpoint_extension_second_hit_enabled = bool(endpoint_extension_second_hit_enabled)
        self.endpoint_extension_require_topology = bool(endpoint_extension_require_topology)
        self.endpoint_extension_candidate_hit_limit = max(1, int(endpoint_extension_candidate_hit_limit))
        self.endpoint_extension_max_selected_hits = max(1, int(endpoint_extension_max_selected_hits))
        self.endpoint_extension_topology_max_hops = max(0, int(endpoint_extension_topology_max_hops))
        self.endpoint_extension_target_neighbor_hops = max(0, int(endpoint_extension_target_neighbor_hops))
        self.endpoint_extension_fallback_to_nearest_geometry = bool(endpoint_extension_fallback_to_nearest_geometry)
        self.endpoint_extension_tangent_variation_threshold_deg = max(0.0, float(endpoint_extension_tangent_variation_threshold_deg))
        self.endpoint_extension_tangent_high_variation_threshold_deg = max(self.endpoint_extension_tangent_variation_threshold_deg, float(endpoint_extension_tangent_high_variation_threshold_deg))
        self.endpoint_extension_tangent_short_span = max(1, int(endpoint_extension_tangent_short_span))
        self.endpoint_extension_tangent_min_span = max(1, int(endpoint_extension_tangent_min_span))
        self.endpoint_extension_second_hit_min_gap = max(0.0, float(endpoint_extension_second_hit_min_gap))
        self.endpoint_extension_second_hit_max_gap = max(self.endpoint_extension_second_hit_min_gap, float(endpoint_extension_second_hit_max_gap))
        self.endpoint_extension_second_hit_parallel_cosine = max(0.0, min(1.0, float(endpoint_extension_second_hit_parallel_cosine)))
        self.endpoint_extension_insert_points_enabled = bool(endpoint_extension_insert_points_enabled)
        self.endpoint_extension_insert_spacing_factor = max(0.1, float(endpoint_extension_insert_spacing_factor))
        self.endpoint_extension_insert_spacing_sample_points = max(1, int(endpoint_extension_insert_spacing_sample_points))
        self.endpoint_extension_max_insert_points = max(0, int(endpoint_extension_max_insert_points))
        self.endpoint_extension_debug_examples = max(0, int(endpoint_extension_debug_examples))
        # Endpoint extension compares every dangling endpoint against every
        # draft way. It is useful around small intersections, but its cost
        # grows rapidly on city-scale maps. Zero explicitly disables the limit.
        self.endpoint_extension_max_draft_ways = max(0, int(endpoint_extension_max_draft_ways))
        self.endpoint_extension_skip_reason = ""

        self._source: Optional[Lanelet2Map] = None
        self._out: Optional[OSM] = None
        self._node_id_gen: Optional[_IdGenerator] = None
        self._way_id_gen: Optional[_IdGenerator] = None
        self.debug_output_path = debug_output_path

        # XY geometry is used for all distance/intersection tests.  Some inputs
        # do not carry local_x/local_y; in that case we project lat/lon to a
        # local metric equirectangular frame so thresholds remain in meters.
        self._xy_origin_lat: Optional[float] = None
        self._xy_origin_lon: Optional[float] = None
        self._xy_m_per_deg_lat: float = 111320.0
        self._xy_m_per_deg_lon: float = 111320.0

    def __call__(self, osm: Lanelet2Map) -> OSM:
        return self.convert(osm)

    def convert(self, osm: Lanelet2Map) -> OSM:
        self._source = osm
        self._initialize_xy_projection_origin()
        if self.debug_output_path:
            try:
                with open(self.debug_output_path, "w", encoding="utf-8") as f:
                    f.write("")
            except Exception as e:
                LOGGER.warning("Failed to initialize debug output file %s: %s", self.debug_output_path, e)
        if not self._source.nodes:
            LOGGER.warning("Lanelet2OSMConverter: input OSMLanelet is empty.")
            return OSM()

        self._out = OSM()
        self._node_id_gen = _IdGenerator(self._source.nodes.keys())
        self._way_id_gen = _IdGenerator(self._source.ways.keys())
        self._emitted_regulatory_feature_keys: Set[Tuple[str, str, str]] = set()
        self._preserved_stop_line_refs: Set[str] = set()

        LOGGER.info(
            "Lanelet2OSMConverter: converting %d lanelet nodes, %d ways, %d lanelets, %d multipolygons",
            len(self._source.nodes),
            len(self._source.ways),
            len(self._source.way_relations),
            len(self._source.multipolygons),
        )

        #self._convert_multipolygons()

        # Prefer existing explicit topology (for example official RoutingGraph writeback).
        # Only use boundary-endpoint inference to fill gaps when topology is missing.
        self._infer_lanelet_topology_from_boundary_endpoints(
            overwrite_existing=not self.prefer_existing_topology,
            only_fill_missing=self.fallback_fill_missing_topology_only,
        )

        self._convert_lanelets_to_road_centerlines()
        self._convert_selected_multipolygon_features()
        return self._out
    
    def _debug_print_specific_lanelet_topology(self, lanelet_ids: Sequence[str]) -> None:
        assert self._source is not None
        LOGGER.info("========== DEBUG: specific lanelet topology ==========")
        for lanelet_id in lanelet_ids:
            lanelet_id = str(lanelet_id)
            lanelet = self._source.find_way_rel_by_id(lanelet_id)
            if lanelet is None:
                LOGGER.info("DEBUG: lanelet %s not found.", lanelet_id)
                continue
            preds = getattr(lanelet, "predecessors", None) or []
            succs = getattr(lanelet, "successors", None) or []
            left_neighbors = getattr(lanelet, "left_neighbors", None) or []
            right_neighbors = getattr(lanelet, "right_neighbors", None) or []
            adjacent_left = getattr(lanelet, "adjacent_left", None) or []
            adjacent_right = getattr(lanelet, "adjacent_right", None) or []
            topology_source = getattr(lanelet, "topology_source", None)
            LOGGER.info("DEBUG: lanelet %s", lanelet_id)
            LOGGER.info("DEBUG:   predecessors=%s", ",".join(str(x) for x in preds) if preds else "NONE")
            LOGGER.info("DEBUG:   successors=%s", ",".join(str(x) for x in succs) if succs else "NONE")
            LOGGER.info("DEBUG:   left_neighbors=%s", ",".join(str(x) for x in left_neighbors) if left_neighbors else "NONE")
            LOGGER.info("DEBUG:   right_neighbors=%s", ",".join(str(x) for x in right_neighbors) if right_neighbors else "NONE")
            LOGGER.info("DEBUG:   adjacent_left=%s", ",".join(str(x) for x in adjacent_left) if adjacent_left else "NONE")
            LOGGER.info("DEBUG:   adjacent_right=%s", ",".join(str(x) for x in adjacent_right) if adjacent_right else "NONE")
            LOGGER.info("DEBUG:   topology_source=%s", str(topology_source) if topology_source is not None else "NONE")

    def _convert_multipolygons(self) -> None:
        assert self._source is not None
        assert self._out is not None

        for multipolygon in self._source.multipolygons.values():
            members: List[RelationMember] = []
            for outer_way_id in multipolygon.outer_list:
                self._copy_source_way_with_nodes(outer_way_id)
                members.append(RelationMember("way", outer_way_id, "outer"))
            for inner_way_id in getattr(multipolygon, "inner_list", []) or []:
                self._copy_source_way_with_nodes(inner_way_id)
                members.append(RelationMember("way", inner_way_id, "inner"))

            tag_dict = self._copy_tag_dict(multipolygon.tag_dict)
            tag_dict["type"] = "multipolygon"
            relation = OSMRelation(id_=multipolygon.id_, members=members, tag_dict=tag_dict)
            self._out.add_relation(relation)

    def _copy_source_way_with_nodes(self, way_id: str) -> None:
        assert self._source is not None
        assert self._out is not None

        way_id = str(way_id)
        if self._out.find_way_by_id(way_id) is not None:
            return

        source_way = self._source.find_way_by_id(way_id)
        if source_way is None:
            LOGGER.warning("Referenced source way %s not found while copying to OSM output.", way_id)
            return

        for node_id in source_way.nodes:
            self._copy_source_node(node_id)

        copied_tags = self._copy_tag_dict(source_way.tag_dict) if self.copy_way_tags_for_multipolygon_boundaries else {}
        self._out.add_way(OSMWay(id_=source_way.id_, nodes=list(source_way.nodes), tag_dict=copied_tags))

    def _copy_source_node(self, node_id: str) -> None:
        assert self._source is not None
        assert self._out is not None

        node_id = str(node_id)
        if self._out.find_node_by_id(node_id) is not None:
            return

        source_node = self._source.find_node_by_id(node_id)
        if source_node is None:
            LOGGER.warning("Referenced source node %s not found while copying to OSM output.", node_id)
            return

        self._out.add_node(self._make_osm_node_from_source(source_node))

    def _convert_selected_multipolygon_features(self) -> None:
        """Append selected Lanelet2 multipolygon-derived auxiliary features.

        This method is intentionally called after all road/connector centerline
        ways have already been generated and serialized.  It only appends area
        or control/crossing features to ``self._out`` and never touches draft
        centerline ways, joint smoothing, endpoint extension, or topology logic.

        Policy:
        - area-like subtypes that have ordinary OSM equivalents are kept as
          OSM multipolygon relations with conservative OSM tags added;
        - subtype=ped_crossing: convert to a crossing way plus midpoint node;
        - subtype=stop_area, stop_area_type=TURN_STOP: ignore for now;
        - subtype=stop_area, stop_area_type=PED_CROSSING: convert to crossing node only;
        - subtype=stop_area, stop_area_type=STOP_SIGN: convert to highway=stop node;
        - subtype=stop_area, stop_area_type=TRAFFIC_LIGHT: convert to highway=traffic_signals node.
        """
        assert self._source is not None
        assert self._out is not None

        if not self._source.multipolygons:
            return

        stats: Dict[str, int] = defaultdict(int)
        for multipolygon in self._source.multipolygons.values():
            tags = self._copy_tag_dict(getattr(multipolygon, "tag_dict", {}) or {})
            subtype = str(tags.get("subtype", tags.get("type", ""))).strip().lower()
            stop_area_type = str(tags.get("stop_area_type", tags.get("stop_area", ""))).strip().upper()

            area_tags = self._osm_area_tags_for_multipolygon(subtype, tags)
            if area_tags is not None:
                self._copy_multipolygon_relation(multipolygon, extra_tags=area_tags)
                stats[f"{subtype or 'unknown'}_area_relation"] += 1
                continue

            if subtype == "ped_crossing":
                if self._convert_multipolygon_to_crossing_way_and_node(multipolygon, source_kind="ped_crossing"):
                    stats["ped_crossing_way_node"] += 1
                else:
                    stats["ped_crossing_failed"] += 1
                continue

            if subtype == "stop_area":
                if stop_area_type == "TURN_STOP":
                    stats["stop_area_turn_stop_ignored"] += 1
                    continue
                if stop_area_type == "PED_CROSSING":
                    if self._convert_multipolygon_to_point_feature(
                        multipolygon,
                        {
                            "highway": "crossing",
                            "crossing": "uncontrolled",
                            "lanelet2:source_multipolygon_subtype": "stop_area",
                            "lanelet2:stop_area_type": "PED_CROSSING",
                        },
                    ):
                        stats["stop_area_ped_crossing_node"] += 1
                    else:
                        stats["stop_area_ped_crossing_failed"] += 1
                    continue
                if stop_area_type == "STOP_SIGN":
                    if self._convert_multipolygon_to_point_feature(
                        multipolygon,
                        {
                            "highway": "stop",
                            "lanelet2:source_multipolygon_subtype": "stop_area",
                            "lanelet2:stop_area_type": "STOP_SIGN",
                        },
                    ):
                        stats["stop_area_stop_sign_node"] += 1
                    else:
                        stats["stop_area_stop_sign_failed"] += 1
                    continue
                if stop_area_type == "TRAFFIC_LIGHT":
                    if self._convert_multipolygon_to_point_feature(
                        multipolygon,
                        {
                            "highway": "traffic_signals",
                            "lanelet2:source_multipolygon_subtype": "stop_area",
                            "lanelet2:stop_area_type": "TRAFFIC_LIGHT",
                        },
                    ):
                        stats["stop_area_traffic_light_node"] += 1
                    else:
                        stats["stop_area_traffic_light_failed"] += 1
                    continue

                stats["stop_area_other_ignored"] += 1
                continue

            stats["other_multipolygon_ignored"] += 1

        LOGGER.info(
            "Lanelet2OSMConverter: selective multipolygon conversion stats: %s",
            {key: int(stats[key]) for key in sorted(stats.keys())},
        )

    def _output_relation_exists(self, relation_id: str) -> bool:
        """Return whether an output relation id already exists, without assuming OSM API details."""
        assert self._out is not None
        relation_id = str(relation_id)
        finder = getattr(self._out, "find_relation_by_id", None)
        if callable(finder):
            return finder(relation_id) is not None
        relations = getattr(self._out, "relations", None)
        if isinstance(relations, dict):
            return relation_id in relations
        if isinstance(relations, list):
            return any(str(getattr(rel, "id_", "")) == relation_id for rel in relations)
        return False

    def _osm_area_tags_for_multipolygon(
        self, subtype: str, source_tags: Dict[str, str]
    ) -> Optional[Dict[str, str]]:
        """Return conservative OSM area tags for Lanelet2 multipolygon subtypes."""
        subtype = str(subtype or "").strip().lower()
        if subtype in {"walkway", "footway", "sidewalk"}:
            return {"area": "yes", "highway": "pedestrian"}
        if subtype in {"parking", "parking_space", "parkingspace"}:
            return {"amenity": "parking"}
        if subtype in {"vegetation", "grass", "greenery"}:
            return {"landuse": "grass"}
        if subtype == "building":
            return {"building": "yes"}
        if subtype in {"traffic_island", "trafficisland", "island"}:
            return {"area": "yes", "area:highway": "traffic_island"}
        return None

    def _copy_multipolygon_relation(
        self, multipolygon: Multipolygon, extra_tags: Optional[Dict[str, str]] = None
    ) -> None:
        """Copy one Lanelet2 multipolygon relation with its original outer ways/nodes."""
        assert self._out is not None

        members: List[RelationMember] = []
        for outer_way_id in getattr(multipolygon, "outer_list", []) or []:
            outer_way_id = str(outer_way_id)
            self._copy_source_way_with_nodes(outer_way_id)
            members.append(RelationMember("way", outer_way_id, "outer"))
        for inner_way_id in getattr(multipolygon, "inner_list", []) or []:
            inner_way_id = str(inner_way_id)
            self._copy_source_way_with_nodes(inner_way_id)
            members.append(RelationMember("way", inner_way_id, "inner"))

        if not members:
            return

        tag_dict = self._copy_tag_dict(getattr(multipolygon, "tag_dict", {}) or {})
        if extra_tags:
            for key, value in extra_tags.items():
                tag_dict.setdefault(str(key), str(value))
        tag_dict["type"] = "multipolygon"
        tag_dict["lanelet2:source_multipolygon"] = str(multipolygon.id_)
        relation_id = str(multipolygon.id_)
        if self._output_relation_exists(relation_id):
            LOGGER.warning("Skipping copied multipolygon relation %s because output relation id already exists.", relation_id)
            return
        self._out.add_relation(OSMRelation(id_=relation_id, members=members, tag_dict=tag_dict))

    def _convert_regulatory_elements_to_road_level_features(self) -> None:
        """Map Lanelet2 lane-level regulatory relations onto output road-level OSM ways."""
        assert self._source is not None
        assert self._out is not None
        assert self._node_id_gen is not None

        if not self._source.regulatory_elements or not self._out.ways:
            return

        stats: Dict[str, int] = defaultdict(int)
        for road_way in list(self._out.ways.values()):
            lanelet_ids = self._source_lanelet_ids_for_output_way(road_way)
            if not lanelet_ids:
                continue

            regulatory_elements = self._regulatory_elements_for_lanelet_ids(lanelet_ids)
            if not regulatory_elements:
                continue

            for reg in regulatory_elements:
                subtype = str(reg.tag_dict.get("subtype", "")).strip().lower()
                if self._preserve_regulatory_stop_lines(road_way, reg):
                    stats["stop_line_way"] += 1

                if subtype == "speed_limit":
                    if self._apply_speed_limit_to_road_way(road_way, reg):
                        stats["speed_limit_way"] += 1
                    if self._emit_regulatory_feature_nodes(
                        road_way,
                        reg,
                        "speed_limit",
                        {
                            "traffic_sign": "maxspeed",
                            "lanelet2:regulatory_role": "speed_limit",
                        },
                    ):
                        stats["speed_limit_node"] += 1
                    continue

                if subtype == "traffic_light":
                    road_way.tag_dict["traffic_signals"] = "yes"
                    road_way.tag_dict["lanelet2:traffic_light_relations"] = self._append_csv_value(
                        road_way.tag_dict.get("lanelet2:traffic_light_relations"), str(reg.id_)
                    )
                    road_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level"
                    if self._emit_regulatory_feature_nodes(
                        road_way,
                        reg,
                        "traffic_light",
                        {
                            "highway": "traffic_signals",
                            "traffic_signals": "signal",
                            "lanelet2:regulatory_role": "traffic_light",
                        },
                    ):
                        stats["traffic_light_node"] += 1
                    continue

                if subtype in {"right_of_way", "all_way_stop"}:
                    if self._road_way_has_lanelet_role(road_way, reg, "yield"):
                        road_way.tag_dict["lanelet2:must_yield"] = "yes"
                        road_way.tag_dict["lanelet2:yield_relations"] = self._append_csv_value(
                            road_way.tag_dict.get("lanelet2:yield_relations"), str(reg.id_)
                        )
                        road_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level"
                        if self._emit_right_of_way_nodes(road_way, reg, yielding=True):
                            stats["yield_node"] += 1
                    if self._road_way_has_lanelet_role(road_way, reg, "right_of_way"):
                        road_way.tag_dict["lanelet2:has_right_of_way"] = "yes"
                        road_way.tag_dict["lanelet2:right_of_way_relations"] = self._append_csv_value(
                            road_way.tag_dict.get("lanelet2:right_of_way_relations"), str(reg.id_)
                        )
                        road_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level"
                        if self._emit_right_of_way_nodes(road_way, reg, yielding=False):
                            stats["priority_node"] += 1

        if stats:
            LOGGER.info(
                "Lanelet2OSMConverter: regulatory road-level conversion stats: %s",
                {key: int(stats[key]) for key in sorted(stats.keys())},
            )

    def _preserve_regulatory_stop_lines(self, road_way: OSMWay, reg: object) -> bool:
        """Copy Lanelet2 ref_line stop-line geometry and link it to the output road way."""
        assert self._source is not None
        assert self._out is not None

        ref_lines = [str(ref) for ref in (getattr(reg, "ref_line", []) or []) if str(ref).strip()]
        if not ref_lines:
            return False

        preserved = False
        for ref in ref_lines:
            source_way = self._source.find_way_by_id(ref)
            if source_way is None:
                continue
            source_tags = self._copy_tag_dict(getattr(source_way, "tag_dict", {}) or {})
            source_type = str(source_tags.get("type", "")).strip().lower()
            source_subtype = str(source_tags.get("subtype", "")).strip().lower()
            reg_subtype = str(getattr(reg, "tag_dict", {}).get("subtype", "")).strip().lower()

            # In Lanelet2 traffic_light/right_of_way relations, ref_line is the
            # control line.  Keep explicitly tagged stop lines and conventionally
            # used ref_line geometry, but avoid treating unrelated helper lines as
            # a stop line for unsupported regulatory elements.
            is_control_ref_line = reg_subtype in {"traffic_light", "right_of_way", "all_way_stop"}
            if source_type != "stop_line" and source_subtype != "stop_line" and not is_control_ref_line:
                continue

            self._copy_source_way_with_nodes(ref)
            copied_way = self._out.find_way_by_id(ref)
            if copied_way is None:
                continue

            if copied_way.tag_dict.get("type") and copied_way.tag_dict.get("type") != "stop_line":
                copied_way.tag_dict.setdefault("lanelet2:source_type", copied_way.tag_dict.get("type"))
            if copied_way.tag_dict.get("subtype") and copied_way.tag_dict.get("subtype") != "stop_line":
                copied_way.tag_dict.setdefault("lanelet2:source_subtype", copied_way.tag_dict.get("subtype"))
            copied_way.tag_dict["type"] = "stop_line"
            copied_way.tag_dict["lanelet2:source_way"] = ref
            copied_way.tag_dict["lanelet2:source_regulatory_elements"] = self._append_csv_value(
                copied_way.tag_dict.get("lanelet2:source_regulatory_elements"), str(reg.id_)
            )
            copied_way.tag_dict["lanelet2:target_road_ways"] = self._append_csv_value(
                copied_way.tag_dict.get("lanelet2:target_road_ways"), str(road_way.id_)
            )
            copied_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level_reference"

            road_way.tag_dict["lanelet2:has_stop_line"] = "yes"
            road_way.tag_dict["lanelet2:stop_line_refs"] = self._append_csv_value(
                road_way.tag_dict.get("lanelet2:stop_line_refs"), ref
            )
            road_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level"

            key = f"{road_way.id_}:{reg.id_}:{ref}"
            if key not in self._preserved_stop_line_refs:
                self._preserved_stop_line_refs.add(key)
                preserved = True

        return preserved

    def _source_lanelet_ids_for_output_way(self, road_way: OSMWay) -> Set[str]:
        ids: List[str] = []
        for key in (
            "lanelet2:all_lanelet_ids",
            "lanelet2:lanelet_ids",
            "lanelet2:start_group_lanelet_ids",
            "lanelet2:end_group_lanelet_ids",
            "lanelet2:representative_lanelet",
            "lanelet2:representative_lanelets",
        ):
            ids.extend(self._parse_id_list(road_way.tag_dict.get(key)))
        return {str(x) for x in ids if str(x).strip()}

    def _regulatory_elements_for_lanelet_ids(self, lanelet_ids: Set[str]) -> List[object]:
        assert self._source is not None
        regulatory_ids: Set[str] = set()
        for lanelet_id in lanelet_ids:
            lanelet = self._source.find_way_rel_by_id(str(lanelet_id))
            if lanelet is None:
                continue
            regulatory_ids.update(str(reg_id) for reg_id in (lanelet.regulatory_elements or []))
        elements = []
        for reg_id in sorted(regulatory_ids, key=self._safe_sort_key):
            reg = self._source.find_regulatory_element_by_id(str(reg_id))
            if reg is not None:
                elements.append(reg)
        return elements

    def _apply_speed_limit_to_road_way(self, road_way: OSMWay, reg: object) -> bool:
        assert self._source is not None
        speed_value = ""
        helper = getattr(self._source, "speed_limit_signs", {}).get(str(reg.id_))
        if helper is not None:
            speed_value = str(helper[0])
        elif "sign_type" in reg.tag_dict:
            speed_value = str(reg.tag_dict["sign_type"])

        road_way.tag_dict["lanelet2:speed_limit_relations"] = self._append_csv_value(
            road_way.tag_dict.get("lanelet2:speed_limit_relations"), str(reg.id_)
        )
        road_way.tag_dict["lanelet2:regulatory_mapping"] = "road_level"
        if not speed_value:
            return False
        road_way.tag_dict["lanelet2:maxspeed"] = speed_value
        standard_speed = self._normalize_osm_maxspeed(speed_value)
        if standard_speed:
            road_way.tag_dict.setdefault("maxspeed", standard_speed)
        return True

    @staticmethod
    def _normalize_osm_maxspeed(value: object) -> str:
        """Convert common Lanelet2 speed strings to standard OSM maxspeed syntax."""
        text = str(value or "").strip().lower().replace(",", ".")
        if not text:
            return ""
        match = re.fullmatch(
            r"([0-9]+(?:\.[0-9]+)?)\s*(km/?h|kmh|kph|mph|m/?s)?", text
        )
        if match is None:
            return ""
        number = float(match.group(1))
        unit = match.group(2) or "km/h"
        if unit in {"m/s", "ms"}:
            number *= 3.6
            unit = "km/h"
        rendered = str(int(round(number))) if abs(number - round(number)) < 1e-9 else f"{number:g}"
        return f"{rendered} mph" if unit == "mph" else rendered

    def _emit_right_of_way_nodes(self, road_way: OSMWay, reg: object, yielding: bool) -> bool:
        emitted = False
        refs = list(getattr(reg, "refers", []) or [])
        if not refs:
            refs = list(getattr(reg, "ref_line", []) or [])
        refs = self._filter_right_of_way_refs_by_role(refs, yielding=yielding)
        if not refs:
            return False
        for ref in refs:
            source_tags = self._source_way_tags(ref)
            sign_subtype = str(source_tags.get("subtype", "")).strip().lower()
            if yielding:
                highway_value = "stop" if "206" in sign_subtype or "stop" in sign_subtype else "give_way"
                base_tags = {
                    "highway": highway_value,
                    "lanelet2:regulatory_role": "yield",
                }
            else:
                base_tags = {
                    "priority_road": "yes",
                    "traffic_sign": "priority_road",
                    "lanelet2:regulatory_role": "right_of_way",
                }
            if sign_subtype:
                base_tags["lanelet2:source_sign_subtype"] = sign_subtype
                base_tags.setdefault("traffic_sign", sign_subtype)
            emitted = self._emit_regulatory_feature_node_for_ref(
                road_way, reg, "right_of_way", str(ref), base_tags
            ) or emitted
        return emitted

    def _filter_right_of_way_refs_by_role(self, refs: List[object], yielding: bool) -> List[object]:
        """Keep only signs whose subtype matches the role represented on this road way."""
        if not refs:
            return []

        matched: List[object] = []
        unknown: List[object] = []
        for ref in refs:
            sign_subtype = str(self._source_way_tags(ref).get("subtype", "")).strip().lower()
            if not sign_subtype:
                unknown.append(ref)
                continue
            is_yield_sign = (
                "205" in sign_subtype
                or "206" in sign_subtype
                or "yield" in sign_subtype
                or "give_way" in sign_subtype
                or "stop" in sign_subtype
            )
            is_priority_sign = (
                "301" in sign_subtype
                or "306" in sign_subtype
                or "priority" in sign_subtype
                or "right_of_way" in sign_subtype
            )
            if yielding and is_yield_sign:
                matched.append(ref)
            elif not yielding and is_priority_sign:
                matched.append(ref)

        if matched:
            return matched
        # Only fall back to unknown signs when none of the refers have a known
        # role-specific subtype.  This avoids creating priority-road nodes from
        # explicit yield signs.
        if unknown and len(unknown) == len(refs):
            return unknown
        return []

    def _emit_regulatory_feature_nodes(
        self, road_way: OSMWay, reg: object, feature_kind: str, base_tags: Dict[str, str]
    ) -> bool:
        refs = list(getattr(reg, "refers", []) or [])
        if not refs:
            refs = list(getattr(reg, "ref_line", []) or [])
        if not refs:
            refs = [""]

        emitted = False
        for ref in refs:
            tags = dict(base_tags)
            source_tags = self._source_way_tags(ref)
            sign_subtype = source_tags.get("subtype")
            if sign_subtype:
                tags["lanelet2:source_sign_subtype"] = str(sign_subtype)
                if feature_kind == "speed_limit":
                    tags.setdefault("traffic_sign", str(sign_subtype))
            if feature_kind == "speed_limit":
                speed = road_way.tag_dict.get("maxspeed") or road_way.tag_dict.get("lanelet2:maxspeed")
                if speed:
                    tags["maxspeed"] = str(speed)
            emitted = self._emit_regulatory_feature_node_for_ref(
                road_way, reg, feature_kind, str(ref), tags
            ) or emitted
        return emitted

    def _emit_regulatory_feature_node_for_ref(
        self,
        road_way: OSMWay,
        reg: object,
        feature_kind: str,
        ref: str,
        base_tags: Dict[str, str],
    ) -> bool:
        assert self._out is not None
        assert self._node_id_gen is not None

        key = (str(road_way.id_), str(reg.id_), str(ref or feature_kind))
        if key in self._emitted_regulatory_feature_keys:
            return False

        lat_lon = self._source_way_midpoint_latlon(ref)
        if lat_lon is None:
            lat_lon = self._output_way_midpoint_latlon(road_way)
        if lat_lon is None:
            return False

        tags = dict(base_tags)
        tags["lanelet2:source_regulatory_element"] = str(reg.id_)
        tags["lanelet2:source_refers_way"] = str(ref) if ref else ""
        tags["lanelet2:target_road_way"] = str(road_way.id_)
        tags["lanelet2:target_lanelets"] = ",".join(
            sorted(self._source_lanelet_ids_for_output_way(road_way), key=self._safe_sort_key)
        )
        tags["lanelet2:regulatory_mapping"] = "road_level"
        self._out.add_node(
            OSMNode(
                id_=self._node_id_gen.next(),
                lat=self._format_float(lat_lon[0]),
                lon=self._format_float(lat_lon[1]),
                ele="0.00000000",
                tag_dict=tags,
            )
        )
        self._emitted_regulatory_feature_keys.add(key)
        return True

    def _source_way_midpoint_latlon(self, way_id: object) -> Optional[Tuple[float, float]]:
        assert self._source is not None
        way = self._source.find_way_by_id(str(way_id))
        if way is None:
            return None
        lat_lon_points: List[Tuple[float, float]] = []
        xy_points: List[Tuple[float, float]] = []
        for node_id in way.nodes:
            node = self._source.find_node_by_id(str(node_id))
            if node is not None:
                lat = self._safe_float(getattr(node, "lat", None), None)
                lon = self._safe_float(getattr(node, "lon", None), None)
                if (
                    lat is not None
                    and lon is not None
                    and math.isfinite(lat)
                    and math.isfinite(lon)
                    and -90.0 <= lat <= 90.0
                    and -180.0 <= lon <= 180.0
                ):
                    lat_lon_points.append((lat, lon))
                else:
                    xy_points.append(self._node_xy(node))
        if lat_lon_points:
            return (
                sum(point[0] for point in lat_lon_points) / len(lat_lon_points),
                sum(point[1] for point in lat_lon_points) / len(lat_lon_points),
            )
        if not xy_points:
            return None
        xy = (
            sum(point[0] for point in xy_points) / len(xy_points),
            sum(point[1] for point in xy_points) / len(xy_points),
        )
        return self._xy_to_latlon_if_needed(xy)

    def _output_way_midpoint_latlon(self, road_way: OSMWay) -> Optional[Tuple[float, float]]:
        assert self._out is not None
        nodes = [
            self._out.find_node_by_id(node_id)
            for node_id in getattr(road_way, "nodes", [])
        ]
        nodes = [node for node in nodes if node is not None]
        if not nodes:
            return None
        mid_node = nodes[len(nodes) // 2]
        return (
            self._safe_float(mid_node.lat),
            self._safe_float(mid_node.lon),
        )

    def _source_way_tags(self, way_id: object) -> Dict[str, str]:
        assert self._source is not None
        way = self._source.find_way_by_id(str(way_id))
        if way is None:
            return {}
        return self._copy_tag_dict(getattr(way, "tag_dict", {}) or {})

    def _road_way_has_lanelet_role(self, road_way: OSMWay, reg: object, role: str) -> bool:
        lanelet_ids = self._source_lanelet_ids_for_output_way(road_way)
        if role == "yield":
            role_ids = set(str(x) for x in (getattr(reg, "yield_ways", []) or []))
        else:
            role_ids = set(str(x) for x in (getattr(reg, "right_of_ways", []) or []))
        return bool(lanelet_ids & role_ids)

    @staticmethod
    def _append_csv_value(existing: Optional[str], value: str) -> str:
        values = []
        if existing:
            values.extend(part.strip() for part in str(existing).split(",") if part.strip())
        if value and value not in values:
            values.append(value)
        return ",".join(values)

    def _convert_multipolygon_to_crossing_way_and_node(self, multipolygon: Multipolygon, source_kind: str) -> bool:
        """Convert an actual ped_crossing polygon to a crossing center way and midpoint node.

        The crossing way is based on a PCA-like principal centerline of the
        polygon, not the farthest vertex pair, so rectangular crossings do not
        become diagonal lines.
        """
        assert self._out is not None
        assert self._node_id_gen is not None
        assert self._way_id_gen is not None

        centerline = self._multipolygon_principal_centerline(multipolygon)
        if centerline is None:
            return False
        start_xy, end_xy = centerline
        start_ll = self._xy_to_latlon_if_needed(start_xy)
        end_ll = self._xy_to_latlon_if_needed(end_xy)
        mid_xy = self._midpoint(start_xy, end_xy)
        mid_ll = self._xy_to_latlon_if_needed(mid_xy)

        start_node_id = self._node_id_gen.next()
        mid_node_id = self._node_id_gen.next()
        end_node_id = self._node_id_gen.next()
        way_id = self._way_id_gen.next()

        self._out.add_node(OSMNode(id_=start_node_id, lat=self._format_float(start_ll[0]), lon=self._format_float(start_ll[1]), ele="0.00000000", tag_dict={}))
        self._out.add_node(
            OSMNode(
                id_=mid_node_id,
                lat=self._format_float(mid_ll[0]),
                lon=self._format_float(mid_ll[1]),
                ele="0.00000000",
                tag_dict={
                    "highway": "crossing",
                    "crossing": "uncontrolled",
                    "lanelet2:source_multipolygon": str(multipolygon.id_),
                    "lanelet2:source_multipolygon_subtype": str(source_kind),
                },
            )
        )
        self._out.add_node(OSMNode(id_=end_node_id, lat=self._format_float(end_ll[0]), lon=self._format_float(end_ll[1]), ele="0.00000000", tag_dict={}))
        self._out.add_way(
            OSMWay(
                id_=way_id,
                nodes=[start_node_id, mid_node_id, end_node_id],
                tag_dict={
                    "highway": "footway",
                    "footway": "crossing",
                    "crossing": "uncontrolled",
                    "lanelet2:source_multipolygon": str(multipolygon.id_),
                    "lanelet2:source_multipolygon_subtype": str(source_kind),
                },
            )
        )
        return True

    def _convert_multipolygon_to_point_feature(self, multipolygon: Multipolygon, tag_dict: Dict[str, str]) -> bool:
        assert self._out is not None
        assert self._node_id_gen is not None

        centroid = self._multipolygon_centroid_xy(multipolygon)
        if centroid is None:
            return False
        lat, lon = self._xy_to_latlon_if_needed(centroid)
        node_tags = dict(tag_dict)
        node_tags["lanelet2:source_multipolygon"] = str(multipolygon.id_)
        self._out.add_node(
            OSMNode(
                id_=self._node_id_gen.next(),
                lat=self._format_float(lat),
                lon=self._format_float(lon),
                ele="0.00000000",
                tag_dict=node_tags,
            )
        )
        return True

    def _multipolygon_outer_points_xy(self, multipolygon: Multipolygon) -> List[Tuple[float, float]]:
        assert self._source is not None
        points: List[Tuple[float, float]] = []
        for outer_way_id in getattr(multipolygon, "outer_list", []) or []:
            way = self._source.find_way_by_id(str(outer_way_id))
            if way is None:
                continue
            for node_id in way.nodes:
                node = self._source.find_node_by_id(str(node_id))
                if node is None:
                    continue
                xy = self._node_xy(node)
                if not points or self._distance(points[-1], xy) > 1e-9:
                    points.append(xy)
        if len(points) >= 2 and self._distance(points[0], points[-1]) < 1e-9:
            points.pop()
        return points

    def _multipolygon_centroid_xy(self, multipolygon: Multipolygon) -> Optional[Tuple[float, float]]:
        points = self._multipolygon_outer_points_xy(multipolygon)
        if not points:
            return None
        return (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))

    def _multipolygon_principal_centerline(self, multipolygon: Multipolygon) -> Optional[Tuple[Tuple[float, float], Tuple[float, float]]]:
        points = self._multipolygon_outer_points_xy(multipolygon)
        if len(points) < 2:
            return None
        centroid = (sum(p[0] for p in points) / len(points), sum(p[1] for p in points) / len(points))

        # 2D PCA principal direction for the polygon point cloud.
        sxx = syy = sxy = 0.0
        for x, y in points:
            dx = x - centroid[0]
            dy = y - centroid[1]
            sxx += dx * dx
            syy += dy * dy
            sxy += dx * dy
        if abs(sxy) < 1e-12 and abs(sxx - syy) < 1e-12:
            # Degenerate/square-ish case: fall back to the longest polygon edge.
            direction = self._longest_edge_direction(points)
        else:
            angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
            direction = (math.cos(angle), math.sin(angle))

        direction = self._unit_vector(direction)
        if direction == (0.0, 0.0):
            return None

        projections = [(p[0] - centroid[0]) * direction[0] + (p[1] - centroid[1]) * direction[1] for p in points]
        min_proj = min(projections)
        max_proj = max(projections)
        if max_proj - min_proj < 1e-6:
            return None
        start_xy = (centroid[0] + direction[0] * min_proj, centroid[1] + direction[1] * min_proj)
        end_xy = (centroid[0] + direction[0] * max_proj, centroid[1] + direction[1] * max_proj)
        return start_xy, end_xy

    def _longest_edge_direction(self, points: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
        if len(points) < 2:
            return (0.0, 0.0)
        best_len = -1.0
        best_vec = (0.0, 0.0)
        for idx in range(len(points)):
            a = points[idx]
            b = points[(idx + 1) % len(points)]
            vec = (b[0] - a[0], b[1] - a[1])
            length = math.hypot(vec[0], vec[1])
            if length > best_len:
                best_len = length
                best_vec = vec
        return best_vec

    def _xy_to_latlon_if_needed(self, xy: Tuple[float, float]) -> Tuple[float, float]:
        """Convert local metric XY back to lat/lon for newly generated auxiliary features."""
        if self._xy_origin_lat is None or self._xy_origin_lon is None:
            self._xy_origin_lat = 0.0
            self._xy_origin_lon = 0.0
        lat = self._xy_origin_lat + xy[1] / self._xy_m_per_deg_lat
        lon = self._xy_origin_lon + xy[0] / self._xy_m_per_deg_lon
        return lat, lon

    # ------------------------------------------------------------------
    # Part 2: lanelet -> road-level centerline way + longitudinal stitching
    # ------------------------------------------------------------------
    def _convert_lanelets_to_road_centerlines(self) -> None:
        assert self._source is not None

        lanelets = list(self._source.way_relations.values())
        if not lanelets:
            return

        main_lanelets, filtered_lanelets = self._split_lanelets_for_centerline_conversion(lanelets)
        if not main_lanelets:
            LOGGER.info("Lanelet2OSMConverter: no lanelets remain after boundary-type filtering.")
            return
        groups = self._build_lateral_lanelet_groups(main_lanelets)
        LOGGER.info("Lanelet2OSMConverter: detected %d lateral lanelet groups after filtering.", len(groups))

        ordered_groups: List[List[WayRelation]] = []
        for group_ids in groups:
            ordered_lanelet_ids = self._order_lateral_lanelet_group(group_ids)
            lanelet_group = [self._source.find_way_rel_by_id(lanelet_id) for lanelet_id in ordered_lanelet_ids]
            lanelet_group = [lanelet for lanelet in lanelet_group if lanelet is not None]
            if lanelet_group:
                ordered_groups.append(lanelet_group)

        segments: List[_CenterlineSegment] = []
        for group_index, lanelet_group in enumerate(ordered_groups, start=1):
            seg = self._build_group_centerline_segment_with_policy(
                lanelet_group=lanelet_group,
                segment_id=f"group-{group_index}",
            )
            if seg is not None:
                segments.append(seg)


        chains = self._build_longitudinal_chains(segments)
        LOGGER.info(
            "Lanelet2OSMConverter: stitched %d centerline segments into %d continuous road chains.",
            len(segments),
            len(chains),
        )
        selected_connectors = self._build_selected_straight_connector_paths(filtered_lanelets, chains)
        LOGGER.info(
            "Lanelet2OSMConverter: selected %d straight junction connector paths from %d filtered lanelets.",
            len(selected_connectors),
            len(filtered_lanelets),
        )

        # Build all output centerline ways first, then resolve road <-> connector
        # joints before serializing.  This keeps road chains and connectors as
        # separate OSM ways, but makes their shared boundaries geometrically and
        # topologically continuous.
        draft_ways = self._build_draft_centerline_ways(chains, selected_connectors)
        self._link_and_smooth_draft_way_joints(draft_ways)

        for draft_way in draft_ways:
            self._write_draft_way_as_osm(draft_way)
        self._convert_regulatory_elements_to_road_level_features()
    
    def _debug_append_lines_to_file(self, lines: Sequence[str]) -> None:
        if not self.debug_output_path:
            return
        try:
            with open(self.debug_output_path, "a", encoding="utf-8") as f:
                for line in lines:
                    f.write(str(line) + "\n")
        except Exception as e:
            LOGGER.warning("Failed to write debug output to %s: %s", self.debug_output_path, e)

    def _debug_dump_lanelets_without_topology_to_file(self) -> None:
        assert self._source is not None

        lines: List[str] = []
        lines.append("========== DEBUG: lanelets without predecessor/successor topology ==========")

        missing_topology_ids: List[str] = []
        for lanelet in self._source.way_relations.values():
            lanelet_id = str(lanelet.id_)
            preds = getattr(lanelet, "predecessors", None) or []
            succs = getattr(lanelet, "successors", None) or []
            if not preds and not succs:
                missing_topology_ids.append(lanelet_id)

        if not missing_topology_ids:
            lines.append("DEBUG: all lanelets have predecessor/successor topology.")
        else:
            lines.append(
                "DEBUG: lanelets without predecessor/successor topology (%d): %s"
                % (
                    len(missing_topology_ids),
                    ",".join(sorted(missing_topology_ids, key=self._safe_sort_key)),
                )
            )

        lines.append("========== DEBUG: lanelets without topology_source info ==========")

        missing_source_ids: List[str] = []
        for lanelet in self._source.way_relations.values():
            lanelet_id = str(lanelet.id_)
            source_info = getattr(lanelet, "topology_source", None)

            if not isinstance(source_info, dict):
                missing_source_ids.append(lanelet_id)
                continue

            if not any(str(v).strip() for v in source_info.values() if v is not None):
                missing_source_ids.append(lanelet_id)

        if not missing_source_ids:
            lines.append("DEBUG: all lanelets have non-empty topology_source info.")
        else:
            lines.append(
                "DEBUG: lanelets without topology_source info (%d): %s"
                % (
                    len(missing_source_ids),
                    ",".join(sorted(missing_source_ids, key=self._safe_sort_key)),
                )
            )

        lines.append("")
        self._debug_append_lines_to_file(lines)

    def _debug_dump_chains_to_file(self, chains: Sequence[Sequence[_CenterlineSegment]]) -> None:
        lines: List[str] = []
        lines.append("========== DEBUG: current chains ==========")

        if not chains:
            lines.append("DEBUG: no chains.")
            lines.append("")
            self._debug_append_lines_to_file(lines)
            return

        for chain_index, chain in enumerate(chains):
            if not chain:
                lines.append("DEBUG: chain[%d] is empty" % chain_index)
                continue

            start_seg = chain[0]
            end_seg = chain[-1]

            start_reps = [str(x) for x in (start_seg.representative_lanelet_ids or [])]
            end_reps = [str(x) for x in (end_seg.representative_lanelet_ids or [])]

            lines.append(
                "DEBUG: chain[%d] start_reps=%s end_reps=%s segment_count=%d"
                % (
                    chain_index,
                    ",".join(start_reps) if start_reps else "NONE",
                    ",".join(end_reps) if end_reps else "NONE",
                    len(chain),
                )
            )

            for seg_index, seg in enumerate(chain):
                lines.append(
                    "DEBUG:   chain[%d].seg[%d] segment_id=%s reps=%s ordered_lanelets=%s policy=%s"
                    % (
                        chain_index,
                        seg_index,
                        str(seg.segment_id),
                        ",".join(str(x) for x in (seg.representative_lanelet_ids or []))
                        if seg.representative_lanelet_ids else "NONE",
                        ",".join(str(x) for x in (seg.ordered_lanelet_ids or seg.lanelet_ids))
                        if (seg.ordered_lanelet_ids or seg.lanelet_ids) else "NONE",
                        str(seg.representative_policy),
                    )
                )

        lines.append("")
        self._debug_append_lines_to_file(lines)

    def _build_group_centerline_segment_with_policy(
        self,
        lanelet_group: Sequence[WayRelation],
        segment_id: str,
    ) -> Optional[_CenterlineSegment]:
        """
        Base policy before inheritance:
        - 1 lanelet  -> directly use its centerline
        - odd count  -> directly use the middle lanelet's centerline
        - even count -> compute midpoint using outer boundaries
        """
        assert self._source is not None

        if not lanelet_group:
            return None

        ordered_ids = [str(x.id_) for x in lanelet_group]
        count = len(lanelet_group)

        if count == 1:
            lanelet = lanelet_group[0]
            seg = self._build_centerline_segment([lanelet], segment_id)
            if seg is not None:
                seg.tags["lanes"] = "1"
                seg.representative_lanelet_ids = [str(lanelet.id_)]
                seg.representative_lanelet_count = 1
                seg.representative_policy = "single_lanelet_direct"
                seg.ordered_lanelet_ids = ordered_ids
                seg.tags["lanelet2:centerline_policy"] = seg.representative_policy
                seg.tags["lanelet2:representative_lanelet"] = str(lanelet.id_)
                seg.tags["lanelet2:lanelet_ids"] = ",".join(ordered_ids)
                self._normalize_centerline_tags(seg.tags)
            return seg

        if count % 2 == 1:
            middle_idx = count // 2
            middle_lanelet = lanelet_group[middle_idx]
            seg = self._build_centerline_segment([middle_lanelet], segment_id)
            if seg is not None:
                tags = self._build_centerline_way_tags(
                    lanelet_group=lanelet_group,
                    outer_pair=(str(middle_lanelet.left_way), str(middle_lanelet.right_way)),
                )
                tags["lanes"] = str(count)
                tags["lanelet2:centerline_policy"] = "odd_lanelets_use_middle_lanelet_centerline"
                tags["lanelet2:representative_lanelet"] = str(middle_lanelet.id_)
                tags["lanelet2:lanelet_ids"] = ",".join(ordered_ids)
                self._normalize_centerline_tags(tags)
                seg.tags = tags
                seg.representative_lanelet_ids = [str(middle_lanelet.id_)]
                seg.representative_lanelet_count = 1
                seg.representative_policy = "odd_lanelets_use_middle_lanelet_centerline"
                seg.ordered_lanelet_ids = ordered_ids
            return seg

        outer_pair = self._find_outer_boundaries(lanelet_group)
        if outer_pair is None:
            LOGGER.warning(
                "Could not determine a unique outer-left/outer-right pair for even-count lanelet group %s. "
                "Falling back to middle-pair lanelets.",
                sorted(str(x.id_) for x in lanelet_group),
            )
            left_idx = count // 2 - 1
            right_idx = count // 2
            pair = [lanelet_group[left_idx], lanelet_group[right_idx]]
            seg = self._build_centerline_segment(pair, segment_id, (str(pair[0].left_way), str(pair[1].right_way)))
            if seg is not None:
                tags = self._build_centerline_way_tags(
                    lanelet_group=lanelet_group,
                    outer_pair=(str(pair[0].left_way), str(pair[1].right_way)),
                )
                tags["lanes"] = str(count)
                tags["lanelet2:centerline_policy"] = "even_lanelets_fallback_middle_pair_midpoint"
                tags["lanelet2:representative_lanelets"] = ",".join(str(x.id_) for x in pair)
                tags["lanelet2:lanelet_ids"] = ",".join(ordered_ids)
                self._normalize_centerline_tags(tags)
                seg.tags = tags
                seg.representative_lanelet_ids = [str(pair[0].id_), str(pair[1].id_)]
                seg.representative_lanelet_count = 2
                seg.representative_policy = "even_lanelets_fallback_middle_pair_midpoint"
                seg.ordered_lanelet_ids = ordered_ids
            return seg

        seg = self._build_centerline_segment(lanelet_group, segment_id, outer_pair)
        if seg is not None:
            left_idx = count // 2 - 1
            right_idx = count // 2
            pair = [lanelet_group[left_idx], lanelet_group[right_idx]]
            seg.tags["lanes"] = str(count)
            seg.tags["lanelet2:centerline_policy"] = "even_lanelets_outer_boundary_midpoint"
            seg.tags["lanelet2:representative_lanelets"] = ",".join(str(x.id_) for x in pair)
            seg.tags["lanelet2:lanelet_ids"] = ",".join(ordered_ids)
            self._normalize_centerline_tags(seg.tags)
            seg.representative_lanelet_ids = [str(pair[0].id_), str(pair[1].id_)]
            seg.representative_lanelet_count = 2
            seg.representative_policy = "even_lanelets_outer_boundary_midpoint"
            seg.ordered_lanelet_ids = ordered_ids
        return seg

    def _split_lanelets_for_centerline_conversion(
        self,
        lanelets: Sequence[WayRelation],
    ) -> Tuple[List[WayRelation], List[WayRelation]]:
        kept: List[WayRelation] = []
        filtered: List[WayRelation] = []
        removed_virtual = 0
        removed_traffic_sign = 0
        removed_missing_boundary = 0

        for lanelet in lanelets:
            keep, reason = self._lanelet_is_valid_for_centerline_conversion(lanelet)
            if keep:
                kept.append(lanelet)
            else:
                filtered.append(lanelet)
                if reason == "has_virtual_boundary":
                    removed_virtual += 1
                elif reason == "traffic_sign_boundary":
                    removed_traffic_sign += 1
                elif reason == "missing_boundary_way":
                    removed_missing_boundary += 1

        LOGGER.info(
            "Lanelet2OSMConverter: boundary-type filtering kept %d / %d lanelets "
            "(filtered virtual=%d, traffic_sign=%d, missing_boundary=%d).",
            len(kept),
            len(lanelets),
            removed_virtual,
            removed_traffic_sign,
            removed_missing_boundary,
        )
        return kept, filtered

    def _lanelet_is_valid_for_centerline_conversion(self, lanelet: WayRelation) -> Tuple[bool, str]:
        assert self._source is not None

        left_way = self._source.find_way_by_id(str(lanelet.left_way))
        right_way = self._source.find_way_by_id(str(lanelet.right_way))
        if left_way is None or right_way is None:
            return False, "missing_boundary_way"

        left_type = self._way_type(left_way)
        right_type = self._way_type(right_way)

        if left_type == "traffic_sign" or right_type == "traffic_sign":
            return False, "traffic_sign_boundary"
        if left_type == "virtual" or right_type == "virtual":
            return False, "has_virtual_boundary"
        return True, "keep"

    def _way_type(self, way: Lanelet2Way) -> str:
        tag_dict = self._copy_tag_dict(getattr(way, "tag_dict", {}) or {})
        return str(tag_dict.get("type", "")).strip().lower()
    def _build_selected_straight_connector_paths(
        self,
        filtered_lanelets: Sequence[WayRelation],
        chains: Sequence[Sequence[_CenterlineSegment]],
    ) -> List[_CenterlineSegment]:
        assert self._source is not None

        if not filtered_lanelets or not chains:
            return []

        terminal_lanelet_segments, terminal_lanelet_to_chain, terminal_roles = self._collect_terminal_lanelet_segments(chains)
        if not terminal_lanelet_segments:
            return []

        chain_terminal_reps = self._collect_chain_terminal_representatives(chains)
        chain_terminal_groups: Dict[Tuple[int, str], List[str]] = {}
        for chain_index, chain in enumerate(chains):
            if not chain:
                continue
            chain_terminal_groups[(chain_index, "start")] = self._segment_full_lanelet_group_ids(chain[0])
            chain_terminal_groups[(chain_index, "end")] = self._segment_full_lanelet_group_ids(chain[-1])

        filtered_map: Dict[str, WayRelation] = {str(l.id_): l for l in filtered_lanelets}

        def endpoint_key(chain_index: int, side: str) -> Tuple[int, str]:
            return (int(chain_index), str(side))

        def tag_connector(seg: _CenterlineSegment, selection_pass: str, angle_deg: float) -> _CenterlineSegment:
            seg.tags["lanelet2:connector_selection_pass"] = selection_pass
            seg.tags["lanelet2:connector_local_angle_deg"] = self._format_float(float(angle_deg))
            return seg

        def lanelet_segment(lanelet_id: str, role: str) -> Optional[_CenterlineSegment]:
            # Representative terminal lanelets already have cached segments.
            cached = terminal_lanelet_segments.get(str(lanelet_id))
            if cached is not None:
                return cached
            lanelet = self._source.find_way_rel_by_id(str(lanelet_id))
            if lanelet is None:
                return None
            return self._build_centerline_segment([lanelet], f"terminal-{role}-{lanelet_id}")

        def collect_best_paths(
            outgoing_ids: Iterable[str],
            incoming_ids: Iterable[str],
            outgoing_to_chain: Dict[str, int],
            incoming_to_chain: Dict[str, int],
            role_label: str,
        ) -> Dict[Tuple[str, str], Tuple[float, _CenterlineSegment, float, int, int]]:
            """Collect best filtered-lanelet paths for the supplied terminal lanelet sets."""
            incoming_set = {str(x) for x in incoming_ids}
            result: Dict[Tuple[str, str], Tuple[float, _CenterlineSegment, float, int, int]] = {}

            for pred_lanelet_id in sorted({str(x) for x in outgoing_ids}, key=self._safe_sort_key):
                pred_lanelet = self._source.find_way_rel_by_id(pred_lanelet_id)
                pred_chain = outgoing_to_chain.get(pred_lanelet_id)
                if pred_lanelet is None or pred_chain is None:
                    continue

                pred_seg = lanelet_segment(pred_lanelet_id, f"{role_label}-out")
                if pred_seg is None:
                    continue

                first_filtered_ids = [
                    str(succ)
                    for succ in self._lanelet_successors(pred_lanelet)
                    if str(succ) in filtered_map
                ]
                if not first_filtered_ids:
                    continue

                queue: List[Tuple[List[str], str]] = [([fid], fid) for fid in first_filtered_ids]
                visited_paths: Set[Tuple[str, ...]] = set()

                while queue:
                    path_ids, current_id = queue.pop(0)
                    path_key = tuple(path_ids)
                    if path_key in visited_paths:
                        continue
                    visited_paths.add(path_key)

                    current_lanelet = filtered_map[current_id]
                    end_terminal_ids = [
                        str(succ)
                        for succ in self._lanelet_successors(current_lanelet)
                        if str(succ) in incoming_set
                    ]
                    if end_terminal_ids:
                        for succ_lanelet_id in end_terminal_ids:
                            succ_chain = incoming_to_chain.get(succ_lanelet_id)
                            if succ_chain is None or succ_chain == pred_chain:
                                continue
                            succ_seg = lanelet_segment(succ_lanelet_id, f"{role_label}-in")
                            if succ_seg is None:
                                continue

                            angle_deg = self._angle_between_vectors_deg(
                                self._segment_out_direction(pred_seg),
                                self._segment_in_direction(succ_seg),
                            )

                            connector_seg = self._build_connector_segment_from_lanelet_path(
                                [filtered_map[lid] for lid in path_ids],
                                pred_lanelet_id,
                                succ_lanelet_id,
                                pred_chain,
                                succ_chain,
                            )
                            if connector_seg is None:
                                continue

                            score = self._polyline_length(connector_seg.xy)
                            key = (pred_lanelet_id, succ_lanelet_id)
                            cand = (score, connector_seg, angle_deg, pred_chain, succ_chain)
                            prev_best = result.get(key)
                            if prev_best is None or score < prev_best[0]:
                                result[key] = cand

                    if len(path_ids) >= self.connector_path_max_hops:
                        continue

                    for succ_id in self._lanelet_successors(current_lanelet):
                        succ_id = str(succ_id)
                        if succ_id not in filtered_map:
                            continue
                        if succ_id in path_ids:
                            continue
                        queue.append((path_ids + [succ_id], succ_id))
            return result

        # ------------------------------------------------------------------
        # Collect original representative -> representative candidates first.
        # This preserves the old geometric/topological candidate-generation logic.
        # ------------------------------------------------------------------
        outgoing_terminal_ids = [
            lanelet_id for lanelet_id, roles in terminal_roles.items() if "end" in roles
        ]
        incoming_terminal_ids = {
            lanelet_id for lanelet_id, roles in terminal_roles.items() if "start" in roles
        }
        all_best_by_lane_pair = collect_best_paths(
            outgoing_ids=outgoing_terminal_ids,
            incoming_ids=incoming_terminal_ids,
            outgoing_to_chain=terminal_lanelet_to_chain,
            incoming_to_chain=terminal_lanelet_to_chain,
            role_label="rep-rep",
        )

        # --------------------------------------------------------------
        # Pass 1: original primary logic: representative -> representative
        # with the original angle threshold.
        # --------------------------------------------------------------
        primary_best_by_lane_pair: Dict[Tuple[str, str], Tuple[float, _CenterlineSegment]] = {
            lane_pair: (score, tag_connector(seg, "primary_rep_to_rep", angle_deg))
            for lane_pair, (score, seg, angle_deg, _pred_chain, _succ_chain) in all_best_by_lane_pair.items()
            if angle_deg <= self.straight_connector_angle_threshold_deg
        }

        primary_connectors: List[_CenterlineSegment] = []
        used_lane_pairs: Set[Tuple[str, str]] = set()
        used_terminal_ids: Set[str] = set()
        used_endpoints: Set[Tuple[int, str]] = set()
        used_chain_pairs: Set[Tuple[int, int]] = set()
        best_single_by_chain_pair: Dict[Tuple[int, int], Tuple[float, _CenterlineSegment]] = {}

        chain_pair_keys: Set[Tuple[int, int]] = set()
        for pred_lanelet_id, succ_lanelet_id in primary_best_by_lane_pair.keys():
            pred_chain = terminal_lanelet_to_chain.get(pred_lanelet_id)
            succ_chain = terminal_lanelet_to_chain.get(succ_lanelet_id)
            if pred_chain is None or succ_chain is None:
                continue
            chain_pair_keys.add((pred_chain, succ_chain))

        for pred_chain, succ_chain in sorted(chain_pair_keys):
            src_reps = list(chain_terminal_reps.get((pred_chain, "end"), []))
            dst_reps = list(chain_terminal_reps.get((succ_chain, "start"), []))
            if not src_reps or not dst_reps:
                continue

            chosen_pairs: Optional[List[Tuple[str, str]]] = None
            if len(src_reps) == 2 or len(dst_reps) == 2:
                pair_options: List[List[Tuple[str, str]]] = []
                if len(src_reps) == 2 and len(dst_reps) == 2:
                    pair_options.append([(src_reps[0], dst_reps[0]), (src_reps[1], dst_reps[1])])
                    pair_options.append([(src_reps[0], dst_reps[1]), (src_reps[1], dst_reps[0])])
                elif len(src_reps) == 2 and len(dst_reps) == 1:
                    pair_options.append([(src_reps[0], dst_reps[0]), (src_reps[1], dst_reps[0])])
                elif len(src_reps) == 1 and len(dst_reps) == 2:
                    pair_options.append([(src_reps[0], dst_reps[0]), (src_reps[0], dst_reps[1])])

                best_option_score: Optional[Tuple[float, float]] = None
                best_option_pairs: Optional[List[Tuple[str, str]]] = None
                for option in pair_options:
                    connectors: List[_CenterlineSegment] = []
                    ok = True
                    for lane_pair in option:
                        cand = primary_best_by_lane_pair.get(lane_pair)
                        if cand is None:
                            ok = False
                            break
                        connectors.append(cand[1])
                    if not ok or len(connectors) != 2:
                        continue
                    score = (
                        self._polyline_length(connectors[0].xy) + self._polyline_length(connectors[1].xy),
                        self._distance(connectors[0].xy[0], connectors[1].xy[0]) + self._distance(connectors[0].xy[-1], connectors[1].xy[-1]),
                    )
                    if best_option_score is None or score < best_option_score:
                        best_option_score = score
                        best_option_pairs = option
                chosen_pairs = best_option_pairs

                if chosen_pairs is not None:
                    connector_a = primary_best_by_lane_pair[chosen_pairs[0]][1]
                    connector_b = primary_best_by_lane_pair[chosen_pairs[1]][1]
                    merged = self._build_mid_connector_from_two_connectors(
                        connector_a,
                        connector_b,
                        pred_chain=pred_chain,
                        succ_chain=succ_chain,
                    )
                    if merged is not None:
                        merged.tags["lanelet2:connector_selection_pass"] = "primary_rep_pair_midpoint"
                        primary_connectors.append(merged)
                        used_lane_pairs.update(chosen_pairs)
                        used_chain_pairs.add((pred_chain, succ_chain))
                        used_endpoints.add(endpoint_key(pred_chain, "end"))
                        used_endpoints.add(endpoint_key(succ_chain, "start"))
                        for src, dst in chosen_pairs:
                            used_terminal_ids.add(str(src))
                            used_terminal_ids.add(str(dst))
                        continue

            for src_rep in src_reps:
                for dst_rep in dst_reps:
                    cand = primary_best_by_lane_pair.get((src_rep, dst_rep))
                    if cand is None:
                        continue
                    key = (pred_chain, succ_chain)
                    prev_best = best_single_by_chain_pair.get(key)
                    if prev_best is None or cand[0] < prev_best[0]:
                        best_single_by_chain_pair[key] = cand

        for chain_pair, (_, seg) in sorted(best_single_by_chain_pair.items()):
            if any(
                (src, dst) in used_lane_pairs
                for src in chain_terminal_reps.get((chain_pair[0], "end"), [])
                for dst in chain_terminal_reps.get((chain_pair[1], "start"), [])
            ):
                continue
            primary_connectors.append(seg)
            used_chain_pairs.add(chain_pair)
            used_endpoints.add(endpoint_key(chain_pair[0], "end"))
            used_endpoints.add(endpoint_key(chain_pair[1], "start"))
            pred_lanelet_id = str(seg.tags.get("lanelet2:connector_from_lanelet", ""))
            succ_lanelet_id = str(seg.tags.get("lanelet2:connector_to_lanelet", ""))
            if pred_lanelet_id and succ_lanelet_id:
                used_lane_pairs.add((pred_lanelet_id, succ_lanelet_id))
                used_terminal_ids.add(pred_lanelet_id)
                used_terminal_ids.add(succ_lanelet_id)

        # --------------------------------------------------------------
        # Pass 2: original relaxed repair pass, but endpoint occupancy is now
        # road-endpoint based.  If one representative of a two-representative
        # endpoint was already connected, the whole road endpoint is occupied.
        # --------------------------------------------------------------
        secondary_grouped_by_pred: Dict[str, List[Tuple[float, float, int, Tuple[int, int], Tuple[str, str], _CenterlineSegment]]] = defaultdict(list)
        for lane_pair, (score, seg, angle_deg, pred_chain, succ_chain) in all_best_by_lane_pair.items():
            pred_lanelet_id, succ_lanelet_id = lane_pair
            chain_pair = (pred_chain, succ_chain)
            if chain_pair in used_chain_pairs:
                continue
            if endpoint_key(pred_chain, "end") in used_endpoints or endpoint_key(succ_chain, "start") in used_endpoints:
                continue
            if pred_lanelet_id in used_terminal_ids or succ_lanelet_id in used_terminal_ids:
                continue
            hops = int(seg.tags.get("lanelet2:connector_hops", "0") or 0)
            tag_connector(seg, "secondary_rep_to_rep_endpoint_repair", angle_deg)
            secondary_grouped_by_pred[pred_lanelet_id].append(
                (angle_deg, score, hops, chain_pair, lane_pair, seg)
            )

        secondary_candidates: List[Tuple[float, float, int, Tuple[int, int], Tuple[str, str], _CenterlineSegment]] = []
        for pred_lanelet_id, candidates in secondary_grouped_by_pred.items():
            candidates.sort(key=lambda item: (item[0], item[1], item[2], self._safe_sort_key(item[4][1])))
            secondary_candidates.append(candidates[0])

        secondary_candidates.sort(
            key=lambda item: (item[0], item[1], item[2], self._safe_sort_key(item[4][0]), self._safe_sort_key(item[4][1]))
        )

        secondary_connectors: List[_CenterlineSegment] = []
        secondary_used_terminals: Set[str] = set()
        secondary_used_chain_pairs: Set[Tuple[int, int]] = set()
        secondary_used_lane_pairs: Set[Tuple[str, str]] = set()

        for angle_deg, score, hops, chain_pair, lane_pair, seg in secondary_candidates:
            pred_lanelet_id, succ_lanelet_id = lane_pair
            pred_chain, succ_chain = chain_pair
            if chain_pair in used_chain_pairs or chain_pair in secondary_used_chain_pairs:
                continue
            if endpoint_key(pred_chain, "end") in used_endpoints or endpoint_key(succ_chain, "start") in used_endpoints:
                continue
            if lane_pair in used_lane_pairs or lane_pair in secondary_used_lane_pairs:
                continue
            if pred_lanelet_id in used_terminal_ids or succ_lanelet_id in used_terminal_ids:
                continue
            if pred_lanelet_id in secondary_used_terminals or succ_lanelet_id in secondary_used_terminals:
                continue

            secondary_connectors.append(seg)
            secondary_used_chain_pairs.add(chain_pair)
            secondary_used_lane_pairs.add(lane_pair)
            secondary_used_terminals.add(pred_lanelet_id)
            secondary_used_terminals.add(succ_lanelet_id)
            used_endpoints.add(endpoint_key(pred_chain, "end"))
            used_endpoints.add(endpoint_key(succ_chain, "start"))

        used_chain_pairs.update(secondary_used_chain_pairs)
        used_lane_pairs.update(secondary_used_lane_pairs)
        used_terminal_ids.update(secondary_used_terminals)

        # --------------------------------------------------------------
        # Pass 3: conservative representative-to-group repair.
        # This pass runs only after original rep->rep primary/secondary.
        # It tries to recover main-line connector paths where one side is not
        # the representative lanelet but still belongs to the same terminal
        # lanelet group.  group->group is intentionally forbidden.
        # --------------------------------------------------------------
        outgoing_group_ids: Set[str] = set()
        incoming_group_ids: Set[str] = set()
        group_out_to_chain: Dict[str, int] = {}
        group_in_to_chain: Dict[str, int] = {}
        for chain_index, chain in enumerate(chains):
            if not chain:
                continue
            if endpoint_key(chain_index, "end") not in used_endpoints:
                for lid in chain_terminal_groups.get((chain_index, "end"), []):
                    lid = str(lid)
                    outgoing_group_ids.add(lid)
                    group_out_to_chain[lid] = chain_index
            if endpoint_key(chain_index, "start") not in used_endpoints:
                for lid in chain_terminal_groups.get((chain_index, "start"), []):
                    lid = str(lid)
                    incoming_group_ids.add(lid)
                    group_in_to_chain[lid] = chain_index

        third_best_by_lane_pair = collect_best_paths(
            outgoing_ids=outgoing_group_ids,
            incoming_ids=incoming_group_ids,
            outgoing_to_chain=group_out_to_chain,
            incoming_to_chain=group_in_to_chain,
            role_label="rep-group-repair",
        )

        rep_out_ids = {str(x) for x in outgoing_terminal_ids}
        rep_in_ids = {str(x) for x in incoming_terminal_ids}
        third_candidates: List[Tuple[int, float, float, int, Tuple[int, int], Tuple[str, str], _CenterlineSegment, str]] = []
        for lane_pair, (score, seg, angle_deg, pred_chain, succ_chain) in third_best_by_lane_pair.items():
            pred_lanelet_id, succ_lanelet_id = lane_pair
            pred_is_rep = pred_lanelet_id in rep_out_ids
            succ_is_rep = succ_lanelet_id in rep_in_ids
            if pred_is_rep and succ_is_rep:
                # Already covered by primary/secondary rep->rep logic.
                continue
            if not (pred_is_rep or succ_is_rep):
                # Explicitly reject group->group.
                continue
            if endpoint_key(pred_chain, "end") in used_endpoints or endpoint_key(succ_chain, "start") in used_endpoints:
                continue
            if angle_deg > self.straight_connector_angle_threshold_deg:
                continue
            relax_label = "third_rep_to_group_repair" if pred_is_rep else "third_group_to_rep_repair"
            relax_penalty = 1
            hops = int(seg.tags.get("lanelet2:connector_hops", "0") or 0)
            tag_connector(seg, relax_label, angle_deg)
            seg.tags["lanelet2:connector_relax_level"] = str(relax_penalty)
            third_candidates.append((relax_penalty, angle_deg, score, hops, (pred_chain, succ_chain), lane_pair, seg, relax_label))

        third_candidates.sort(
            key=lambda item: (item[0], item[1], item[2], item[3], self._safe_sort_key(item[5][0]), self._safe_sort_key(item[5][1]))
        )

        third_connectors: List[_CenterlineSegment] = []
        third_used_chain_pairs: Set[Tuple[int, int]] = set()
        third_used_lane_pairs: Set[Tuple[str, str]] = set()
        for _penalty, _angle, _score, _hops, chain_pair, lane_pair, seg, _label in third_candidates:
            pred_chain, succ_chain = chain_pair
            pred_lanelet_id, succ_lanelet_id = lane_pair
            if chain_pair in used_chain_pairs or chain_pair in third_used_chain_pairs:
                continue
            if endpoint_key(pred_chain, "end") in used_endpoints or endpoint_key(succ_chain, "start") in used_endpoints:
                continue
            if lane_pair in used_lane_pairs or lane_pair in third_used_lane_pairs:
                continue
            third_connectors.append(seg)
            third_used_chain_pairs.add(chain_pair)
            third_used_lane_pairs.add(lane_pair)
            used_endpoints.add(endpoint_key(pred_chain, "end"))
            used_endpoints.add(endpoint_key(succ_chain, "start"))
            used_terminal_ids.add(pred_lanelet_id)
            used_terminal_ids.add(succ_lanelet_id)

        final_connectors = primary_connectors + secondary_connectors + third_connectors
        final_connectors.sort(key=lambda seg: self._safe_sort_key(seg.segment_id))
        return final_connectors

    def _collect_chain_terminal_representatives(
        self,
        chains: Sequence[Sequence[_CenterlineSegment]],
    ) -> Dict[Tuple[int, str], List[str]]:
        chain_terminal_reps: Dict[Tuple[int, str], List[str]] = {}
        for chain_index, chain in enumerate(chains):
            if not chain:
                continue
            chain_terminal_reps[(chain_index, "start")] = [str(x) for x in (chain[0].representative_lanelet_ids or [])]
            chain_terminal_reps[(chain_index, "end")] = [str(x) for x in (chain[-1].representative_lanelet_ids or [])]
        return chain_terminal_reps

    def _build_mid_connector_from_two_connectors(
        self,
        connector_a: _CenterlineSegment,
        connector_b: _CenterlineSegment,
        pred_chain: int,
        succ_chain: int,
    ) -> Optional[_CenterlineSegment]:
        if not connector_a.xy or not connector_b.xy:
            return None

        sample_count = max(
            self.min_centerline_points,
            len(connector_a.xy),
            len(connector_b.xy),
        )
        a_xy = self._resample_polyline(connector_a.xy, sample_count)
        b_xy = self._resample_polyline(connector_b.xy, sample_count)
        a_ll = self._resample_polyline(connector_a.ll, sample_count)
        b_ll = self._resample_polyline(connector_b.ll, sample_count)
        a_ele = self._resample_scalar_series(connector_a.ele, sample_count)
        b_ele = self._resample_scalar_series(connector_b.ele, sample_count)

        mid_xy: List[Tuple[float, float]] = []
        mid_ll: List[Tuple[float, float]] = []
        mid_ele: List[float] = []
        for idx in range(sample_count):
            mid_xy.append(self._midpoint(a_xy[idx], b_xy[idx]))
            mid_ll.append(self._midpoint(a_ll[idx], b_ll[idx]))
            mid_ele.append((a_ele[idx] + b_ele[idx]) / 2.0)

        tags = dict(connector_a.tags)
        tags["lanelet2:centerline_policy"] = "junction_straight_connector_pair_midpoint"
        tags["lanelet2:paired_connector_ids"] = f"{connector_a.segment_id},{connector_b.segment_id}"
        tags["lanelet2:connector_from_chain"] = str(pred_chain)
        tags["lanelet2:connector_to_chain"] = str(succ_chain)
        tags["lanelet2:paired_representative_lanelets"] = ",".join(
            sorted(
                set(connector_a.tags.get("lanelet2:connector_from_lanelet", "").split(","))
                | set(connector_b.tags.get("lanelet2:connector_from_lanelet", "").split(","))
                | set(connector_a.tags.get("lanelet2:connector_to_lanelet", "").split(","))
                | set(connector_b.tags.get("lanelet2:connector_to_lanelet", "").split(",")),
                key=self._safe_sort_key,
            )
        ).strip(",")
        self._normalize_centerline_tags(tags)

        return _CenterlineSegment(
            segment_id=f"mid-{connector_a.segment_id}__{connector_b.segment_id}",
            lanelet_ids=list(dict.fromkeys(connector_a.lanelet_ids + connector_b.lanelet_ids)),
            outer_pair=connector_a.outer_pair,
            left_start_node=connector_a.left_start_node,
            left_end_node=connector_a.left_end_node,
            right_start_node=connector_b.right_start_node,
            right_end_node=connector_b.right_end_node,
            xy=mid_xy,
            ll=mid_ll,
            ele=mid_ele,
            tags=tags,
            representative_lanelet_ids=[],
            representative_lanelet_count=2,
            representative_policy="junction_straight_connector_pair_midpoint",
            ordered_lanelet_ids=list(dict.fromkeys(connector_a.ordered_lanelet_ids + connector_b.ordered_lanelet_ids)),
        )

    def _collect_terminal_lanelet_segments(
        self,
        chains: Sequence[Sequence[_CenterlineSegment]],
    ) -> Tuple[Dict[str, _CenterlineSegment], Dict[str, int], Dict[str, Set[str]]]:
        assert self._source is not None

        terminal_segments: Dict[str, _CenterlineSegment] = {}
        terminal_to_chain: Dict[str, int] = {}
        terminal_roles: Dict[str, Set[str]] = defaultdict(set)

        for chain_index, chain in enumerate(chains):
            if not chain:
                continue
            start_seg = chain[0]
            end_seg = chain[-1]

            for lanelet_id in start_seg.representative_lanelet_ids or []:
                lanelet_id = str(lanelet_id)
                lanelet = self._source.find_way_rel_by_id(lanelet_id)
                if lanelet is None:
                    continue
                lanelet_seg = self._build_centerline_segment([lanelet], f"terminal-start-{lanelet_id}")
                if lanelet_seg is None:
                    continue
                terminal_segments[lanelet_id] = lanelet_seg
                terminal_to_chain[lanelet_id] = chain_index
                terminal_roles[lanelet_id].add("start")

            for lanelet_id in end_seg.representative_lanelet_ids or []:
                lanelet_id = str(lanelet_id)
                lanelet = self._source.find_way_rel_by_id(lanelet_id)
                if lanelet is None:
                    continue
                lanelet_seg = self._build_centerline_segment([lanelet], f"terminal-end-{lanelet_id}")
                if lanelet_seg is None:
                    continue
                terminal_segments[lanelet_id] = lanelet_seg
                terminal_to_chain[lanelet_id] = chain_index
                terminal_roles[lanelet_id].add("end")
        if "8170" in terminal_roles:
            LOGGER.info("DEBUG terminal_roles[8170]=%s", sorted(terminal_roles["8170"]))
        else:
            LOGGER.info("DEBUG terminal_roles[8170]=MISSING")

        return terminal_segments, terminal_to_chain, terminal_roles

    def _lane_pair_is_straight(
        self,
        pred_lanelet_seg: _CenterlineSegment,
        succ_lanelet_seg: _CenterlineSegment,
    ) -> bool:
        pred_dir = self._segment_out_direction(pred_lanelet_seg)
        succ_dir = self._segment_in_direction(succ_lanelet_seg)
        angle = self._angle_between_vectors_deg(pred_dir, succ_dir)
        return angle <= self.straight_connector_angle_threshold_deg

    def _build_connector_segment_from_lanelet_path(
        self,
        lanelet_path: Sequence[WayRelation],
        pred_lanelet_id: str,
        succ_lanelet_id: str,
        pred_chain: int,
        succ_chain: int,
    ) -> Optional[_CenterlineSegment]:
        if not lanelet_path:
            return None

        lanelet_segments: List[_CenterlineSegment] = []
        for idx, lanelet in enumerate(lanelet_path, start=1):
            seg = self._build_centerline_segment(
                [lanelet],
                f"connector-path-{pred_lanelet_id}-{succ_lanelet_id}-{idx}-{lanelet.id_}",
            )
            if seg is None:
                return None
            seg.tags["lanes"] = "1"
            seg.representative_lanelet_ids = [str(lanelet.id_)]
            seg.representative_lanelet_count = 1
            seg.representative_policy = "junction_straight_connector_path_lanelet"
            seg.ordered_lanelet_ids = [str(lanelet.id_)]
            lanelet_segments.append(seg)

        merged_xy, merged_ll, merged_ele = self._merge_chain_geometry(lanelet_segments)
        first_seg = lanelet_segments[0]
        last_seg = lanelet_segments[-1]
        lanelet_ids = [str(l.id_) for l in lanelet_path]
        tags = dict(first_seg.tags)
        tags["lanes"] = "1"
        tags["lanelet2:centerline_policy"] = "junction_straight_connector_path"
        tags["lanelet2:lanelet_ids"] = ",".join(lanelet_ids)
        tags["lanelet2:connector_from_chain"] = str(pred_chain)
        tags["lanelet2:connector_to_chain"] = str(succ_chain)
        tags["lanelet2:connector_from_lanelet"] = str(pred_lanelet_id)
        tags["lanelet2:connector_to_lanelet"] = str(succ_lanelet_id)
        tags["lanelet2:connector_hops"] = str(len(lanelet_path))
        self._normalize_centerline_tags(tags)

        return _CenterlineSegment(
            segment_id=f"straight-connector-{pred_lanelet_id}-{succ_lanelet_id}-{'-'.join(lanelet_ids)}",
            lanelet_ids=lanelet_ids,
            outer_pair=first_seg.outer_pair,
            left_start_node=first_seg.left_start_node,
            left_end_node=last_seg.left_end_node,
            right_start_node=first_seg.right_start_node,
            right_end_node=last_seg.right_end_node,
            xy=merged_xy,
            ll=merged_ll,
            ele=merged_ele,
            tags=tags,
            representative_lanelet_ids=[lanelet_ids[0]],
            representative_lanelet_count=1,
            representative_policy="junction_straight_connector_path",
            ordered_lanelet_ids=lanelet_ids,
        )

    def _polyline_length(self, points: Sequence[Tuple[float, float]]) -> float:
        if len(points) < 2:
            return 0.0
        total = 0.0
        for idx in range(1, len(points)):
            total += self._distance(points[idx - 1], points[idx])
        return total

    def _angle_between_vectors_deg(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        cosine = max(-1.0, min(1.0, self._vector_cosine(a, b)))
        return math.degrees(math.acos(cosine))

    def _build_lateral_lanelet_groups(self, lanelets: Sequence[WayRelation]) -> List[Set[str]]:
        left_index: Dict[str, Set[str]] = defaultdict(set)
        right_index: Dict[str, Set[str]] = defaultdict(set)
        adjacency: Dict[str, Set[str]] = defaultdict(set)

        for lanelet in lanelets:
            left_index[str(lanelet.left_way)].add(str(lanelet.id_))
            right_index[str(lanelet.right_way)].add(str(lanelet.id_))
            adjacency[str(lanelet.id_)]

        shared_boundary_ids = set(left_index.keys()) & set(right_index.keys())
        for boundary_id in shared_boundary_ids:
            left_lanelets = left_index[boundary_id]
            right_lanelets = right_index[boundary_id]
            for left_lanelet_id in left_lanelets:
                for right_lanelet_id in right_lanelets:
                    if left_lanelet_id == right_lanelet_id:
                        continue
                    left_lanelet = self._source.find_way_rel_by_id(left_lanelet_id) if self._source is not None else None
                    right_lanelet = self._source.find_way_rel_by_id(right_lanelet_id) if self._source is not None else None
                    if left_lanelet is None or right_lanelet is None:
                        continue
                    if self._lanelets_laterally_compatible(left_lanelet, right_lanelet):
                        adjacency[left_lanelet_id].add(right_lanelet_id)
                        adjacency[right_lanelet_id].add(left_lanelet_id)

        visited: Set[str] = set()
        components: List[Set[str]] = []
        for lanelet in lanelets:
            start_id = str(lanelet.id_)
            if start_id in visited:
                continue
            stack = [start_id]
            component: Set[str] = set()
            visited.add(start_id)
            while stack:
                current = stack.pop()
                component.add(current)
                for neighbor in adjacency[current]:
                    if neighbor not in visited:
                        visited.add(neighbor)
                        stack.append(neighbor)
            components.append(component)
        return components

    def _order_lateral_lanelet_group(self, group_ids: Set[str]) -> List[str]:
        assert self._source is not None

        group = set(str(x) for x in group_ids)
        if not group:
            return []

        lanelets: Dict[str, WayRelation] = {}
        for lanelet_id in group:
            lanelet = self._source.find_way_rel_by_id(lanelet_id)
            if lanelet is not None:
                lanelets[lanelet_id] = lanelet

        if not lanelets:
            return []

        left_to_id: Dict[str, str] = {}
        right_to_id: Dict[str, str] = {}
        for lanelet_id, lanelet in lanelets.items():
            left_to_id[str(lanelet.left_way)] = lanelet_id
            right_to_id[str(lanelet.right_way)] = lanelet_id

        leftmost_candidates: List[str] = []
        for lanelet_id, lanelet in lanelets.items():
            if str(lanelet.left_way) not in right_to_id:
                leftmost_candidates.append(lanelet_id)

        start_id = sorted(leftmost_candidates or list(lanelets.keys()), key=self._safe_sort_key)[0]

        ordered: List[str] = []
        seen: Set[str] = set()
        current_id: Optional[str] = start_id
        while current_id is not None and current_id not in seen:
            seen.add(current_id)
            ordered.append(current_id)
            current_lanelet = lanelets[current_id]
            next_id = left_to_id.get(str(current_lanelet.right_way))
            if next_id in group and next_id not in seen:
                current_id = next_id
            else:
                current_id = None

        for lanelet_id in sorted(group - set(ordered), key=self._safe_sort_key):
            ordered.append(lanelet_id)
        return ordered

    def _find_outer_boundaries(self, lanelet_group: Sequence[WayRelation]) -> Optional[Tuple[str, str]]:
        side_usage: Dict[str, Set[str]] = defaultdict(set)
        for lanelet in lanelet_group:
            side_usage[str(lanelet.left_way)].add("left")
            side_usage[str(lanelet.right_way)].add("right")

        left_candidates = [way_id for way_id, sides in side_usage.items() if sides == {"left"}]
        right_candidates = [way_id for way_id, sides in side_usage.items() if sides == {"right"}]
        if len(left_candidates) == 1 and len(right_candidates) == 1:
            return left_candidates[0], right_candidates[0]
        return None

    def _build_centerline_segment(
        self,
        lanelet_group: Sequence[WayRelation],
        segment_id: str,
        outer_pair: Optional[Tuple[str, str]] = None,
    ) -> Optional[_CenterlineSegment]:
        assert self._source is not None

        if not lanelet_group:
            return None
        if outer_pair is None:
            outer_pair = (str(lanelet_group[0].left_way), str(lanelet_group[0].right_way))

        left_way = self._source.find_way_by_id(outer_pair[0])
        right_way = self._source.find_way_by_id(outer_pair[1])
        if left_way is None or right_way is None:
            LOGGER.warning("Skipping centerline for %s because outer boundary ways are missing.", segment_id)
            return None

        left_nodes = list(left_way.nodes)
        right_nodes = list(right_way.nodes)
        left_geom = self._build_way_geometry(left_way)
        right_geom = self._build_way_geometry(right_way)
        if len(left_geom["xy"]) < 2 or len(right_geom["xy"]) < 2:
            LOGGER.warning("Skipping centerline for %s because a boundary way has fewer than two valid points.", segment_id)
            return None

        left_xy, left_ll, left_ele, right_xy, right_ll, right_ele, right_reversed = self._align_boundaries(
            left_xy=left_geom["xy"],
            left_ll=left_geom["ll"],
            left_ele=left_geom["ele"],
            right_xy=right_geom["xy"],
            right_ll=right_geom["ll"],
            right_ele=right_geom["ele"],
        )
        if right_reversed:
            right_nodes = list(reversed(right_nodes))

        sample_count = max(self.min_centerline_points, len(left_xy), len(right_xy))
        left_xy_rs = self._resample_polyline(left_xy, sample_count)
        right_xy_rs = self._resample_polyline(right_xy, sample_count)
        left_ll_rs = self._resample_polyline(left_ll, sample_count)
        right_ll_rs = self._resample_polyline(right_ll, sample_count)
        left_ele_rs = self._resample_scalar_series(left_ele, sample_count)
        right_ele_rs = self._resample_scalar_series(right_ele, sample_count)

        center_xy: List[Tuple[float, float]] = []
        center_ll: List[Tuple[float, float]] = []
        center_ele: List[float] = []
        for idx in range(sample_count):
            center_xy.append(self._midpoint(left_xy_rs[idx], right_xy_rs[idx]))
            center_ll.append(self._midpoint(left_ll_rs[idx], right_ll_rs[idx]))
            center_ele.append((left_ele_rs[idx] + right_ele_rs[idx]) / 2.0)

        tags = self._build_centerline_way_tags(lanelet_group, outer_pair)
        return _CenterlineSegment(
            segment_id=segment_id,
            lanelet_ids=[str(x.id_) for x in lanelet_group],
            outer_pair=outer_pair,
            left_start_node=str(left_nodes[0]),
            left_end_node=str(left_nodes[-1]),
            right_start_node=str(right_nodes[0]),
            right_end_node=str(right_nodes[-1]),
            xy=center_xy,
            ll=center_ll,
            ele=center_ele,
            tags=tags,
        )

    def _build_longitudinal_chains(self, segments: Sequence[_CenterlineSegment]) -> List[List[_CenterlineSegment]]:
        if not segments:
            return []

        lanelet_to_segment: Dict[str, int] = {}
        for idx, seg in enumerate(segments):
            for lanelet_id in seg.ordered_lanelet_ids or seg.lanelet_ids:
                lanelet_to_segment[lanelet_id] = idx

        successors: Dict[int, List[int]] = defaultdict(list)
        predecessors: Dict[int, List[int]] = defaultdict(list)

        for idx, seg in enumerate(segments):
            explicit_next: Set[int] = set()
            for lanelet_id in seg.ordered_lanelet_ids or seg.lanelet_ids:
                lanelet = self._source.find_way_rel_by_id(lanelet_id) if self._source is not None else None
                if lanelet is None:
                    continue
                for succ in self._lanelet_successors(lanelet):
                    succ_idx = lanelet_to_segment.get(str(succ))
                    if succ_idx is not None and succ_idx != idx:
                        explicit_next.add(succ_idx)
                for pred in self._lanelet_predecessors(lanelet):
                    pred_idx = lanelet_to_segment.get(str(pred))
                    if pred_idx is not None and pred_idx != idx:
                        predecessors[idx].append(pred_idx)
                        successors[pred_idx].append(idx)

            for succ_idx in explicit_next:
                successors[idx].append(succ_idx)
                predecessors[succ_idx].append(idx)

        start_index: Dict[Tuple[str, str], List[int]] = defaultdict(list)
        for idx, seg in enumerate(segments):
            start_index[seg.start_signature].append(idx)

        for idx, seg in enumerate(segments):
            if successors.get(idx):
                continue
            for nxt in start_index.get(seg.end_signature, []):
                if idx == nxt:
                    continue
                if self._segments_compatible(seg, segments[nxt]):
                    successors[idx].append(nxt)
                    predecessors[nxt].append(idx)

        for idx in list(successors.keys()):
            successors[idx] = sorted(set(successors[idx]))
        for idx in list(predecessors.keys()):
            predecessors[idx] = sorted(set(predecessors[idx]))

        chains: List[List[_CenterlineSegment]] = []
        used: Set[int] = set()
        starts = [idx for idx in range(len(segments)) if len(predecessors.get(idx, [])) == 0]
        order = starts + [idx for idx in range(len(segments)) if idx not in starts]

        for start_idx in order:
            if start_idx in used:
                continue
            chain = [segments[start_idx]]
            used.add(start_idx)
            current = start_idx
            while True:
                next_candidates = [n for n in successors.get(current, []) if n not in used]
                if not next_candidates:
                    break
                chosen = self._choose_best_successor(current, next_candidates, segments)
                if chosen is None:
                    break
                used.add(chosen)
                chain.append(segments[chosen])
                current = chosen
            chains.append(chain)
        return chains

    def _choose_best_successor(
        self,
        current_idx: int,
        next_candidates: Sequence[int],
        segments: Sequence[_CenterlineSegment],
    ) -> Optional[int]:
        current = segments[current_idx]
        best_idx: Optional[int] = None
        best_score: Optional[Tuple[float, float, float, float]] = None
        current_lane_count = int(current.tags.get("lanes", "1"))
        current_dir = self._segment_out_direction(current)

        for nxt_idx in next_candidates:
            nxt = segments[nxt_idx]
            if not self._segments_compatible(current, nxt):
                continue
            lane_delta = abs(current_lane_count - int(nxt.tags.get("lanes", "1")))
            dist = self._distance(current.xy[-1], nxt.xy[0])
            cosine = self._vector_cosine(current_dir, self._segment_in_direction(nxt))
            topology_score = self._segment_topology_score(current, nxt)
            lateral_shift = self._lateral_shift_between_segments(current, nxt)
            score = (-topology_score, lane_delta, lateral_shift + dist, -cosine)
            if best_score is None or score < best_score:
                best_score = score
                best_idx = nxt_idx
        return best_idx

    def _segments_compatible(self, a: _CenterlineSegment, b: _CenterlineSegment) -> bool:
        a_lanes = int(a.tags.get("lanes", "1"))
        b_lanes = int(b.tags.get("lanes", "1"))
        lane_delta = abs(a_lanes - b_lanes)
        dist = self._distance(a.xy[-1], b.xy[0])
        cosine = self._vector_cosine(self._segment_out_direction(a), self._segment_in_direction(b))
        lateral_shift = self._lateral_shift_between_segments(a, b)
        topology_score = self._segment_topology_score(a, b)

        if lane_delta > 1:
            return False
        if cosine < self.direction_cosine_threshold:
            return False

        max_dist = self.longitudinal_join_distance
        if topology_score > 0:
            max_dist = self.bridge_join_distance
        if dist > max_dist:
            return False
        if topology_score > 0 and lateral_shift > self.bridge_lateral_shift_tolerance:
            return False
        if topology_score <= 0 and dist > self.longitudinal_join_distance:
            return False
        return True

    # ------------------------------------------------------------------
    # Draft way joint linking / smoothing
    # ------------------------------------------------------------------
    def _build_draft_centerline_ways(
        self,
        chains: Sequence[Sequence[_CenterlineSegment]],
        connectors: Sequence[_CenterlineSegment],
    ) -> List[_DraftCenterlineWay]:
        """Create road and connector draft ways without immediately writing nodes.

        This replaces the old write-as-you-go behavior.  The goal is to keep
        each road chain / connector as a separate OSM way while allowing a
        later pass to share boundary nodes and smooth the local joint geometry.
        """
        draft_ways: List[_DraftCenterlineWay] = []

        for chain_index, chain in enumerate(chains):
            if not chain:
                continue
            xy, ll, ele = self._merge_chain_geometry(chain)
            if len(xy) < 2 or len(ll) < 2:
                continue

            tags = dict(chain[0].tags)
            tags["name"] = f"road-{chain_index + 1}"
            tags["lanelet2:draft_kind"] = "road"
            tags["lanelet2:chain_index"] = str(chain_index)
            tags["lanelet2:joint_smoothing"] = "candidate"

            start_lanelet_ids = self._segment_endpoint_lanelet_ids(chain[0])
            end_lanelet_ids = self._segment_endpoint_lanelet_ids(chain[-1])
            start_group_lanelet_ids = self._segment_full_lanelet_group_ids(chain[0])
            end_group_lanelet_ids = self._segment_full_lanelet_group_ids(chain[-1])
            all_lanelet_ids = list(dict.fromkeys(
                lid
                for seg in chain
                for lid in self._segment_full_lanelet_group_ids(seg)
            ))
            tags["lanelet2:start_lanelet_ids"] = ",".join(start_lanelet_ids)
            tags["lanelet2:end_lanelet_ids"] = ",".join(end_lanelet_ids)
            tags["lanelet2:start_group_lanelet_ids"] = ",".join(start_group_lanelet_ids)
            tags["lanelet2:end_group_lanelet_ids"] = ",".join(end_group_lanelet_ids)
            tags["lanelet2:all_lanelet_ids"] = ",".join(all_lanelet_ids)
            tags["lanelet2:connector_start_lanelet"] = ",".join(start_lanelet_ids)
            tags["lanelet2:connector_end_lanelet"] = ",".join(end_lanelet_ids)
            tags["lanelet2:start_representative_lanelets"] = ",".join(start_lanelet_ids)
            tags["lanelet2:end_representative_lanelets"] = ",".join(end_lanelet_ids)

            draft_ways.append(
                _DraftCenterlineWay(
                    draft_id=f"road-{chain_index + 1}",
                    kind="road",
                    xy=list(xy),
                    ll=list(ll),
                    ele=list(ele),
                    tags=tags,
                    chain_index=chain_index,
                    start_lanelet_ids=start_lanelet_ids,
                    end_lanelet_ids=end_lanelet_ids,
                    start_group_lanelet_ids=start_group_lanelet_ids,
                    end_group_lanelet_ids=end_group_lanelet_ids,
                    all_lanelet_ids=all_lanelet_ids,
                )
            )

        for connector_index, connector in enumerate(connectors):
            xy, ll, ele = self._merge_chain_geometry([connector])
            if len(xy) < 2 or len(ll) < 2:
                continue

            tags = dict(connector.tags)
            tags["name"] = f"junction-straight-connector-{connector_index + 1}"
            tags["lanelet2:draft_kind"] = "connector"
            tags["lanelet2:joint_smoothing"] = "candidate"

            connector_lanelet_ids = self._parse_id_list(tags.get("lanelet2:lanelet_ids"))
            start_lanelet_ids = connector_lanelet_ids[:1]
            end_lanelet_ids = connector_lanelet_ids[-1:]
            if not start_lanelet_ids and tags.get("lanelet2:connector_from_lanelet"):
                start_lanelet_ids = [str(tags.get("lanelet2:connector_from_lanelet"))]
            if not end_lanelet_ids and tags.get("lanelet2:connector_to_lanelet"):
                end_lanelet_ids = [str(tags.get("lanelet2:connector_to_lanelet"))]
            connector_group_lanelet_ids = list(dict.fromkeys(str(x) for x in connector_lanelet_ids if str(x).strip()))
            tags["lanelet2:start_lanelet_ids"] = ",".join(start_lanelet_ids)
            tags["lanelet2:end_lanelet_ids"] = ",".join(end_lanelet_ids)
            tags["lanelet2:start_group_lanelet_ids"] = ",".join(connector_group_lanelet_ids or start_lanelet_ids)
            tags["lanelet2:end_group_lanelet_ids"] = ",".join(connector_group_lanelet_ids or end_lanelet_ids)
            tags["lanelet2:all_lanelet_ids"] = ",".join(connector_group_lanelet_ids or list(dict.fromkeys(start_lanelet_ids + end_lanelet_ids)))

            draft_ways.append(
                _DraftCenterlineWay(
                    draft_id=f"junction-straight-connector-{connector_index + 1}",
                    kind="connector",
                    xy=list(xy),
                    ll=list(ll),
                    ele=list(ele),
                    tags=tags,
                    from_chain=self._safe_int_or_none(tags.get("lanelet2:connector_from_chain")),
                    to_chain=self._safe_int_or_none(tags.get("lanelet2:connector_to_chain")),
                    from_lanelet=tags.get("lanelet2:connector_from_lanelet"),
                    to_lanelet=tags.get("lanelet2:connector_to_lanelet"),
                    start_lanelet_ids=start_lanelet_ids,
                    end_lanelet_ids=end_lanelet_ids,
                    start_group_lanelet_ids=connector_group_lanelet_ids or start_lanelet_ids,
                    end_group_lanelet_ids=connector_group_lanelet_ids or end_lanelet_ids,
                    all_lanelet_ids=connector_group_lanelet_ids or list(dict.fromkeys(start_lanelet_ids + end_lanelet_ids)),
                )
            )

        return draft_ways

    @staticmethod
    def _parse_id_list(value: object) -> List[str]:
        if value is None:
            return []
        if isinstance(value, (list, tuple, set)):
            return [str(x).strip() for x in value if str(x).strip()]
        return [part.strip() for part in str(value).split(",") if part.strip()]

    def _segment_endpoint_lanelet_ids(self, seg: _CenterlineSegment) -> List[str]:
        """Return only representative endpoint lanelets for topology sharing.

        This intentionally does NOT fall back to ``ordered_lanelet_ids`` or
        ``lanelet_ids``.  A road-level centerline may represent multiple
        lateral lanelets, but topology-based no-smoothing endpoint sharing
        should only use the representative lanelet(s):
        - odd lane count: the middle lanelet;
        - even lane count: the two middle lanelets.

        If no representative lanelet exists, return an empty list so the
        conservative no-smoothing fast path will not trigger; the joint can
        still be handled by the normal pairwise smoothing path.
        """
        ids = [str(x).strip() for x in (seg.representative_lanelet_ids or []) if str(x).strip()]
        return list(dict.fromkeys(ids))

    def _segment_full_lanelet_group_ids(self, seg: _CenterlineSegment) -> List[str]:
        """Return the complete lateral lanelet group represented by a centerline segment."""
        ids = seg.ordered_lanelet_ids or seg.lanelet_ids or seg.representative_lanelet_ids or []
        return list(dict.fromkeys(str(x).strip() for x in ids if str(x).strip()))

    def _draft_endpoint_lanelet_ids(self, way: _DraftCenterlineWay, side: str) -> List[str]:
        if side == "start":
            return list(dict.fromkeys(str(x) for x in way.start_lanelet_ids if str(x).strip()))
        if side == "end":
            return list(dict.fromkeys(str(x) for x in way.end_lanelet_ids if str(x).strip()))
        return []

    def _draft_way_lanelets_topologically_connected(
        self,
        a: _DraftCenterlineWay,
        a_side: str,
        b: _DraftCenterlineWay,
        b_side: str,
    ) -> bool:
        """Return True when representative endpoint lanelets encode a -> b topology.

        This intentionally checks only the robust forward pattern used by the
        generated draft joints: ``a.end -> b.start``.  For the no-smoothing
        topology-shared fast path, the representative lanelet counts must also
        match.  In other words, 1 representative may only match 1 representative,
        and 2 representatives may only match 2 representatives.  A 1-to-2 or
        2-to-1 topology relation is treated as not eligible for the fast path so
        it can fall back to normal pairwise smoothing.
        """
        assert self._source is not None
        if a_side != "end" or b_side != "start":
            return False

        a_lanelet_ids = self._draft_endpoint_lanelet_ids(a, a_side)
        b_lanelet_ids = self._draft_endpoint_lanelet_ids(b, b_side)
        if not a_lanelet_ids or not b_lanelet_ids:
            return False

        # Strict representative-count constraint: avoid treating one-to-two or
        # two-to-one representative connections as already smooth/centered.
        if len(a_lanelet_ids) != len(b_lanelet_ids):
            return False

        def pair_connected(src_id: str, dst_id: str) -> bool:
            src_lanelet = self._source.find_way_rel_by_id(str(src_id))
            dst_lanelet = self._source.find_way_rel_by_id(str(dst_id))
            if src_lanelet is None or dst_lanelet is None:
                return False
            src_succs = set(str(x) for x in self._lanelet_successors(src_lanelet))
            dst_preds = set(str(x) for x in self._lanelet_predecessors(dst_lanelet))
            return str(dst_id) in src_succs or str(src_id) in dst_preds

        if len(a_lanelet_ids) == 1:
            return pair_connected(a_lanelet_ids[0], b_lanelet_ids[0])

        if len(a_lanelet_ids) == 2:
            # Preserve robustness to lateral ordering: accept either direct or
            # crossed pairing, but require both representatives to connect.
            direct = pair_connected(a_lanelet_ids[0], b_lanelet_ids[0]) and pair_connected(a_lanelet_ids[1], b_lanelet_ids[1])
            crossed = pair_connected(a_lanelet_ids[0], b_lanelet_ids[1]) and pair_connected(a_lanelet_ids[1], b_lanelet_ids[0])
            return direct or crossed

        # Defensive fallback for unexpected counts: require same-order all-pair
        # connectivity.  In normal use representative counts should be 1 or 2.
        return all(pair_connected(src_id, dst_id) for src_id, dst_id in zip(a_lanelet_ids, b_lanelet_ids))

    def _share_topological_endpoint_if_already_connected(
        self,
        a: _DraftCenterlineWay,
        a_side: str,
        b: _DraftCenterlineWay,
        b_side: str,
        gap: Optional[float] = None,
    ) -> bool:
        """Share one OSM node when topology says the endpoints are already connected.

        This is the conservative no-smoothing fast path.  It is used only when
        the lanelet topology explicitly connects the endpoint lanelets and the
        generated centerline endpoints are already nearly coincident.
        """
        assert self._out is not None
        assert self._node_id_gen is not None

        if a_side != "end" or b_side != "start":
            return False
        if len(a.xy) < 2 or len(b.xy) < 2 or len(a.ll) < 2 or len(b.ll) < 2:
            return False
        if not self._draft_way_lanelets_topologically_connected(a, a_side, b, b_side):
            return False

        if gap is None:
            gap = self._distance(a.xy[-1], b.xy[0])
        if gap > self.topology_endpoint_snap_tolerance:
            return False

        joint_xy = self._midpoint(a.xy[-1], b.xy[0])
        joint_ll = self._midpoint(a.ll[-1], b.ll[0])
        joint_ele = 0.5 * (float(a.ele[-1]) + float(b.ele[0]))

        a.xy[-1] = joint_xy
        b.xy[0] = joint_xy
        a.ll[-1] = joint_ll
        b.ll[0] = joint_ll
        a.ele[-1] = joint_ele
        b.ele[0] = joint_ele

        shared_node_id = a.end_shared_node_id or b.start_shared_node_id or self._node_id_gen.next()
        a.end_shared_node_id = shared_node_id
        b.start_shared_node_id = shared_node_id

        if self._out.find_node_by_id(shared_node_id) is None:
            self._out.add_node(
                OSMNode(
                    id_=shared_node_id,
                    lat=self._format_float(joint_ll[0]),
                    lon=self._format_float(joint_ll[1]),
                    ele=self._format_float(joint_ele),
                    tag_dict={
                        "lanelet2:shared_centerline_joint": "yes",
                        "lanelet2:topology_shared_endpoint": "yes",
                    },
                )
            )

        gap_text = self._format_float(gap)
        a.tags["lanelet2:joint_smoothing"] = "topology_shared_endpoint"
        b.tags["lanelet2:joint_smoothing"] = "topology_shared_endpoint"
        a.tags["lanelet2:topology_endpoint_gap"] = gap_text
        b.tags["lanelet2:topology_endpoint_gap"] = gap_text
        return True

    def _debug_lanelet_topology_summary(self, lanelet_id: str) -> str:
        assert self._source is not None
        lanelet_id = str(lanelet_id)
        lanelet = self._source.find_way_rel_by_id(lanelet_id)
        if lanelet is None:
            return f"lanelet {lanelet_id}: NOT_FOUND"
        preds = [str(x) for x in (getattr(lanelet, "predecessors", None) or [])]
        succs = [str(x) for x in (getattr(lanelet, "successors", None) or [])]
        topology_source = getattr(lanelet, "topology_source", None)
        return (
            f"lanelet {lanelet_id}: "
            f"predecessors={','.join(preds) if preds else 'NONE'}; "
            f"successors={','.join(succs) if succs else 'NONE'}; "
            f"topology_source={topology_source if topology_source is not None else 'NONE'}"
        )

    def _debug_way_summary(self, way: Optional[_DraftCenterlineWay]) -> List[str]:
        if way is None:
            return ["NOT_FOUND"]
        lines = [
            f"draft_id={way.draft_id}",
            f"kind={way.kind}",
            f"chain_index={way.chain_index}",
            f"from_chain={way.from_chain}",
            f"to_chain={way.to_chain}",
            f"from_lanelet={way.from_lanelet}",
            f"to_lanelet={way.to_lanelet}",
            f"start_lanelet_ids={','.join(way.start_lanelet_ids) if way.start_lanelet_ids else 'NONE'}",
            f"end_lanelet_ids={','.join(way.end_lanelet_ids) if way.end_lanelet_ids else 'NONE'}",
            f"tag_lanelet_ids={way.tags.get('lanelet2:lanelet_ids', 'NONE')}",
            f"tag_representative_lanelet={way.tags.get('lanelet2:representative_lanelet', 'NONE')}",
            f"tag_representative_lanelets={way.tags.get('lanelet2:representative_lanelets', 'NONE')}",
            f"tag_centerline_policy={way.tags.get('lanelet2:centerline_policy', 'NONE')}",
            f"tag_connector_from_lanelet={way.tags.get('lanelet2:connector_from_lanelet', 'NONE')}",
            f"tag_connector_to_lanelet={way.tags.get('lanelet2:connector_to_lanelet', 'NONE')}",
            f"tag_connector_hops={way.tags.get('lanelet2:connector_hops', 'NONE')}",
            f"point_count={len(way.xy)}",
        ]
        return lines

    def _debug_evaluate_draft_joint(
        self,
        a: _DraftCenterlineWay,
        a_side: str,
        b: _DraftCenterlineWay,
        b_side: str,
    ) -> Dict[str, object]:
        result: Dict[str, object] = {
            "a": a.draft_id,
            "a_side": a_side,
            "b": b.draft_id,
            "b_side": b_side,
            "valid_forward_pattern": a_side == "end" and b_side == "start",
            "a_endpoint_lanelets": self._draft_endpoint_lanelet_ids(a, a_side),
            "b_endpoint_lanelets": self._draft_endpoint_lanelet_ids(b, b_side),
            "topology_connected": False,
            "gap": None,
            "gap_le_topology_snap": False,
            "gap_le_bridge_join": False,
            "cosine": None,
            "angle_deg": None,
            "lateral_shift": None,
            "first_class_candidate": False,
            "first_class_skip_reason": "not_evaluated",
        }
        if a_side != "end" or b_side != "start":
            result["first_class_skip_reason"] = "not_a_end_to_b_start"
            return result
        if len(a.xy) < 2 or len(b.xy) < 2 or len(a.ll) < 2 or len(b.ll) < 2:
            result["first_class_skip_reason"] = "not_enough_points"
            return result

        topology_connected = self._draft_way_lanelets_topologically_connected(a, a_side, b, b_side)
        result["topology_connected"] = topology_connected
        gap = self._distance(a.xy[-1], b.xy[0])
        result["gap"] = gap
        result["gap_le_topology_snap"] = gap <= self.topology_endpoint_snap_tolerance
        result["gap_le_bridge_join"] = gap <= self.bridge_join_distance
        try:
            cosine = self._vector_cosine(self._draft_end_direction(a), self._draft_start_direction(b))
            result["cosine"] = cosine
        except Exception:
            pass
        try:
            span = min(self.joint_tangent_span, len(a.xy) - 1, len(b.xy) - 1)
            if span >= 1:
                angle_deg = self._angle_between_vectors_deg(
                    self._draft_end_direction_window(a, span),
                    self._draft_start_direction_window(b, span),
                )
                result["angle_deg"] = angle_deg
        except Exception:
            pass
        try:
            result["lateral_shift"] = self._lateral_shift_between_draft_ends(a, b)
        except Exception:
            pass

        if not topology_connected:
            a_ids = result.get("a_endpoint_lanelets") or []
            b_ids = result.get("b_endpoint_lanelets") or []
            if a_ids and b_ids and len(a_ids) != len(b_ids):
                result["first_class_skip_reason"] = (
                    f"representative_lanelet_count_mismatch({len(a_ids)}!={len(b_ids)})"
                )
            else:
                result["first_class_skip_reason"] = "representative_lanelets_not_topologically_connected"
        elif gap > self.topology_endpoint_snap_tolerance:
            result["first_class_skip_reason"] = (
                f"gap_gt_topology_endpoint_snap_tolerance({self._format_float(gap)}>{self._format_float(self.topology_endpoint_snap_tolerance)})"
            )
        else:
            result["first_class_candidate"] = True
            result["first_class_skip_reason"] = "NONE"
        return result

    @staticmethod
    def _debug_format_value(value: object) -> str:
        if isinstance(value, float):
            return f"{value:.6f}"
        if isinstance(value, list):
            return ",".join(str(x) for x in value) if value else "NONE"
        return str(value)

    def _debug_describe_pair_lanelet_topology(
        self,
        a: _DraftCenterlineWay,
        a_side: str,
        b: _DraftCenterlineWay,
        b_side: str,
    ) -> List[str]:
        lines: List[str] = []
        a_ids = self._draft_endpoint_lanelet_ids(a, a_side)
        b_ids = self._draft_endpoint_lanelet_ids(b, b_side)
        lines.append(
            f"lanelet_pair_check {a.draft_id}.{a_side}({','.join(a_ids) if a_ids else 'NONE'}) -> "
            f"{b.draft_id}.{b_side}({','.join(b_ids) if b_ids else 'NONE'})"
        )
        if not a_ids or not b_ids:
            lines.append("  no endpoint lanelets to compare")
            return lines
        if len(a_ids) != len(b_ids):
            lines.append(
                f"  representative_count_mismatch: {len(a_ids)} vs {len(b_ids)}; "
                "not eligible for CLASS_1 topology-shared no-smoothing fast path"
            )
        assert self._source is not None
        for a_id in a_ids:
            a_lanelet = self._source.find_way_rel_by_id(str(a_id))
            a_succs = set(self._lanelet_successors(a_lanelet)) if a_lanelet is not None else set()
            for b_id in b_ids:
                b_lanelet = self._source.find_way_rel_by_id(str(b_id))
                b_preds = set(self._lanelet_predecessors(b_lanelet)) if b_lanelet is not None else set()
                a_to_b = str(b_id) in a_succs
                b_has_a_pred = str(a_id) in b_preds
                lines.append(
                    f"  pair {a_id}->{b_id}: "
                    f"b_in_a_successors={a_to_b}; a_in_b_predecessors={b_has_a_pred}"
                )
        return lines

    def _debug_dump_target_road_connector_pair(
        self,
        draft_ways: Sequence[_DraftCenterlineWay],
        joints: Sequence[Tuple[_DraftCenterlineWay, str, _DraftCenterlineWay, str]],
        joint_categories: Dict[Tuple[str, str, str, str], str],
    ) -> None:
        target_road_id = "road-290"
        target_connector_id = "junction-straight-connector-104"
        by_id = {way.draft_id: way for way in draft_ways}
        road = by_id.get(target_road_id)
        connector = by_id.get(target_connector_id)

        lines: List[str] = []
        lines.append("========== DEBUG TARGET ROAD / CONNECTOR ==========")
        lines.append(f"target_road={target_road_id}")
        lines.append(f"target_connector={target_connector_id}")
        lines.append("")

        lines.append("--- ROAD DRAFT WAY REPRESENTATIVE LANELET RETRIEVAL ---")
        for line in self._debug_way_summary(road):
            lines.append("road: " + line)
        if road is not None:
            for lanelet_id in list(dict.fromkeys(road.start_lanelet_ids + road.end_lanelet_ids)):
                lines.append("road_lanelet_topology: " + self._debug_lanelet_topology_summary(lanelet_id))
        lines.append("")

        lines.append("--- CONNECTOR DRAFT WAY REPRESENTATIVE LANELET RETRIEVAL ---")
        for line in self._debug_way_summary(connector):
            lines.append("connector: " + line)
        if connector is not None:
            for lanelet_id in list(dict.fromkeys(connector.start_lanelet_ids + connector.end_lanelet_ids)):
                lines.append("connector_lanelet_topology: " + self._debug_lanelet_topology_summary(lanelet_id))
        lines.append("")

        if road is None or connector is None:
            lines.append("RESULT: road or connector not found in draft_ways; cannot classify pair.")
            lines.append("========== END DEBUG TARGET ROAD / CONNECTOR ==========")
            self._debug_append_lines_to_file(lines)
            for line in lines:
                LOGGER.info(line)
            return

        joint_keys = {(a.draft_id, a_side, b.draft_id, b_side) for a, a_side, b, b_side in joints}
        lines.append("--- TARGET JOINT CLASSIFICATION ---")
        for a, a_side, b, b_side in [
            (road, "end", connector, "start"),
            (connector, "end", road, "start"),
        ]:
            key = (a.draft_id, a_side, b.draft_id, b_side)
            lines.append(f"orientation={a.draft_id}.{a_side} -> {b.draft_id}.{b_side}")
            lines.append(f"  present_in_actual_joints={key in joint_keys}")
            lines.append(f"  actual_category={joint_categories.get(key, 'NOT_PROCESSED_AS_A_JOINT')}")
            evaluation = self._debug_evaluate_draft_joint(a, a_side, b, b_side)
            for k in [
                "a_endpoint_lanelets",
                "b_endpoint_lanelets",
                "topology_connected",
                "gap",
                "gap_le_topology_snap",
                "gap_le_bridge_join",
                "cosine",
                "angle_deg",
                "lateral_shift",
                "first_class_candidate",
                "first_class_skip_reason",
            ]:
                lines.append(f"  {k}={self._debug_format_value(evaluation.get(k))}")
            lines.extend("  " + x for x in self._debug_describe_pair_lanelet_topology(a, a_side, b, b_side))
            lines.append("")

        lines.append("--- ALL ACTUAL JOINTS INVOLVING TARGET ROAD OR CONNECTOR ---")
        found_any = False
        for a, a_side, b, b_side in joints:
            if a.draft_id in {target_road_id, target_connector_id} or b.draft_id in {target_road_id, target_connector_id}:
                found_any = True
                key = (a.draft_id, a_side, b.draft_id, b_side)
                lines.append(
                    f"joint={a.draft_id}.{a_side}->{b.draft_id}.{b_side}; "
                    f"category={joint_categories.get(key, 'UNKNOWN')}"
                )
        if not found_any:
            lines.append("No actual joints involve the target road or connector.")
        lines.append("========== END DEBUG TARGET ROAD / CONNECTOR ==========")
        self._debug_append_lines_to_file(lines)
        for line in lines:
            LOGGER.info(line)

    def _link_and_smooth_draft_way_joints(self, draft_ways: Sequence[_DraftCenterlineWay]) -> None:
        """Share boundary nodes and locally smooth road <-> connector joints.

        First version deliberately handles only the robust topology pattern:
            road.end -> connector.start
            connector.end -> road.start
        It does not infer road-road or connector-connector joints geometrically,
        which helps avoid accidentally smoothing across a wrong branch in a junction.
        """
        assert self._out is not None
        assert self._node_id_gen is not None

        roads_by_chain: Dict[int, _DraftCenterlineWay] = {}
        connectors: List[_DraftCenterlineWay] = []
        for way in draft_ways:
            if way.kind == "road" and way.chain_index is not None:
                roads_by_chain[way.chain_index] = way
            elif way.kind == "connector":
                connectors.append(way)

        joints: List[Tuple[_DraftCenterlineWay, str, _DraftCenterlineWay, str]] = []
        for connector in connectors:
            if connector.from_chain is not None:
                prev_road = roads_by_chain.get(connector.from_chain)
                if prev_road is not None:
                    joints.append((prev_road, "end", connector, "start"))
            if connector.to_chain is not None:
                next_road = roads_by_chain.get(connector.to_chain)
                if next_road is not None:
                    joints.append((connector, "end", next_road, "start"))

        linked_count = 0
        smoothed_count = 0
        joint_categories: Dict[Tuple[str, str, str, str], str] = {}
        for a, a_side, b, b_side in joints:
            evaluation_before = self._debug_evaluate_draft_joint(a, a_side, b, b_side)
            linked, smoothed = self._link_and_smooth_one_draft_joint(a, a_side, b, b_side)
            key = (a.draft_id, a_side, b.draft_id, b_side)
            if linked:
                linked_count += 1
            if smoothed:
                smoothed_count += 1
            if not linked:
                joint_categories[key] = "SKIPPED"
            elif bool(evaluation_before.get("first_class_candidate")):
                joint_categories[key] = "CLASS_1_TOPOLOGY_SHARED_ENDPOINT_NO_SMOOTHING"
            elif smoothed:
                joint_categories[key] = "CLASS_2_SHARED_ENDPOINT_AND_BEZIER_SMOOTHED"
            else:
                joint_categories[key] = "CLASS_2_SHARED_ENDPOINT_BUT_NOT_SMOOTHED"


        extended_count = self._extend_dangling_draft_way_endpoints_by_true_intersection(draft_ways)

        LOGGER.info(
            "Lanelet2OSMConverter: linked %d road/connector draft joints, smoothed %d of them, and extension-linked %d dangling endpoints.",
            linked_count,
            smoothed_count,
            extended_count,
        )

    def _link_and_smooth_one_draft_joint(
        self,
        a: _DraftCenterlineWay,
        a_side: str,
        b: _DraftCenterlineWay,
        b_side: str,
    ) -> Tuple[bool, bool]:
        """Link one a.end -> b.start joint and optionally rebuild local geometry."""
        assert self._out is not None
        assert self._node_id_gen is not None

        if a_side != "end" or b_side != "start":
            return False, False
        if len(a.xy) < 2 or len(b.xy) < 2 or len(a.ll) < 2 or len(b.ll) < 2:
            return False, False

        gap = self._distance(a.xy[-1], b.xy[0])
        if self._share_topological_endpoint_if_already_connected(a, a_side, b, b_side, gap=gap):
            return True, False
        if gap > self.bridge_join_distance:
            return False, False

        out_dir = self._draft_end_direction(a)
        in_dir = self._draft_start_direction(b)
        cosine = self._vector_cosine(out_dir, in_dir)
        if cosine < self.direction_cosine_threshold:
            return False, False

        lateral_shift = self._lateral_shift_between_draft_ends(a, b)
        if lateral_shift > self.bridge_lateral_shift_tolerance:
            return False, False

        # Make both way endpoints exactly identical in geometry.
        joint_xy = self._midpoint(a.xy[-1], b.xy[0])
        joint_ll = self._midpoint(a.ll[-1], b.ll[0])
        joint_ele = 0.5 * (float(a.ele[-1]) + float(b.ele[0]))
        a.xy[-1] = joint_xy
        b.xy[0] = joint_xy
        a.ll[-1] = joint_ll
        b.ll[0] = joint_ll
        a.ele[-1] = joint_ele
        b.ele[0] = joint_ele

        # Reuse an existing side id if this endpoint has already been linked;
        # otherwise allocate one shared OSM node for both ways.
        shared_node_id = a.end_shared_node_id or b.start_shared_node_id or self._node_id_gen.next()
        a.end_shared_node_id = shared_node_id
        b.start_shared_node_id = shared_node_id

        used_points = self._estimate_joint_blend_points(a, b, gap)
        smoothed = self._smooth_draft_joint_window(a, b, blend_points=used_points)

        # After optional smoothing, force exact boundary equality again and write
        # the shared node at the final joint coordinate.
        joint_xy = self._midpoint(a.xy[-1], b.xy[0])
        joint_ll = self._midpoint(a.ll[-1], b.ll[0])
        joint_ele = 0.5 * (float(a.ele[-1]) + float(b.ele[0]))
        a.xy[-1] = joint_xy
        b.xy[0] = joint_xy
        a.ll[-1] = joint_ll
        b.ll[0] = joint_ll
        a.ele[-1] = joint_ele
        b.ele[0] = joint_ele
        if self._out.find_node_by_id(shared_node_id) is None:
            self._out.add_node(
                OSMNode(
                    id_=shared_node_id,
                    lat=self._format_float(joint_ll[0]),
                    lon=self._format_float(joint_ll[1]),
                    ele=self._format_float(joint_ele),
                    tag_dict={"lanelet2:shared_centerline_joint": "yes"},
                )
            )

        a.tags["lanelet2:joint_smoothing"] = "linked"
        b.tags["lanelet2:joint_smoothing"] = "linked"
        a.tags["lanelet2:joint_blend_points"] = str(used_points)
        b.tags["lanelet2:joint_blend_points"] = str(used_points)
        if smoothed:
            a.tags["lanelet2:joint_smoothing"] = "linked_and_smoothed"
            b.tags["lanelet2:joint_smoothing"] = "linked_and_smoothed"
        return True, smoothed

    def _smooth_draft_joint_window(
        self,
        a: _DraftCenterlineWay,
        b: _DraftCenterlineWay,
        blend_points: int,
    ) -> bool:
        """Locally rebuild a.end -> b.start using two larger Bezier windows.

        Compared with the first implementation, this version uses:
        1. an adaptive / larger blend window;
        2. tangent estimation over a wider span instead of only the last segment;
        3. an optional post-smoothing relaxation pass inside the rebuilt window.
        """
        k = max(3, int(blend_points))
        k = min(k, len(a.xy), len(b.xy), len(a.ll), len(b.ll), len(a.ele), len(b.ele))
        if k < 3:
            return False

        joint_xy = a.xy[-1]
        joint_ll = a.ll[-1]
        joint_ele = a.ele[-1]

        # If the incoming and outgoing tangent directions are very different,
        # keep only the shared node and avoid rounding an actual turn.
        angle_deg = self._angle_between_vectors_deg(
            self._draft_end_direction_window(a, min(self.joint_tangent_span, k - 1)),
            self._draft_start_direction_window(b, min(self.joint_tangent_span, k - 1)),
        )
        if angle_deg > self.joint_smoothing_angle_threshold_deg:
            return False

        a_dir_xy = self._draft_end_direction_window(a, min(self.joint_tangent_span, k - 1))
        b_dir_xy = self._draft_start_direction_window(b, min(self.joint_tangent_span, k - 1))
        a_dir_ll = self._draft_end_direction_ll_window(a, min(self.joint_tangent_span, k - 1))
        b_dir_ll = self._draft_start_direction_ll_window(b, min(self.joint_tangent_span, k - 1))

        a_tail_xy = self._bezier_to_joint_points(a.xy[-k], joint_xy, a_dir_xy, k)
        b_head_xy = self._bezier_from_joint_points(joint_xy, b.xy[k - 1], b_dir_xy, k)
        a_tail_ll = self._bezier_to_joint_points(a.ll[-k], joint_ll, a_dir_ll, k)
        b_head_ll = self._bezier_from_joint_points(joint_ll, b.ll[k - 1], b_dir_ll, k)

        a_tail_ele = self._linear_values(float(a.ele[-k]), float(joint_ele), k)
        b_head_ele = self._linear_values(float(joint_ele), float(b.ele[k - 1]), k)

        if self.joint_post_smooth_passes > 0:
            a_tail_xy = self._relax_polyline_points(a_tail_xy, self.joint_post_smooth_passes)
            b_head_xy = self._relax_polyline_points(b_head_xy, self.joint_post_smooth_passes)
            a_tail_ll = self._relax_polyline_points(a_tail_ll, self.joint_post_smooth_passes)
            b_head_ll = self._relax_polyline_points(b_head_ll, self.joint_post_smooth_passes)
            a_tail_ele = self._relax_scalar_values(a_tail_ele, self.joint_post_smooth_passes)
            b_head_ele = self._relax_scalar_values(b_head_ele, self.joint_post_smooth_passes)

        a.xy[-k:] = a_tail_xy
        b.xy[:k] = b_head_xy
        a.ll[-k:] = a_tail_ll
        b.ll[:k] = b_head_ll
        a.ele[-k:] = a_tail_ele
        b.ele[:k] = b_head_ele

        # Preserve exact shared boundary after replacement.
        a.xy[-1] = joint_xy
        b.xy[0] = joint_xy
        a.ll[-1] = joint_ll
        b.ll[0] = joint_ll
        a.ele[-1] = joint_ele
        b.ele[0] = joint_ele
        return True

    def _bezier_to_joint_points(
        self,
        start: Tuple[float, float],
        joint: Tuple[float, float],
        direction: Tuple[float, float],
        count: int,
    ) -> List[Tuple[float, float]]:
        distance = self._distance(start, joint)
        control_len = max(1e-6, distance * self.joint_control_ratio)
        p0 = start
        p3 = joint
        p1 = self._offset_point(p0, direction, control_len)
        p2 = self._offset_point(p3, direction, -control_len)
        return [self._cubic_bezier(p0, p1, p2, p3, i / float(count - 1)) for i in range(count)]

    def _bezier_from_joint_points(
        self,
        joint: Tuple[float, float],
        end: Tuple[float, float],
        direction: Tuple[float, float],
        count: int,
    ) -> List[Tuple[float, float]]:
        distance = self._distance(joint, end)
        control_len = max(1e-6, distance * self.joint_control_ratio)
        p0 = joint
        p3 = end
        p1 = self._offset_point(p0, direction, control_len)
        p2 = self._offset_point(p3, direction, -control_len)
        return [self._cubic_bezier(p0, p1, p2, p3, i / float(count - 1)) for i in range(count)]

    @staticmethod
    def _linear_values(start: float, end: float, count: int) -> List[float]:
        if count <= 1:
            return [float(end)]
        return [float(start) + (float(end) - float(start)) * i / float(count - 1) for i in range(count)]

    def _estimate_joint_blend_points(
        self,
        a: _DraftCenterlineWay,
        b: _DraftCenterlineWay,
        gap: float,
    ) -> int:
        available = min(len(a.xy), len(b.xy), len(a.ll), len(b.ll), len(a.ele), len(b.ele))
        if available < 3:
            return available
        spacing_samples: List[float] = []
        tail_count = min(4, len(a.xy) - 1)
        head_count = min(4, len(b.xy) - 1)
        for i in range(1, tail_count + 1):
            spacing_samples.append(self._distance(a.xy[-i], a.xy[-i - 1]))
        for i in range(head_count):
            spacing_samples.append(self._distance(b.xy[i], b.xy[i + 1]))
        positive_spacing = [s for s in spacing_samples if s > 1e-6]
        avg_spacing = sum(positive_spacing) / len(positive_spacing) if positive_spacing else 1.0
        adaptive = max(self.joint_smoothing_min_points, int(math.ceil(gap / avg_spacing)) + 3)
        return min(self.joint_smoothing_max_points, available, adaptive)

    @staticmethod
    def _relax_polyline_points(points: List[Tuple[float, float]], passes: int) -> List[Tuple[float, float]]:
        result = list(points)
        if len(result) <= 2 or passes <= 0:
            return result
        for _ in range(passes):
            updated = list(result)
            for idx in range(1, len(result) - 1):
                updated[idx] = (
                    0.25 * result[idx - 1][0] + 0.5 * result[idx][0] + 0.25 * result[idx + 1][0],
                    0.25 * result[idx - 1][1] + 0.5 * result[idx][1] + 0.25 * result[idx + 1][1],
                )
            result = updated
        return result

    @staticmethod
    def _relax_scalar_values(values: List[float], passes: int) -> List[float]:
        result = [float(v) for v in values]
        if len(result) <= 2 or passes <= 0:
            return result
        for _ in range(passes):
            updated = list(result)
            for idx in range(1, len(result) - 1):
                updated[idx] = 0.25 * result[idx - 1] + 0.5 * result[idx] + 0.25 * result[idx + 1]
            result = updated
        return result


    # ------------------------------------------------------------------
    # Geometry endpoint-extension pass
    # ------------------------------------------------------------------
    def _extend_dangling_draft_way_endpoints_by_true_intersection(
        self,
        draft_ways: Sequence[_DraftCenterlineWay],
    ) -> int:
        """Extend unshared endpoints along the single outward direction.

        Current reconstruction policy:
        - Only endpoints without an existing shared node are processed.
        - Only the outward endpoint direction is used.  The previous reverse
          fallback is intentionally removed because it can pull already-valid
          junction endpoints inward.
        - The source ray may create one or two shared intersections:
            Q1 = nearest valid target-way crossing;
            Q2 = optional second crossing only when it is close behind Q1 and
                 its target direction is parallel / anti-parallel to Q1's target.
          This supports the desired "井字形" two-crossing topology without
          allowing arbitrary far-away ray hits.
        - The source endpoint is extended through Q1/Q2 and intermediate points
          are filled according to the source way's endpoint point density.
        - Target ways are never extended; shared nodes are inserted into their
          existing segments.
        """
        if not self.endpoint_extension_enabled:
            return 0
        if (
            self.endpoint_extension_max_draft_ways
            and len(draft_ways) > self.endpoint_extension_max_draft_ways
        ):
            self.endpoint_extension_skip_reason = (
                "draft_way_count_exceeds_limit"
            )
            LOGGER.info(
                "Lanelet2OSMConverter: skipped endpoint extension for %d draft ways "
                "(limit=%d); preserving existing topology without the expensive "
                "geometric intersection refinement.",
                len(draft_ways),
                self.endpoint_extension_max_draft_ways,
            )
            return 0
        assert self._node_id_gen is not None

        stats: Dict[str, int] = defaultdict(int)
        distance_reject_examples: List[str] = []
        topology_reject_examples: List[str] = []
        candidate_decision_examples: List[str] = []
        accepted_examples: List[str] = []

        topology_context = self._build_endpoint_extension_topology_context(draft_ways)

        extended_endpoint_count = 0
        shared_intersection_count = 0

        # Use a snapshot of source endpoints.  Target geometries may gain inserted
        # shared nodes during this pass, but each original dangling endpoint should
        # be considered at most once.
        source_endpoints: List[Tuple[_DraftCenterlineWay, str]] = []
        for source_way in list(draft_ways):
            if len(source_way.xy) < 2 or len(source_way.ll) < 2:
                continue
            source_endpoints.append((source_way, "start"))
            source_endpoints.append((source_way, "end"))

        for source_way, source_side in source_endpoints:
            stats["endpoint_total"] += 1
            if len(source_way.xy) < 2 or len(source_way.ll) < 2:
                stats["skip_source_too_short_after_updates"] += 1
                continue
            if self._draft_endpoint_has_shared_node(source_way, source_side):
                stats["skip_already_shared_endpoint"] += 1
                continue

            original_p_xy, original_p_ll, original_p_ele = self._draft_endpoint_geometry(source_way, source_side)
            hits = self._select_outward_intersection_hits_for_endpoint(
                source_way=source_way,
                source_side=source_side,
                draft_ways=draft_ways,
                stats=stats,
                distance_reject_examples=distance_reject_examples,
                topology_context=topology_context,
                topology_reject_examples=topology_reject_examples,
                candidate_decision_examples=candidate_decision_examples,
            )
            if not hits:
                stats["skip_no_accepted_candidate"] += 1
                continue

            # Allocate one shared node id per accepted crossing and extend the
            # source way through all crossings in ray-distance order.
            shared_node_ids = [self._node_id_gen.next() for _ in hits]
            self._extend_draft_endpoint_through_hits(
                way=source_way,
                side=source_side,
                hits=hits,
                shared_node_ids=shared_node_ids,
            )

            for hit, shared_node_id in zip(hits, shared_node_ids):
                target_way = hit["target_way"]
                self._insert_shared_point_into_draft_way(
                    way=target_way,
                    insert_after_seg_idx=int(hit["target_seg_idx"]),
                    q_xy=hit["q_xy"],
                    q_ll=hit["q_ll"],
                    q_ele=float(hit["q_ele"]),
                    shared_node_id=shared_node_id,
                )
                self._upsert_shared_osm_node(
                    shared_node_id=shared_node_id,
                    ll=hit["q_ll"],
                    ele=float(hit["q_ele"]),
                    tag_dict={
                        "lanelet2:shared_centerline_joint": "yes",
                        "lanelet2:endpoint_extension_joint": "yes",
                        "lanelet2:endpoint_extension_true_intersection": "yes",
                        "lanelet2:endpoint_extension_hit_rank": str(int(hit["hit_rank"])),
                    },
                )
                target_way.tags["lanelet2:endpoint_extension_target_inserted"] = "yes"

                if len(accepted_examples) < self.endpoint_extension_debug_examples:
                    accepted_examples.append(
                        self._format_endpoint_extension_example(
                            prefix="ACCEPT",
                            source_way=source_way,
                            source_side=source_side,
                            target_way=target_way,
                            target_seg_idx=int(hit["target_seg_idx"]),
                            q_xy=hit["q_xy"],
                            q_ll=hit["q_ll"],
                            target_ratio=float(hit["target_ratio"]),
                            direction_label="outward",
                            extension_distance=float(hit["extension_distance"]),
                            angle_deg=float(hit["angle_deg"]),
                            reject_reason=f"accepted_hit_{int(hit['hit_rank'])}",
                            source_p_xy=original_p_xy,
                            source_p_ll=original_p_ll,
                        )
                    )

            source_way.tags["lanelet2:endpoint_extension"] = "source_endpoint_extended"
            source_way.tags["lanelet2:endpoint_extension_side"] = source_side
            source_way.tags["lanelet2:endpoint_extension_target_draft_id"] = ",".join(str(hit["target_way"].draft_id) for hit in hits)
            source_way.tags["lanelet2:endpoint_extension_distance"] = ",".join(self._format_float(float(hit["extension_distance"])) for hit in hits)
            source_way.tags["lanelet2:endpoint_extension_direction"] = "outward"
            source_way.tags["lanelet2:endpoint_extension_hit_count"] = str(len(hits))

            stats["accepted_endpoints"] += 1
            stats["accepted_intersections"] += len(hits)
            extended_endpoint_count += 1
            shared_intersection_count += len(hits)

        self._log_endpoint_extension_debug(stats, distance_reject_examples, topology_reject_examples, candidate_decision_examples, accepted_examples)
        LOGGER.info(
            "Lanelet2OSMConverter: endpoint extension created %d shared intersections on %d source endpoints.",
            shared_intersection_count,
            extended_endpoint_count,
        )
        return extended_endpoint_count

    def _select_outward_intersection_hits_for_endpoint(
        self,
        source_way: _DraftCenterlineWay,
        source_side: str,
        draft_ways: Sequence[_DraftCenterlineWay],
        stats: Dict[str, int],
        distance_reject_examples: List[str],
        topology_context: Dict[str, object],
        topology_reject_examples: List[str],
        candidate_decision_examples: List[str],
    ) -> List[Dict[str, object]]:
        p_xy, _p_ll, _p_ele = self._draft_endpoint_geometry(source_way, source_side)
        outward = self._unit_vector(self._draft_endpoint_outward_direction(source_way, source_side))
        if outward == (0.0, 0.0):
            stats["skip_zero_direction"] += 1
            return []

        all_hits: List[Dict[str, object]] = []
        ray_len = max(self.endpoint_extension_search_radius, self.endpoint_extension_max_distance)
        ray_end = (p_xy[0] + outward[0] * ray_len, p_xy[1] + outward[1] * ray_len)

        for target_way in draft_ways:
            if target_way is source_way:
                stats["skip_target_self"] += 1
                continue
            if len(target_way.xy) < 2 or len(target_way.ll) < 2:
                stats["skip_target_too_short"] += 1
                continue

            # Keep the broad search as a cheap prefilter.  Intersection tests are
            # still exact ray/segment checks.
            nearest_dist = self._nearest_distance_from_point_to_polyline(p_xy, target_way.xy)
            if nearest_dist > self.endpoint_extension_search_radius:
                stats["skip_search_radius"] += 1
                continue

            stats["target_tested"] += 1
            for seg_idx in range(len(target_way.xy) - 1):
                a_xy = target_way.xy[seg_idx]
                b_xy = target_way.xy[seg_idx + 1]
                target_vec = (b_xy[0] - a_xy[0], b_xy[1] - a_xy[1])
                if self._distance(a_xy, b_xy) == 0.0:
                    stats["skip_zero_target_segment"] += 1
                    continue

                intersection = self._segment_intersection_with_params(p_xy, ray_end, a_xy, b_xy)
                if intersection is None:
                    stats["skip_no_true_intersection"] += 1
                    continue

                q_xy, _ray_t, seg_t = intersection
                extension_distance = self._distance(p_xy, q_xy)
                if extension_distance < self.endpoint_extension_min_distance:
                    stats["skip_extension_too_near"] += 1
                    self._append_distance_reject_example(
                        distance_reject_examples, source_way, source_side, target_way, seg_idx,
                        q_xy, seg_t, "outward", extension_distance, "too_near"
                    )
                    continue
                if extension_distance > self.endpoint_extension_max_distance:
                    stats["skip_extension_too_far"] += 1
                    self._append_distance_reject_example(
                        distance_reject_examples, source_way, source_side, target_way, seg_idx,
                        q_xy, seg_t, "outward", extension_distance, "too_far"
                    )
                    continue

                if self._target_hit_too_close_to_endpoint(target_way, seg_idx, seg_t):
                    stats["skip_target_endpoint_margin"] += 1
                    continue

                v = (q_xy[0] - p_xy[0], q_xy[1] - p_xy[1])
                dir_cosine = self._vector_cosine(outward, v)
                if dir_cosine < self.endpoint_extension_direction_cosine_threshold:
                    stats["skip_direction_cosine"] += 1
                    continue

                # The endpoint-extension ray is meant to cross target ways, not
                # attach to a nearly collinear way.  This prevents long same-road
                # or opposite-direction grabs.
                target_parallel_cos = abs(self._vector_cosine(outward, target_vec))
                if target_parallel_cos > self.endpoint_extension_crossing_max_parallel_cosine:
                    stats["skip_source_target_too_parallel"] += 1
                    continue

                q_ll, q_ele = self._interpolate_draft_way_ll_ele(target_way, seg_idx, seg_t)
                angle_deg = self._angle_between_vectors_deg(outward, v)
                all_hits.append({
                    "target_way": target_way,
                    "target_seg_idx": seg_idx,
                    "target_ratio": seg_t,
                    "target_dir": self._unit_vector(target_vec),
                    "q_xy": q_xy,
                    "q_ll": q_ll,
                    "q_ele": q_ele,
                    "angle_deg": angle_deg,
                    "extension_distance": extension_distance,
                })

        if not all_hits:
            return []

        all_hits.sort(key=lambda hit: (float(hit["extension_distance"]), float(hit["angle_deg"]), str(hit["target_way"].draft_id)))
        deduped: List[Dict[str, object]] = []
        for hit in all_hits:
            if any(self._distance(hit["q_xy"], prev["q_xy"]) < 0.30 for prev in deduped):
                stats["skip_duplicate_hit_same_location"] += 1
                continue
            deduped.append(hit)

        if not deduped:
            return []

        # v13 geometry is preserved up to this point.  New behavior starts here:
        # evaluate the nearest K geometry hits against a group-aware topology gate.
        selected: List[Dict[str, object]] = []
        max_selected = self.endpoint_extension_max_selected_hits if self.endpoint_extension_second_hit_enabled else 1
        max_selected = max(1, max_selected)
        candidate_hits = deduped[: self.endpoint_extension_candidate_hit_limit]
        stats["topology_candidates_checked"] += len(candidate_hits)

        first_rejected_hit: Optional[Dict[str, object]] = None
        for candidate_rank, hit in enumerate(candidate_hits, start=1):
            hit["candidate_rank"] = candidate_rank
            reachable_info = self._endpoint_extension_hit_topology_reachable_group_aware(
                source_way=source_way,
                source_side=source_side,
                hit=hit,
                topology_context=topology_context,
            )
            hit["topology_info"] = reachable_info

            if self.endpoint_extension_require_topology and not bool(reachable_info.get("reachable")):
                stats["skip_topology_not_reachable"] += 1
                if first_rejected_hit is None:
                    first_rejected_hit = hit
                self._append_topology_reject_example(
                    topology_reject_examples,
                    source_way,
                    source_side,
                    hit,
                    reachable_info,
                    reason="topology_not_reachable",
                )
                self._append_candidate_decision_example(
                    candidate_decision_examples,
                    source_way,
                    source_side,
                    hit,
                    reachable_info,
                    decision="REJECT_TOPOLOGY",
                )
                continue

            hit["hit_rank"] = len(selected) + 1
            selected.append(hit)
            stats["accepted_topology_reachable_hit"] += 1
            self._append_candidate_decision_example(
                candidate_decision_examples,
                source_way,
                source_side,
                hit,
                reachable_info,
                decision="ACCEPT_TOPOLOGY",
            )
            if len(selected) >= max_selected:
                break

        if selected:
            if len(selected) >= 2:
                stats["accepted_second_hit"] += 1
            return selected

        if self.endpoint_extension_require_topology and self.endpoint_extension_fallback_to_nearest_geometry and deduped:
            hit = deduped[0]
            hit["hit_rank"] = 1
            hit["candidate_rank"] = int(hit.get("candidate_rank", 1))
            stats["accepted_fallback_nearest_geometry"] += 1
            self._append_candidate_decision_example(
                candidate_decision_examples,
                source_way,
                source_side,
                hit,
                hit.get("topology_info", {}) if isinstance(hit.get("topology_info"), dict) else {},
                decision="ACCEPT_FALLBACK_NEAREST_GEOMETRY",
            )
            return [hit]

        if self.endpoint_extension_require_topology and first_rejected_hit is not None:
            stats["skip_no_topology_reachable_hit"] += 1
        return []

    def _build_endpoint_extension_topology_context(
        self,
        draft_ways: Sequence[_DraftCenterlineWay],
    ) -> Dict[str, object]:
        """Build lanelet -> group indexes for group-aware topology checks."""
        lanelet_to_groups: Dict[str, Set[str]] = defaultdict(set)
        draft_way_all_groups: Dict[str, Set[str]] = {}
        for way in draft_ways:
            all_ids = set(self._draft_way_all_group_lanelets(way))
            start_ids = set(self._draft_way_endpoint_group_lanelets(way, "start"))
            end_ids = set(self._draft_way_endpoint_group_lanelets(way, "end"))
            combined = set(all_ids) | start_ids | end_ids
            if not combined:
                continue
            draft_way_all_groups[way.draft_id] = combined
            # Map each lanelet to the complete road/connector group containing it.
            for lid in combined:
                lanelet_to_groups[str(lid)].update(combined)
        return {
            "lanelet_to_groups": lanelet_to_groups,
            "draft_way_all_groups": draft_way_all_groups,
        }

    def _draft_way_endpoint_group_lanelets(self, way: _DraftCenterlineWay, side: str) -> List[str]:
        if side == "start":
            ids = list(way.start_group_lanelet_ids or [])
            if not ids:
                ids = self._parse_id_list(way.tags.get("lanelet2:start_group_lanelet_ids"))
            if not ids:
                ids = list(way.start_lanelet_ids or [])
        else:
            ids = list(way.end_group_lanelet_ids or [])
            if not ids:
                ids = self._parse_id_list(way.tags.get("lanelet2:end_group_lanelet_ids"))
            if not ids:
                ids = list(way.end_lanelet_ids or [])
        return list(dict.fromkeys(str(x).strip() for x in ids if str(x).strip()))

    def _draft_way_all_group_lanelets(self, way: _DraftCenterlineWay) -> List[str]:
        ids = list(way.all_lanelet_ids or [])
        if not ids:
            ids = self._parse_id_list(way.tags.get("lanelet2:all_lanelet_ids"))
        if not ids:
            ids = self._parse_id_list(way.tags.get("lanelet2:lanelet_ids"))
        if not ids:
            ids = list(dict.fromkeys((way.start_group_lanelet_ids or []) + (way.end_group_lanelet_ids or [])))
        if not ids:
            ids = list(dict.fromkeys((way.start_lanelet_ids or []) + (way.end_lanelet_ids or [])))
        return list(dict.fromkeys(str(x).strip() for x in ids if str(x).strip()))

    def _lanelet_group_from_context(self, lanelet_id: str, topology_context: Dict[str, object]) -> Set[str]:
        mapping = topology_context.get("lanelet_to_groups", {})
        group: Set[str] = set()
        if isinstance(mapping, dict):
            raw = mapping.get(str(lanelet_id))
            if raw:
                group.update(str(x) for x in raw)
        if not group:
            group.add(str(lanelet_id))
        return group

    def _expand_lanelet_set_to_groups(self, lanelet_ids: Iterable[str], topology_context: Dict[str, object]) -> Set[str]:
        result: Set[str] = set()
        for lanelet_id in lanelet_ids:
            if not str(lanelet_id).strip():
                continue
            result.update(self._lanelet_group_from_context(str(lanelet_id), topology_context))
        return result

    def _target_group_aware_expanded_lanelets(
        self,
        target_way: _DraftCenterlineWay,
        topology_context: Dict[str, object],
    ) -> Tuple[Set[str], Set[str]]:
        """Return target base group and group-aware pred/succ expansion."""
        assert self._source is not None
        base = self._expand_lanelet_set_to_groups(self._draft_way_all_group_lanelets(target_way), topology_context)
        if not base:
            base = set(self._draft_way_all_group_lanelets(target_way))
        expanded: Set[str] = set(base)
        frontier: Set[str] = set(base)
        for _hop in range(self.endpoint_extension_target_neighbor_hops):
            next_frontier: Set[str] = set()
            for lanelet_id in frontier:
                lanelet = self._source.find_way_rel_by_id(str(lanelet_id))
                if lanelet is None:
                    continue
                neighbors = list(self._lanelet_predecessors(lanelet)) + list(self._lanelet_successors(lanelet))
                for nb in neighbors:
                    group = self._lanelet_group_from_context(str(nb), topology_context)
                    for gid in group:
                        if gid not in expanded:
                            expanded.add(gid)
                            next_frontier.add(gid)
            if not next_frontier:
                break
            frontier = next_frontier
        return base, expanded

    def _source_group_aware_search_seeds(
        self,
        source_way: _DraftCenterlineWay,
        source_side: str,
        topology_context: Dict[str, object],
    ) -> Set[str]:
        endpoint_group = self._draft_way_endpoint_group_lanelets(source_way, source_side)
        if not endpoint_group:
            endpoint_group = self._draft_endpoint_lanelet_ids(source_way, source_side)
        return self._expand_lanelet_set_to_groups(endpoint_group, topology_context)

    def _topology_neighbors_for_direction(self, lanelet_id: str, direction: str) -> List[str]:
        assert self._source is not None
        lanelet = self._source.find_way_rel_by_id(str(lanelet_id))
        if lanelet is None:
            return []
        if direction == "successors":
            return self._lanelet_successors(lanelet)
        return self._lanelet_predecessors(lanelet)

    def _group_aware_reachable_bfs(
        self,
        source_seeds: Iterable[str],
        target_lanelets: Set[str],
        direction: str,
        topology_context: Dict[str, object],
    ) -> Dict[str, object]:
        seeds = self._expand_lanelet_set_to_groups(source_seeds, topology_context)
        queue: List[Tuple[str, int]] = [(lid, 0) for lid in sorted(seeds, key=self._safe_sort_key)]
        visited: Set[str] = set(seeds)
        parent: Dict[str, Optional[str]] = {lid: None for lid in seeds}
        hit_id: Optional[str] = None
        if visited & target_lanelets:
            hit_id = sorted(visited & target_lanelets, key=self._safe_sort_key)[0]
        while queue and hit_id is None:
            current, depth = queue.pop(0)
            if depth >= self.endpoint_extension_topology_max_hops:
                continue
            for nb in self._topology_neighbors_for_direction(current, direction):
                nb_group = self._lanelet_group_from_context(str(nb), topology_context)
                # One longitudinal hop reaches the neighbor lanelet; then the
                # whole lanelet group containing that neighbor is available.
                for gid in sorted(nb_group, key=self._safe_sort_key):
                    if gid in visited:
                        continue
                    visited.add(gid)
                    parent[gid] = current
                    if gid in target_lanelets:
                        hit_id = gid
                        break
                    queue.append((gid, depth + 1))
                if hit_id is not None:
                    break
        path: List[str] = []
        if hit_id is not None:
            cur: Optional[str] = hit_id
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            path.reverse()
        return {
            "reachable": hit_id is not None,
            "hit_lanelet": hit_id or "",
            "visited": visited,
            "visited_count": len(visited),
            "visited_sample": sorted(visited, key=self._safe_sort_key)[:80],
            "path": path,
            "source_seeds": sorted(seeds, key=self._safe_sort_key),
            "direction": direction,
        }

    def _endpoint_extension_hit_topology_reachable_group_aware(
        self,
        source_way: _DraftCenterlineWay,
        source_side: str,
        hit: Dict[str, object],
        topology_context: Dict[str, object],
    ) -> Dict[str, object]:
        target_way = hit["target_way"]
        assert isinstance(target_way, _DraftCenterlineWay)
        source_seeds = self._source_group_aware_search_seeds(source_way, source_side, topology_context)
        target_base, target_expanded = self._target_group_aware_expanded_lanelets(target_way, topology_context)
        direction = "predecessors" if source_side == "start" else "successors"
        result = self._group_aware_reachable_bfs(
            source_seeds=source_seeds,
            target_lanelets=target_expanded,
            direction=direction,
            topology_context=topology_context,
        )
        intersection = sorted(set(result.get("visited", set())) & target_expanded, key=self._safe_sort_key)
        return {
            "reachable": bool(result.get("reachable")),
            "search_direction": direction,
            "source_group_lanelets": sorted(source_seeds, key=self._safe_sort_key),
            "target_base_group_lanelets": sorted(target_base, key=self._safe_sort_key),
            "target_expanded_group_lanelets": sorted(target_expanded, key=self._safe_sort_key),
            "visited_count": int(result.get("visited_count", 0)),
            "visited_lanelet_sample": result.get("visited_sample", []),
            "intersection_visited_target": intersection[:80],
            "hit_lanelet": str(result.get("hit_lanelet", "")),
            "path": result.get("path", []),
        }

    def _format_lanelet_list_for_debug(self, values: Iterable[str], limit: int = 80) -> str:
        vals = list(dict.fromkeys(str(x) for x in values if str(x).strip()))
        vals = sorted(vals, key=self._safe_sort_key)
        if len(vals) > limit:
            return ",".join(vals[:limit]) + f",...(+{len(vals)-limit})"
        return ",".join(vals) if vals else "NONE"

    def _append_topology_reject_example(
        self,
        examples: List[str],
        source_way: _DraftCenterlineWay,
        source_side: str,
        hit: Dict[str, object],
        info: Dict[str, object],
        reason: str,
    ) -> None:
        if len(examples) >= self.endpoint_extension_debug_examples:
            return
        target_way = hit["target_way"]
        p_xy, _p_ll, _p_ele = self._draft_endpoint_geometry(source_way, source_side)
        examples.append(
            f"TOPOLOGY_REJECT: reason={reason}; "
            f"source_endpoint={source_way.draft_id}.{source_side}; "
            f"source_representative_lanelets={','.join(self._draft_endpoint_lanelet_ids(source_way, source_side)) or 'NONE'}; "
            f"source_group_lanelets={self._format_lanelet_list_for_debug(info.get('source_group_lanelets', []))}; "
            f"search_direction={info.get('search_direction', 'NONE')}; "
            f"target_way={target_way.draft_id}; target_kind={target_way.kind}; "
            f"target_base_group_lanelets={self._format_lanelet_list_for_debug(info.get('target_base_group_lanelets', []))}; "
            f"target_expanded_group_lanelets={self._format_lanelet_list_for_debug(info.get('target_expanded_group_lanelets', []))}; "
            f"candidate_rank={int(hit.get('candidate_rank', -1))}; "
            f"extension_distance={self._format_float(float(hit['extension_distance']))}; "
            f"target_seg_idx={int(hit['target_seg_idx'])}; target_seg_ratio={self._format_float(float(hit['target_ratio']))}; "
            f"source_p_xy=({self._format_float(p_xy[0])},{self._format_float(p_xy[1])}); "
            f"q_xy=({self._format_float(hit['q_xy'][0])},{self._format_float(hit['q_xy'][1])}); "
            f"topology_max_hops={self.endpoint_extension_topology_max_hops}; "
            f"target_neighbor_hops={self.endpoint_extension_target_neighbor_hops}; "
            f"visited_count={info.get('visited_count', 0)}; "
            f"visited_lanelet_sample={self._format_lanelet_list_for_debug(info.get('visited_lanelet_sample', []))}; "
            f"intersection_visited_target={self._format_lanelet_list_for_debug(info.get('intersection_visited_target', []))}; "
            f"path={self._format_lanelet_list_for_debug(info.get('path', []))}"
        )

    def _append_candidate_decision_example(
        self,
        examples: List[str],
        source_way: _DraftCenterlineWay,
        source_side: str,
        hit: Dict[str, object],
        info: Dict[str, object],
        decision: str,
    ) -> None:
        if len(examples) >= self.endpoint_extension_debug_examples:
            return
        target_way = hit["target_way"]
        examples.append(
            f"CANDIDATE_DECISION: decision={decision}; "
            f"source_endpoint={source_way.draft_id}.{source_side}; "
            f"source_group_lanelets={self._format_lanelet_list_for_debug(info.get('source_group_lanelets', []))}; "
            f"target_way={target_way.draft_id}; target_kind={target_way.kind}; "
            f"candidate_rank={int(hit.get('candidate_rank', -1))}; "
            f"accepted_rank={int(hit.get('hit_rank', 0)) if 'hit_rank' in hit else 'NONE'}; "
            f"extension_distance={self._format_float(float(hit['extension_distance']))}; "
            f"target_base_group_lanelets={self._format_lanelet_list_for_debug(info.get('target_base_group_lanelets', []))}; "
            f"hit_lanelet={info.get('hit_lanelet', 'NONE')}; "
            f"path={self._format_lanelet_list_for_debug(info.get('path', []), limit=30)}"
        )

    def _append_distance_reject_example(
        self,
        examples: List[str],
        source_way: _DraftCenterlineWay,
        source_side: str,
        target_way: _DraftCenterlineWay,
        target_seg_idx: int,
        q_xy: Tuple[float, float],
        target_ratio: float,
        direction_label: str,
        extension_distance: float,
        reject_reason: str,
    ) -> None:
        if len(examples) >= self.endpoint_extension_debug_examples:
            return
        q_ll, _q_ele = self._interpolate_draft_way_ll_ele(target_way, target_seg_idx, target_ratio)
        examples.append(
            self._format_endpoint_extension_example(
                prefix="DISTANCE_REJECT",
                source_way=source_way,
                source_side=source_side,
                target_way=target_way,
                target_seg_idx=target_seg_idx,
                q_xy=q_xy,
                q_ll=q_ll,
                target_ratio=target_ratio,
                direction_label=direction_label,
                extension_distance=extension_distance,
                angle_deg=0.0,
                reject_reason=reject_reason,
            )
        )

    def _format_endpoint_extension_example(
        self,
        prefix: str,
        source_way: _DraftCenterlineWay,
        source_side: str,
        target_way: _DraftCenterlineWay,
        target_seg_idx: int,
        q_xy: Tuple[float, float],
        q_ll: Tuple[float, float],
        target_ratio: float,
        direction_label: str,
        extension_distance: float,
        angle_deg: float,
        reject_reason: str,
        source_p_xy: Optional[Tuple[float, float]] = None,
        source_p_ll: Optional[Tuple[float, float]] = None,
    ) -> str:
        if source_p_xy is None or source_p_ll is None:
            p_xy, p_ll, _p_ele = self._draft_endpoint_geometry(source_way, source_side)
        else:
            p_xy, p_ll = source_p_xy, source_p_ll
        return (
            f"{prefix}: reason={reject_reason}; "
            f"source_endpoint={source_way.draft_id}.{source_side}; "
            f"source_kind={source_way.kind}; "
            f"source_chain={source_way.chain_index}; "
            f"source_lanelets={','.join(source_way.start_lanelet_ids if source_side == 'start' else source_way.end_lanelet_ids) or 'NONE'}; "
            f"source_group_lanelets={','.join(self._draft_way_endpoint_group_lanelets(source_way, source_side)) or 'NONE'}; "
            f"source_p_xy=({self._format_float(p_xy[0])},{self._format_float(p_xy[1])}); "
            f"source_p_ll=({self._format_float(p_ll[0])},{self._format_float(p_ll[1])}); "
            f"target_way={target_way.draft_id}; "
            f"target_kind={target_way.kind}; "
            f"target_chain={target_way.chain_index}; "
            f"target_lanelets={target_way.tags.get('lanelet2:lanelet_ids', 'NONE')}; "
            f"target_seg_idx={target_seg_idx}; "
            f"target_seg_ratio={self._format_float(target_ratio)}; "
            f"q_xy=({self._format_float(q_xy[0])},{self._format_float(q_xy[1])}); "
            f"q_ll=({self._format_float(q_ll[0])},{self._format_float(q_ll[1])}); "
            f"direction={direction_label}; "
            f"extension_distance={self._format_float(extension_distance)}; "
            f"angle_deg={self._format_float(angle_deg)}; "
            f"allowed_min={self._format_float(self.endpoint_extension_min_distance)}; "
            f"allowed_max={self._format_float(self.endpoint_extension_max_distance)}"
        )

    def _log_endpoint_extension_debug(
        self,
        stats: Dict[str, int],
        distance_reject_examples: Sequence[str],
        topology_reject_examples: Sequence[str],
        candidate_decision_examples: Sequence[str],
        accepted_examples: Sequence[str],
    ) -> None:
        sorted_stats = {key: int(stats[key]) for key in sorted(stats.keys())}
        LOGGER.info("Lanelet2OSMConverter endpoint extension stats: %s", sorted_stats)

        lines: List[str] = []
        lines.append("========== DEBUG ENDPOINT EXTENSION STATS ==========")
        lines.append(str(sorted_stats))
        lines.append("--- distance reject examples ---")
        if distance_reject_examples:
            for idx, example in enumerate(distance_reject_examples):
                line = f"endpoint_extension_distance_reject_example[{idx}]: {example}"
                LOGGER.info(line)
                lines.append(line)
        else:
            lines.append("NONE")
        lines.append("--- topology reject examples ---")
        if topology_reject_examples:
            for idx, example in enumerate(topology_reject_examples):
                line = f"endpoint_extension_topology_reject_example[{idx}]: {example}"
                LOGGER.info(line)
                lines.append(line)
        else:
            lines.append("NONE")
        lines.append("--- candidate decision examples ---")
        if candidate_decision_examples:
            for idx, example in enumerate(candidate_decision_examples):
                line = f"endpoint_extension_candidate_decision_example[{idx}]: {example}"
                LOGGER.info(line)
                lines.append(line)
        else:
            lines.append("NONE")
        lines.append("--- accepted examples ---")
        if accepted_examples:
            for idx, example in enumerate(accepted_examples):
                line = f"endpoint_extension_accepted_example[{idx}]: {example}"
                LOGGER.info(line)
                lines.append(line)
        else:
            lines.append("NONE")
        lines.append("========== END DEBUG ENDPOINT EXTENSION STATS ==========")
        self._debug_append_lines_to_file(lines)

    def _draft_endpoint_geometry(
        self,
        way: _DraftCenterlineWay,
        side: str,
    ) -> Tuple[Tuple[float, float], Tuple[float, float], float]:
        if side == "start":
            return way.xy[0], way.ll[0], float(way.ele[0])
        return way.xy[-1], way.ll[-1], float(way.ele[-1])

    def _draft_endpoint_has_shared_node(self, way: _DraftCenterlineWay, side: str) -> bool:
        if side == "start":
            return way.start_shared_node_id is not None
        if side == "end":
            return way.end_shared_node_id is not None
        return False

    def _draft_endpoint_outward_direction(self, way: _DraftCenterlineWay, side: str) -> Tuple[float, float]:
        span = self._endpoint_extension_tangent_span(way, side)
        if side == "start":
            d = self._draft_start_direction_window(way, span)
            return -d[0], -d[1]
        return self._draft_end_direction_window(way, span)

    def _endpoint_extension_tangent_span(self, way: _DraftCenterlineWay, side: str) -> int:
        base_span = min(self.joint_tangent_span, max(1, len(way.xy) - 1))
        if base_span <= 1:
            return base_span
        variation = self._endpoint_tangent_variation_deg(way, side, base_span)
        if variation >= self.endpoint_extension_tangent_high_variation_threshold_deg:
            return min(self.endpoint_extension_tangent_min_span, base_span)
        if variation >= self.endpoint_extension_tangent_variation_threshold_deg:
            return min(self.endpoint_extension_tangent_short_span, base_span)
        return base_span

    def _endpoint_tangent_variation_deg(self, way: _DraftCenterlineWay, side: str, span: int) -> float:
        if len(way.xy) < 3:
            return 0.0
        use_span = max(1, min(int(span), len(way.xy) - 1))
        vectors: List[Tuple[float, float]] = []
        if side == "start":
            for idx in range(use_span):
                a = way.xy[idx]
                b = way.xy[idx + 1]
                vectors.append((b[0] - a[0], b[1] - a[1]))
        else:
            start_idx = len(way.xy) - 1 - use_span
            for idx in range(start_idx, len(way.xy) - 1):
                a = way.xy[idx]
                b = way.xy[idx + 1]
                vectors.append((b[0] - a[0], b[1] - a[1]))
        vectors = [v for v in vectors if math.hypot(v[0], v[1]) > 1e-9]
        if len(vectors) < 2:
            return 0.0
        ref = vectors[0]
        return max(self._angle_between_vectors_deg(ref, v) for v in vectors[1:])

    def _extend_draft_endpoint_through_hits(
        self,
        way: _DraftCenterlineWay,
        side: str,
        hits: Sequence[Dict[str, object]],
        shared_node_ids: Sequence[str],
    ) -> None:
        """Extend source way through one or two outward hits, filling points.

        Hits must be sorted by distance from the original endpoint.  For an end
        endpoint we append P->Q1->Q2.  For a start endpoint we prepend the same
        geometry in reverse order so the way orientation remains valid.
        """
        if not hits:
            return
        p_xy, p_ll, p_ele = self._draft_endpoint_geometry(way, side)
        endpoint_spacing = self._estimate_endpoint_spacing(way, side)

        forward_items: List[Tuple[Tuple[float, float], Tuple[float, float], float, Optional[str]]] = []
        prev_xy, prev_ll, prev_ele = p_xy, p_ll, p_ele
        for hit, shared_node_id in zip(hits, shared_node_ids):
            q_xy = hit["q_xy"]
            q_ll = hit["q_ll"]
            q_ele = float(hit["q_ele"])
            for xy, ll, ele in self._interpolate_extension_between_points(
                prev_xy, prev_ll, prev_ele, q_xy, q_ll, q_ele, endpoint_spacing
            ):
                forward_items.append((xy, ll, ele, None))
            forward_items.append((q_xy, q_ll, q_ele, shared_node_id))
            prev_xy, prev_ll, prev_ele = q_xy, q_ll, q_ele

        if side == "end":
            for xy, ll, ele, shared_node_id in forward_items:
                idx = len(way.xy)
                way.xy.append(xy)
                way.ll.append(ll)
                way.ele.append(ele)
                if shared_node_id is not None:
                    way.shared_node_ids_by_index[idx] = shared_node_id
            # Last accepted hit is the new endpoint and should use the endpoint id slot.
            last_shared = shared_node_ids[-1]
            last_idx = len(way.xy) - 1
            way.shared_node_ids_by_index.pop(last_idx, None)
            way.end_shared_node_id = last_shared
            return

        # start side: prepend farthest->nearest->old P.
        reversed_items = list(reversed(forward_items))
        self._shift_middle_shared_indices_after_bulk_insert(way, 0, len(reversed_items))
        for offset, (xy, ll, ele, shared_node_id) in enumerate(reversed_items):
            way.xy.insert(offset, xy)
            way.ll.insert(offset, ll)
            way.ele.insert(offset, ele)
            if shared_node_id is not None:
                way.shared_node_ids_by_index[offset] = shared_node_id
        first_shared = shared_node_ids[-1]
        way.shared_node_ids_by_index.pop(0, None)
        way.start_shared_node_id = first_shared

    def _interpolate_extension_between_points(
        self,
        p_xy: Tuple[float, float],
        p_ll: Tuple[float, float],
        p_ele: float,
        q_xy: Tuple[float, float],
        q_ll: Tuple[float, float],
        q_ele: float,
        endpoint_spacing: float,
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float], float]]:
        if not self.endpoint_extension_insert_points_enabled:
            return []
        distance = self._distance(p_xy, q_xy)
        spacing = max(0.1, endpoint_spacing * self.endpoint_extension_insert_spacing_factor)
        insert_count = max(0, int(math.ceil(distance / spacing)) - 1)
        insert_count = min(self.endpoint_extension_max_insert_points, insert_count)
        points: List[Tuple[Tuple[float, float], Tuple[float, float], float]] = []
        for i in range(1, insert_count + 1):
            t = i / float(insert_count + 1)
            xy = (p_xy[0] + (q_xy[0] - p_xy[0]) * t, p_xy[1] + (q_xy[1] - p_xy[1]) * t)
            ll = (p_ll[0] + (q_ll[0] - p_ll[0]) * t, p_ll[1] + (q_ll[1] - p_ll[1]) * t)
            ele = float(p_ele) + (float(q_ele) - float(p_ele)) * t
            points.append((xy, ll, ele))
        return points

    def _estimate_endpoint_spacing(self, way: _DraftCenterlineWay, side: str) -> float:
        if len(way.xy) < 2:
            return 2.0
        sample_count = min(self.endpoint_extension_insert_spacing_sample_points, len(way.xy) - 1)
        distances: List[float] = []
        if side == "start":
            for idx in range(sample_count):
                distances.append(self._distance(way.xy[idx], way.xy[idx + 1]))
        else:
            for offset in range(sample_count):
                idx = len(way.xy) - 1 - offset
                distances.append(self._distance(way.xy[idx], way.xy[idx - 1]))
        distances = [d for d in distances if d > 1e-6]
        return sum(distances) / len(distances) if distances else 2.0

    @staticmethod
    def _shift_middle_shared_indices_after_bulk_insert(way: _DraftCenterlineWay, insert_idx: int, count: int) -> None:
        if count <= 0 or not way.shared_node_ids_by_index:
            return
        shifted: Dict[int, str] = {}
        for idx, node_id in way.shared_node_ids_by_index.items():
            shifted[idx + count if idx >= insert_idx else idx] = node_id
        way.shared_node_ids_by_index = shifted

    def _extend_draft_endpoint_to_point(
        self,
        way: _DraftCenterlineWay,
        side: str,
        q_xy: Tuple[float, float],
        q_ll: Tuple[float, float],
        q_ele: float,
        shared_node_id: str,
    ) -> None:
        if side == "start":
            self._shift_middle_shared_indices_after_insert(way, 0)
            way.xy.insert(0, q_xy)
            way.ll.insert(0, q_ll)
            way.ele.insert(0, q_ele)
            way.start_shared_node_id = shared_node_id
        else:
            way.xy.append(q_xy)
            way.ll.append(q_ll)
            way.ele.append(q_ele)
            way.end_shared_node_id = shared_node_id

    def _insert_shared_point_into_draft_way(
        self,
        way: _DraftCenterlineWay,
        insert_after_seg_idx: int,
        q_xy: Tuple[float, float],
        q_ll: Tuple[float, float],
        q_ele: float,
        shared_node_id: str,
    ) -> None:
        insert_idx = max(1, min(int(insert_after_seg_idx) + 1, len(way.xy)))
        self._shift_middle_shared_indices_after_insert(way, insert_idx)
        way.xy.insert(insert_idx, q_xy)
        way.ll.insert(insert_idx, q_ll)
        way.ele.insert(insert_idx, q_ele)
        way.shared_node_ids_by_index[insert_idx] = shared_node_id

    @staticmethod
    def _shift_middle_shared_indices_after_insert(way: _DraftCenterlineWay, insert_idx: int) -> None:
        if not way.shared_node_ids_by_index:
            return
        shifted: Dict[int, str] = {}
        for idx, node_id in way.shared_node_ids_by_index.items():
            shifted[idx + 1 if idx >= insert_idx else idx] = node_id
        way.shared_node_ids_by_index = shifted

    def _upsert_shared_osm_node(
        self,
        shared_node_id: str,
        ll: Tuple[float, float],
        ele: float,
        tag_dict: Optional[Dict[str, str]] = None,
    ) -> None:
        assert self._out is not None
        node = self._out.find_node_by_id(shared_node_id)
        if node is None:
            self._out.add_node(
                OSMNode(
                    id_=shared_node_id,
                    lat=self._format_float(ll[0]),
                    lon=self._format_float(ll[1]),
                    ele=self._format_float(ele),
                    tag_dict=tag_dict or {},
                )
            )
            return
        try:
            node.lat = self._format_float(ll[0])
            node.lon = self._format_float(ll[1])
            node.ele = self._format_float(ele)
            existing_tags = dict(getattr(node, "tag_dict", {}) or {})
            existing_tags.update(tag_dict or {})
            node.tag_dict = existing_tags
        except Exception:
            pass

    def _interpolate_draft_way_ll_ele(
        self,
        way: _DraftCenterlineWay,
        seg_idx: int,
        ratio: float,
    ) -> Tuple[Tuple[float, float], float]:
        ratio = max(0.0, min(1.0, float(ratio)))
        a_ll = way.ll[seg_idx]
        b_ll = way.ll[seg_idx + 1]
        a_ele = float(way.ele[seg_idx])
        b_ele = float(way.ele[seg_idx + 1])
        q_ll = (a_ll[0] + (b_ll[0] - a_ll[0]) * ratio, a_ll[1] + (b_ll[1] - a_ll[1]) * ratio)
        q_ele = a_ele + (b_ele - a_ele) * ratio
        return q_ll, q_ele

    def _target_hit_too_close_to_endpoint(self, way: _DraftCenterlineWay, seg_idx: int, ratio: float) -> bool:
        if self.endpoint_extension_endpoint_margin <= 0.0:
            return False
        if seg_idx == 0 and ratio <= self.endpoint_extension_endpoint_margin:
            return True
        if seg_idx == len(way.xy) - 2 and ratio >= 1.0 - self.endpoint_extension_endpoint_margin:
            return True
        return False

    def _nearest_distance_from_point_to_polyline(
        self,
        p: Tuple[float, float],
        polyline: Sequence[Tuple[float, float]],
    ) -> float:
        if not polyline:
            return float("inf")
        if len(polyline) == 1:
            return self._distance(p, polyline[0])
        best = float("inf")
        for idx in range(len(polyline) - 1):
            q, _ratio = self._project_point_to_segment(p, polyline[idx], polyline[idx + 1])
            best = min(best, self._distance(p, q))
        return best

    @staticmethod
    def _project_point_to_segment(
        p: Tuple[float, float],
        a: Tuple[float, float],
        b: Tuple[float, float],
    ) -> Tuple[Tuple[float, float], float]:
        ab = (b[0] - a[0], b[1] - a[1])
        denom = ab[0] * ab[0] + ab[1] * ab[1]
        if denom == 0.0:
            return a, 0.0
        ap = (p[0] - a[0], p[1] - a[1])
        t = (ap[0] * ab[0] + ap[1] * ab[1]) / denom
        t = max(0.0, min(1.0, t))
        return (a[0] + ab[0] * t, a[1] + ab[1] * t), t

    @staticmethod
    def _segment_intersection_with_params(
        a0: Tuple[float, float],
        a1: Tuple[float, float],
        b0: Tuple[float, float],
        b1: Tuple[float, float],
    ) -> Optional[Tuple[Tuple[float, float], float, float]]:
        r = (a1[0] - a0[0], a1[1] - a0[1])
        s = (b1[0] - b0[0], b1[1] - b0[1])
        denom = r[0] * s[1] - r[1] * s[0]
        if abs(denom) < 1e-12:
            return None
        qp = (b0[0] - a0[0], b0[1] - a0[1])
        t = (qp[0] * s[1] - qp[1] * s[0]) / denom
        u = (qp[0] * r[1] - qp[1] * r[0]) / denom
        eps = 1e-9
        if t < -eps or t > 1.0 + eps or u < -eps or u > 1.0 + eps:
            return None
        q = (a0[0] + r[0] * t, a0[1] + r[1] * t)
        return q, max(0.0, min(1.0, t)), max(0.0, min(1.0, u))

    @staticmethod
    def _unit_vector(v: Tuple[float, float]) -> Tuple[float, float]:
        norm = math.hypot(v[0], v[1])
        if norm == 0.0:
            return 0.0, 0.0
        return v[0] / norm, v[1] / norm

    def _write_draft_way_as_osm(self, way: _DraftCenterlineWay) -> None:
        """Serialize one already-linked/smoothed draft way to the output OSM."""
        assert self._out is not None
        assert self._node_id_gen is not None
        assert self._way_id_gen is not None

        if len(way.ll) < 2:
            return

        node_ids: List[str] = []
        last_index = len(way.ll) - 1
        for idx, (ll, ele) in enumerate(zip(way.ll, way.ele)):
            shared_node_id: Optional[str] = None
            if idx == 0:
                shared_node_id = way.start_shared_node_id
            elif idx == last_index:
                shared_node_id = way.end_shared_node_id
            else:
                shared_node_id = way.shared_node_ids_by_index.get(idx)

            if shared_node_id is not None:
                # Shared nodes are normally created by the joint-linking or endpoint-extension pass.
                # Create defensively if an id was registered but no node was emitted yet.
                if self._out.find_node_by_id(shared_node_id) is None:
                    self._out.add_node(
                        OSMNode(
                            id_=shared_node_id,
                            lat=self._format_float(ll[0]),
                            lon=self._format_float(ll[1]),
                            ele=self._format_float(ele),
                            tag_dict={"lanelet2:shared_centerline_joint": "yes"},
                        )
                    )
                node_ids.append(shared_node_id)
                continue

            node_id = self._node_id_gen.next()
            node = OSMNode(
                id_=node_id,
                lat=self._format_float(ll[0]),
                lon=self._format_float(ll[1]),
                ele=self._format_float(ele),
                tag_dict={},
            )
            self._out.add_node(node)
            node_ids.append(node_id)

        if len(node_ids) < 2:
            return

        way_id = self._way_id_gen.next()
        out_tags = dict(way.tags)
        out_tags["lanelet2:draft_id"] = way.draft_id
        self._out.add_way(OSMWay(id_=way_id, nodes=node_ids, tag_dict=out_tags))

    @staticmethod
    def _safe_int_or_none(value: object) -> Optional[int]:
        try:
            if value is None:
                return None
            return int(str(value))
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _draft_start_direction(way: _DraftCenterlineWay) -> Tuple[float, float]:
        if len(way.xy) < 2:
            return 1.0, 0.0
        return way.xy[1][0] - way.xy[0][0], way.xy[1][1] - way.xy[0][1]

    @staticmethod
    def _draft_end_direction(way: _DraftCenterlineWay) -> Tuple[float, float]:
        if len(way.xy) < 2:
            return 1.0, 0.0
        return way.xy[-1][0] - way.xy[-2][0], way.xy[-1][1] - way.xy[-2][1]

    @staticmethod
    def _draft_start_direction_ll(way: _DraftCenterlineWay) -> Tuple[float, float]:
        if len(way.ll) < 2:
            return 1.0, 0.0
        return way.ll[1][0] - way.ll[0][0], way.ll[1][1] - way.ll[0][1]

    @staticmethod
    def _draft_end_direction_ll(way: _DraftCenterlineWay) -> Tuple[float, float]:
        if len(way.ll) < 2:
            return 1.0, 0.0
        return way.ll[-1][0] - way.ll[-2][0], way.ll[-1][1] - way.ll[-2][1]

    @staticmethod
    def _draft_start_direction_window(way: _DraftCenterlineWay, span: int) -> Tuple[float, float]:
        if len(way.xy) < 2:
            return 1.0, 0.0
        use_span = max(1, min(int(span), len(way.xy) - 1))
        end_idx = use_span
        return way.xy[end_idx][0] - way.xy[0][0], way.xy[end_idx][1] - way.xy[0][1]

    @staticmethod
    def _draft_end_direction_window(way: _DraftCenterlineWay, span: int) -> Tuple[float, float]:
        if len(way.xy) < 2:
            return 1.0, 0.0
        use_span = max(1, min(int(span), len(way.xy) - 1))
        start_idx = len(way.xy) - 1 - use_span
        return way.xy[-1][0] - way.xy[start_idx][0], way.xy[-1][1] - way.xy[start_idx][1]

    @staticmethod
    def _draft_start_direction_ll_window(way: _DraftCenterlineWay, span: int) -> Tuple[float, float]:
        if len(way.ll) < 2:
            return 1.0, 0.0
        use_span = max(1, min(int(span), len(way.ll) - 1))
        end_idx = use_span
        return way.ll[end_idx][0] - way.ll[0][0], way.ll[end_idx][1] - way.ll[0][1]

    @staticmethod
    def _draft_end_direction_ll_window(way: _DraftCenterlineWay, span: int) -> Tuple[float, float]:
        if len(way.ll) < 2:
            return 1.0, 0.0
        use_span = max(1, min(int(span), len(way.ll) - 1))
        start_idx = len(way.ll) - 1 - use_span
        return way.ll[-1][0] - way.ll[start_idx][0], way.ll[-1][1] - way.ll[start_idx][1]

    def _lateral_shift_between_draft_ends(self, a: _DraftCenterlineWay, b: _DraftCenterlineWay) -> float:
        a_dir = self._draft_end_direction(a)
        norm = math.hypot(a_dir[0], a_dir[1])
        if norm == 0.0:
            return 0.0
        dx = b.xy[0][0] - a.xy[-1][0]
        dy = b.xy[0][1] - a.xy[-1][1]
        cross = abs(dx * a_dir[1] - dy * a_dir[0])
        return cross / norm

    def _write_chain_as_osm_way(self, chain: Sequence[_CenterlineSegment], road_name: str) -> None:
        assert self._out is not None
        assert self._node_id_gen is not None
        assert self._way_id_gen is not None

        if not chain:
            return

        merged_xy, merged_ll, merged_ele = self._merge_chain_geometry(chain)

        node_ids: List[str] = []
        for _xy, ll, ele in zip(merged_xy, merged_ll, merged_ele):
            node_id = self._node_id_gen.next()
            node = OSMNode(
                id_=node_id,
                lat=self._format_float(ll[0]),
                lon=self._format_float(ll[1]),
                ele=self._format_float(ele),
                tag_dict={},
            )
            self._out.add_node(node)
            node_ids.append(node_id)

        way_tags = dict(chain[0].tags)
        way_tags["name"] = road_name
        way_id = self._way_id_gen.next()
        self._out.add_way(OSMWay(id_=way_id, nodes=node_ids, tag_dict=way_tags))

    def _merge_chain_geometry(
        self,
        chain: Sequence[_CenterlineSegment],
    ) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]], List[float]]:
        merged_xy: List[Tuple[float, float]] = []
        merged_ll: List[Tuple[float, float]] = []
        merged_ele: List[float] = []

        for idx, seg in enumerate(chain):
            if idx == 0:
                merged_xy.extend(seg.xy)
                merged_ll.extend(seg.ll)
                merged_ele.extend(seg.ele)
                continue

            prev_seg = chain[idx - 1]
            bridge = self._build_controlled_bridge(prev_seg, seg)
            if bridge is not None:
                bridge_xy, bridge_ll, bridge_ele = bridge
                merged_xy.extend(bridge_xy)
                merged_ll.extend(bridge_ll)
                merged_ele.extend(bridge_ele)
                merged_xy.extend(seg.xy[1:])
                merged_ll.extend(seg.ll[1:])
                merged_ele.extend(seg.ele[1:])
            else:
                merged_xy.extend(seg.xy[1:])
                merged_ll.extend(seg.ll[1:])
                merged_ele.extend(seg.ele[1:])
        return merged_xy, merged_ll, merged_ele

    def _build_controlled_bridge(
        self,
        prev_seg: _CenterlineSegment,
        next_seg: _CenterlineSegment,
    ) -> Optional[Tuple[List[Tuple[float, float]], List[Tuple[float, float]], List[float]]]:
        gap = self._distance(prev_seg.xy[-1], next_seg.xy[0])
        if gap <= 1e-6:
            return None
        if gap > self.bridge_join_distance:
            return None
        if self._segment_topology_score(prev_seg, next_seg) <= 0:
            return None
        if self._lateral_shift_between_segments(prev_seg, next_seg) > self.bridge_lateral_shift_tolerance:
            return None
        if self._vector_cosine(self._segment_out_direction(prev_seg), self._segment_in_direction(next_seg)) < self.direction_cosine_threshold:
            return None

        p0_xy = prev_seg.xy[-1]
        p1_xy = self._offset_point(p0_xy, self._segment_out_direction(prev_seg), gap / 3.0)
        p3_xy = next_seg.xy[0]
        p2_xy = self._offset_point(p3_xy, self._segment_in_direction(next_seg), -gap / 3.0)

        p0_ll = prev_seg.ll[-1]
        p3_ll = next_seg.ll[0]
        # LL coordinates are degrees, while gap is measured in metric XY.
        # Use simple fractional LL controls to avoid mixing meter distances into lat/lon.
        p1_ll = (p0_ll[0] + (p3_ll[0] - p0_ll[0]) / 3.0, p0_ll[1] + (p3_ll[1] - p0_ll[1]) / 3.0)
        p2_ll = (p0_ll[0] + 2.0 * (p3_ll[0] - p0_ll[0]) / 3.0, p0_ll[1] + 2.0 * (p3_ll[1] - p0_ll[1]) / 3.0)

        e0 = prev_seg.ele[-1]
        e3 = next_seg.ele[0]
        bridge_xy: List[Tuple[float, float]] = []
        bridge_ll: List[Tuple[float, float]] = []
        bridge_ele: List[float] = []

        for i in range(1, self.bridge_insert_points + 1):
            t = i / float(self.bridge_insert_points + 1)
            bridge_xy.append(self._cubic_bezier(p0_xy, p1_xy, p2_xy, p3_xy, t))
            bridge_ll.append(self._cubic_bezier(p0_ll, p1_ll, p2_ll, p3_ll, t))
            bridge_ele.append((1.0 - t) * e0 + t * e3)

        return bridge_xy, bridge_ll, bridge_ele

    def _infer_lanelet_topology_from_boundary_endpoints(
        self,
        overwrite_existing: bool = False,
        only_fill_missing: bool = True,
    ) -> None:
        """
        Boundary-endpoint fallback topology inference.

        Strategy:
        1. Prefer existing explicit topology, especially official RoutingGraph writeback.
        2. Only fill missing predecessor/successor when topology is absent.
        3. Never clear all lanelets globally before rebuilding.
        """
        assert self._source is not None

        lanelets = list(self._source.way_relations.values())
        if not lanelets:
            return

        endpoint_cache: Dict[str, Dict[str, str]] = {}
        start_left_index: Dict[str, List[str]] = defaultdict(list)
        start_right_index: Dict[str, List[str]] = defaultdict(list)

        for lanelet in lanelets:
            self._ensure_lanelet_topology_fields(lanelet)

        for lanelet in lanelets:
            left_way = self._source.find_way_by_id(str(lanelet.left_way))
            right_way = self._source.find_way_by_id(str(lanelet.right_way))
            if left_way is None or right_way is None:
                continue
            if len(left_way.nodes) < 2 or len(right_way.nodes) < 2:
                continue

            left_start = str(left_way.nodes[0])
            left_end = str(left_way.nodes[-1])
            right_start = str(right_way.nodes[0])
            right_end = str(right_way.nodes[-1])

            endpoint_cache[str(lanelet.id_)] = {
                "left_start": left_start,
                "left_end": left_end,
                "right_start": right_start,
                "right_end": right_end,
            }
            start_left_index[left_start].append(str(lanelet.id_))
            start_right_index[right_start].append(str(lanelet.id_))

        for lanelet in lanelets:
            lanelet_id = str(lanelet.id_)
            ep = endpoint_cache.get(lanelet_id)
            if ep is None:
                continue

            has_official_topology = self._lanelet_topology_source_contains_official(lanelet)

            if self.prefer_existing_topology and has_official_topology:
                continue

            preds = getattr(lanelet, "predecessors", None) or []
            succs = getattr(lanelet, "successors", None) or []

            if only_fill_missing and (preds or succs):
                continue

            if overwrite_existing and not has_official_topology:
                lanelet.predecessors = []
                lanelet.successors = []
                if isinstance(getattr(lanelet, "topology_source", None), dict):
                    lanelet.topology_source["predecessors"] = ""
                    lanelet.topology_source["successors"] = ""

            candidate_ids: Set[str] = set()
            candidate_ids.update(start_left_index.get(ep["left_end"], []))
            candidate_ids.update(start_right_index.get(ep["right_end"], []))
            candidate_ids.discard(lanelet_id)

            scored: List[Tuple[float, str]] = []
            for cand_id in candidate_ids:
                cand_lanelet = self._source.find_way_rel_by_id(cand_id)
                cand_ep = endpoint_cache.get(cand_id)
                if cand_lanelet is None or cand_ep is None:
                    continue
                score = self._boundary_topology_match_score(lanelet, ep, cand_lanelet, cand_ep)
                if score > 0.0:
                    scored.append((score, cand_id))

            scored.sort(key=lambda item: (-item[0], self._safe_sort_key(item[1])))

            for score, succ_id in scored:
                if score < 40.0:
                    continue

                succ_lanelet = self._source.find_way_rel_by_id(succ_id)
                if succ_lanelet is None:
                    continue

                if only_fill_missing and self.prefer_existing_topology:
                    succ_has_official = self._lanelet_topology_source_contains_official(succ_lanelet)
                    succ_preds = getattr(succ_lanelet, "predecessors", None) or []
                    if succ_has_official and succ_preds:
                        if succ_id not in lanelet.successors:
                            lanelet.successors.append(succ_id)
                        if isinstance(getattr(lanelet, "topology_source", None), dict):
                            if not lanelet.topology_source.get("successors"):
                                lanelet.topology_source["successors"] = "boundary_endpoint_fallback"
                        continue

                self._set_or_append_successor_predecessor(
                    lanelet=lanelet,
                    succ_lanelet=succ_lanelet,
                    overwrite_existing=True,
                )

        for lanelet in lanelets:
            lanelet.predecessors = sorted(
                set(str(x) for x in (getattr(lanelet, "predecessors", []) or []) if str(x) != str(lanelet.id_)),
                key=self._safe_sort_key,
            )
            lanelet.successors = sorted(
                set(str(x) for x in (getattr(lanelet, "successors", []) or []) if str(x) != str(lanelet.id_)),
                key=self._safe_sort_key,
            )

    def _boundary_topology_match_score(
        self,
        from_lanelet: WayRelation,
        from_ep: Dict[str, str],
        to_lanelet: WayRelation,
        to_ep: Dict[str, str],
    ) -> float:
        score = 0.0

        if from_ep['left_end'] == to_ep['left_start']:
            score += 40.0
        if from_ep['right_end'] == to_ep['right_start']:
            score += 40.0

        if from_ep['left_end'] == to_ep['right_start']:
            score += 10.0
        if from_ep['right_end'] == to_ep['left_start']:
            score += 10.0

        from_seg = self._build_centerline_segment([from_lanelet], f'topo-from-{from_lanelet.id_}')
        to_seg = self._build_centerline_segment([to_lanelet], f'topo-to-{to_lanelet.id_}')
        if from_seg is None or to_seg is None:
            return score

        cosine = self._vector_cosine(
            self._segment_out_direction(from_seg),
            self._segment_in_direction(to_seg),
        )
        if cosine < self.direction_cosine_threshold:
            return 0.0
        score += 20.0 * max(0.0, cosine)

        gap = self._distance(from_seg.xy[-1], to_seg.xy[0])
        if gap > self.longitudinal_join_distance * 2.0:
            return 0.0
        score += max(0.0, 20.0 - gap * 2.0)

        if not self._lanelet_tags_compatible(
            self._copy_tag_dict(from_lanelet.tag_dict),
            self._copy_tag_dict(to_lanelet.tag_dict),
            lateral=False,
        ):
            return 0.0

        return score

    # ------------------------------------------------------------------
    # Tag construction
    # ------------------------------------------------------------------
    def _build_centerline_way_tags(
        self,
        lanelet_group: Sequence[WayRelation],
        outer_pair: Tuple[str, str],
    ) -> Dict[str, str]:
        assert self._source is not None

        tags: Dict[str, str] = {
            "lanes": str(len(lanelet_group)),
        }
        if self.add_default_highway_tag:
            tags["highway"] = "road"

        common_tags = self._common_lanelet_tags(lanelet_group)
        keep_keys = {
            "subtype",
            "location",
            "participant:vehicle",
            "participant:car",
            "participant:bus",
            "participant:bicycle",
            "participant:pedestrian",
            "name",
            "ref",
        }
        for key, value in common_tags.items():
            if key in {"type", "one_way", "oneway"}:
                continue
            if key in keep_keys or key.startswith("participant:"):
                tags[key] = value

        one_way_value = common_tags.get("one_way") or common_tags.get("oneway")
        if one_way_value is not None:
            lowered = str(one_way_value).strip().lower()
            if lowered in {"yes", "true", "1"}:
                tags["oneway"] = "yes"
            elif lowered in {"no", "false", "0"}:
                tags["oneway"] = "no"
        else:
            # A Lanelet2 lanelet has an intrinsic travel direction. Opposing
            # traffic is represented by another lanelet, so the road-level OSM
            # centerline must default to one-way unless the source says otherwise.
            tags["oneway"] = "yes"

        regulatory_ids: Set[str] = set()
        speed_rel_ids: List[str] = []
        speed_values: List[str] = []
        traffic_light_ids: List[str] = []
        row_priority_ids: List[str] = []
        row_yield_ids: List[str] = []

        for lanelet in lanelet_group:
            regulatory_ids.update(str(reg_id) for reg_id in (lanelet.regulatory_elements or []))

        for reg_id in sorted(regulatory_ids, key=self._safe_sort_key):
            reg = self._source.find_regulatory_element_by_id(reg_id)
            if reg is None:
                continue
            subtype = str(reg.tag_dict.get("subtype", ""))
            if subtype == "speed_limit":
                speed_rel_ids.append(str(reg.id_))
                helper = getattr(self._source, "speed_limit_signs", {}).get(str(reg.id_))
                if helper is not None:
                    speed_values.append(str(helper[0]))
                elif "sign_type" in reg.tag_dict:
                    speed_values.append(str(reg.tag_dict["sign_type"]))
            elif subtype == "traffic_light":
                traffic_light_ids.append(str(reg.id_))
            elif subtype in {"right_of_way", "all_way_stop"}:
                referring_lanelet_ids = set(str(x) for x in (getattr(reg, "right_of_ways", []) or []))
                yielding_lanelet_ids = set(str(x) for x in (getattr(reg, "yield_ways", []) or []))
                if referring_lanelet_ids:
                    if any(str(lanelet.id_) in referring_lanelet_ids for lanelet in lanelet_group):
                        row_priority_ids.append(str(reg.id_))
                if yielding_lanelet_ids:
                    if any(str(lanelet.id_) in yielding_lanelet_ids for lanelet in lanelet_group):
                        row_yield_ids.append(str(reg.id_))

        if regulatory_ids:
            tags["lanelet2:regulatory_elements"] = ",".join(sorted(regulatory_ids, key=self._safe_sort_key))
        if speed_rel_ids:
            tags["lanelet2:speed_limit_relations"] = ",".join(sorted(speed_rel_ids, key=self._safe_sort_key))
            unique_speed_values = sorted(set(speed_values), key=self._safe_sort_key)
            if len(unique_speed_values) == 1:
                tags["lanelet2:maxspeed"] = unique_speed_values[0]
                standard_speed = self._normalize_osm_maxspeed(unique_speed_values[0])
                if standard_speed and "maxspeed" not in tags:
                    tags["maxspeed"] = standard_speed
            elif len(unique_speed_values) > 1:
                tags["lanelet2:maxspeed_list"] = ",".join(unique_speed_values)
        if traffic_light_ids:
            tags["traffic_signals"] = "yes"
            tags["lanelet2:traffic_light_relations"] = ",".join(sorted(traffic_light_ids, key=self._safe_sort_key))
        if row_priority_ids:
            tags["lanelet2:right_of_way_relations"] = ",".join(sorted(set(row_priority_ids), key=self._safe_sort_key))
            tags["lanelet2:has_right_of_way"] = "yes"
        if row_yield_ids:
            tags["lanelet2:yield_relations"] = ",".join(sorted(set(row_yield_ids), key=self._safe_sort_key))
            tags["lanelet2:must_yield"] = "yes"

        return tags

    def _normalize_centerline_tags(self, tags: Dict[str, str]) -> None:
        subtype = str(tags.get("subtype", "")).lower()
        participant_vehicle = str(tags.get("participant:vehicle", "")).lower()
        participant_car = str(tags.get("participant:car", "")).lower()

        if "highway" not in tags or tags.get("highway") == "road":
            if "crosswalk" in subtype or str(tags.get("participant:pedestrian", "")).lower() == "yes":
                tags["highway"] = "footway"
            elif str(tags.get("participant:bicycle", "")).lower() == "yes" and participant_vehicle not in {"yes", "true", "1"}:
                tags["highway"] = "cycleway"
            elif "service" in subtype:
                tags["highway"] = "service"
            elif "exit" in subtype or "entry" in subtype or "link" in subtype or "ramp" in subtype:
                tags["highway"] = "service"
            elif participant_vehicle in {"yes", "true", "1"} or participant_car in {"yes", "true", "1"}:
                tags["highway"] = "unclassified"
            else:
                # OSM permits highway=road for an unspecified road, but the
                # OSM-to-CommonRoad parser does not include it in its routable
                # road inventory. Keep an unknown motor road round-trippable.
                tags["highway"] = "unclassified"

    def _common_lanelet_tags(self, lanelet_group: Sequence[WayRelation]) -> Dict[str, str]:
        if not lanelet_group:
            return {}
        common = self._copy_tag_dict(lanelet_group[0].tag_dict)
        for lanelet in lanelet_group[1:]:
            current = self._copy_tag_dict(lanelet.tag_dict)
            remove_keys = [key for key, value in common.items() if current.get(key) != value]
            for key in remove_keys:
                common.pop(key, None)
        return common

    # ------------------------------------------------------------------
    # Compatibility rules
    # ------------------------------------------------------------------
    def _lanelets_laterally_compatible(self, a: WayRelation, b: WayRelation) -> bool:
        seg_a = self._build_centerline_segment([a], "compat-a")
        seg_b = self._build_centerline_segment([b], "compat-b")
        if seg_a is None or seg_b is None:
            return False

        if self._vector_cosine(self._segment_out_direction(seg_a), self._segment_out_direction(seg_b)) < self.lateral_direction_cosine_threshold:
            return False
        if self._distance(seg_a.xy[0], seg_b.xy[0]) > self.lateral_offset_tolerance:
            return False
        if self._distance(seg_a.xy[-1], seg_b.xy[-1]) > self.lateral_offset_tolerance:
            return False
        if not self._lanelet_tags_compatible(self._copy_tag_dict(a.tag_dict), self._copy_tag_dict(b.tag_dict)):
            return False
        return True

    def _lanelet_tags_compatible(self, a: Dict[str, str], b: Dict[str, str], lateral: bool = True) -> bool:
        keys = [
            "location",
            "subtype",
            "participant:vehicle",
            "participant:car",
            "participant:bus",
            "participant:bicycle",
            "participant:pedestrian",
            "one_way",
            "oneway",
        ]
        for key in keys:
            a_value = a.get(key)
            b_value = b.get(key)
            if a_value is not None and b_value is not None and str(a_value) != str(b_value):
                return False

        if lateral:
            a_name = a.get("name")
            b_name = b.get("name")
            if a_name and b_name and str(a_name) != str(b_name):
                return False
        return True

    def _lanelet_has_explicit_topology(self, lanelet: WayRelation) -> bool:
        preds = getattr(lanelet, "predecessors", None) or []
        succs = getattr(lanelet, "successors", None) or []
        lefts = getattr(lanelet, "left_neighbors", None) or []
        rights = getattr(lanelet, "right_neighbors", None) or []
        adj_lefts = getattr(lanelet, "adjacent_left", None) or []
        adj_rights = getattr(lanelet, "adjacent_right", None) or []
        return any([preds, succs, lefts, rights, adj_lefts, adj_rights])

    def _lanelet_topology_source_contains_official(self, lanelet: WayRelation) -> bool:
        source_info = getattr(lanelet, "topology_source", None)
        if not isinstance(source_info, dict):
            return False

        for value in source_info.values():
            if not value:
                continue
            lowered = str(value).lower()
            if "official" in lowered or "routing_graph" in lowered:
                return True
        return False

    def _ensure_lanelet_topology_fields(self, lanelet: WayRelation) -> None:
        if not hasattr(lanelet, "predecessors") or getattr(lanelet, "predecessors") is None:
            setattr(lanelet, "predecessors", [])
        if not hasattr(lanelet, "successors") or getattr(lanelet, "successors") is None:
            setattr(lanelet, "successors", [])
        if not hasattr(lanelet, "left_neighbors") or getattr(lanelet, "left_neighbors") is None:
            setattr(lanelet, "left_neighbors", [])
        if not hasattr(lanelet, "right_neighbors") or getattr(lanelet, "right_neighbors") is None:
            setattr(lanelet, "right_neighbors", [])
        if not hasattr(lanelet, "adjacent_left") or getattr(lanelet, "adjacent_left") is None:
            setattr(lanelet, "adjacent_left", [])
        if not hasattr(lanelet, "adjacent_right") or getattr(lanelet, "adjacent_right") is None:
            setattr(lanelet, "adjacent_right", [])

        if not hasattr(lanelet, "topology_source") or getattr(lanelet, "topology_source") is None:
            setattr(
                lanelet,
                "topology_source",
                {
                    "predecessors": "",
                    "successors": "",
                    "left_neighbors": "",
                    "right_neighbors": "",
                    "adjacent_left": "",
                    "adjacent_right": "",
                },
            )

    def _append_unique_topology_id(self, values: List[str], lanelet_id: str) -> None:
        lanelet_id = str(lanelet_id)
        if lanelet_id not in values:
            values.append(lanelet_id)

    def _set_or_append_successor_predecessor(
        self,
        lanelet: WayRelation,
        succ_lanelet: WayRelation,
        overwrite_existing: bool,
    ) -> None:
        lanelet_id = str(lanelet.id_)
        succ_id = str(succ_lanelet.id_)

        self._ensure_lanelet_topology_fields(lanelet)
        self._ensure_lanelet_topology_fields(succ_lanelet)

        if overwrite_existing:
            if succ_id not in lanelet.successors:
                lanelet.successors.append(succ_id)
            if lanelet_id not in succ_lanelet.predecessors:
                succ_lanelet.predecessors.append(lanelet_id)
        else:
            self._append_unique_topology_id(lanelet.successors, succ_id)
            self._append_unique_topology_id(succ_lanelet.predecessors, lanelet_id)

        if isinstance(getattr(lanelet, "topology_source", None), dict):
            if not lanelet.topology_source.get("successors"):
                lanelet.topology_source["successors"] = "boundary_endpoint_fallback"
        if isinstance(getattr(succ_lanelet, "topology_source", None), dict):
            if not succ_lanelet.topology_source.get("predecessors"):
                succ_lanelet.topology_source["predecessors"] = "boundary_endpoint_fallback"

    # ------------------------------------------------------------------
    # Topology helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _lanelet_successors(lanelet: WayRelation) -> List[str]:
        raw = getattr(lanelet, "successors", None)
        if raw is None:
            raw = []
        return sorted({str(x) for x in raw if str(x) != str(getattr(lanelet, "id_", ""))})

    @staticmethod
    def _lanelet_predecessors(lanelet: WayRelation) -> List[str]:
        raw = getattr(lanelet, "predecessors", None)
        if raw is None:
            raw = []
        return sorted({str(x) for x in raw if str(x) != str(getattr(lanelet, "id_", ""))})

    def _lanelet_is_inheritance_excluded(self, lanelet: WayRelation) -> bool:
        tags = self._copy_tag_dict(getattr(lanelet, "tag_dict", {}) or {})
        subtype = str(tags.get("subtype", "")).lower()
        location = str(tags.get("location", "")).lower()
        turn_direction = str(tags.get("turn_direction", "")).lower()
        turn_lane = str(tags.get("turn:lanes", "")).lower()
        participant_ped = str(tags.get("participant:pedestrian", "")).lower()

        if participant_ped in {"yes", "true", "1"} or "crosswalk" in subtype:
            return True
        if turn_direction not in {"", "straight"}:
            return True
        if turn_lane and turn_lane not in {"through", "straight"}:
            return True
        if location in {"intersection", "junction"}:
            return True

        excluded_keywords = (
            "intersection",
            "junction",
            "connector",
            "turn",
            "merge",
            "diverge",
            "split",
            "fork",
            "entry",
            "exit",
            "link",
            "ramp",
            "service",
            "slip",
        )
        return any(keyword in subtype for keyword in excluded_keywords)

    def _group_is_inheritance_excluded(self, lanelet_group: Sequence[WayRelation]) -> bool:
        if not lanelet_group:
            return True
        return any(self._lanelet_is_inheritance_excluded(lanelet) for lanelet in lanelet_group)

    def _group_has_branching_lanelets(self, lanelet_group: Sequence[WayRelation]) -> bool:
        for lanelet in lanelet_group:
            if len(self._lanelet_predecessors(lanelet)) > 1:
                return True
            if len(self._lanelet_successors(lanelet)) > 1:
                return True
        return False

    def _topology_match_score(self, prev_rep_ids: Sequence[str], candidate_lanelet_id: str) -> int:
        if self._source is None:
            return 0
        candidate = self._source.find_way_rel_by_id(candidate_lanelet_id)
        if candidate is None:
            return 0
        if self._lanelet_is_inheritance_excluded(candidate):
            return 0
        preds = set(self._lanelet_predecessors(candidate))
        succs = set(self._lanelet_successors(candidate))
        score = 0
        for prev_id in prev_rep_ids:
            prev_id = str(prev_id)
            if prev_id in preds:
                score += 3
            if prev_id in succs:
                score += 2
            prev_lanelet = self._source.find_way_rel_by_id(prev_id)
            if prev_lanelet is not None:
                prev_succs = set(self._lanelet_successors(prev_lanelet))
                if candidate_lanelet_id in prev_succs:
                    score += 4
        return score

    def _segment_topology_score(self, a: _CenterlineSegment, b: _CenterlineSegment) -> int:
        score = 0
        for lanelet_id in b.ordered_lanelet_ids or b.lanelet_ids:
            score += self._topology_match_score(a.representative_lanelet_ids or a.lanelet_ids, lanelet_id)
        return score

    def _lateral_shift_between_segments(self, a: _CenterlineSegment, b: _CenterlineSegment) -> float:
        a_dir = self._segment_out_direction(a)
        norm = math.hypot(a_dir[0], a_dir[1])
        if norm == 0.0:
            return 0.0
        dx = b.xy[0][0] - a.xy[-1][0]
        dy = b.xy[0][1] - a.xy[-1][1]
        cross = abs(dx * a_dir[1] - dy * a_dir[0])
        return cross / norm

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    def _build_way_geometry(self, way: Lanelet2Way) -> Dict[str, List]:
        assert self._source is not None
        xy: List[Tuple[float, float]] = []
        ll: List[Tuple[float, float]] = []
        ele: List[float] = []

        for node_id in way.nodes:
            node = self._source.find_node_by_id(node_id)
            if node is None:
                continue
            point_xy = self._node_xy(node)
            xy.append(point_xy)
            lat = self._safe_float(node.lat, None)
            lon = self._safe_float(node.lon, None)
            ll.append(
                (lat, lon)
                if lat is not None and lon is not None
                else self._xy_to_latlon_if_needed(point_xy)
            )
            ele.append(self._safe_float(getattr(node, "ele", 0.0)))
        return {"xy": xy, "ll": ll, "ele": ele}

    def _align_boundaries(
        self,
        left_xy: List[Tuple[float, float]],
        left_ll: List[Tuple[float, float]],
        left_ele: List[float],
        right_xy: List[Tuple[float, float]],
        right_ll: List[Tuple[float, float]],
        right_ele: List[float],
    ) -> Tuple[
        List[Tuple[float, float]],
        List[Tuple[float, float]],
        List[float],
        List[Tuple[float, float]],
        List[Tuple[float, float]],
        List[float],
        bool,
    ]:
        same_dir = self._distance(left_xy[0], right_xy[0]) + self._distance(left_xy[-1], right_xy[-1])
        reversed_dir = self._distance(left_xy[0], right_xy[-1]) + self._distance(left_xy[-1], right_xy[0])

        if reversed_dir < same_dir:
            return (
                left_xy,
                left_ll,
                left_ele,
                list(reversed(right_xy)),
                list(reversed(right_ll)),
                list(reversed(right_ele)),
                True,
            )
        return left_xy, left_ll, left_ele, right_xy, right_ll, right_ele, False

    def _resample_polyline(self, points: Sequence[Tuple[float, float]], num_points: int) -> List[Tuple[float, float]]:
        if not points:
            return []
        if len(points) == 1:
            return [tuple(points[0]) for _ in range(num_points)]

        cumulative = [0.0]
        for idx in range(1, len(points)):
            cumulative.append(cumulative[-1] + self._distance(points[idx - 1], points[idx]))
        total_length = cumulative[-1]

        if total_length == 0.0:
            return [tuple(points[0]) for _ in range(num_points)]

        targets = [total_length * i / (num_points - 1) for i in range(num_points)]
        result: List[Tuple[float, float]] = []
        seg_idx = 0
        for target in targets:
            while seg_idx < len(cumulative) - 2 and target > cumulative[seg_idx + 1]:
                seg_idx += 1
            seg_start = cumulative[seg_idx]
            seg_end = cumulative[seg_idx + 1]
            ratio = 0.0 if seg_end == seg_start else (target - seg_start) / (seg_end - seg_start)
            x = points[seg_idx][0] + ratio * (points[seg_idx + 1][0] - points[seg_idx][0])
            y = points[seg_idx][1] + ratio * (points[seg_idx + 1][1] - points[seg_idx][1])
            result.append((x, y))
        return result

    def _resample_scalar_series(self, values: Sequence[float], num_values: int) -> List[float]:
        if not values:
            return [0.0 for _ in range(num_values)]
        if len(values) == 1:
            return [float(values[0]) for _ in range(num_values)]
        points = [(float(i), float(v)) for i, v in enumerate(values)]
        resampled = self._resample_polyline(points, num_values)
        return [value for _, value in resampled]

    @staticmethod
    def _midpoint(a: Tuple[float, float], b: Tuple[float, float]) -> Tuple[float, float]:
        return 0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])

    @staticmethod
    def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _vector_cosine(a: Tuple[float, float], b: Tuple[float, float]) -> float:
        norm_a = math.hypot(a[0], a[1])
        norm_b = math.hypot(b[0], b[1])
        if norm_a == 0.0 or norm_b == 0.0:
            return 1.0
        return (a[0] * b[0] + a[1] * b[1]) / (norm_a * norm_b)

    @staticmethod
    def _segment_in_direction(seg: _CenterlineSegment) -> Tuple[float, float]:
        if len(seg.xy) < 2:
            return 1.0, 0.0
        return seg.xy[1][0] - seg.xy[0][0], seg.xy[1][1] - seg.xy[0][1]

    @staticmethod
    def _segment_out_direction(seg: _CenterlineSegment) -> Tuple[float, float]:
        if len(seg.xy) < 2:
            return 1.0, 0.0
        return seg.xy[-1][0] - seg.xy[-2][0], seg.xy[-1][1] - seg.xy[-2][1]

    @staticmethod
    def _offset_point(point: Tuple[float, float], direction: Tuple[float, float], distance: float) -> Tuple[float, float]:
        norm = math.hypot(direction[0], direction[1])
        if norm == 0.0:
            return point
        return point[0] + direction[0] / norm * distance, point[1] + direction[1] / norm * distance

    @staticmethod
    def _cubic_bezier(
        p0: Tuple[float, float],
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        p3: Tuple[float, float],
        t: float,
    ) -> Tuple[float, float]:
        u = 1.0 - t
        x = (u ** 3) * p0[0] + 3.0 * (u ** 2) * t * p1[0] + 3.0 * u * (t ** 2) * p2[0] + (t ** 3) * p3[0]
        y = (u ** 3) * p0[1] + 3.0 * (u ** 2) * t * p1[1] + 3.0 * u * (t ** 2) * p2[1] + (t ** 3) * p3[1]
        return x, y

    # ------------------------------------------------------------------
    # Primitive / format helpers
    # ------------------------------------------------------------------
    def _make_osm_node_from_source(self, source_node: Lanelet2Node) -> OSMNode:
        lat = self._safe_float(source_node.lat, None)
        lon = self._safe_float(source_node.lon, None)
        if lat is None or lon is None:
            lat, lon = self._xy_to_latlon_if_needed(self._node_xy(source_node))
        return OSMNode(
            id_=source_node.id_,
            lat=self._format_float(lat),
            lon=self._format_float(lon),
            ele=self._format_float(self._safe_float(getattr(source_node, "ele", 0.0))),
            tag_dict={},
        )

    @staticmethod
    def _copy_tag_dict(tag_dict: Optional[Dict[str, str]]) -> Dict[str, str]:
        if tag_dict is None:
            return {}
        return {str(k): str(v) for k, v in tag_dict.items()}

    @staticmethod
    def _safe_float(value: object, default: Optional[float] = 0.0) -> Optional[float]:
        try:
            return float(value)
        except (TypeError, ValueError):
            if default is None:
                return None
            return float(default)

    @staticmethod
    def _safe_sort_key(value: object):
        try:
            return (0, int(str(value)))
        except (TypeError, ValueError):
            return (1, str(value))

    @staticmethod
    def _format_float(value: float, precision: int = 8) -> str:
        return f"{float(value):.{precision}f}"

    def _initialize_xy_projection_origin(self) -> None:
        """Initialize a local metric XY frame for maps without local_x/local_y.

        The fallback used to return (lon, lat), which made all geometry thresholds
        accidentally operate in degrees.  Endpoint extension parameters such as
        search radius and max distance are intended to be meters, so we project
        lat/lon to an equirectangular local meter frame around the first valid
        map node.  If source nodes already provide local_x/local_y, this origin is
        harmless because _node_xy will use local_x/local_y directly.
        """
        self._xy_origin_lat = None
        self._xy_origin_lon = None
        self._xy_m_per_deg_lat = 111320.0
        self._xy_m_per_deg_lon = 111320.0

        if self._source is None:
            return

        for node in self._source.nodes.values():
            lat = self._safe_float(getattr(node, "lat", None), None)
            lon = self._safe_float(getattr(node, "lon", None), None)
            if lat is None or lon is None:
                continue
            if not math.isfinite(lat) or not math.isfinite(lon):
                continue
            self._xy_origin_lat = lat
            self._xy_origin_lon = lon
            # Good enough for city-scale maps and avoids extra dependencies.
            self._xy_m_per_deg_lat = 111320.0
            self._xy_m_per_deg_lon = 111320.0 * math.cos(math.radians(lat))
            if abs(self._xy_m_per_deg_lon) < 1e-9:
                self._xy_m_per_deg_lon = 111320.0
            LOGGER.info(
                "Lanelet2OSMConverter: using metric XY fallback projection origin lat=%.8f lon=%.8f "
                "(meters_per_deg_lat=%.3f, meters_per_deg_lon=%.3f).",
                self._xy_origin_lat,
                self._xy_origin_lon,
                self._xy_m_per_deg_lat,
                self._xy_m_per_deg_lon,
            )
            return
        self._xy_origin_lat = 0.0
        self._xy_origin_lon = 0.0
        LOGGER.warning(
            "Lanelet2OSMConverter: source has local XY but no geographic reference; "
            "using a synthetic WGS84 origin at 0,0. Relative geometry is preserved, "
            "but the absolute location is unknown."
        )

    def _latlon_to_metric_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if self._xy_origin_lat is None or self._xy_origin_lon is None:
            self._xy_origin_lat = lat
            self._xy_origin_lon = lon
            self._xy_m_per_deg_lat = 111320.0
            self._xy_m_per_deg_lon = 111320.0 * math.cos(math.radians(lat))
            if abs(self._xy_m_per_deg_lon) < 1e-9:
                self._xy_m_per_deg_lon = 111320.0
        x = (lon - self._xy_origin_lon) * self._xy_m_per_deg_lon
        y = (lat - self._xy_origin_lat) * self._xy_m_per_deg_lat
        return x, y

    def _node_xy(self, node: Lanelet2Node) -> Tuple[float, float]:
        local_x = getattr(node, "local_x", None)
        local_y = getattr(node, "local_y", None)
        if local_x is not None and local_y is not None:
            return self._safe_float(local_x), self._safe_float(local_y)

        lat = self._safe_float(getattr(node, "lat", 0.0))
        lon = self._safe_float(getattr(node, "lon", 0.0))
        return self._latlon_to_metric_xy(lat, lon)
