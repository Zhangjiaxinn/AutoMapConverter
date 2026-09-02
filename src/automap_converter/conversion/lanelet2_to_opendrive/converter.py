import logging
import math
import re
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Set, Tuple

from lxml import etree
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from automap_converter.core.config.opendrive_config import OpenDriveConfig, open_drive_config
from automap_converter.formats.lanelet2.map import (
    Multipolygon,
    Node as Lanelet2Node,
    OSMLanelet as Lanelet2Map,
    Way as Lanelet2Way,
    WayRelation,
)

LOGGER = logging.getLogger(__name__)


Point2D = Tuple[float, float]


@dataclass
class RoadGroup:
    """
    Internal intermediate road group.

    road_lanes:
        Lanelets belonging to one OpenDRIVE road.
        Stored from right to left.

    od_road_id:
        OpenDRIVE road id.

    ref_points:
        Reference line points of this OpenDRIVE road.
        In this first version, reference line is the left boundary of the left-most lane.
        Therefore all lanes are written on the right side of lane id 0.
    """
    od_road_id: int
    road_lanes: List[WayRelation]
    ref_points: List[Point2D]
    lane_width_samples: Dict[int, List[Tuple[float, float]]] = field(default_factory=dict)
    length: float = 0.0
    road_xml: Optional[etree._Element] = None
    successors: Set[int] = field(default_factory=set)
    predecessors: Set[int] = field(default_factory=set)
    is_virtual_boundary_road: bool = False
    is_connecting_road: bool = False
    junction_id: Optional[int] = None

    # When several short virtual+virtual roads form one OpenDRIVE connectingRoad,
    # the first road id is kept as the representative road. The following fields
    # keep the original longitudinal sections so geometry / width / lane mappings
    # can be rebuilt as one road.
    is_removed: bool = False
    chain_member_road_ids: List[int] = field(default_factory=list)
    longitudinal_road_lanes: Optional[List[List[WayRelation]]] = None
    longitudinal_ref_points: Optional[List[List[Point2D]]] = None


class Lanelet2OpendriveConverter:
    """
    Convert parsed Lanelet2 OSMLanelet content into OpenDRIVE.

    Input:
        lanelet2_content should be an OSMLanelet object from lanelet2.py.

    This implementation is intentionally close to the CommonRoad -> OpenDRIVE converter idea:
        1. construct roads from lateral adjacent lanelets
        2. build road-level linkage from lanelet-level topology
        3. build coarse junctions from road-level multi-successor / multi-predecessor relation
        4. populate regulatory elements and multipolygons
        5. write XML directly
    """

    def __init__(
        self,
        lanelet2_content: Lanelet2Map,
        step_size: float = 2.0,
        center: bool = False,
        default_lane_type: str = "driving",
        start_road_id: int = 1,
        start_junction_id: int = 100000,
        ref_line_fit: bool = True,
        ref_line_fit_max_error: float = 0.20,
        width_fit: bool = True,
        width_fit_max_error: float = 0.15,
        max_single_connector_length: float = 30.0,
        max_connector_chain_length: float = 80.0,
        nested_connector_front_length: float = 8.0,
        min_split_segment_length: float = 1.0,
        debug_road_ids: Optional[Sequence[int]] = None,
        debug_topology: bool = False,
        config: OpenDriveConfig = open_drive_config,
    ) -> None:
        self.lanelet2_content = lanelet2_content
        self.config = config
        self.step_size = float(step_size)
        self.center = center
        self.default_lane_type = default_lane_type
        self.ref_line_fit = bool(ref_line_fit)
        self.ref_line_fit_max_error = float(ref_line_fit_max_error)
        self.width_fit = bool(width_fit)
        self.width_fit_max_error = float(width_fit_max_error)
        self.max_single_connector_length = float(max_single_connector_length)
        self.max_connector_chain_length = float(max_connector_chain_length)
        self.nested_connector_front_length = float(nested_connector_front_length)
        self.min_split_segment_length = float(min_split_segment_length)
        self.debug_topology = bool(debug_topology)
        self.debug_road_ids: Set[int] = set(int(x) for x in (debug_road_ids or []))

        self.nodes: Dict[str, Lanelet2Node] = self.lanelet2_content.nodes
        self.ways: Dict[str, Lanelet2Way] = self.lanelet2_content.ways
        self.lanelets: Dict[str, WayRelation] = self.lanelet2_content.way_relations
        self.multipolygons: Dict[str, Multipolygon] = self.lanelet2_content.multipolygons
        self.regulatory_elements = self.lanelet2_content.regulatory_elements

        self.origin_lat: Optional[float] = None
        self.origin_lon: Optional[float] = None
        self.origin_local_x: float = 0.0
        self.origin_local_y: float = 0.0

        self.next_road_id = int(start_road_id)
        self.next_junction_id = int(start_junction_id)

        self.root: Optional[etree._Element] = None

        # Conversion state
        self.id_dict: Dict[str, bool] = self.prepare_id_dict()
        self.road_groups: Dict[int, RoadGroup] = {}

        # Mapping tables. These are important for later regulatory_element back-reference.
        self.lanelet_to_road: Dict[str, int] = {}
        self.lanelet_to_lane: Dict[str, int] = {}

        # XML nodes for each converted lanelet lane.
        # These are filled in _write_lanes(), and later populated by populate_lane_links().
        self.lanelet_to_lane_xml: Dict[str, etree._Element] = {}
        self.lanelet_to_lane_link_xml: Dict[str, etree._Element] = {}

        self.way_to_lanelets: Dict[str, Set[str]] = defaultdict(set)
        self.regulatory_to_roads: Dict[str, Set[int]] = defaultdict(set)
        self.multipolygon_to_object: Dict[str, str] = {}
        self._stop_line_cache: Dict[
            Tuple[str, int], Tuple[str, etree._Element, etree._Element]
        ] = {}
        self._traffic_light_pole_cache: Dict[Tuple[str, int], etree._Element] = {}

        # Explicit OpenDRIVE junction-role maps.
        # These are filled by build_topological_connecting_road_chains().
        # They prevent road-level link writing from guessing "outgoing roads"
        # only because a road happens to touch a connectingRoad.
        self.connecting_road_to_incoming: Dict[int, int] = {}
        self.connecting_road_to_outgoing: Dict[int, int] = {}
        self.junction_to_incoming_roads: Dict[int, Set[int]] = defaultdict(set)
        self.junction_to_outgoing_roads: Dict[int, Set[int]] = defaultdict(set)
        self.incoming_road_to_junction: Dict[int, int] = {}
        self.outgoing_road_to_junction: Dict[int, int] = {}

        # Manual road-link overrides created when a road must be physically split.
        # Use case: a road is first selected as a connectingRoad, but it also has
        # multiple successors and therefore must become the incomingRoad of a next
        # junction. To avoid the same OpenDRIVE road being both connectingRoad and
        # incomingRoad, we split it into front/back road ids and override topology:
        #     old road(front, short connectingRoad) -> new road(back, ordinary/incoming main road)
        #     new road(back) -> old successors
        self.manual_road_successors: Dict[int, Set[int]] = {}
        self.manual_road_predecessors: Dict[int, Set[int]] = {}

        # Manual lane-level link overrides for physically split / synthetic roads.
        # Road-level topology after split is stored in manual_road_successors /
        # manual_road_predecessors. Lane-level links must follow the same split
        # topology instead of the original Lanelet2 lanelet.successors, because
        # several OpenDRIVE road pieces may reuse the same Lanelet2 lanelet.
        # Key: (from_road_id, to_road_id), value: [(from_lane_id, to_lane_id), ...].
        self.manual_lane_links: Dict[Tuple[int, int], List[Tuple[int, int]]] = {}

        self.split_front_to_back: Dict[int, int] = {}
        self.split_back_to_front: Dict[int, int] = {}
        # Tail splits are created for multi-predecessor merge junctions:
        #     pred_front -> pred_back(connectingRoad) -> merge/outgoing road
        self.merge_tail_front_to_back: Dict[Tuple[int, int], int] = {}
        # Terminal connector splits are created when a formal connectingRoad has
        # no successor. The old road id remains the short connectingRoad and a
        # new ordinary back road becomes its outgoing road / map boundary tail.
        self.terminal_connector_front_to_back: Dict[int, int] = {}
        self.terminal_connector_back_to_front: Dict[int, int] = {}
        self.conv_time = 0.0

        # Geometry-only fixes added on top of v19:
        # 1) Cache oriented lanelet boundaries so a lanelet whose boundary order
        #    is opposite to both predecessor and successor can be corrected during
        #    OpenDRIVE geometry writing without changing Lanelet2 topology.
        # 2) Write constant lane widths from robust median samples instead of
        #    fitting varying width polynomials, because some Lanelet2 boundaries
        #    are not parallel and TESS NG can amplify varying width into twisted
        #    lane meshes.
        self._oriented_lanelet_boundary_cache: Dict[str, Tuple[List[Point2D], List[Point2D]]] = {}
        self.boundary_orientation_fix_count = 0
        self.constant_width_fallback_count = 0
        self.constant_width_clamp_count = 0

        # Ref-line simplification/smoothing added on top of v25.
        # The converter still uses the left boundary of the left-most lanelet as
        # the road reference line, but before writing OpenDRIVE we remove local
        # short-segment zigzags and smooth interior points. This is meant to
        # reduce offset-line self-intersection in simulators when a 3-lane road
        # is generated by accumulating 10+ meters of constant width from a very
        # jagged left boundary.
        self.refline_simplify_enabled = True
        self.refline_min_segment_length = max(1.20, float(self.step_size) * 0.80)
        self.refline_rdp_tolerance = 0.35
        self.refline_short_turn_threshold_deg = 8.0
        self.refline_smooth_iterations = 2
        self.refline_smooth_alpha = 0.25
        self.refline_simplified_count = 0
        self.refline_removed_point_count = 0
        self.refline_smoothed_count = 0

        self._build_way_to_lanelets_index()

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------

    def convert(self, file_path_out: str) -> None:
        """
        Convert Lanelet2 parsed content into an OpenDRIVE .xodr file.
        """
        start = time.time()

        self.prepare_geometry()
        self.root = self._create_opendrive_root()

        start_lanelet_id = self.select_start_lanelet_id()
        if start_lanelet_id is None:
            raise RuntimeError("No lanelet found in lanelet2_content.way_relations.")

        self.construct_roads([start_lanelet_id])
        self.check_all_visited_and_fix_missing()

        # Build road-level topology first. Then infer OpenDRIVE connectingRoads
        # purely from topology: every direct successor of a multi-successor road
        # starts a connector chain. The chain may continue through one-to-one,
        # length-limited roads. Lanelet boundary type is NOT used as a required
        # condition here.
        self.process_road_linkages()
        self.build_topological_connecting_road_chains()

        # Recompute lane-level and road-level topology after possible chain merging.
        # lanelet_to_road and lanelet_to_lane must be complete before this step.
        self.populate_lane_links()
        self.process_road_linkages(clear_existing=True)

        # Targeted debug: print the lanelet-level predecessor/successor source
        # and the current road-level mappings before XML road links are written.
        self.debug_selected_roads_topology(stage="before_construct_junctions")

        self.construct_junctions_and_links()

        self.populate_regulatory_elements()
        self.construct_regulatory_elements()

        self.construct_multipolygons()

        self.finalize(file_path_out)

        self.conv_time = time.time() - start
        self.print_time()

    # ---------------------------------------------------------------------
    # Basic preparation
    # ---------------------------------------------------------------------

    def prepare_id_dict(self) -> Dict[str, bool]:
        return {str(lanelet_id): False for lanelet_id in self.lanelets.keys()}

    def prepare_geometry(self) -> None:
        """
        Placeholder for future geometry preprocessing.

        In this first version:
        - Node local_x/local_y is used when available.
        - Otherwise lat/lon is projected to local x/y by a simple equirectangular projection.
        - Optional global centering can be added here later.
        """
        if not self.nodes:
            return

        for node in self.nodes.values():
            try:
                lat = float(node.lat)
                lon = float(node.lon)
            except Exception:
                continue

            if self.origin_lat is None or self.origin_lon is None:
                self.origin_lat = lat
                self.origin_lon = lon
                if node.local_x is not None and node.local_y is not None:
                    self.origin_local_x = float(node.local_x)
                    self.origin_local_y = float(node.local_y)
                break

    def _build_way_to_lanelets_index(self) -> None:
        for lanelet_id, lanelet in self.lanelets.items():
            self.way_to_lanelets[str(lanelet.left_way)].add(str(lanelet_id))
            self.way_to_lanelets[str(lanelet.right_way)].add(str(lanelet_id))

    def select_start_lanelet_id(self) -> Optional[str]:
        """
        Select a start lanelet.

        Prefer a lanelet with no predecessor, because BFS from a source is more stable.
        If all lanelets have predecessors, use the first one.
        """
        for lanelet_id, lanelet in self.lanelets.items():
            if not lanelet.predecessors:
                return str(lanelet_id)

        for lanelet_id in self.lanelets.keys():
            return str(lanelet_id)

        return None

    # ---------------------------------------------------------------------
    # Road construction
    # ---------------------------------------------------------------------

    def construct_roads(self, frontier: List[str]) -> None:
        """
        Construct OpenDRIVE roads using BFS.

        Similar idea to CommonRoad converter:
        - take one lanelet as seed
        - extend it laterally to form one OpenDRIVE road
        - add successors/predecessors to frontier
        """
        queue = deque([str(x) for x in frontier])

        while queue:
            lanelet_id = queue.popleft()

            if lanelet_id not in self.lanelets:
                continue

            if self.id_dict.get(lanelet_id, False):
                continue

            seed = self.lanelets[lanelet_id]

            road_lanes = [seed]
            road_lanes = self.extend_road(seed, road_lanes, left=False, append=False)
            road_lanes = self.extend_road(seed, road_lanes, left=True, append=True)

            road_id = self.create_road_from_lanelets(road_lanes)

            self.mark_road_lanes_visited(road_lanes, road_id, queue)

            for lanelet in road_lanes:
                for succ in lanelet.successors:
                    succ = str(succ)
                    if succ in self.lanelets and not self.id_dict.get(succ, False):
                        queue.append(succ)

                for pred in lanelet.predecessors:
                    pred = str(pred)
                    if pred in self.lanelets and not self.id_dict.get(pred, False):
                        queue.append(pred)

    def extend_road(
        self,
        current: WayRelation,
        road_lanes: List[WayRelation],
        left: bool,
        append: bool,
    ) -> List[WayRelation]:
        """
        Extend road laterally.

        road_lanes is stored from right to left.

        For Lanelet2:
        - adjacent_left/adjacent_right mean physically adjacent neighbors.
        - left_neighbors/right_neighbors usually mean lane-change reachable neighbors.
        For road geometry construction, physical adjacency is preferred, so this
        function uses adjacent_left/adjacent_right first and only falls back to
        left_neighbors/right_neighbors.

        Direction check is performed geometrically to avoid adding reverse-direction lanes into the same road.
        """
        if left:
            candidates = list(current.adjacent_left) or list(current.left_neighbors)
        else:
            candidates = list(current.adjacent_right) or list(current.right_neighbors)

        for next_id in candidates:
            next_id = str(next_id)

            if next_id not in self.lanelets:
                continue

            if self.id_dict.get(next_id, False):
                continue

            if any(str(x.id_) == next_id for x in road_lanes):
                continue

            next_lanelet = self.lanelets[next_id]

            if not self.is_same_direction(current, next_lanelet):
                continue

            if append:
                road_lanes.append(next_lanelet)
            else:
                road_lanes.insert(0, next_lanelet)

            return self.extend_road(next_lanelet, road_lanes, left=left, append=append)

        return road_lanes
    def create_road_from_lanelets(self, road_lanes: List[WayRelation]) -> int:
        """
        Create one OpenDRIVE road from a group of lanelets.

        Current geometric simplification:
        - road_lanes is right -> left
        - use left boundary of left-most lanelet as OpenDRIVE reference line
        - write all driving lanes to the right side of lane id 0
        """
        if self.root is None:
            raise RuntimeError("OpenDRIVE root is not initialized.")

        road_id = self.next_road_id
        self.next_road_id += 1

        # right -> left, so left-most is last.
        left_most_lanelet = road_lanes[-1]
        ref_points = self.get_left_points(left_most_lanelet)
        ref_points = self.prepare_refline_points(ref_points)

        if len(ref_points) < 2:
            LOGGER.warning("Skip road %s because reference line has less than 2 points.", road_id)
            return road_id

        road_length = self.polyline_length(ref_points)
        if road_length <= 1e-3:
            LOGGER.warning("Skip road %s because reference line length is too small.", road_id)
            return road_id

        road_group = RoadGroup(
            od_road_id=road_id,
            road_lanes=road_lanes,
            ref_points=ref_points,
            length=road_length,
        )

        road_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{road_id}",
            length=f"{road_length:.6f}",
            id=str(road_id),
            junction="-1",
        )
        road_group.road_xml = road_el

        self._write_road_type(road_el)
        self._write_plan_view(road_el, ref_points)
        self._write_lanes(road_el, road_group)

        self.road_groups[road_id] = road_group

        # Assign OpenDRIVE lane ids.
        # all lanes are placed on right side of reference line:
        # left-most lane -> -1, next to right -> -2, ...
        left_to_right = list(reversed(road_lanes))
        for index, lanelet in enumerate(left_to_right, start=1):
            lanelet_id = str(lanelet.id_)
            lane_id = -index
            self.lanelet_to_road[lanelet_id] = road_id
            self.lanelet_to_lane[lanelet_id] = lane_id

        return road_id

    def mark_road_lanes_visited(
        self,
        road_lanes: List[WayRelation],
        road_id: int,
        frontier: deque,
    ) -> None:
        for lanelet in road_lanes:
            lanelet_id = str(lanelet.id_)
            self.id_dict[lanelet_id] = True
            self.lanelet_to_road[lanelet_id] = road_id

            try:
                frontier.remove(lanelet_id)
            except ValueError:
                pass

    def check_all_visited_and_fix_missing(self) -> None:
        """
        Ensure all lanelets are converted.

        Unlike CommonRoad converter which raises error, this Lanelet2 version continues
        and starts another BFS component if the graph is disconnected.
        """
        missing = [lanelet_id for lanelet_id, visited in self.id_dict.items() if not visited]

        while missing:
            LOGGER.warning(
                "There are %d unvisited lanelets. Start another BFS component from lanelet %s.",
                len(missing),
                missing[0],
            )
            self.construct_roads([missing[0]])
            missing = [lanelet_id for lanelet_id, visited in self.id_dict.items() if not visited]

    def is_virtual_virtual_lanelet(self, lanelet: WayRelation) -> bool:
        """Return True if both Lanelet2 boundaries of this lanelet are virtual."""
        left_marking, right_marking = self.get_lanelet_boundary_markings(lanelet)
        return left_marking == "virtual" and right_marking == "virtual"

    def is_virtual_boundary_road(self, group: RoadGroup) -> bool:
        """Return True if all lanelets represented by this road are virtual+virtual."""
        lanelets = self.iter_group_lanelets(group) if hasattr(self, "iter_group_lanelets") else group.road_lanes
        if not lanelets:
            return False
        return all(self.is_virtual_virtual_lanelet(lanelet) for lanelet in lanelets)

    # ---------------------------------------------------------------------
    # Lane-level topology
    # ---------------------------------------------------------------------

    def populate_lane_links(self) -> None:
        """
        Populate OpenDRIVE lane-level predecessor/successor links.

        In merged connectingRoad chains, several Lanelet2 lanelets may map to the
        same OpenDRIVE <lane> XML node. Therefore this function aggregates topology
        by XML link element first, and writes each <lane><link> only once.

        For physically split/synthetic roads, the original Lanelet2 lanelet links
        are no longer sufficient. Those road pieces reuse lanelets, but their
        OpenDRIVE road-level topology is overridden manually. Therefore we first
        build manual lane-link overrides from manual_road_successors and apply
        them after the original lanelet-derived links are written.
        """
        self.refresh_manual_lane_links()
        link_to_lanelets: Dict[int, List[str]] = defaultdict(list)
        link_id_to_xml: Dict[int, etree._Element] = {}

        for lanelet_id in self.lanelets.keys():
            lanelet_id = str(lanelet_id)
            link_el = self.lanelet_to_lane_link_xml.get(lanelet_id)
            if link_el is None:
                continue
            key = id(link_el)
            link_to_lanelets[key].append(lanelet_id)
            link_id_to_xml[key] = link_el

        for key, lanelet_ids in link_to_lanelets.items():
            link_el = link_id_to_xml[key]

            # Remove possible old link children before repopulating.
            for child in list(link_el):
                link_el.remove(child)

            current_roads = {
                self.lanelet_to_road[lid]
                for lid in lanelet_ids
                if lid in self.lanelet_to_road
            }

            predecessor_lane_ids: List[int] = []
            successor_lane_ids: List[int] = []

            for lanelet_id in lanelet_ids:
                lanelet = self.lanelets.get(str(lanelet_id))
                if lanelet is None:
                    continue

                for pred_id in lanelet.predecessors:
                    pred_id = str(pred_id)
                    pred_road = self.lanelet_to_road.get(pred_id)
                    # Internal predecessor inside the same merged road/lane should not
                    # be written as an external lane link.
                    if pred_road in current_roads:
                        continue
                    pred_lane_id = self.lanelet_to_lane.get(pred_id)
                    if pred_lane_id is not None and pred_lane_id not in predecessor_lane_ids:
                        predecessor_lane_ids.append(pred_lane_id)

                for succ_id in lanelet.successors:
                    succ_id = str(succ_id)
                    succ_road = self.lanelet_to_road.get(succ_id)
                    if succ_road in current_roads:
                        continue
                    succ_lane_id = self.lanelet_to_lane.get(succ_id)
                    if succ_lane_id is not None and succ_lane_id not in successor_lane_ids:
                        successor_lane_ids.append(succ_lane_id)

            for pred_lane_id in sorted(predecessor_lane_ids):
                etree.SubElement(link_el, "predecessor", id=str(pred_lane_id))

            for succ_lane_id in sorted(successor_lane_ids):
                etree.SubElement(link_el, "successor", id=str(succ_lane_id))

        # Finally overwrite lane links for road pairs whose topology was created
        # by physical split / manual road overrides. This prevents split road
        # pieces from inheriting stale Lanelet2 successor/predecessor links.
        self.apply_manual_lane_links()

    def refresh_manual_lane_links(self) -> None:
        """Build lane-level overrides from manual road-level topology."""
        self.manual_lane_links.clear()

        for from_road_id, succs in sorted(self.manual_road_successors.items()):
            if from_road_id not in self.road_groups:
                continue
            for to_road_id in sorted(succs):
                if to_road_id not in self.road_groups:
                    continue
                links = self.compute_lane_links_between_road_groups(from_road_id, to_road_id)
                if not links and self.have_same_lane_layout(from_road_id, to_road_id):
                    links = self.same_layout_lane_links(from_road_id, to_road_id)
                if links:
                    self.manual_lane_links[(from_road_id, to_road_id)] = self.unique_lane_links(links)

    def apply_manual_lane_links(self) -> None:
        """Apply manual lane links to XML <lane><link> elements.

        Successor links of manual-from roads and predecessor links of manual-to
        roads are replaced by manual links. The opposite direction is preserved.
        """
        if not self.manual_lane_links:
            return

        from_roads = {a for a, _ in self.manual_lane_links.keys()}
        to_roads = {b for _, b in self.manual_lane_links.keys()}

        # Remove stale successor links on from roads.
        for road_id in from_roads:
            group = self.road_groups.get(road_id)
            if group is None:
                continue
            for lane_id in self.group_lane_ids(group):
                link_el = self.get_lane_link_xml_by_road_lane(road_id, lane_id)
                if link_el is None:
                    continue
                for child in list(link_el.findall("successor")):
                    link_el.remove(child)

        # Remove stale predecessor links on to roads.
        for road_id in to_roads:
            group = self.road_groups.get(road_id)
            if group is None:
                continue
            for lane_id in self.group_lane_ids(group):
                link_el = self.get_lane_link_xml_by_road_lane(road_id, lane_id)
                if link_el is None:
                    continue
                for child in list(link_el.findall("predecessor")):
                    link_el.remove(child)

        # Add corrected manual successors/predecessors.
        for (from_road_id, to_road_id), links in sorted(self.manual_lane_links.items()):
            for from_lane, to_lane in links:
                from_link = self.get_lane_link_xml_by_road_lane(from_road_id, from_lane)
                if from_link is not None and not self.link_child_exists(from_link, "successor", to_lane):
                    etree.SubElement(from_link, "successor", id=str(to_lane))

                to_link = self.get_lane_link_xml_by_road_lane(to_road_id, to_lane)
                if to_link is not None and not self.link_child_exists(to_link, "predecessor", from_lane):
                    etree.SubElement(to_link, "predecessor", id=str(from_lane))

    @staticmethod
    def link_child_exists(link_el: etree._Element, tag: str, lane_id: int) -> bool:
        return any(child.get("id") == str(lane_id) for child in link_el.findall(tag))

    def get_lane_link_xml_by_road_lane(self, road_id: int, lane_id: int) -> Optional[etree._Element]:
        group = self.road_groups.get(road_id)
        if group is None or group.road_xml is None:
            return None
        lane_el = group.road_xml.find(f".//lane[@id='{lane_id}']")
        if lane_el is None:
            return None
        link_el = lane_el.find("link")
        if link_el is None:
            link_el = etree.SubElement(lane_el, "link")
        return link_el

    def group_lane_ids(self, group: RoadGroup) -> List[int]:
        lane_count = len(group.road_lanes)
        return [-idx for idx in range(1, lane_count + 1)]

    def lane_id_in_group(self, group: RoadGroup, lanelet_id: str) -> Optional[int]:
        left_to_right = list(reversed(group.road_lanes))
        for idx, lanelet in enumerate(left_to_right, start=1):
            if str(lanelet.id_) == str(lanelet_id):
                return -idx
        return None

    def compute_lane_links_between_road_groups(self, from_road_id: int, to_road_id: int) -> List[Tuple[int, int]]:
        """Compute lane links using road-group lane membership, not global lanelet_to_road.

        This is crucial after real split, because synthetic road pieces may reuse
        the same Lanelet2 lanelets while global lanelet_to_road remains assigned
        to the original/front road for regulatory-element stability.
        """
        from_group = self.road_groups.get(from_road_id)
        to_group = self.road_groups.get(to_road_id)
        if from_group is None or to_group is None:
            return []

        to_lanelets = {str(lanelet.id_) for lanelet in self.iter_group_lanelets(to_group)}
        result: List[Tuple[int, int]] = []

        for lanelet in self.iter_group_lanelets(from_group):
            from_lane = self.lane_id_in_group(from_group, str(lanelet.id_))
            if from_lane is None:
                continue
            for succ_id in lanelet.successors:
                succ_id = str(succ_id)
                if succ_id not in to_lanelets:
                    continue
                to_lane = self.lane_id_in_group(to_group, succ_id)
                if to_lane is not None:
                    result.append((from_lane, to_lane))

        return self.unique_lane_links(result)

    def have_same_lane_layout(self, from_road_id: int, to_road_id: int) -> bool:
        a = self.road_groups.get(from_road_id)
        b = self.road_groups.get(to_road_id)
        return a is not None and b is not None and len(a.road_lanes) == len(b.road_lanes)

    def same_layout_lane_links(self, from_road_id: int, to_road_id: int) -> List[Tuple[int, int]]:
        a = self.road_groups.get(from_road_id)
        b = self.road_groups.get(to_road_id)
        if a is None or b is None:
            return []
        lane_count = min(len(a.road_lanes), len(b.road_lanes))
        return [(-idx, -idx) for idx in range(1, lane_count + 1)]

    # ---------------------------------------------------------------------
    # Debug helpers
    # ---------------------------------------------------------------------

    def debug_selected_roads_topology(self, stage: str = "") -> None:
        """
        Print detailed lanelet-level and road-level topology for selected road ids.

        This is intentionally verbose and is used to debug selected roads:
        - which Lanelet2 lanelets are inside the road
        - lanelet predecessor/successor ids
        - how those lanelets map to OpenDRIVE road ids
        - manual split topology overrides
        - explicit junction role maps

        It also expands one-hop road neighbors around the requested ids, because
        the actual wrong relation is often located on an adjacent predecessor or
        successor road rather than on the selected road itself.
        """
        if not self.debug_topology or not self.debug_road_ids:
            return

        road_ids: Set[int] = set(self.debug_road_ids)
        for rid in list(self.debug_road_ids):
            group = self.road_groups.get(rid)
            if group is None:
                continue
            road_ids.update(group.predecessors)
            road_ids.update(group.successors)
            road_ids.update(self.manual_road_predecessors.get(rid, set()))
            road_ids.update(self.manual_road_successors.get(rid, set()))

        # Also include roads whose manual successors/predecessors point to the target.
        for rid, succs in self.manual_road_successors.items():
            if succs & self.debug_road_ids:
                road_ids.add(rid)
        for rid, preds in self.manual_road_predecessors.items():
            if preds & self.debug_road_ids:
                road_ids.add(rid)

        title = f"[TOPOLOGY DEBUG {stage}]" if stage else "[TOPOLOGY DEBUG]"
        print("\n" + "=" * 100)
        print(f"{title} requested={sorted(self.debug_road_ids)} expanded={sorted(road_ids)}")
        print("=" * 100)

        print("[DEBUG] explicit junction-role maps around selected roads:")
        for rid in sorted(road_ids):
            roles = []
            if rid in self.incoming_road_to_junction:
                roles.append(f"incoming_of_J{self.incoming_road_to_junction[rid]}")
            if rid in self.outgoing_road_to_junction:
                roles.append(f"outgoing_of_J{self.outgoing_road_to_junction[rid]}")
            if rid in self.connecting_road_to_incoming:
                roles.append(f"connector_from_{self.connecting_road_to_incoming[rid]}")
            if rid in self.connecting_road_to_outgoing:
                roles.append(f"connector_to_{self.connecting_road_to_outgoing[rid]}")
            if roles:
                print(f"  road {rid}: " + ", ".join(roles))

        print("[DEBUG] split maps:")
        print(f"  split_front_to_back={self._filter_mapping_for_debug(self.split_front_to_back, road_ids)}")
        print(f"  split_back_to_front={self._filter_mapping_for_debug(self.split_back_to_front, road_ids)}")
        print(f"  merge_tail_front_to_back={self._filter_tuple_mapping_for_debug(self.merge_tail_front_to_back, road_ids)}")

        for road_id in sorted(road_ids):
            self.debug_road_topology(road_id)

        print("=" * 100 + "\n")

    @staticmethod
    def _filter_mapping_for_debug(mapping: Dict[int, int], road_ids: Set[int]) -> Dict[int, int]:
        return {k: v for k, v in mapping.items() if k in road_ids or v in road_ids}

    @staticmethod
    def _filter_tuple_mapping_for_debug(mapping: Dict[Tuple[int, int], int], road_ids: Set[int]) -> Dict[Tuple[int, int], int]:
        return {k: v for k, v in mapping.items() if v in road_ids or any(x in road_ids for x in k)}

    def debug_road_topology(self, road_id: int) -> None:
        group = self.road_groups.get(road_id)
        print("\n" + "-" * 100)
        if group is None:
            print(f"[DEBUG] road {road_id}: not found")
            return

        print(f"[DEBUG] road {road_id}")
        print(f"  is_removed={group.is_removed}")
        print(f"  is_connecting_road={group.is_connecting_road}, junction_id={group.junction_id}")
        print(f"  length={group.length:.6f}, lane_count={len(group.road_lanes)}")
        print(f"  road-level predecessors={sorted(group.predecessors)}")
        print(f"  road-level successors  ={sorted(group.successors)}")
        print(f"  manual predecessors={sorted(self.manual_road_predecessors.get(road_id, set()))}")
        print(f"  manual successors  ={sorted(self.manual_road_successors.get(road_id, set()))}")
        manual_lane_out = {k: v for k, v in self.manual_lane_links.items() if k[0] == road_id or k[1] == road_id}
        if manual_lane_out:
            print(f"  manual lane links={manual_lane_out}")
        print(f"  explicit incoming junction={self.incoming_road_to_junction.get(road_id)}")
        print(f"  explicit outgoing junction={self.outgoing_road_to_junction.get(road_id)}")
        print(f"  connector incoming={self.connecting_road_to_incoming.get(road_id)}")
        print(f"  connector outgoing={self.connecting_road_to_outgoing.get(road_id)}")

        if group.ref_points:
            print(f"  ref start={group.ref_points[0]}, ref end={group.ref_points[-1]}")

        lanelets = self.iter_group_lanelets(group)
        print(f"  represented lanelets={ [str(ll.id_) for ll in lanelets] }")
        for lanelet in lanelets:
            llid = str(lanelet.id_)
            pred_lanelets = [str(x) for x in lanelet.predecessors]
            succ_lanelets = [str(x) for x in lanelet.successors]
            pred_roads = [(pid, self.lanelet_to_road.get(pid)) for pid in pred_lanelets]
            succ_roads = [(sid, self.lanelet_to_road.get(sid)) for sid in succ_lanelets]
            left_mark, right_mark = self.get_lanelet_boundary_markings(lanelet)
            print(f"    lanelet {llid}: mapped_road={self.lanelet_to_road.get(llid)}, lane={self.lanelet_to_lane.get(llid)}")
            print(f"      boundaries: left_way={lanelet.left_way}({left_mark}), right_way={lanelet.right_way}({right_mark})")
            print(f"      predecessor lanelets={pred_lanelets}")
            print(f"      successor lanelets  ={succ_lanelets}")
            print(f"      predecessor roads={pred_roads}")
            print(f"      successor roads  ={succ_roads}")

    # ---------------------------------------------------------------------
    # Road linkage and junction
    # ---------------------------------------------------------------------

    def process_road_linkages(self, clear_existing: bool = False) -> None:
        """
        Convert lanelet-level predecessor/successor into road-level predecessor/successor.

        If clear_existing=True, road-level predecessor/successor sets are rebuilt.
        This is required after connecting-road chain merging because several old
        short roads may have been collapsed into one representative road.
        """
        if clear_existing:
            for group in self.road_groups.values():
                group.successors.clear()
                group.predecessors.clear()

        for road_id, group in self.road_groups.items():
            if group.is_removed:
                continue

            for lanelet in self.iter_group_lanelets(group):
                for succ in lanelet.successors:
                    succ = str(succ)
                    succ_road = self.lanelet_to_road.get(succ)
                    if succ_road is not None and succ_road != road_id:
                        group.successors.add(succ_road)

                for pred in lanelet.predecessors:
                    pred = str(pred)
                    pred_road = self.lanelet_to_road.get(pred)
                    if pred_road is not None and pred_road != road_id:
                        group.predecessors.add(pred_road)

        # Apply explicit topology overrides from physical road splitting. These
        # overrides must be applied after lanelet-derived topology is rebuilt,
        # because split front/back roads deliberately share Lanelet2 lanelets and
        # cannot be inferred correctly from lanelet_to_road alone.
        for rid, succs in self.manual_road_successors.items():
            group = self.road_groups.get(rid)
            if group is not None and not group.is_removed:
                group.successors = set(succs)
        for rid, preds in self.manual_road_predecessors.items():
            group = self.road_groups.get(rid)
            if group is not None and not group.is_removed:
                group.predecessors = set(preds)


    # ---------------------------------------------------------------------
    # ConnectingRoad chain merging
    # ---------------------------------------------------------------------

    def iter_group_lanelets(self, group: RoadGroup) -> List[WayRelation]:
        """
        Return all Lanelet2 lanelets represented by a RoadGroup.

        For a normal road, this is just group.road_lanes.
        For a merged connectingRoad chain, this is the flattened list of all
        longitudinal sections and their lateral lanelets.
        """
        if group.longitudinal_road_lanes:
            result: List[WayRelation] = []
            for section_lanes in group.longitudinal_road_lanes:
                result.extend(section_lanes)
            return result
        return list(group.road_lanes)

    def build_topological_connecting_road_chains(self) -> None:
        """
        Build connectingRoad chains by topology only.

        Core idea:
        - A road with more than one successor is an incoming-road trigger.
        - Each direct successor of that road starts one OpenDRIVE connectingRoad chain.
        - The chain may continue through one-to-one short roads.
        - Lanelet boundary type, including virtual+virtual, is not a required condition.

        After initial chains are clustered by same-role sharing:
        - same incoming road, or
        - same outgoing road,
        each component becomes one OpenDRIVE junction.

        Important in this version:
        - single-incoming components are NOT shrunk back to direct successors.
          The chain boundary is controlled only by topology and length guards:
          max_single_connector_length and max_connector_chain_length.
        """
        # Reset connector state and explicit junction-role maps. Existing road
        # geometry remains unchanged until final chains are decided and applied.
        self.connecting_road_to_incoming.clear()
        self.connecting_road_to_outgoing.clear()
        self.junction_to_incoming_roads.clear()
        self.junction_to_outgoing_roads.clear()
        self.incoming_road_to_junction.clear()
        self.outgoing_road_to_junction.clear()

        for group in self.road_groups.values():
            group.is_virtual_boundary_road = self.is_virtual_boundary_road(group)
            group.is_connecting_road = False
            group.junction_id = None

        raw_chains = self.collect_topological_connector_chains()
        if not raw_chains:
            return

        # If a road appears both as a connector-chain member and as an incoming
        # trigger for another multi-successor junction, split it physically into:
        #     front part: previous junction's connectingRoad
        #     back part : next junction's incomingRoad
        # Then rebuild topology and collect chains again. Repeat a few times to
        # handle cascaded split points.
        for _ in range(8):
            changed = False

            # 1) If an already selected connector later becomes an incoming road
            # for another split, split the connector into a short front connector
            # and a normal back/incoming road.
            if self.split_nested_connector_incomings(raw_chains):
                changed = True
                self.process_road_linkages(clear_existing=True)
                raw_chains = self.collect_topological_connector_chains()
                if not raw_chains:
                    return

            # NOTE: Do NOT split an incoming tail merely because the branch start
            # road already has multiple predecessors. That old v14 branch-tail
            # heuristic could hide lane-level mapping errors and produced synthetic
            # connectors such as 1848. A connectingRoad with multiple predecessors
            # should be debugged/fixed through correct lane-level links and the
            # normal merge-junction logic, not by blindly cutting the incoming road.

            if not changed:
                break

        components = self.cluster_connector_chain_indices(raw_chains)

        # Apply topology-based chain merges directly. In this version there is no
        # single-incoming shrink step; connector-chain length is limited by
        # trace_topological_connector_chain().
        for chain in raw_chains:
            members = list(chain["members"])
            if not members:
                continue

            # If a chain has already been collapsed by another chain, keep only its start.
            members = [rid for rid in members if rid in self.road_groups and not self.road_groups[rid].is_removed]
            if not members:
                continue

            # Keep merging conservative for lane ids: all longitudinal sections must
            # have the same lateral lane count. If not, keep only the start road as
            # the connector representative for now; full multi-laneSection chain
            # merging can be added later.
            if len(members) > 1:
                lane_counts = {len(self.road_groups[rid].road_lanes) for rid in members}
                if len(lane_counts) == 1:
                    self.merge_road_group_chain(members)
                else:
                    members = [members[0]]
                    chain["members"] = members

        # Rebuild topology after any physical chain merge. This is needed before
        # assigning junction ids and before road links are written.
        self.process_road_linkages(clear_existing=True)

        # Assign one junction id per component and mark representative road of each
        # chain as connectingRoad. The representative is always the first member.
        #
        # Important: also build explicit incoming/outgoing maps here. Later XML
        # link writing must use these maps, not "any road adjacent to a connector",
        # otherwise roads restored by the single-incoming shrink rule can be
        # incorrectly written as predecessor=the junction.
        for component in components:
            # Drop chains that became empty for safety.
            valid_indices = [idx for idx in component if raw_chains[idx].get("members")]
            if not valid_indices:
                continue

            junction_id = self.next_junction_id
            self.next_junction_id += 1

            for idx in valid_indices:
                chain = raw_chains[idx]
                members = chain.get("members")
                if not members:
                    continue

                rep_id = members[0]
                group = self.road_groups.get(rep_id)
                if group is None or group.is_removed:
                    continue

                incoming = chain.get("incoming")
                outgoing = chain.get("outgoing")

                # Recompute outgoing after chain merge/topology rebuild whenever possible.
                # For a merged chain, rep_id now represents all members; for a shrunken
                # one-to-many case, rep_id is the direct successor road.
                current_outgoing = self.get_chain_outgoing_road_id([rep_id])
                if isinstance(current_outgoing, int):
                    outgoing = current_outgoing

                group.is_connecting_road = True
                group.junction_id = junction_id

                if isinstance(incoming, int):
                    self.connecting_road_to_incoming[rep_id] = incoming
                    self.junction_to_incoming_roads[junction_id].add(incoming)
                    self.incoming_road_to_junction[incoming] = junction_id

                if isinstance(outgoing, int):
                    self.connecting_road_to_outgoing[rep_id] = outgoing
                    self.junction_to_outgoing_roads[junction_id].add(outgoing)
                    self.outgoing_road_to_junction[outgoing] = junction_id

        # Handle the dual topology case: a road with multiple predecessors is a
        # merge/outgoing road and must also be represented through a junction.
        # If its predecessors are ordinary one-to-one roads, split their tail part
        # into connectingRoads, then build a normal single-direction OpenDRIVE
        # junction: incoming_front -> connecting_tail -> outgoing_merge_road.
        self.process_road_linkages(clear_existing=True)
        self.build_merge_junctions_for_multi_predecessors()

        # Final role-consistency guard:
        # an ordinary road may sit between two topology events, but it should not
        # be wrapped by the same junction on both ends:
        #     J -> ordinary road R -> J
        # If that happens, the successor-side connections of R are split into a
        # new junction, yielding:
        #     J_old -> R -> J_new
        self.resolve_same_junction_wrapped_ordinary_roads()

        # Last structural cleanup: a formal connectingRoad without a successor is
        # usually too much geometry swallowed by the junction. Split off its tail
        # as an ordinary terminal/outgoing road so the connectingRoad still has
        # the standard incomingRoad -> connectingRoad -> outgoingRoad form.
        self.split_terminal_connecting_roads_without_successor()

        self.warn_connecting_roads_with_complex_links()

    def resolve_same_junction_wrapped_ordinary_roads(self, max_passes: int = 8) -> None:
        """
        Split role mappings when an ordinary road is wrapped by the same junction
        at both ends.

        Problem pattern after clustering / merge handling:
            ordinary road R:
                predecessor = junction J
                successor   = junction J

        This means the same OpenDRIVE junction component swallowed two adjacent
        topology events with R in the middle. R should instead be the ordinary
        boundary between them:
            predecessor = junction J_old
            successor   = junction J_new

        We do not change geometry here. We only move the successor-side
        connection(s), i.e. connectingRoads whose explicit incoming road is R,
        from J_old to a new junction id. Then explicit role maps are rebuilt from
        the connectingRoad records.
        """
        for _ in range(max_passes):
            changed = False

            # Work on a snapshot because moving connections updates maps.
            conflicts: List[Tuple[int, int]] = []
            for road_id, group in sorted(self.road_groups.items()):
                if group.is_removed or group.is_connecting_road:
                    continue
                pred_junction = self.outgoing_road_to_junction.get(road_id)
                succ_junction = self.incoming_road_to_junction.get(road_id)
                if pred_junction is not None and pred_junction == succ_junction:
                    conflicts.append((road_id, pred_junction))

            if not conflicts:
                break

            for road_id, old_junction_id in conflicts:
                # Move only successor-side connections of this ordinary road:
                # connections where road_id is the incomingRoad and some connector
                # road currently belongs to the same old junction.
                connectors_to_move = []
                for conn_id, incoming_id in sorted(self.connecting_road_to_incoming.items()):
                    if incoming_id != road_id:
                        continue
                    conn_group = self.road_groups.get(conn_id)
                    if conn_group is None or conn_group.is_removed:
                        continue
                    if not conn_group.is_connecting_road:
                        continue
                    if conn_group.junction_id == old_junction_id:
                        connectors_to_move.append(conn_id)

                if not connectors_to_move:
                    LOGGER.warning(
                        "Road %s is wrapped by junction %s on both ends, but no successor-side connectors were found to move.",
                        road_id,
                        old_junction_id,
                    )
                    continue

                new_junction_id = self.next_junction_id
                self.next_junction_id += 1

                for conn_id in connectors_to_move:
                    self.road_groups[conn_id].junction_id = new_junction_id

                LOGGER.info(
                    "Split same-junction wrapper around ordinary road %s: moved connectors %s from junction %s to new junction %s.",
                    road_id,
                    connectors_to_move,
                    old_junction_id,
                    new_junction_id,
                )
                changed = True

            if changed:
                self.rebuild_explicit_junction_role_maps_from_connecting_roads()
            else:
                break

    def rebuild_explicit_junction_role_maps_from_connecting_roads(self) -> None:
        """
        Rebuild explicit incoming/outgoing role maps from each connectingRoad's
        current junction_id and explicit incoming/outgoing road records.

        This is used after moving connection roads between junctions without
        changing road geometry/topology.
        """
        self.junction_to_incoming_roads.clear()
        self.junction_to_outgoing_roads.clear()
        self.incoming_road_to_junction.clear()
        self.outgoing_road_to_junction.clear()

        for conn_id, group in sorted(self.road_groups.items()):
            if group.is_removed or not group.is_connecting_road or group.junction_id is None:
                continue
            junction_id = group.junction_id

            incoming = self.connecting_road_to_incoming.get(conn_id)
            if isinstance(incoming, int):
                self.junction_to_incoming_roads[junction_id].add(incoming)
                old = self.incoming_road_to_junction.get(incoming)
                if old is not None and old != junction_id:
                    LOGGER.warning(
                        "Incoming road %s is assigned to multiple junctions (%s and %s); keeping the later assignment.",
                        incoming,
                        old,
                        junction_id,
                    )
                self.incoming_road_to_junction[incoming] = junction_id

            outgoing = self.connecting_road_to_outgoing.get(conn_id)
            if isinstance(outgoing, int):
                self.junction_to_outgoing_roads[junction_id].add(outgoing)
                old = self.outgoing_road_to_junction.get(outgoing)
                if old is not None and old != junction_id:
                    LOGGER.warning(
                        "Outgoing road %s is assigned to multiple junctions (%s and %s); keeping the later assignment.",
                        outgoing,
                        old,
                        junction_id,
                    )
                self.outgoing_road_to_junction[outgoing] = junction_id

    def split_nested_connector_incomings(self, chains: List[Dict[str, object]]) -> bool:
        """
        Split roads that would otherwise become both connectingRoad and incomingRoad.

        OpenDRIVE modeling target:
            previous incoming road -> C_front as the short connectingRoad of the previous junction
            C_front -> C_back as an ordinary/main-road continuation
            C_back -> next junction -> branch connectors

        This is the key separation discussed in the design: when the candidate
        connectingRoad itself has multiple successors, the front part remains the
        auxiliary connectingRoad, while the back part becomes the ordinary incoming
        road of the next topology junction.

        Detection:
            - Build the set of all roads that are members of any connector chain.
            - If another chain uses one of those roads as its incoming road, that road
              would become a nested-junction incomingRoad while also being a connectingRoad.
            - Physically split that road into front/back road ids and use the back id
              as the future incoming road for the next junction.
        """
        connector_members: Set[int] = set()
        for chain in chains:
            for rid in chain.get("members", []):
                if isinstance(rid, int):
                    connector_members.add(rid)

        nested_incomings = sorted(
            {
                chain.get("incoming")
                for chain in chains
                if isinstance(chain.get("incoming"), int)
                and chain.get("incoming") in connector_members
            }
        )

        changed = False
        for road_id in nested_incomings:
            if road_id in self.split_front_to_back:
                continue
            if self.split_road_for_nested_incoming(int(road_id)):
                changed = True
        return changed

    def split_road_for_nested_incoming(self, road_id: int) -> bool:
        """
        Physically split one road into front/back OpenDRIVE roads.

        The original road id is kept as the front part. A new road id is created
        for the back part. Topology is overridden as:
            predecessors -> short front connectingRoad -> back ordinary/incoming road
            back ordinary/incoming road -> original successors

        Unlike the earlier midpoint split, this version intentionally keeps the
        front segment short. That matches the OpenDRIVE role: connectingRoad is
        only the auxiliary segment that leaves the previous junction; the remaining
        back segment is the main-road continuation that can become the incomingRoad
        of the next junction.

        This is an engineering split at OpenDRIVE-road level. It keeps the original
        Lanelet2 lane objects for both front and back road XML, but lanelet_to_road
        remains mapped to the front road so regulatory back-references stay stable.
        Road-level topology for split roads is maintained by manual overrides.
        """
        if self.root is None:
            return False
        if road_id not in self.road_groups:
            return False
        group = self.road_groups[road_id]
        if group.is_removed or group.length <= 1e-3 or len(group.ref_points) < 2:
            return False
        if len(group.successors) <= 1:
            return False

        old_successors = set(group.successors)
        old_predecessors = set(group.predecessors)
        if not old_successors:
            return False

        # Keep the previous-junction connectingRoad short and put the rest of the
        # geometry into the ordinary back road. This prevents a long road from being
        # swallowed as a connectingRoad when it is actually the main route before the
        # next split/merge topology.
        min_seg = max(0.5, self.min_split_segment_length)
        if group.length <= 2.0 * min_seg:
            split_s = group.length * 0.5
        else:
            front_len = min(self.nested_connector_front_length, group.length * 0.35)
            front_len = max(min_seg, front_len)
            split_s = min(front_len, group.length - min_seg)

        front_pts, back_pts = self.split_polyline_at_s(group.ref_points, split_s)
        front_pts = self.prepare_refline_points(front_pts)
        back_pts = self.prepare_refline_points(back_pts)
        if len(front_pts) < 2 or len(back_pts) < 2:
            return False

        back_id = self.next_road_id
        self.next_road_id += 1

        # Remove old XML for the original road; it will be rebuilt as the front part.
        if group.road_xml is not None and group.road_xml.getparent() is not None:
            group.road_xml.getparent().remove(group.road_xml)

        # Rebuild original/front road.
        group.ref_points = front_pts
        group.length = self.polyline_length(front_pts)
        group.successors = {back_id}
        group.predecessors = set(old_predecessors)
        group.lane_width_samples = {}
        front_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{road_id}",
            length=f"{group.length:.6f}",
            id=str(road_id),
            junction="-1",
        )
        group.road_xml = front_el
        self._write_road_type(front_el)
        self._write_plan_view(front_el, group.ref_points)
        self._write_lanes(front_el, group)

        # Create back road. It reuses the same lateral lane objects for geometry
        # writing, but we restore global lanelet mappings immediately afterwards.
        back_group = RoadGroup(
            od_road_id=back_id,
            road_lanes=list(group.road_lanes),
            ref_points=back_pts,
            length=self.polyline_length(back_pts),
        )
        back_group.predecessors = {road_id}
        back_group.successors = set(old_successors)
        back_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{back_id}_split_from_{road_id}",
            length=f"{back_group.length:.6f}",
            id=str(back_id),
            junction="-1",
        )
        back_group.road_xml = back_el
        self._write_road_type(back_el)
        self._write_plan_view(back_el, back_group.ref_points)

        affected_lanelets = [str(lanelet.id_) for lanelet in back_group.road_lanes]
        saved_maps = {
            lid: (
                self.lanelet_to_road.get(lid),
                self.lanelet_to_lane.get(lid),
                self.lanelet_to_lane_xml.get(lid),
                self.lanelet_to_lane_link_xml.get(lid),
            )
            for lid in affected_lanelets
        }
        self._write_lanes(back_el, back_group)
        for lid, (road_val, lane_val, lane_xml_val, link_xml_val) in saved_maps.items():
            if road_val is None:
                self.lanelet_to_road.pop(lid, None)
            else:
                self.lanelet_to_road[lid] = road_val
            if lane_val is None:
                self.lanelet_to_lane.pop(lid, None)
            else:
                self.lanelet_to_lane[lid] = lane_val
            if lane_xml_val is None:
                self.lanelet_to_lane_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_xml[lid] = lane_xml_val
            if link_xml_val is None:
                self.lanelet_to_lane_link_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_link_xml[lid] = link_xml_val

        self.road_groups[back_id] = back_group
        self.split_front_to_back[road_id] = back_id
        self.split_back_to_front[back_id] = road_id

        # Manual topology overrides. These are reapplied after every topology rebuild.
        self.manual_road_predecessors[road_id] = set(old_predecessors)
        self.manual_road_successors[road_id] = {back_id}
        self.manual_road_predecessors[back_id] = {road_id}
        self.manual_road_successors[back_id] = set(old_successors)

        # Successor roads should now see the back road as predecessor, not the front.
        for succ_id in old_successors:
            succ_group = self.road_groups.get(succ_id)
            if succ_group is None:
                continue
            base_preds = set(self.manual_road_predecessors.get(succ_id, succ_group.predecessors))
            base_preds.discard(road_id)
            base_preds.add(back_id)
            self.manual_road_predecessors[succ_id] = base_preds

        LOGGER.info(
            "Split nested connector/incoming road %s into front=%s and back=%s; successors=%s",
            road_id,
            road_id,
            back_id,
            sorted(old_successors),
        )
        return True

    def build_merge_junctions_for_multi_predecessors(self) -> None:
        """
        Build main-logic junctions for roads with multiple predecessors.

        This replaces the old fallback behavior for merge topology:
            P1 -> O
            P2 -> O

        If some predecessors are already connectingRoads of one existing junction,
        ordinary predecessors are split at their tails and added into that same
        junction. This is important for T/ramp-like areas where one branch was
        already represented as a split-junction connector while the same outgoing
        road also receives ordinary predecessors.
        """
        for _ in range(8):
            self.process_road_linkages(clear_existing=True)
            changed = False

            candidates = []
            for out_id, out_group in sorted(self.road_groups.items()):
                if out_group.is_removed or out_group.is_connecting_road:
                    continue
                if len(out_group.predecessors) > 1:
                    candidates.append(out_id)

            if not candidates:
                break

            for out_id in candidates:
                out_group = self.road_groups.get(out_id)
                if out_group is None or out_group.is_removed:
                    continue
                preds = sorted(pid for pid in out_group.predecessors if pid in self.road_groups and not self.road_groups[pid].is_removed)
                if len(preds) < 2:
                    continue

                connector_preds = [pid for pid in preds if self.road_groups[pid].is_connecting_road]
                ordinary_preds = [pid for pid in preds if not self.road_groups[pid].is_connecting_road]
                pred_junctions = {
                    self.road_groups[pid].junction_id
                    for pid in connector_preds
                    if self.road_groups[pid].junction_id is not None
                }

                # Case 1: at least one predecessor is already a connectingRoad of
                # exactly one junction. Reuse that junction and add tail connectors
                # for every ordinary predecessor that also feeds the same outgoing road.
                if connector_preds and len(pred_junctions) == 1:
                    junction_id = next(iter(pred_junctions))
                    connector_ids = list(connector_preds)
                    incoming_ids = [self.connecting_road_to_incoming.get(cid) for cid in connector_preds]

                    can_build = True
                    for pred_id in ordinary_preds:
                        pred_group = self.road_groups.get(pred_id)
                        if pred_group is None or pred_group.is_removed:
                            can_build = False
                            break
                        if len(pred_group.successors) != 1 or out_id not in pred_group.successors:
                            can_build = False
                            break
                        back_id = self.merge_tail_front_to_back.get((pred_id, out_id))
                        if back_id is None:
                            back_id = self.split_road_tail_for_merge(pred_id, out_id)
                            if back_id is None:
                                can_build = False
                                break
                            changed = True
                        connector_ids.append(back_id)
                        incoming_ids.append(pred_id)

                    if not can_build:
                        continue

                    self.process_road_linkages(clear_existing=True)
                    self.junction_to_outgoing_roads[junction_id].add(out_id)
                    self.outgoing_road_to_junction[out_id] = junction_id

                    for conn_id, incoming_id in zip(connector_ids, incoming_ids):
                        conn_group = self.road_groups.get(conn_id)
                        if conn_group is None or conn_group.is_removed:
                            continue
                        conn_group.is_connecting_road = True
                        conn_group.junction_id = junction_id
                        if isinstance(incoming_id, int):
                            self.connecting_road_to_incoming[conn_id] = incoming_id
                            self.junction_to_incoming_roads[junction_id].add(incoming_id)
                            self.incoming_road_to_junction[incoming_id] = junction_id
                        self.connecting_road_to_outgoing[conn_id] = out_id
                    changed = True
                    continue

                # Case 2: all merge predecessors are already connectingRoads of the same junction.
                if connector_preds and not ordinary_preds and len(pred_junctions) == 1:
                    junction_id = next(iter(pred_junctions))
                    self.junction_to_outgoing_roads[junction_id].add(out_id)
                    self.outgoing_road_to_junction[out_id] = junction_id
                    for conn_id in connector_preds:
                        self.connecting_road_to_outgoing[conn_id] = out_id
                    changed = True
                    continue

                # Case 3: direct ordinary predecessors merge into O. Split each ordinary
                # one-to-one predecessor tail into a connectingRoad and create a new junction.
                if connector_preds:
                    # Multiple connector predecessor junctions are ambiguous; leave a warning.
                    continue

                connector_ids = []
                incoming_ids = []
                can_build = True
                for pred_id in ordinary_preds:
                    pred_group = self.road_groups.get(pred_id)
                    if pred_group is None or pred_group.is_removed:
                        can_build = False
                        break
                    if len(pred_group.successors) != 1 or out_id not in pred_group.successors:
                        can_build = False
                        break

                    back_id = self.merge_tail_front_to_back.get((pred_id, out_id))
                    if back_id is None:
                        back_id = self.split_road_tail_for_merge(pred_id, out_id)
                        if back_id is None:
                            can_build = False
                            break
                        changed = True
                    connector_ids.append(back_id)
                    incoming_ids.append(pred_id)

                if not can_build or len(connector_ids) < 2:
                    continue

                self.process_road_linkages(clear_existing=True)
                junction_id = self.next_junction_id
                self.next_junction_id += 1
                self.junction_to_outgoing_roads[junction_id].add(out_id)
                self.outgoing_road_to_junction[out_id] = junction_id

                for incoming_id, conn_id in zip(incoming_ids, connector_ids):
                    conn_group = self.road_groups.get(conn_id)
                    if conn_group is None or conn_group.is_removed:
                        continue
                    conn_group.is_connecting_road = True
                    conn_group.junction_id = junction_id
                    self.connecting_road_to_incoming[conn_id] = incoming_id
                    self.connecting_road_to_outgoing[conn_id] = out_id
                    self.junction_to_incoming_roads[junction_id].add(incoming_id)
                    self.incoming_road_to_junction[incoming_id] = junction_id
                changed = True

            if not changed:
                break

        self.process_road_linkages(clear_existing=True)

    def split_road_tail_for_merge(self, road_id: int, outgoing_id: int) -> Optional[int]:
        """
        Split an ordinary predecessor road into front/back for a merge junction.

        The original road id is kept as the incoming/front road. A new road id is
        created for the tail/back road, which becomes the OpenDRIVE connectingRoad:
            road_id(front/incoming) -> back_id(connectingRoad) -> outgoing_id
        """
        if self.root is None:
            return None
        if road_id not in self.road_groups or outgoing_id not in self.road_groups:
            return None
        if (road_id, outgoing_id) in self.merge_tail_front_to_back:
            return self.merge_tail_front_to_back[(road_id, outgoing_id)]

        group = self.road_groups[road_id]
        if group.is_removed or group.is_connecting_road:
            return None
        if len(group.ref_points) < 2 or group.length <= 1e-3:
            return None
        if len(group.successors) != 1 or outgoing_id not in group.successors:
            return None

        old_predecessors = set(group.predecessors)
        # Keep the tail connector reasonably short but non-degenerate.
        tail_len = min(max(5.0, group.length * 0.25), group.length * 0.5)
        split_s = max(1e-6, group.length - tail_len)
        front_pts, back_pts = self.split_polyline_at_s(group.ref_points, split_s)
        front_pts = self.prepare_refline_points(front_pts)
        back_pts = self.prepare_refline_points(back_pts)
        if len(front_pts) < 2 or len(back_pts) < 2:
            return None

        back_id = self.next_road_id
        self.next_road_id += 1

        if group.road_xml is not None and group.road_xml.getparent() is not None:
            group.road_xml.getparent().remove(group.road_xml)

        # Rebuild original/front road.
        group.ref_points = front_pts
        group.length = self.polyline_length(front_pts)
        group.successors = {back_id}
        group.predecessors = set(old_predecessors)
        group.lane_width_samples = {}
        front_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{road_id}",
            length=f"{group.length:.6f}",
            id=str(road_id),
            junction="-1",
        )
        group.road_xml = front_el
        self._write_road_type(front_el)
        self._write_plan_view(front_el, group.ref_points)
        self._write_lanes(front_el, group)

        # Create tail/back connecting road.
        back_group = RoadGroup(
            od_road_id=back_id,
            road_lanes=list(group.road_lanes),
            ref_points=back_pts,
            length=self.polyline_length(back_pts),
        )
        back_group.predecessors = {road_id}
        back_group.successors = {outgoing_id}
        back_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{back_id}_merge_tail_from_{road_id}",
            length=f"{back_group.length:.6f}",
            id=str(back_id),
            junction="-1",
        )
        back_group.road_xml = back_el
        self._write_road_type(back_el)
        self._write_plan_view(back_el, back_group.ref_points)

        # Writing lanes for the synthetic tail reuses lanelet ids. Restore the global
        # lanelet maps afterwards so regulatory references continue to point to the
        # original/front road. Road-level topology is handled by manual overrides.
        affected_lanelets = [str(lanelet.id_) for lanelet in back_group.road_lanes]
        saved_maps = {
            lid: (
                self.lanelet_to_road.get(lid),
                self.lanelet_to_lane.get(lid),
                self.lanelet_to_lane_xml.get(lid),
                self.lanelet_to_lane_link_xml.get(lid),
            )
            for lid in affected_lanelets
        }
        self._write_lanes(back_el, back_group)
        for lid, (road_val, lane_val, lane_xml_val, link_xml_val) in saved_maps.items():
            if road_val is None:
                self.lanelet_to_road.pop(lid, None)
            else:
                self.lanelet_to_road[lid] = road_val
            if lane_val is None:
                self.lanelet_to_lane.pop(lid, None)
            else:
                self.lanelet_to_lane[lid] = lane_val
            if lane_xml_val is None:
                self.lanelet_to_lane_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_xml[lid] = lane_xml_val
            if link_xml_val is None:
                self.lanelet_to_lane_link_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_link_xml[lid] = link_xml_val

        self.road_groups[back_id] = back_group
        self.merge_tail_front_to_back[(road_id, outgoing_id)] = back_id

        self.manual_road_predecessors[road_id] = set(old_predecessors)
        self.manual_road_successors[road_id] = {back_id}
        self.manual_road_predecessors[back_id] = {road_id}
        self.manual_road_successors[back_id] = {outgoing_id}

        out_group = self.road_groups.get(outgoing_id)
        if out_group is not None:
            base_preds = set(self.manual_road_predecessors.get(outgoing_id, out_group.predecessors))
            base_preds.discard(road_id)
            base_preds.add(back_id)
            self.manual_road_predecessors[outgoing_id] = base_preds

        LOGGER.info(
            "Split merge predecessor road %s into front=%s and tail=%s for outgoing=%s",
            road_id,
            road_id,
            back_id,
            outgoing_id,
        )
        return back_id

    def split_polyline_at_s(self, pts: Sequence[Point2D], s_split: float) -> Tuple[List[Point2D], List[Point2D]]:
        """Split a polyline at longitudinal distance s_split."""
        pts = self.clean_polyline(pts)
        if len(pts) < 2:
            return list(pts), []

        total = self.polyline_length(pts)
        if total <= 1e-9:
            return list(pts), []
        s_split = max(1e-6, min(float(s_split), total - 1e-6))

        front: List[Point2D] = [pts[0]]
        acc = 0.0
        for idx, (p0, p1) in enumerate(zip(pts[:-1], pts[1:])):
            seg_len = self.distance(p0, p1)
            if seg_len <= 1e-12:
                continue
            if acc + seg_len < s_split - 1e-9:
                front.append(p1)
                acc += seg_len
                continue

            ratio = (s_split - acc) / seg_len
            split_pt = (
                p0[0] + (p1[0] - p0[0]) * ratio,
                p0[1] + (p1[1] - p0[1]) * ratio,
            )
            front.append(split_pt)
            back = [split_pt, p1]
            back.extend(pts[idx + 2:])
            return self.clean_polyline(front), self.clean_polyline(back)

        # Fallback: split near the last point if numerical issues occur.
        return self.clean_polyline(pts[:-1]), self.clean_polyline(pts[-2:])

    def collect_topological_connector_chains(self) -> List[Dict[str, object]]:
        """
        Collect initial connector chains from every multi-successor incoming road.

        Returns a list of dictionaries:
            {
                "incoming": int,
                "start": int,
                "members": List[int],
                "outgoing": Optional[int],
            }
        """
        chains: List[Dict[str, object]] = []
        seen_pairs: Set[Tuple[int, int]] = set()

        for incoming_id, incoming_group in sorted(self.road_groups.items()):
            if incoming_group.is_removed:
                continue
            if len(incoming_group.successors) <= 1:
                continue

            for start_id in sorted(incoming_group.successors):
                if start_id not in self.road_groups:
                    continue
                if self.road_groups[start_id].is_removed:
                    continue
                key = (incoming_id, start_id)
                if key in seen_pairs:
                    continue
                seen_pairs.add(key)

                members = self.trace_topological_connector_chain(
                    incoming_id=incoming_id,
                    start_id=start_id,
                )
                if not members:
                    continue

                outgoing = self.get_chain_outgoing_road_id(members)
                chains.append(
                    {
                        "incoming": incoming_id,
                        "start": start_id,
                        "members": members,
                        "outgoing": outgoing,
                    }
                )

        return chains

    def trace_topological_connector_chain(self, incoming_id: int, start_id: int) -> List[int]:
        """
        Trace a topology-based connector chain.

        Boundary types are deliberately ignored. The chain starts at a direct
        successor of a multi-successor incoming road and continues only through
        simple one-to-one, length-limited roads.
        """
        if start_id not in self.road_groups:
            return []

        chain: List[int] = [start_id]
        seen: Set[int] = {incoming_id, start_id}
        total_length = self.road_groups[start_id].length
        cur_id = start_id

        while True:
            cur_group = self.road_groups.get(cur_id)
            if cur_group is None or cur_group.is_removed:
                break

            # If current does not have exactly one successor, the connector branch ends here.
            if len(cur_group.successors) != 1:
                break

            next_id = next(iter(cur_group.successors))
            if next_id in seen:
                break

            next_group = self.road_groups.get(next_id)
            if next_group is None or next_group.is_removed:
                break

            # next with multiple predecessors is the merged outgoing road, not part of connector.
            if len(next_group.predecessors) > 1:
                break

            # next with multiple successors is another branch/incoming road, not part of this connector.
            if len(next_group.successors) > 1:
                break

            # Length guards prevent a connector chain from swallowing a normal road.
            if next_group.length > self.max_single_connector_length:
                break

            if total_length + next_group.length > self.max_connector_chain_length:
                break

            # The next road must really continue from current.
            if cur_id not in next_group.predecessors:
                break

            chain.append(next_id)
            seen.add(next_id)
            total_length += next_group.length
            cur_id = next_id

        return chain

    def get_chain_outgoing_road_id(self, members: Sequence[int]) -> Optional[int]:
        """Return the first external successor of a connector chain, if unique."""
        if not members:
            return None
        member_set = set(members)
        last_group = self.road_groups.get(members[-1])
        if last_group is None:
            return None
        external_successors = [sid for sid in sorted(last_group.successors) if sid not in member_set]
        if len(external_successors) == 1:
            return external_successors[0]
        return None

    def cluster_connector_chain_indices(self, chains: List[Dict[str, object]]) -> List[List[int]]:
        """
        Cluster connector chains into junction components.

        Conservative rule:
        - same incoming road, or
        - same outgoing road.
        Cross incoming/outgoing matching is not used because it can merge neighboring
        junctions through a short road between them.
        """
        if not chains:
            return []

        parent = {idx: idx for idx in range(len(chains))}

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        incoming_to_indices: Dict[int, List[int]] = defaultdict(list)
        outgoing_to_indices: Dict[int, List[int]] = defaultdict(list)

        for idx, chain in enumerate(chains):
            incoming = chain.get("incoming")
            outgoing = chain.get("outgoing")
            if isinstance(incoming, int):
                incoming_to_indices[incoming].append(idx)
            if isinstance(outgoing, int):
                outgoing_to_indices[outgoing].append(idx)

        for index_list in list(incoming_to_indices.values()) + list(outgoing_to_indices.values()):
            if len(index_list) < 2:
                continue
            first = index_list[0]
            for other in index_list[1:]:
                union(first, other)

        comps: Dict[int, List[int]] = defaultdict(list)
        for idx in range(len(chains)):
            comps[find(idx)].append(idx)

        return [sorted(v) for v in comps.values()]

    def merge_virtual_connecting_road_chains(self) -> None:
        """
        Merge consecutive virtual+virtual road groups that form one connectingRoad.

        This implements the requested lanelet-chain idea at road-group level:
        - First all lanelets are converted into small road groups as before.
        - A virtual+virtual road whose predecessor has multiple successors becomes
          a connector-chain start candidate.
        - Starting from this first virtual+virtual road, follow the successor road
          while the next road is still virtual+virtual and the chain is one-to-one.
        - The chain is rebuilt into one OpenDRIVE road using the first road id as
          representative; old member road XML elements are removed.
        """
        # Refresh virtual-boundary flags.
        for group in self.road_groups.values():
            group.is_virtual_boundary_road = self.is_virtual_boundary_road(group)

        starts: List[int] = []
        for road_id, group in sorted(self.road_groups.items()):
            if not group.is_virtual_boundary_road or group.is_removed:
                continue
            for pred_id in group.predecessors:
                pred_group = self.road_groups.get(pred_id)
                if pred_group is not None and len(pred_group.successors) > 1:
                    starts.append(road_id)
                    break

        used: Set[int] = set()
        for start_id in starts:
            if start_id in used:
                continue
            if start_id not in self.road_groups:
                continue

            chain_ids = self.trace_virtual_connecting_road_chain(start_id)
            if len(chain_ids) <= 1:
                used.update(chain_ids)
                continue

            # Only merge chains with stable lateral lane count. This keeps the
            # first version conservative and avoids corrupting lane ids.
            lane_counts = {len(self.road_groups[rid].road_lanes) for rid in chain_ids}
            if len(lane_counts) != 1:
                used.update(chain_ids)
                continue

            self.merge_road_group_chain(chain_ids)
            used.update(chain_ids)

    def trace_virtual_connecting_road_chain(self, start_id: int) -> List[int]:
        """Trace a one-to-one virtual+virtual successor chain from start_id."""
        chain = [start_id]
        seen = {start_id}
        cur_id = start_id

        while True:
            cur_group = self.road_groups.get(cur_id)
            if cur_group is None:
                break

            # Continue only when there is exactly one successor.
            if len(cur_group.successors) != 1:
                break
            next_id = next(iter(cur_group.successors))
            if next_id in seen:
                break

            next_group = self.road_groups.get(next_id)
            if next_group is None or next_group.is_removed:
                break
            if not next_group.is_virtual_boundary_road:
                break

            # Preserve only simple longitudinal chains. If the next virtual road
            # has several predecessors, it is a merge point and should be a new
            # connector candidate or left unmerged.
            if len(next_group.predecessors) != 1:
                break
            if cur_id not in next_group.predecessors:
                break

            # Direction continuity guard.
            if not self.are_road_groups_direction_continuous(cur_group, next_group):
                break

            chain.append(next_id)
            seen.add(next_id)
            cur_id = next_id

        return chain

    def are_road_groups_direction_continuous(self, a: RoadGroup, b: RoadGroup) -> bool:
        """Return True when two road groups have roughly continuous reference-line direction."""
        if len(a.ref_points) < 2 or len(b.ref_points) < 2:
            return True
        va = (
            a.ref_points[-1][0] - a.ref_points[-2][0],
            a.ref_points[-1][1] - a.ref_points[-2][1],
        )
        vb = (
            b.ref_points[1][0] - b.ref_points[0][0],
            b.ref_points[1][1] - b.ref_points[0][1],
        )
        na = math.hypot(va[0], va[1])
        nb = math.hypot(vb[0], vb[1])
        if na <= 1e-9 or nb <= 1e-9:
            return True
        cos_val = (va[0] * vb[0] + va[1] * vb[1]) / (na * nb)
        return cos_val > -0.2

    def merge_road_group_chain(self, chain_ids: List[int]) -> None:
        """
        Collapse several already-created road groups into the first road id.

        The representative road is rebuilt with:
        - stitched reference line
        - one laneSection
        - lane width records fitted over the whole chain
        - lanelet_to_road / lanelet_to_lane mappings for all member lanelets
        """
        if not chain_ids:
            return
        rep_id = chain_ids[0]
        rep_group = self.road_groups[rep_id]

        section_lanes = [list(self.road_groups[rid].road_lanes) for rid in chain_ids]
        section_refs = [list(self.road_groups[rid].ref_points) for rid in chain_ids]

        merged_ref: List[Point2D] = []
        for pts in section_refs:
            merged_ref = self.append_polyline_aligned(merged_ref, pts)

        merged_ref = self.prepare_refline_points(merged_ref)
        if len(merged_ref) < 2:
            return

        # Remove old XML elements of all chain members.
        for rid in chain_ids:
            old_group = self.road_groups[rid]
            if old_group.road_xml is not None and old_group.road_xml.getparent() is not None:
                old_group.road_xml.getparent().remove(old_group.road_xml)

        # Mark non-representative road groups as removed and redirect lanelet mappings.
        for rid in chain_ids[1:]:
            self.road_groups[rid].is_removed = True

        rep_group.chain_member_road_ids = list(chain_ids)
        rep_group.longitudinal_road_lanes = section_lanes
        rep_group.longitudinal_ref_points = section_refs
        rep_group.road_lanes = section_lanes[0]
        rep_group.ref_points = merged_ref
        rep_group.length = self.polyline_length(merged_ref)
        rep_group.successors = set(self.road_groups[chain_ids[-1]].successors) - set(chain_ids)
        rep_group.predecessors = set(self.road_groups[chain_ids[0]].predecessors) - set(chain_ids)

        # Rebuild road XML.
        road_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{rep_id}",
            length=f"{rep_group.length:.6f}",
            id=str(rep_id),
            junction="-1",
        )
        rep_group.road_xml = road_el
        self._write_road_type(road_el)
        self._write_plan_view(road_el, rep_group.ref_points)
        self._write_lanes(road_el, rep_group)

        # Re-map every lanelet in every section to the representative road and
        # corresponding lane id. road_lanes are stored right->left, and all
        # OpenDRIVE lanes are written on the right side.
        for section in section_lanes:
            left_to_right = list(reversed(section))
            for index, lanelet in enumerate(left_to_right, start=1):
                lanelet_id = str(lanelet.id_)
                self.lanelet_to_road[lanelet_id] = rep_id
                self.lanelet_to_lane[lanelet_id] = -index

    def append_polyline_aligned(self, base: List[Point2D], pts: List[Point2D]) -> List[Point2D]:
        """Append pts to base, reversing pts if needed for endpoint continuity."""
        pts = self.clean_polyline(pts)
        if not pts:
            return base
        if not base:
            return list(pts)

        if self.distance(base[-1], pts[-1]) < self.distance(base[-1], pts[0]):
            pts = list(reversed(pts))

        result = list(base)
        if self.distance(result[-1], pts[0]) > 1e-6:
            # Insert a simple straight bridge for tiny gaps instead of dropping topology.
            result.append(pts[0])
            result.extend(pts[1:])
        else:
            result.extend(pts[1:])
        return self.clean_polyline(result)

    def identify_connecting_roads(self) -> None:
        """
        Identify OpenDRIVE connectingRoads and cluster them into physical junctions.

        Rules implemented:
        1. All Lanelet2 way_relations are still converted into OpenDRIVE roads.
        2. A road is a connector candidate when:
             - it is made entirely from virtual+virtual lanelets, and
             - at least one predecessor road has multiple successors.
           Consecutive virtual+virtual candidates should already have been merged by
           merge_virtual_connecting_road_chains().
        3. Multiple connector candidates are grouped into the same physical junction
           only through same-role sharing:
               - shared predecessor road, or
               - shared successor road.
           Cross predecessor/successor matching, lanelet adjacency, and endpoint-distance
           matching are intentionally not used here to avoid merging neighboring
           physical junctions.
        """
        for group in self.road_groups.values():
            group.is_virtual_boundary_road = self.is_virtual_boundary_road(group)
            group.is_connecting_road = False
            group.junction_id = None

        candidate_ids: List[int] = []
        for road_id, group in sorted(self.road_groups.items()):
            if group.is_removed:
                continue
            if not group.is_virtual_boundary_road:
                continue

            is_candidate = False
            for pred_road_id in group.predecessors:
                pred_group = self.road_groups.get(pred_road_id)
                if pred_group is not None and len(pred_group.successors) > 1:
                    is_candidate = True
                    break

            if is_candidate:
                candidate_ids.append(road_id)
                group.is_connecting_road = True

        if not candidate_ids:
            return

        components = self.cluster_connecting_roads(candidate_ids)

        for component in components:
            junction_id = self.next_junction_id
            self.next_junction_id += 1
            for road_id in component:
                group = self.road_groups.get(road_id)
                if group is None:
                    continue
                group.is_connecting_road = True
                group.junction_id = junction_id

    def cluster_connecting_roads(self, candidate_ids: List[int]) -> List[List[int]]:
        """
        Cluster connectingRoad candidates into physical junction components.

        Conservative rule for this version:
        - Two connectingRoads are placed into the same junction component only if
          they share at least one predecessor road, or share at least one successor road.
        - predecessor-successor cross sharing is intentionally NOT used. For example,
          C1.successor == C2.predecessor often means two neighboring physical junctions
          are connected by a short ordinary road, and merging on that condition would
          incorrectly fuse adjacent junctions.
        - Endpoint distance, lanelet adjacency, and broad road-neighborhood tests are
          also intentionally not used here, because they were too permissive and could
          merge several nearby intersections into one OpenDRIVE <junction>.

        With the same-type sharing rule, all connector roads from the same incoming
        road are grouped; then connectors that merge into the same outgoing road group
        additional approaches, and the connected components normally cover one physical
        intersection without spilling into the next one.
        """
        parent = {rid: rid for rid in candidate_ids}

        def find(x: int) -> int:
            while parent[x] != x:
                parent[x] = parent[parent[x]]
                x = parent[x]
            return x

        def union(a: int, b: int) -> None:
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        # Index connector roads by same-role ordinary roads.
        pred_to_connectors: Dict[int, List[int]] = defaultdict(list)
        succ_to_connectors: Dict[int, List[int]] = defaultdict(list)

        for conn_id in candidate_ids:
            group = self.road_groups.get(conn_id)
            if group is None:
                continue

            for pred_id in group.predecessors:
                pred_group = self.road_groups.get(pred_id)
                if pred_group is not None and not pred_group.is_connecting_road:
                    pred_to_connectors[pred_id].append(conn_id)

            for succ_id in group.successors:
                succ_group = self.road_groups.get(succ_id)
                if succ_group is not None and not succ_group.is_connecting_road:
                    succ_to_connectors[succ_id].append(conn_id)

        for connector_ids in list(pred_to_connectors.values()) + list(succ_to_connectors.values()):
            if len(connector_ids) < 2:
                continue
            first = connector_ids[0]
            for other in connector_ids[1:]:
                union(first, other)

        comps: Dict[int, List[int]] = defaultdict(list)
        for rid in candidate_ids:
            comps[find(rid)].append(rid)

        return [sorted(v) for v in comps.values()]

    def are_connecting_roads_same_junction(self, a_id: int, b_id: int) -> bool:
        """
        Conservative relation test retained for debugging/experiments.

        The active clustering implementation above does not call this pairwise helper;
        it uses predecessor-predecessor and successor-successor indexing instead. This
        helper intentionally mirrors that same conservative semantics.
        """
        a = self.road_groups.get(a_id)
        b = self.road_groups.get(b_id)
        if a is None or b is None:
            return False
        return bool(a.predecessors & b.predecessors) or bool(a.successors & b.successors)

    def get_road_group_endpoints(self, group: RoadGroup) -> List[Point2D]:
        if len(group.ref_points) >= 2:
            return [group.ref_points[0], group.ref_points[-1]]
        return list(group.ref_points)

    def are_road_groups_junction_neighbors(self, road_a_id: int, road_b_id: int) -> bool:
        """
        Decide whether two ordinary roads are likely part of the same physical
        junction neighborhood. This is used only for junction clustering, not for
        merging roads.
        """
        if road_a_id not in self.road_groups or road_b_id not in self.road_groups:
            return False
        ga = self.road_groups[road_a_id]
        gb = self.road_groups[road_b_id]

        for la in self.iter_group_lanelets(ga):
            for lb in self.iter_group_lanelets(gb):
                if self.are_lanelets_junction_neighbors(str(la.id_), str(lb.id_)):
                    return True

        # Geometry fallback.
        pts_a = self.get_road_group_endpoints(ga)
        pts_b = self.get_road_group_endpoints(gb)
        for pa in pts_a:
            for pb in pts_b:
                if self.distance(pa, pb) <= 25.0:
                    return True
        return False

    def are_lanelets_junction_neighbors(self, a_id: str, b_id: str) -> bool:
        """
        Lanelet-level neighborhood test for junction clustering.

        It intentionally accepts both same-direction adjacency and opposite-direction
        proximity because both can belong to the same physical intersection. This
        function is NOT used for lateral road merging.
        """
        a = self.lanelets.get(str(a_id))
        b = self.lanelets.get(str(b_id))
        if a is None or b is None:
            return False

        b_id = str(b_id)
        a_id = str(a_id)

        if b_id in {str(x) for x in a.adjacent_left} or b_id in {str(x) for x in a.adjacent_right}:
            return True
        if a_id in {str(x) for x in b.adjacent_left} or a_id in {str(x) for x in b.adjacent_right}:
            return True

        a_ways = {str(a.left_way), str(a.right_way)}
        b_ways = {str(b.left_way), str(b.right_way)}
        if a_ways & b_ways:
            return True

        ca = self.get_center_points(a, n_samples=5)
        cb = self.get_center_points(b, n_samples=5)
        if ca and cb:
            min_dist = min(self.distance(pa, pb) for pa in ca for pb in cb)
            if min_dist <= 8.0:
                return True

        return False

    def construct_junctions_and_links(self) -> None:
        """
        Write OpenDRIVE road links and junctions.

        Current staged logic:
        1. Prefer true connectingRoad structure when a virtual+virtual road has been
           identified as a junction-internal connector.
        2. For such roads:
              incoming road  -> successor junction
              connectingRoad -> junction="J", predecessor/successor ordinary roads
              outgoing road  -> predecessor junction
              junction       -> connection incomingRoad=... connectingRoad=...
        3. If no connectingRoad is involved, fall back to the older direct road link /
           coarse junction logic.
        """
        self._write_connecting_road_junctions()
        self._write_road_links_with_connecting_roads()
        # Fallback simple junction writing is intentionally disabled. All
        # multi-successor and multi-predecessor topology should be handled by
        # the main split/connectingRoad logic above; otherwise we prefer a
        # visible warning over writing non-standard coarse junctions.
        self.warn_unhandled_multi_links()

    def _write_connecting_road_junctions(self) -> None:
        """Write junction elements based on roads marked as connectingRoad."""
        if self.root is None:
            return

        junction_to_connections: Dict[int, List[Tuple[int, int, str]]] = defaultdict(list)

        for conn_road_id, conn_group in self.road_groups.items():
            if not conn_group.is_connecting_road or conn_group.junction_id is None:
                continue

            junction_id = conn_group.junction_id

            # OpenDRIVE junction.connection represents incomingRoad -> connectingRoad.
            # Use the explicit incoming map recorded during chain construction.
            # Do not infer incoming roads from conn_group.predecessors, because after
            # chain shrinking/restoration, ordinary neighboring roads can remain
            # adjacent to a connector without being the junction's incoming road.
            pred_id = self.connecting_road_to_incoming.get(conn_road_id)
            if pred_id is None:
                pred_id = self._pick_best_direct_predecessor(conn_group)

            if pred_id is not None:
                pred_group = self.road_groups.get(pred_id)
                if pred_group is not None and not pred_group.is_connecting_road:
                    junction_to_connections[junction_id].append((pred_id, conn_road_id, "start"))

        for junction_id, connections in sorted(junction_to_connections.items()):
            junction_el = etree.SubElement(
                self.root,
                "junction",
                name=f"lanelet2_junction_{junction_id}",
                id=str(junction_id),
            )

            seen = set()
            connection_count = 0
            for incoming_id, conn_road_id, contact_point in connections:
                key = (incoming_id, conn_road_id, contact_point)
                if key in seen:
                    continue
                seen.add(key)

                connection_el = etree.SubElement(
                    junction_el,
                    "connection",
                    id=str(connection_count),
                    incomingRoad=str(incoming_id),
                    connectingRoad=str(conn_road_id),
                    contactPoint=contact_point,
                )

                lane_links = self._compute_lane_links_between_roads_any_direction(incoming_id, conn_road_id)
                for idx, (from_lane, to_lane) in enumerate(lane_links):
                    etree.SubElement(
                        connection_el,
                        "laneLink",
                        id=str(idx),
                        **{"from": str(from_lane), "to": str(to_lane)},
                    )

                connection_count += 1

    def _write_road_links_with_connecting_roads(self) -> None:
        """Write road-level <link> while respecting connectingRoad/junction roles."""
        for road_id, group in self.road_groups.items():
            if group.road_xml is None:
                continue

            # Remove any existing road-level link before rewriting.
            old_link = group.road_xml.find("link")
            if old_link is not None:
                group.road_xml.remove(old_link)

            if group.is_connecting_road and group.junction_id is not None:
                group.road_xml.set("junction", str(group.junction_id))
            else:
                group.road_xml.set("junction", "-1")

            link_el = etree.Element("link")
            has_link = False

            # Ordinary roads may reference a junction only when they were explicitly
            # recorded as an incoming/outgoing road of that junction. This avoids the
            # previous broad rule: "if predecessor/successor is a connectingRoad then
            # this road belongs to the junction", which created false
            # predecessor=junction links for roads restored by chain shrinking.
            pred_junction = self.outgoing_road_to_junction.get(road_id)
            succ_junction = self.incoming_road_to_junction.get(road_id)

            if group.is_connecting_road:
                pred_id = self.connecting_road_to_incoming.get(road_id)
                if pred_id is None:
                    pred_id = self._pick_best_direct_predecessor(group)

                succ_id = self.connecting_road_to_outgoing.get(road_id)
                if succ_id is None:
                    succ_id = self._pick_best_direct_successor(group)

                if pred_id is not None:
                    etree.SubElement(
                        link_el,
                        "predecessor",
                        elementType="road",
                        elementId=str(pred_id),
                        contactPoint="end",
                    )
                    has_link = True

                if succ_id is not None:
                    etree.SubElement(
                        link_el,
                        "successor",
                        elementType="road",
                        elementId=str(succ_id),
                        contactPoint="start",
                    )
                    has_link = True

            else:
                if pred_junction is not None:
                    etree.SubElement(
                        link_el,
                        "predecessor",
                        elementType="junction",
                        elementId=str(pred_junction),
                    )
                    has_link = True
                elif len(group.predecessors) == 1:
                    pred_id = next(iter(group.predecessors))
                    etree.SubElement(
                        link_el,
                        "predecessor",
                        elementType="road",
                        elementId=str(pred_id),
                        contactPoint="end",
                    )
                    has_link = True

                if succ_junction is not None:
                    etree.SubElement(
                        link_el,
                        "successor",
                        elementType="junction",
                        elementId=str(succ_junction),
                    )
                    has_link = True
                elif len(group.successors) == 1:
                    succ_id = next(iter(group.successors))
                    etree.SubElement(
                        link_el,
                        "successor",
                        elementType="road",
                        elementId=str(succ_id),
                        contactPoint="start",
                    )
                    has_link = True

            if has_link:
                group.road_xml.insert(0, link_el)

    def split_terminal_connecting_roads_without_successor(self, max_passes: int = 4) -> None:
        """
        Split formal connectingRoads that have no successor.

        Target OpenDRIVE pattern:
            incomingRoad -> connectingRoad -> outgoingRoad

        If a connectingRoad has no successor, it often means the converter made
        the whole terminal/map-boundary segment a junction-internal connector.
        For a cleaner OpenDRIVE structure, keep the old road id as a short
        connectingRoad and create a new ordinary tail road as its outgoing road:
            incoming -> C_front(connectingRoad, old id) -> C_back(ordinary terminal)

        The tail road may still have no successor; that is acceptable because it
        represents a map boundary / dead-end outside the junction. The important
        point is that the connectingRoad itself now has a road successor.
        """
        for _ in range(max_passes):
            self.process_road_linkages(clear_existing=True)
            changed = False

            candidates = []
            for road_id, group in sorted(self.road_groups.items()):
                if group.is_removed or not group.is_connecting_road:
                    continue
                if road_id in self.terminal_connector_front_to_back:
                    continue
                if len(group.successors) == 0:
                    candidates.append(road_id)

            if not candidates:
                break

            for road_id in candidates:
                if self.split_terminal_connecting_road_tail(road_id):
                    changed = True

            if not changed:
                break

        self.process_road_linkages(clear_existing=True)

    def split_terminal_connecting_road_tail(self, road_id: int) -> bool:
        """Split a terminal connectingRoad into front connector + ordinary tail."""
        if self.root is None:
            return False
        group = self.road_groups.get(road_id)
        if group is None or group.is_removed or not group.is_connecting_road:
            return False
        if road_id in self.terminal_connector_front_to_back:
            return False
        if len(group.successors) != 0:
            return False
        if group.length <= 1e-3 or len(group.ref_points) < 2:
            return False

        old_predecessors = set(group.predecessors)
        junction_id = group.junction_id
        incoming_id = self.connecting_road_to_incoming.get(road_id)

        min_seg = max(0.5, self.min_split_segment_length)
        if group.length <= 2.0 * min_seg:
            # Too short to split safely. Leaving a terminal connector is better
            # than creating degenerate OpenDRIVE geometry.
            LOGGER.warning(
                "Skip terminal connectingRoad split for road %s because length %.3f is too short.",
                road_id,
                group.length,
            )
            return False

        # Keep the connectingRoad front part short; the back part is the ordinary
        # terminal/outgoing road outside the junction.
        front_len = min(self.nested_connector_front_length, group.length * 0.35)
        front_len = max(min_seg, front_len)
        split_s = min(front_len, group.length - min_seg)

        front_pts, back_pts = self.split_polyline_at_s(group.ref_points, split_s)
        front_pts = self.prepare_refline_points(front_pts)
        back_pts = self.prepare_refline_points(back_pts)
        if len(front_pts) < 2 or len(back_pts) < 2:
            return False

        back_id = self.next_road_id
        self.next_road_id += 1

        # Remove old XML; rebuild the old id as the front connector.
        if group.road_xml is not None and group.road_xml.getparent() is not None:
            group.road_xml.getparent().remove(group.road_xml)

        group.ref_points = front_pts
        group.length = self.polyline_length(front_pts)
        group.successors = {back_id}
        group.predecessors = set(old_predecessors)
        group.lane_width_samples = {}
        front_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{road_id}",
            length=f"{group.length:.6f}",
            id=str(road_id),
            junction=str(junction_id) if junction_id is not None else "-1",
        )
        group.road_xml = front_el
        self._write_road_type(front_el)
        self._write_plan_view(front_el, group.ref_points)
        self._write_lanes(front_el, group)

        # Create ordinary terminal/outgoing back road.
        back_group = RoadGroup(
            od_road_id=back_id,
            road_lanes=list(group.road_lanes),
            ref_points=back_pts,
            length=self.polyline_length(back_pts),
        )
        back_group.predecessors = {road_id}
        back_group.successors = set()
        back_el = etree.SubElement(
            self.root,
            "road",
            name=f"lanelet2_road_{back_id}_terminal_tail_from_{road_id}",
            length=f"{back_group.length:.6f}",
            id=str(back_id),
            junction="-1",
        )
        back_group.road_xml = back_el
        self._write_road_type(back_el)
        self._write_plan_view(back_el, back_group.ref_points)

        # The synthetic back road reuses lanelet objects. Preserve global mappings
        # for regulatory-element back-references; road/lane topology is supplied
        # by manual_road_* and manual_lane_links.
        affected_lanelets = [str(lanelet.id_) for lanelet in back_group.road_lanes]
        saved_maps = {
            lid: (
                self.lanelet_to_road.get(lid),
                self.lanelet_to_lane.get(lid),
                self.lanelet_to_lane_xml.get(lid),
                self.lanelet_to_lane_link_xml.get(lid),
            )
            for lid in affected_lanelets
        }
        self._write_lanes(back_el, back_group)
        for lid, (road_val, lane_val, lane_xml_val, link_xml_val) in saved_maps.items():
            if road_val is None:
                self.lanelet_to_road.pop(lid, None)
            else:
                self.lanelet_to_road[lid] = road_val
            if lane_val is None:
                self.lanelet_to_lane.pop(lid, None)
            else:
                self.lanelet_to_lane[lid] = lane_val
            if lane_xml_val is None:
                self.lanelet_to_lane_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_xml[lid] = lane_xml_val
            if link_xml_val is None:
                self.lanelet_to_lane_link_xml.pop(lid, None)
            else:
                self.lanelet_to_lane_link_xml[lid] = link_xml_val

        self.road_groups[back_id] = back_group
        self.terminal_connector_front_to_back[road_id] = back_id
        self.terminal_connector_back_to_front[back_id] = road_id

        self.manual_road_predecessors[road_id] = set(old_predecessors)
        self.manual_road_successors[road_id] = {back_id}
        self.manual_road_predecessors[back_id] = {road_id}
        self.manual_road_successors[back_id] = set()

        # Update explicit role maps: old road remains connectingRoad; new back road
        # becomes the outgoing road of the same junction.
        self.connecting_road_to_outgoing[road_id] = back_id
        if junction_id is not None:
            self.junction_to_outgoing_roads[junction_id].add(back_id)
            self.outgoing_road_to_junction[back_id] = junction_id
        if incoming_id is not None and junction_id is not None:
            self.connecting_road_to_incoming[road_id] = incoming_id
            self.junction_to_incoming_roads[junction_id].add(incoming_id)
            self.incoming_road_to_junction[incoming_id] = junction_id

        LOGGER.info(
            "Split terminal connectingRoad %s into connector front=%s and ordinary tail=%s.",
            road_id,
            road_id,
            back_id,
        )
        return True

    def warn_connecting_roads_with_complex_links(self) -> None:
        """Warn if a connectingRoad still has a complex road-level link.

        After the real split logic, a formal connectingRoad should normally have
        one predecessor and one successor road. If a connectingRoad still has
        multiple successors, it means the road was not separated cleanly from a
        later split topology. If it has multiple predecessors, it should generally
        be handled by merge-junction tail split logic.
        """
        bad_successors = []
        bad_predecessors = []
        for road_id, group in sorted(self.road_groups.items()):
            if group.is_removed or not group.is_connecting_road:
                continue
            if len(group.successors) > 1:
                bad_successors.append(road_id)
            if len(group.predecessors) > 1:
                bad_predecessors.append(road_id)
        if bad_successors:
            LOGGER.warning(
                "ConnectingRoads still having multiple successors after real split: %s",
                bad_successors[:50],
            )
        if bad_predecessors:
            LOGGER.warning(
                "ConnectingRoads still having multiple predecessors after real split: %s",
                bad_predecessors[:50],
            )

    def warn_unhandled_multi_links(self) -> None:
        """Report remaining multi-link roads that were not handled by the main logic."""
        unhandled_successors = []
        unhandled_predecessors = []
        for road_id, group in sorted(self.road_groups.items()):
            if group.is_removed:
                continue
            if len(group.successors) > 1 and road_id not in self.incoming_road_to_junction:
                unhandled_successors.append(road_id)
            if len(group.predecessors) > 1 and road_id not in self.outgoing_road_to_junction:
                unhandled_predecessors.append(road_id)
        if unhandled_successors:
            LOGGER.warning("Unhandled multi-successor roads after main junction logic: %s", unhandled_successors[:50])
        if unhandled_predecessors:
            LOGGER.warning("Unhandled multi-predecessor roads after main junction logic: %s", unhandled_predecessors[:50])

    def _write_fallback_junctions_for_unhandled_multi_links(self) -> None:
        """
        Fallback for multi-successor / multi-predecessor cases that are not covered
        by connectingRoad recognition. This preserves the older coarse behavior.
        """
        if self.root is None:
            return

        handled_successor_roads = set(self.incoming_road_to_junction.keys())
        handled_predecessor_roads = set(self.outgoing_road_to_junction.keys())

        road_to_successor_junction: Dict[int, int] = {}
        road_to_predecessor_junction: Dict[int, int] = {}

        for road_id, group in self.road_groups.items():
            if road_id in handled_successor_roads:
                continue
            if len(group.successors) > 1:
                junction_id = self.next_junction_id
                self.next_junction_id += 1
                road_to_successor_junction[road_id] = junction_id
                self._write_simple_junction(
                    junction_id=junction_id,
                    incoming_road_id=road_id,
                    outgoing_road_ids=sorted(group.successors),
                    direction="successor",
                )

        for road_id, group in self.road_groups.items():
            if road_id in handled_predecessor_roads:
                continue
            if len(group.predecessors) > 1:
                junction_id = self.next_junction_id
                self.next_junction_id += 1
                road_to_predecessor_junction[road_id] = junction_id
                self._write_simple_junction(
                    junction_id=junction_id,
                    incoming_road_id=road_id,
                    outgoing_road_ids=sorted(group.predecessors),
                    direction="predecessor",
                )

        # Patch links for fallback junctions where no link was written.
        for road_id, group in self.road_groups.items():
            if group.road_xml is None:
                continue
            link_el = group.road_xml.find("link")
            if link_el is None:
                link_el = etree.Element("link")
                group.road_xml.insert(0, link_el)

            if road_id in road_to_predecessor_junction and link_el.find("predecessor") is None:
                etree.SubElement(
                    link_el,
                    "predecessor",
                    elementType="junction",
                    elementId=str(road_to_predecessor_junction[road_id]),
                )

            if road_id in road_to_successor_junction and link_el.find("successor") is None:
                etree.SubElement(
                    link_el,
                    "successor",
                    elementType="junction",
                    elementId=str(road_to_successor_junction[road_id]),
                )

    def _junction_from_connected_predecessors(self, group: RoadGroup) -> Optional[int]:
        junction_ids = {
            self.road_groups[pred_id].junction_id
            for pred_id in group.predecessors
            if pred_id in self.road_groups
            and self.road_groups[pred_id].is_connecting_road
            and self.road_groups[pred_id].junction_id is not None
        }
        if len(junction_ids) == 1:
            return next(iter(junction_ids))
        return None

    def _junction_from_connected_successors(self, group: RoadGroup) -> Optional[int]:
        junction_ids = {
            self.road_groups[succ_id].junction_id
            for succ_id in group.successors
            if succ_id in self.road_groups
            and self.road_groups[succ_id].is_connecting_road
            and self.road_groups[succ_id].junction_id is not None
        }
        if len(junction_ids) == 1:
            return next(iter(junction_ids))
        return None

    def _pick_best_direct_predecessor(self, group: RoadGroup) -> Optional[int]:
        if not group.predecessors:
            return None
        # Prefer incoming roads that created this junction condition.
        triggering = []
        for pred_id in group.predecessors:
            pred_group = self.road_groups.get(pred_id)
            if pred_group is not None and len(pred_group.successors) > 1:
                triggering.append(pred_id)
        return sorted(triggering or list(group.predecessors))[0]

    def _pick_best_direct_successor(self, group: RoadGroup) -> Optional[int]:
        if not group.successors:
            return None
        return sorted(group.successors)[0]

    def _write_simple_junction(
        self,
        junction_id: int,
        incoming_road_id: int,
        outgoing_road_ids: Sequence[int],
        direction: str,
    ) -> None:
        """
        Write a coarse OpenDRIVE junction.

        This is a first-version approximation.
        In a strict OpenDRIVE network, connectingRoad is usually an internal connector road.
        Here we temporarily use outgoing roads as connectingRoad to keep topology visible.
        """
        if self.root is None:
            return

        junction_el = etree.SubElement(
            self.root,
            "junction",
            name=f"lanelet2_junction_{junction_id}",
            id=str(junction_id),
        )

        connection_count = 0
        for out_road_id in outgoing_road_ids:
            connection_el = etree.SubElement(
                junction_el,
                "connection",
                id=str(connection_count),
                incomingRoad=str(incoming_road_id),
                connectingRoad=str(out_road_id),
                contactPoint="start" if direction == "successor" else "end",
            )

            lane_links = self._compute_lane_links_between_roads(incoming_road_id, out_road_id)

            for from_lane, to_lane in lane_links:
                etree.SubElement(
                    connection_el,
                    "laneLink",
                    **{"from": str(from_lane), "to": str(to_lane)},
                )

            connection_count += 1

    def _compute_lane_links_between_roads(self, from_road_id: int, to_road_id: int) -> List[Tuple[int, int]]:
        """Compute lane links in successor direction from one road to another."""
        result: List[Tuple[int, int]] = []

        from_group = self.road_groups.get(from_road_id)
        if from_group is None:
            return result

        for lanelet in from_group.road_lanes:
            lanelet_id = str(lanelet.id_)
            from_lane = self.lanelet_to_lane.get(lanelet_id)
            if from_lane is None:
                continue

            for succ in lanelet.successors:
                succ = str(succ)
                if self.lanelet_to_road.get(succ) == to_road_id:
                    to_lane = self.lanelet_to_lane.get(succ)
                    if to_lane is not None:
                        result.append((from_lane, to_lane))

        return self.unique_lane_links(result)

    def _compute_lane_links_between_roads_any_direction(
        self,
        incoming_road_id: int,
        connecting_road_id: int,
    ) -> List[Tuple[int, int]]:
        """
        Compute laneLink from incomingRoad lane ids to connectingRoad lane ids.

        For contactPoint=start, this is usually incoming.successor -> connecting.
        For contactPoint=end, this can be incoming.predecessor -> connecting.
        We therefore check both predecessor and successor relations.
        """
        manual = self.manual_lane_links.get((incoming_road_id, connecting_road_id))
        if manual:
            return list(manual)

        if self._should_use_same_layout_lane_link(incoming_road_id, connecting_road_id):
            return self._same_layout_lane_links(incoming_road_id, connecting_road_id)

        result: List[Tuple[int, int]] = []
        incoming_group = self.road_groups.get(incoming_road_id)
        if incoming_group is None:
            return result

        for lanelet in incoming_group.road_lanes:
            lanelet_id = str(lanelet.id_)
            from_lane = self.lanelet_to_lane.get(lanelet_id)
            if from_lane is None:
                continue

            neighbor_ids = [str(x) for x in lanelet.successors] + [str(x) for x in lanelet.predecessors]
            for neighbor_id in neighbor_ids:
                if self.lanelet_to_road.get(neighbor_id) == connecting_road_id:
                    to_lane = self.lanelet_to_lane.get(neighbor_id)
                    if to_lane is not None:
                        result.append((from_lane, to_lane))

        return self.unique_lane_links(result)

    def _should_use_same_layout_lane_link(self, from_road_id: int, to_road_id: int) -> bool:
        """Return True when two synthetic split roads share the same lane layout."""
        gf = self.road_groups.get(from_road_id)
        gt = self.road_groups.get(to_road_id)
        if gf is None or gt is None:
            return False
        if len(gf.road_lanes) != len(gt.road_lanes):
            return False
        # Direct synthetic split patterns.
        if self.manual_road_successors.get(from_road_id) == {to_road_id}:
            return True
        if self.split_front_to_back.get(from_road_id) == to_road_id:
            return True
        if self.merge_tail_front_to_back.get((from_road_id, next(iter(self.manual_road_successors.get(to_road_id, set())), -1))) == to_road_id:
            return True
        # Same lanelet objects after split.
        return [str(x.id_) for x in gf.road_lanes] == [str(x.id_) for x in gt.road_lanes]

    def _same_layout_lane_links(self, from_road_id: int, to_road_id: int) -> List[Tuple[int, int]]:
        """Build laneLink pairs by lane ids for front/back roads sharing layout."""
        gf = self.road_groups.get(from_road_id)
        gt = self.road_groups.get(to_road_id)
        if gf is None or gt is None:
            return []
        n = min(len(gf.road_lanes), len(gt.road_lanes))
        # all lanes are on the right side: -1, -2, ...
        return [(-idx, -idx) for idx in range(1, n + 1)]

    @staticmethod
    def unique_lane_links(links: Sequence[Tuple[int, int]]) -> List[Tuple[int, int]]:
        seen = set()
        result = []
        for link in links:
            if link in seen:
                continue
            seen.add(link)
            result.append(link)
        return result

    # ---------------------------------------------------------------------
    # Regulatory elements
    # ---------------------------------------------------------------------

    def populate_regulatory_elements(self) -> None:
        """
        Build regulatory_element -> roads mapping.

        It uses lanelet.regulatory_elements first.
        If not found, it falls back to way references in the regulatory element.
        """
        self.regulatory_to_roads.clear()

        for lanelet_id, lanelet in self.lanelets.items():
            road_id = self.lanelet_to_road.get(str(lanelet_id))
            if road_id is None:
                continue

            for reg_id in lanelet.regulatory_elements:
                self.regulatory_to_roads[str(reg_id)].add(road_id)

        for reg_id, reg in self.regulatory_elements.items():
            for way_id in list(reg.refers) + list(reg.ref_line):
                for lanelet_id in self.way_to_lanelets.get(str(way_id), set()):
                    road_id = self.lanelet_to_road.get(str(lanelet_id))
                    if road_id is not None:
                        self.regulatory_to_roads[str(reg_id)].add(road_id)

            for lanelet_id in list(getattr(reg, "yield_ways", []) or []) + list(getattr(reg, "right_of_ways", []) or []):
                road_id = self.lanelet_to_road.get(str(lanelet_id))
                if road_id is not None:
                    self.regulatory_to_roads[str(reg_id)].add(road_id)

            if not self.regulatory_to_roads[str(reg_id)]:
                point = self._get_regulatory_reference_point(reg)
                if point is not None:
                    road_id, _, _ = self.find_nearest_road_st(point)
                    if road_id is not None:
                        self.regulatory_to_roads[str(reg_id)].add(road_id)

    def construct_regulatory_elements(self) -> None:
        """
        Convert Lanelet2 regulatory elements and standalone traffic ways to OpenDRIVE signals.

        Lanelet2 stores traffic semantics either as regulatory_element relations
        or, in some exported maps, directly as traffic_sign / traffic_light ways.
        Both variants are preserved here as OpenDRIVE signals with Lanelet2 tags
        carried in userData for downstream tools.
        """
        written_signal_ids: Set[str] = set()
        written_controller_ids: Set[str] = set()
        traffic_way_signal_ids: Dict[str, str] = {}
        self._stop_line_cache.clear()

        for reg_id, reg in self.regulatory_elements.items():
            road_ids = self.regulatory_to_roads.get(str(reg_id), set())
            if not road_ids:
                continue

            subtype = self._get_regulatory_subtype(reg)
            relation_stop_lines: List[Tuple[str, etree._Element]] = []
            if subtype == "traffic_light":
                self._construct_traffic_light_relation(
                    str(reg_id),
                    reg,
                    road_ids,
                    written_signal_ids,
                    written_controller_ids,
                    traffic_way_signal_ids,
                )
                continue

            if subtype == "right_of_way":
                self._construct_right_of_way_relation(
                    str(reg_id), reg, road_ids, written_signal_ids
                )
                continue

            point = self._get_regulatory_reference_point(reg)

            for road_id in sorted(road_ids):
                group = self.road_groups.get(road_id)
                if group is None or group.road_xml is None:
                    continue

                s, t = self.project_point_to_polyline(point, group.ref_points) if point is not None else (0.0, 0.0)
                country, signal_type, signal_subtype = self._map_regulatory_to_signal_type(reg)
                signal_id = self._unique_xml_id(f"reg_{reg_id}_{road_id}", written_signal_ids)
                signal_el = self._add_signal_to_road(
                    group,
                    signal_id=signal_id,
                    name=f"lanelet2_{subtype}_{reg_id}",
                    s=s,
                    t=t,
                    country=country,
                    signal_type=signal_type,
                    signal_subtype=signal_subtype,
                    dynamic="no",
                    value=self._get_signal_value(reg),
                    unit=self._get_signal_unit(reg),
                    lane_ids=self._get_regulatory_lane_validity(str(reg_id), road_id),
                )
                self._append_lanelet2_metadata(
                    signal_el, "regulatory_element", str(reg_id), reg.tag_dict
                )
                self._apply_signal_geometry_pose(
                    signal_el,
                    group,
                    s,
                    reg_id=str(reg_id),
                    reg=reg,
                )
                self._append_signal_mapping_metadata(signal_el, subtype)
                if self._should_reference_right_of_way_line(
                    subtype, country, signal_type
                ):
                    for stop_signal_id, _ in relation_stop_lines:
                        etree.SubElement(
                            signal_el,
                            "reference",
                            elementId=stop_signal_id,
                            elementType="signal",
                            type="yield_line",
                        )
                if (
                    subtype == "speed_limit"
                    and signal_el.get("type") != "-1"
                    and signal_el.get("value") is None
                ):
                    self._append_user_data(
                        signal_el,
                        "lanelet2:value_mapping_status",
                        "unresolved",
                    )

        self._construct_standalone_traffic_way_signals(written_signal_ids)
        self._construct_standalone_stop_lines(written_signal_ids)

    def _construct_right_of_way_relation(
        self,
        reg_id: str,
        reg,
        road_ids: Set[int],
        written_signal_ids: Set[str],
    ) -> None:
        """Convert every referred sign and preserve yield/priority role membership."""
        stop_lines = self._construct_stop_lines_for_relation(
            reg_id, reg, road_ids, written_signal_ids
        )
        yield_lanelets = [str(x) for x in getattr(reg, "yield_ways", []) or []]
        priority_lanelets = [
            str(x) for x in getattr(reg, "right_of_ways", []) or []
        ]
        role_locations = {
            "yield": self._lanelet_role_locations(yield_lanelets),
            "right_of_way": self._lanelet_role_locations(priority_lanelets),
        }

        referred_ways = [
            self.ways.get(str(way_id))
            for way_id in getattr(reg, "refers", []) or []
        ]
        referred_ways = [way for way in referred_ways if way is not None]

        if not referred_ways:
            for road_id in sorted(
                {road for road, _ in role_locations["yield"]} or road_ids
            ):
                group = self.road_groups.get(road_id)
                if group is None or group.road_xml is None:
                    continue
                point = self._get_regulatory_reference_point(reg)
                s, t = (
                    self.project_point_to_polyline(point, group.ref_points)
                    if point is not None
                    else (0.0, 0.0)
                )
                signal_id = self._unique_xml_id(
                    f"reg_{reg_id}_{road_id}", written_signal_ids
                )
                mapping = self._configured_signal_mapping("right_of_way")
                signal_el = self._add_signal_to_road(
                    group,
                    signal_id,
                    f"lanelet2_right_of_way_{reg_id}",
                    s,
                    t,
                    mapping["country"],
                    mapping["type"],
                    mapping["subtype"],
                    mapping["dynamic"],
                    "",
                    "",
                    self._role_lane_ids_on_road(yield_lanelets, road_id),
                )
                self._append_lanelet2_metadata(
                    signal_el, "regulatory_element", reg_id, reg.tag_dict
                )
                self._append_signal_mapping_metadata(signal_el, "right_of_way")
                self._append_right_of_way_role_metadata(
                    signal_el, reg, role_locations, "yield"
                )
                for stop_signal_id, _ in stop_lines:
                    etree.SubElement(
                        signal_el,
                        "reference",
                        elementId=stop_signal_id,
                        elementType="signal",
                        type="yield_line",
                    )
            return

        for way in referred_ways:
            way_id = str(way.id_)
            points = self.get_way_points(way_id)
            if not points:
                continue
            country, signal_type, signal_subtype = self._map_way_to_signal_type(way)
            is_yield_sign = self._should_reference_right_of_way_line(
                "right_of_way", country, signal_type
            )
            role = "yield" if is_yield_sign else "right_of_way"
            role_lanelets = yield_lanelets if role == "yield" else priority_lanelets
            candidate_roads = {road for road, _ in role_locations[role]}
            if not candidate_roads:
                candidate_roads = set(road_ids)
            point = self.polyline_midpoint(points)
            road_id, s, t = self._find_nearest_candidate_road_st(
                point, candidate_roads
            )
            if road_id is None:
                continue
            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None:
                continue
            signal_id = self._unique_xml_id(
                f"way_{way_id}_{road_id}", written_signal_ids
            )
            signal_el = self._add_signal_to_road(
                group,
                signal_id,
                f"lanelet2_{role}_{way_id}",
                s,
                t,
                country,
                signal_type,
                signal_subtype,
                "no",
                self._get_way_signal_value(way),
                self._get_way_signal_unit(way),
                self._role_lane_ids_on_road(role_lanelets, road_id),
            )
            self._append_user_data(
                signal_el, "lanelet2:regulatory_element", reg_id
            )
            self._append_lanelet2_metadata(signal_el, "way", way_id, way.tag_dict)
            self._apply_signal_geometry_pose(
                signal_el,
                group,
                s,
                way_points=points,
                reg_id=reg_id,
                reg=reg,
            )
            self._append_right_of_way_role_metadata(
                signal_el, reg, role_locations, role
            )
            if role == "yield":
                for stop_signal_id, _ in stop_lines:
                    etree.SubElement(
                        signal_el,
                        "reference",
                        elementId=stop_signal_id,
                        elementType="signal",
                        type="yield_line",
                    )

    def _lanelet_role_locations(
        self, lanelet_ids: Sequence[str]
    ) -> List[Tuple[int, int]]:
        locations = []
        for lanelet_id in lanelet_ids:
            road_id = self.lanelet_to_road.get(str(lanelet_id))
            lane_id = self.lanelet_to_lane.get(str(lanelet_id))
            if road_id is not None and lane_id is not None:
                locations.append((int(road_id), int(lane_id)))
        return sorted(set(locations))

    def _role_lane_ids_on_road(
        self, lanelet_ids: Sequence[str], road_id: int
    ) -> List[int]:
        return sorted(
            {
                int(self.lanelet_to_lane[str(lanelet_id)])
                for lanelet_id in lanelet_ids
                if self.lanelet_to_road.get(str(lanelet_id)) == road_id
                and str(lanelet_id) in self.lanelet_to_lane
            }
        )

    def _append_right_of_way_role_metadata(
        self,
        signal_el: etree._Element,
        reg,
        role_locations: Dict[str, List[Tuple[int, int]]],
        signal_role: str,
    ) -> None:
        self._append_user_data(signal_el, "lanelet2:role", signal_role)
        self._append_user_data(
            signal_el,
            "lanelet2:fallback",
            str(reg.tag_dict.get("fallback", "no")),
        )
        for key in sorted(reg.tag_dict):
            self._append_user_data(
                signal_el,
                f"lanelet2:regulatory_tag:{key}",
                str(reg.tag_dict[key]),
            )
        for ref_line_id in getattr(reg, "ref_line", []) or []:
            self._append_user_data(
                signal_el,
                "lanelet2:ref_line_source_id",
                str(ref_line_id),
            )
        for role in ("yield", "right_of_way"):
            source_ids = (
                getattr(reg, "yield_ways", [])
                if role == "yield"
                else getattr(reg, "right_of_ways", [])
            )
            for source_id in [str(x) for x in source_ids]:
                road_id = self.lanelet_to_road.get(source_id)
                lane_id = self.lanelet_to_lane.get(source_id)
                if road_id is None or lane_id is None:
                    continue
                self._append_user_data(
                    signal_el,
                    f"lanelet2:role:{role}",
                    f"{source_id}|{road_id}|{lane_id}",
                )

    def _should_reference_right_of_way_line(
        self, subtype: str, country: str, signal_type: str
    ) -> bool:
        if subtype != "right_of_way":
            return False
        if signal_type == "-1":
            return True
        key = f"{country}:{signal_type}"
        return key in self.config.lanelet2_stop_line_reference_signal_types

    def _construct_traffic_light_relation(
        self,
        reg_id: str,
        reg,
        road_ids: Set[int],
        written_signal_ids: Set[str],
        written_controller_ids: Set[str],
        traffic_way_signal_ids: Dict[str, str],
    ) -> None:
        """Write each referred light head as a signal and group them by relation."""
        controlled_signal_ids: List[str] = []
        ref_line_ids = [str(x) for x in getattr(reg, "ref_line", []) or []]
        stop_line_signals = self._construct_stop_lines_for_relation(
            reg_id, reg, road_ids, written_signal_ids
        )

        for way_id_raw in getattr(reg, "refers", []) or []:
            way_id = str(way_id_raw)
            existing_signal_id = traffic_way_signal_ids.get(way_id)
            if existing_signal_id is not None:
                controlled_signal_ids.append(existing_signal_id)
                continue

            way = self.ways.get(way_id)
            pts = self.get_way_points(way_id)
            if way is None or not pts:
                continue

            point = self.polyline_midpoint(pts)
            road_id, s, t = self._find_nearest_candidate_road_st(point, road_ids)
            if road_id is None:
                continue
            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None:
                continue

            country, signal_type, signal_subtype = self._map_way_to_signal_type(
                way, force_traffic_light=True
            )
            signal_id = self._unique_xml_id(f"way_{way_id}_{road_id}", written_signal_ids)
            signal_el = self._add_signal_to_road(
                group,
                signal_id=signal_id,
                name=f"lanelet2_traffic_light_{way_id}",
                s=s,
                t=t,
                country=country,
                signal_type=signal_type,
                signal_subtype=signal_subtype,
                dynamic=self._configured_signal_mapping("traffic_light")["dynamic"],
                value=self._get_way_signal_value(way),
                unit=self._get_way_signal_unit(way),
                lane_ids=self._get_regulatory_lane_validity(reg_id, road_id),
            )
            self._append_user_data(signal_el, "lanelet2:regulatory_element", reg_id)
            self._append_lanelet2_metadata(signal_el, "way", way_id, way.tag_dict)
            self._apply_signal_geometry_pose(
                signal_el,
                group,
                s,
                way_points=pts,
                reg_id=reg_id,
                reg=reg,
            )
            self._append_traffic_light_pole(
                group, way_id, signal_el, s, t, lane_ids=self._get_regulatory_lane_validity(reg_id, road_id)
            )
            if ref_line_ids:
                self._append_user_data(signal_el, "lanelet2:ref_line", ",".join(ref_line_ids))
            for stop_signal_id, _ in stop_line_signals:
                etree.SubElement(
                    signal_el,
                    "reference",
                    elementId=stop_signal_id,
                    elementType="signal",
                    type="stopline",
                )
            traffic_way_signal_ids[way_id] = signal_id
            controlled_signal_ids.append(signal_id)

        # Preserve malformed/minimal relations that have no usable referred way.
        if not controlled_signal_ids:
            point = self._get_regulatory_reference_point(reg)
            if point is not None:
                road_id, s, t = self._find_nearest_candidate_road_st(point, road_ids)
                group = self.road_groups.get(road_id) if road_id is not None else None
                if group is not None and group.road_xml is not None:
                    signal_id = self._unique_xml_id(f"reg_{reg_id}_{road_id}", written_signal_ids)
                    signal_el = self._add_signal_to_road(
                        group,
                        signal_id=signal_id,
                        name=f"lanelet2_traffic_light_{reg_id}",
                        s=s,
                        t=t,
                        country=self._configured_signal_mapping("traffic_light")["country"],
                        signal_type=self._configured_signal_mapping("traffic_light")["type"],
                        signal_subtype=self._configured_signal_mapping("traffic_light")["subtype"],
                        dynamic=self._configured_signal_mapping("traffic_light")["dynamic"],
                        value="",
                        unit="",
                        lane_ids=self._get_regulatory_lane_validity(reg_id, int(road_id)),
                    )
                    self._append_lanelet2_metadata(
                        signal_el, "regulatory_element", reg_id, reg.tag_dict
                    )
                    self._apply_signal_geometry_pose(
                        signal_el,
                        group,
                        s,
                        reg_id=reg_id,
                        reg=reg,
                    )
                    if ref_line_ids:
                        self._append_user_data(signal_el, "lanelet2:ref_line", ",".join(ref_line_ids))
                    for stop_signal_id, _ in stop_line_signals:
                        etree.SubElement(
                            signal_el,
                            "reference",
                            elementId=stop_signal_id,
                            elementType="signal",
                            type="stopline",
                        )
                    controlled_signal_ids.append(signal_id)

        for stop_signal_id, stop_signal_el in stop_line_signals:
            for signal_id in dict.fromkeys(controlled_signal_ids):
                etree.SubElement(
                    stop_signal_el,
                    "dependency",
                    id=signal_id,
                    type="traffic_light",
                )

        if controlled_signal_ids:
            controller_id = self._unique_xml_id(
                f"controller_reg_{reg_id}", written_controller_ids
            )
            controller_el = etree.Element(
                "controller",
                id=controller_id,
                name=f"lanelet2_traffic_light_group_{reg_id}",
                sequence="0",
            )
            for signal_id in dict.fromkeys(controlled_signal_ids):
                etree.SubElement(controller_el, "control", signalId=signal_id)
            self._append_user_data(controller_el, "lanelet2:regulatory_element", reg_id)
            if ref_line_ids:
                self._append_user_data(controller_el, "lanelet2:ref_line", ",".join(ref_line_ids))
            self._insert_root_controller(controller_el)

    def _construct_stop_lines_for_relation(
        self,
        reg_id: str,
        reg,
        road_ids: Set[int],
        written_signal_ids: Set[str],
    ) -> List[Tuple[str, etree._Element]]:
        """Create semantic and visible OpenDRIVE stop lines from ref_line ways."""
        result = []
        for way_id_raw in getattr(reg, "ref_line", []) or []:
            way_id = str(way_id_raw)
            way = self.ways.get(way_id)
            points = self.get_way_points(way_id)
            if way is None or len(points) < 2:
                continue
            midpoint = self.polyline_midpoint(points)
            road_id, s, t = self._find_nearest_candidate_road_st(midpoint, road_ids)
            if road_id is None:
                continue
            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None:
                continue
            lane_ids = self._get_regulatory_lane_validity(reg_id, road_id)
            signal_id, stop_signal, _ = self._get_or_create_stop_line(
                group,
                way_id,
                way,
                points,
                s,
                t,
                lane_ids,
                written_signal_ids,
                regulatory_id=reg_id,
            )
            result.append((signal_id, stop_signal))
        return result

    def _get_or_create_stop_line(
        self,
        group: RoadGroup,
        way_id: str,
        way: Lanelet2Way,
        points: Sequence[Point2D],
        s: float,
        t: float,
        lane_ids: Sequence[int],
        written_signal_ids: Set[str],
        regulatory_id: Optional[str] = None,
    ) -> Tuple[str, etree._Element, etree._Element]:
        cache_key = (str(way_id), int(group.od_road_id))
        cached = self._stop_line_cache.get(cache_key)
        if cached is not None:
            signal_id, stop_signal, stop_object = cached
            self._merge_validities(stop_signal, lane_ids)
            self._merge_validities(stop_object, lane_ids)
            if regulatory_id is not None:
                self._append_user_data(
                    stop_signal, "lanelet2:regulatory_element", regulatory_id
                )
            return cached

        signal_id = self._unique_xml_id(
            f"stop_line_{way_id}_{group.od_road_id}", written_signal_ids
        )
        stop_signal = self._add_signal_to_road(
            group,
            signal_id=signal_id,
            name=f"lanelet2_stop_line_{way_id}",
            s=s,
            t=t,
            country=self._configured_signal_mapping("stop_line")["country"],
            signal_type=self._configured_signal_mapping("stop_line")["type"],
            signal_subtype=self._configured_signal_mapping("stop_line")["subtype"],
            dynamic=self._configured_signal_mapping("stop_line")["dynamic"],
            value="",
            unit="",
            lane_ids=lane_ids,
            dimension_overrides={"width": self.polyline_length(points)},
        )
        if regulatory_id is not None:
            self._append_user_data(
                stop_signal, "lanelet2:regulatory_element", regulatory_id
            )
        self._append_lanelet2_metadata(stop_signal, "way", way_id, way.tag_dict)
        stop_object = self._append_visible_stop_line_object(
            group, way_id, points, s, t, lane_ids
        )
        result = (signal_id, stop_signal, stop_object)
        self._stop_line_cache[cache_key] = result
        return result

    def _construct_standalone_stop_lines(
        self, written_signal_ids: Set[str]
    ) -> None:
        """Convert stop_line ways not already reached through a regulatory relation."""
        line_width = float(
            self.config.lanelet2_stop_line_visualization.get("line_width", 0.15)
        )
        for way_id, way in self.ways.items():
            if str(way.tag_dict.get("type", "")).lower() != "stop_line":
                continue
            if any(key[0] == str(way_id) for key in self._stop_line_cache):
                continue
            points = self.get_way_points(str(way_id))
            if len(points) < 2:
                continue
            midpoint = self.polyline_midpoint(points)
            road_id, s, t = self.find_nearest_road_st(midpoint)
            if road_id is None:
                continue
            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None:
                continue
            geometry = LineString(points).buffer(max(0.01, line_width) * 0.5)
            lane_ids = self._lane_ids_intersecting_geometry(group, geometry)
            self._get_or_create_stop_line(
                group,
                str(way_id),
                way,
                points,
                s,
                t,
                lane_ids,
                written_signal_ids,
            )

    def _lane_ids_intersecting_geometry(
        self, group: RoadGroup, geometry
    ) -> List[int]:
        lane_ids = []
        for lanelet in self.iter_group_lanelets(group):
            left = self.get_left_points(lanelet)
            right = self.get_right_points(lanelet)
            if len(left) < 2 or len(right) < 2:
                continue
            polygon = Polygon(left + list(reversed(right)))
            if not polygon.is_valid:
                polygon = polygon.buffer(0)
            lane_id = self.lanelet_to_lane.get(str(lanelet.id_))
            if (
                lane_id is not None
                and not polygon.is_empty
                and polygon.intersects(geometry)
            ):
                lane_ids.append(int(lane_id))
        return sorted(set(lane_ids))

    def _append_visible_stop_line_object(
        self,
        group: RoadGroup,
        way_id: str,
        points: Sequence[Point2D],
        s: float,
        t: float,
        lane_ids: Sequence[int],
    ) -> etree._Element:
        config = self.config.lanelet2_stop_line_visualization
        if not bool(config.get("enabled", True)):
            return etree.Element("object")
        length = self.polyline_length(points)
        if length <= 1e-6:
            return etree.Element("object")
        line_heading = math.atan2(
            points[-1][1] - points[0][1], points[-1][0] - points[0][0]
        )
        road_heading = self.heading_at_s(group.ref_points, s)
        relative_heading = math.atan2(
            math.sin(line_heading - road_heading),
            math.cos(line_heading - road_heading),
        )
        line_width = max(0.01, float(config.get("line_width", 0.15)))
        objects_el = group.road_xml.find("objects")
        if objects_el is None:
            objects_el = self._get_or_create_objects_element(group)
        object_el = etree.SubElement(
            objects_el,
            "object",
            id=f"stop_line_object_{way_id}_{group.od_road_id}",
            name="StopLine",
            s=f"{self._clamp_road_s(s, group.length):.6f}",
            t=f"{t:.6f}",
            zOffset=str(config.get("zOffset", 0.005)),
            orientation="none",
            type=str(config.get("object_type", "roadMark")),
            subtype="stopLine",
            dynamic="no",
            hdg=f"{relative_heading:.12f}",
            pitch="0.0",
            roll="0.0",
            length=f"{length:.6f}",
            width=f"{line_width:.6f}",
            height=str(config.get("height", 0.01)),
        )
        outlines = etree.SubElement(object_el, "outlines")
        outline = etree.SubElement(
            outlines, "outline", id="0", closed="true", outer="true"
        )
        half_length = length * 0.5
        half_width = line_width * 0.5
        for corner_id, (u, v) in enumerate(
            [
                (-half_length, -half_width),
                (half_length, -half_width),
                (half_length, half_width),
                (-half_length, half_width),
            ]
        ):
            etree.SubElement(
                outline,
                "cornerLocal",
                id=str(corner_id),
                u=f"{u:.6f}",
                v=f"{v:.6f}",
                z="0.0",
                height="0.0",
            )
        self._append_validities(object_el, lane_ids)
        self._append_user_data(object_el, "lanelet2:source_kind", "way")
        self._append_user_data(object_el, "lanelet2:source_id", way_id)
        return object_el

    def _find_nearest_candidate_road_st(
        self, point: Point2D, road_ids: Set[int]
    ) -> Tuple[Optional[int], float, float]:
        best: Optional[Tuple[float, int, float, float]] = None
        for road_id in sorted(road_ids):
            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None or not group.ref_points:
                continue
            s, t = self.project_point_to_polyline(point, group.ref_points)
            candidate = (abs(t), road_id, s, t)
            if best is None or candidate < best:
                best = candidate
        if best is None:
            return None, 0.0, 0.0
        return best[1], best[2], best[3]

    def _insert_root_controller(self, controller_el: etree._Element) -> None:
        """Insert root controllers before junctions to preserve OpenDRIVE XML order."""
        if self.root is None:
            return
        insert_at = len(self.root)
        for index, child in enumerate(self.root):
            if child.tag in {"junction", "junctionGroup", "station"}:
                insert_at = index
                break
        self.root.insert(insert_at, controller_el)

    def _construct_standalone_traffic_way_signals(self, written_signal_ids: Set[str]) -> None:
        """Convert traffic_sign / traffic_light ways that are not wrapped by a regulatory relation."""
        referred_way_ids: Set[str] = set()
        for reg in self.regulatory_elements.values():
            referred_way_ids.update(str(x) for x in getattr(reg, "refers", []) or [])
            referred_way_ids.update(str(x) for x in getattr(reg, "ref_line", []) or [])

        for way_id, way in self.ways.items():
            way_type = str(way.tag_dict.get("type", "")).lower()
            way_subtype = str(way.tag_dict.get("subtype", "")).lower()
            if way_type not in {"traffic_sign", "traffic_light"} and way_subtype not in {"traffic_sign", "traffic_light"}:
                continue
            if str(way_id) in referred_way_ids:
                continue

            pts = self.get_way_points(str(way_id))
            if not pts:
                continue

            point = self.polyline_midpoint(pts)
            road_id, s, t = self.find_nearest_road_st(point)
            if road_id is None:
                continue

            group = self.road_groups.get(road_id)
            if group is None or group.road_xml is None:
                continue

            country, signal_type, signal_subtype = self._map_way_to_signal_type(way)
            source_subtype = way_subtype or way_type or "traffic_sign"
            signal_id = self._unique_xml_id(f"way_{way_id}_{road_id}", written_signal_ids)
            signal_el = self._add_signal_to_road(
                group,
                signal_id=signal_id,
                name=f"lanelet2_{source_subtype}_{way_id}",
                s=s,
                t=t,
                country=country,
                signal_type=signal_type,
                signal_subtype=signal_subtype,
                dynamic=self._configured_signal_mapping(
                    "traffic_light" if way_type == "traffic_light" or way_subtype == "traffic_light" else "traffic_sign"
                )["dynamic"],
                value=self._get_way_signal_value(way),
                unit=self._get_way_signal_unit(way),
                lane_ids=[],
            )
            self._append_lanelet2_metadata(
                signal_el, "way", str(way_id), way.tag_dict
            )
            self._apply_signal_geometry_pose(
                signal_el,
                group,
                s,
                way_points=pts,
                way_id=str(way_id),
            )
            if way_type == "traffic_light" or way_subtype == "traffic_light":
                self._append_traffic_light_pole(
                    group, str(way_id), signal_el, s, t, lane_ids=[]
                )

    def _append_traffic_light_pole(
        self,
        group: RoadGroup,
        way_id: str,
        signal_el: etree._Element,
        s: float,
        t: float,
        lane_ids: Sequence[int],
    ) -> Optional[etree._Element]:
        """Add one visual pole for a concrete Lanelet2 traffic-light way."""
        config = self.config.lanelet2_traffic_light_pole_visualization
        if not bool(config.get("enabled", True)):
            return None

        cache_key = (str(way_id), int(group.od_road_id))
        cached = self._traffic_light_pole_cache.get(cache_key)
        if cached is not None:
            self._append_user_data(
                signal_el, "lanelet2:visual_pole_object", cached.get("id", "")
            )
            self._merge_validities(cached, lane_ids)
            return cached

        object_id = f"traffic_light_pole_{way_id}_{group.od_road_id}"
        objects_el = self._get_or_create_objects_element(group)
        pole_el = etree.SubElement(
            objects_el,
            "object",
            id=object_id,
            name=f"lanelet2_traffic_light_pole_{way_id}",
            s=f"{self._clamp_road_s(s, group.length):.6f}",
            t=f"{float(t):.6f}",
            zOffset=str(config.get("zOffset", 0.0)),
            orientation=str(config.get("orientation", "+")),
            type=str(config.get("object_type", "pole")),
            subtype="trafficLightPole",
            dynamic="no",
            hdg=str(config.get("hdg", 0.0)),
            pitch=str(config.get("pitch", 0.0)),
            roll=str(config.get("roll", 0.0)),
            radius=str(max(0.01, float(config.get("radius", 0.05)))),
            height=str(max(0.0, float(config.get("height", 3.5)))),
        )
        self._append_validities(pole_el, lane_ids)
        self._append_user_data(pole_el, "lanelet2:generated_visual", "traffic_light_pole")
        self._append_user_data(pole_el, "lanelet2:source_way_id", str(way_id))
        self._append_user_data(
            pole_el, "lanelet2:signal_id", signal_el.get("id", "")
        )
        self._append_user_data(signal_el, "lanelet2:visual_pole_object", object_id)
        self._traffic_light_pole_cache[cache_key] = pole_el
        return pole_el

    def _apply_signal_geometry_pose(
        self,
        signal_el: etree._Element,
        group: RoadGroup,
        s: float,
        way_points: Optional[Sequence[Point2D]] = None,
        way_id: Optional[str] = None,
        reg_id: Optional[str] = None,
        reg=None,
    ) -> None:
        """Derive signal orientation and heading offset from Lanelet2 geometry."""
        lanelet = self._signal_reference_lanelet(
            group.od_road_id, reg_id=reg_id, way_id=way_id
        )
        if lanelet is not None:
            centerline = self.raw_lanelet_centerline(lanelet, n_samples=20)
            if len(centerline) >= 2:
                signal_point = self.point_at_s(group.ref_points, s)
                center_s, _ = self.project_point_to_polyline(
                    signal_point, centerline
                )
                lane_heading = self.heading_at_s(centerline, center_s)
                road_heading = self.heading_at_s(group.ref_points, s)
                signal_el.set(
                    "orientation",
                    "+" if math.cos(lane_heading - road_heading) >= 0.0 else "-",
                )

        heading_points = list(way_points or [])
        if len(heading_points) < 2 and reg is not None:
            for ref_line_id in getattr(reg, "ref_line", []) or []:
                candidate = self.get_way_points(str(ref_line_id))
                if len(candidate) >= 2:
                    heading_points = candidate
                    break
        source_heading = self._polyline_endpoint_heading(heading_points)
        if source_heading is not None:
            road_heading = self.heading_at_s(group.ref_points, s)
            relative_heading = math.atan2(
                math.sin(source_heading - road_heading),
                math.cos(source_heading - road_heading),
            )
            signal_el.set("hOffset", f"{relative_heading:.12f}")

    def _signal_reference_lanelet(
        self,
        road_id: int,
        reg_id: Optional[str] = None,
        way_id: Optional[str] = None,
    ) -> Optional[WayRelation]:
        candidate_ids: List[str] = []
        if reg_id is not None:
            candidate_ids.extend(
                str(lanelet_id)
                for lanelet_id, lanelet in self.lanelets.items()
                if str(reg_id)
                in {str(value) for value in lanelet.regulatory_elements}
            )
        if way_id is not None:
            candidate_ids.extend(
                str(lanelet_id)
                for lanelet_id in self.way_to_lanelets.get(str(way_id), set())
            )
        for lanelet_id in dict.fromkeys(candidate_ids):
            if self.lanelet_to_road.get(lanelet_id) == int(road_id):
                return self.lanelets.get(lanelet_id)
        group = self.road_groups.get(int(road_id))
        if group is not None and group.road_lanes:
            return group.road_lanes[0]
        return None

    @staticmethod
    def _polyline_endpoint_heading(
        points: Sequence[Point2D],
    ) -> Optional[float]:
        for start, end in zip(points, points[1:]):
            dx = float(end[0]) - float(start[0])
            dy = float(end[1]) - float(start[1])
            if math.hypot(dx, dy) > 1e-9:
                return math.atan2(dy, dx)
        return None

    @staticmethod
    def _clamp_road_s(s: float, road_length: float) -> float:
        """
        Keep s inside the serialized OpenDRIVE road range.

        Road length is written with 6 decimal places. Values projected exactly
        at the road end may otherwise round to length + 0.000001, which strict
        loaders such as esmini reject when resolving lane sections for signals.
        """
        length = max(0.0, float(road_length))
        if length <= 1e-9:
            return 0.0
        upper = max(0.0, length - 1e-6)
        return max(0.0, min(float(s), upper))

    def _add_signal_to_road(
        self,
        group: RoadGroup,
        signal_id: str,
        name: str,
        s: float,
        t: float,
        country: str,
        signal_type: str,
        signal_subtype: str,
        dynamic: str,
        value: str,
        unit: str,
        lane_ids: Sequence[int],
        dimension_overrides: Optional[Dict[str, float]] = None,
    ) -> etree._Element:
        signals_el = group.road_xml.find("signals") if group.road_xml is not None else None
        if signals_el is None:
            signals_el = etree.SubElement(group.road_xml, "signals")

        if signal_type == self._configured_signal_mapping("traffic_light")["type"]:
            defaults = self._configured_signal_mapping("traffic_light")
        elif signal_type == self._configured_signal_mapping("stop_line")["type"]:
            defaults = self._configured_signal_mapping("stop_line")
        else:
            defaults = self._configured_signal_mapping("traffic_sign")
        dimensions = dict(dimension_overrides or {})
        attributes = {
            "id": str(signal_id),
            "name": name,
            "s": f"{self._clamp_road_s(s, group.length):.6f}",
            "t": f"{t:.6f}",
            "dynamic": dynamic,
            "orientation": defaults["orientation"],
            "zOffset": str(defaults["zOffset"]),
            "country": country,
            "type": signal_type,
            "subtype": signal_subtype,
            "hOffset": str(defaults["hOffset"]),
            "pitch": str(defaults["pitch"]),
            "roll": str(defaults["roll"]),
            "height": str(dimensions.get("height", defaults["height"])),
            "width": str(dimensions.get("width", defaults["width"])),
        }
        if country == "OpenDRIVE" and defaults["countryRevision"]:
            attributes["countryRevision"] = defaults["countryRevision"]
        if value and unit:
            attributes["value"] = value
            attributes["unit"] = unit
        signal_el = etree.SubElement(signals_el, "signal", **attributes)

        self._append_validities(signal_el, lane_ids)

        return signal_el

    @staticmethod
    def _append_validities(parent: etree._Element, lane_ids: Sequence[int]) -> None:
        """Write separate validity ranges when controlled lane IDs are not contiguous."""
        ids = sorted(set(int(lane_id) for lane_id in lane_ids))
        if not ids:
            return
        range_start = range_end = ids[0]
        for lane_id in ids[1:]:
            if lane_id == range_end + 1:
                range_end = lane_id
                continue
            etree.SubElement(
                parent,
                "validity",
                fromLane=str(range_start),
                toLane=str(range_end),
            )
            range_start = range_end = lane_id
        etree.SubElement(
            parent,
            "validity",
            fromLane=str(range_start),
            toLane=str(range_end),
        )

    @classmethod
    def _merge_validities(
        cls, parent: etree._Element, lane_ids: Sequence[int]
    ) -> None:
        merged = set(int(lane_id) for lane_id in lane_ids)
        for validity in parent.findall("validity"):
            start = int(validity.get("fromLane"))
            end = int(validity.get("toLane"))
            merged.update(range(start, end + 1))
            parent.remove(validity)
        cls._append_validities(parent, sorted(merged))

    def _get_regulatory_subtype(self, reg) -> str:
        subtype = str(reg.tag_dict.get("subtype", reg.tag_dict.get("type", ""))).lower()
        if subtype and subtype != "regulatory_element":
            return subtype

        for way_id in list(getattr(reg, "refers", []) or []) + list(getattr(reg, "ref_line", []) or []):
            way = self.ways.get(str(way_id))
            if way is None:
                continue
            way_type = str(way.tag_dict.get("type", "")).lower()
            way_subtype = str(way.tag_dict.get("subtype", "")).lower()
            if way_type in {"traffic_sign", "traffic_light"}:
                return way_type
            if way_subtype:
                return way_subtype

        return "regulatory_element"

    def _map_regulatory_to_signal_type(self, reg) -> Tuple[str, str, str]:
        subtype = self._get_regulatory_subtype(reg)

        if subtype == "traffic_light":
            configured = self._configured_signal_mapping("traffic_light")
            return configured["country"], configured["type"], configured["subtype"]

        for way_id in getattr(reg, "refers", []) or []:
            way = self.ways.get(str(way_id))
            if way is not None:
                mapped = self._map_way_to_signal_type(way)
                if mapped[1] != "-1":
                    return mapped

        configured = self._configured_signal_mapping(
            subtype
            if subtype in self.config.lanelet2_signal_type_mapping
            else ("traffic_sign" if subtype == "traffic_sign" else "unknown")
        )
        return configured["country"], configured["type"], configured["subtype"]

    def _map_way_to_signal_type(
        self, way: Lanelet2Way, force_traffic_light: bool = False
    ) -> Tuple[str, str, str]:
        way_type = str(way.tag_dict.get("type", "")).lower()
        way_subtype = str(way.tag_dict.get("subtype", "")).lower()
        if force_traffic_light or way_type == "traffic_light" or way_subtype == "traffic_light":
            configured = self._configured_signal_mapping("traffic_light")
            return configured["country"], configured["type"], configured["subtype"]

        # Lanelet2 country-specific signs commonly use forms such as de206,
        # de274-50 or usR1-1. Keep the catalog selector and identifiers separate.
        match = re.fullmatch(r"([a-z]{2})([a-z0-9]+)(?:[-_](.+))?", way_subtype, re.IGNORECASE)
        if match:
            country = match.group(1).upper()
            signal_type = match.group(2)
            signal_subtype = match.group(3) or "-1"
            return country, signal_type, signal_subtype

        configured = self._configured_signal_mapping(
            "traffic_sign" if way_type == "traffic_sign" or way_subtype == "traffic_sign" else "unknown"
        )
        return configured["country"], configured["type"], configured["subtype"]

    def _configured_signal_mapping(self, key: str) -> Dict[str, str]:
        table = self.config.lanelet2_signal_type_mapping
        raw = table.get(key, table.get("unknown", {}))
        return {
            "country": str(raw.get("country", "OpenDRIVE")),
            "countryRevision": str(raw.get("countryRevision", "")),
            "type": str(raw.get("type", "-1")),
            "subtype": str(raw.get("subtype", "-1")),
            "dynamic": str(raw.get("dynamic", "no")),
            "orientation": str(raw.get("orientation", "+")),
            "zOffset": str(raw.get("zOffset", 0.0)),
            "hOffset": str(raw.get("hOffset", 0.0)),
            "pitch": str(raw.get("pitch", 0.0)),
            "roll": str(raw.get("roll", 0.0)),
            "height": str(raw.get("height", 0.6)),
            "width": str(raw.get("width", 0.6)),
            "semantic": str(raw.get("semantic", "")),
            "mapping_status": str(raw.get("mapping_status", "resolved")),
        }

    def _append_signal_mapping_metadata(
        self, signal_el: etree._Element, mapping_key: str
    ) -> None:
        mapping = self._configured_signal_mapping(mapping_key)
        if mapping["semantic"]:
            self._append_user_data(
                signal_el, "lanelet2:semantic", mapping["semantic"]
            )
        if (
            signal_el.get("type") == "-1"
            and mapping["mapping_status"] != "resolved"
        ):
            self._append_user_data(
                signal_el, "lanelet2:mapping_status", mapping["mapping_status"]
            )

    def _get_signal_value(self, reg) -> str:
        for key in ["speed_limit", "speed", "value", "sign_type"]:
            if key in reg.tag_dict:
                value = self._extract_numeric_value(reg.tag_dict[key])
                if value:
                    return value

        for way_id in list(getattr(reg, "refers", []) or []) + list(getattr(reg, "ref_line", []) or []):
            way = self.ways.get(str(way_id))
            if way is None:
                continue
            value = self._get_way_signal_value(way)
            if value:
                return value

        return ""

    def _get_signal_unit(self, reg) -> str:
        explicit = self._normalize_signal_unit(reg.tag_dict.get("unit", ""))
        if explicit:
            return explicit
        subtype = self._get_regulatory_subtype(reg)
        if subtype == "speed_limit":
            return "km/h"
        for key in ["speed_limit", "speed", "value", "sign_type"]:
            if key in reg.tag_dict:
                inferred = self._unit_from_text(reg.tag_dict[key])
                if inferred:
                    return inferred
        return ""

    def _get_way_signal_value(self, way: Lanelet2Way) -> str:
        for key in ["speed_limit", "speed", "value", "sign_type"]:
            if key in way.tag_dict:
                value = self._extract_numeric_value(way.tag_dict[key])
                if value:
                    return value
        catalog = self._catalog_signal_value(way)
        return catalog.get("value", "")

    def _get_way_signal_unit(self, way: Lanelet2Way) -> str:
        explicit = self._normalize_signal_unit(way.tag_dict.get("unit", ""))
        if explicit:
            return explicit
        if any(key in way.tag_dict for key in ["speed_limit", "speed"]):
            for key in ["speed_limit", "speed"]:
                if key in way.tag_dict:
                    return self._unit_from_text(way.tag_dict[key]) or "km/h"
        catalog = self._catalog_signal_value(way)
        return catalog.get("unit", "")

    def _catalog_signal_value(self, way: Lanelet2Way) -> Dict[str, str]:
        country, signal_type, signal_subtype = self._map_way_to_signal_type(way)
        key = f"{country}:{signal_type}:{signal_subtype}"
        raw = self.config.lanelet2_country_signal_value_mapping.get(key, {})
        return {
            "value": str(raw.get("value", "")),
            "unit": str(raw.get("unit", "")),
        }

    @staticmethod
    def _extract_numeric_value(raw_value) -> str:
        match = re.search(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", str(raw_value))
        return match.group(0) if match else ""

    @staticmethod
    def _normalize_signal_unit(raw_unit) -> str:
        normalized = str(raw_unit).strip().lower().replace("kph", "km/h")
        allowed = {"m", "km", "ft", "mile", "m/s", "mph", "km/h", "kg", "t", "%"}
        return normalized if normalized in allowed else ""

    def _unit_from_text(self, raw_value) -> str:
        text = str(raw_value).lower()
        for unit in ("km/h", "kph", "m/s", "mph", "mile", "km", "ft", "m", "kg", "%"):
            if re.search(rf"(?<![a-z]){re.escape(unit)}(?![a-z])", text):
                return self._normalize_signal_unit(unit)
        return ""

    def _get_regulatory_reference_point(self, reg) -> Optional[Point2D]:
        candidate_way_ids = list(reg.ref_line) + list(reg.refers)

        for way_id in candidate_way_ids:
            pts = self.get_way_points(str(way_id))
            if pts:
                return self.polyline_midpoint(pts)

        return None

    def _get_regulatory_lane_validity(self, reg_id: str, road_id: int) -> List[int]:
        group = self.road_groups.get(road_id)
        if group is None:
            return []

        lane_ids: List[int] = []
        for lanelet in group.road_lanes:
            lanelet_id = str(lanelet.id_)
            if str(reg_id) not in [str(x) for x in lanelet.regulatory_elements]:
                continue
            lane_id = self.lanelet_to_lane.get(lanelet_id)
            if lane_id is not None:
                lane_ids.append(int(lane_id))

        if lane_ids:
            return sorted(set(lane_ids))

        return sorted(
            {
                int(self.lanelet_to_lane[str(l.id_)])
                for l in group.road_lanes
                if str(l.id_) in self.lanelet_to_lane
            }
        )

    @staticmethod
    def _unique_xml_id(base_id: str, used_ids: Set[str]) -> str:
        candidate = str(base_id)
        suffix = 2
        while candidate in used_ids:
            candidate = f"{base_id}_{suffix}"
            suffix += 1
        used_ids.add(candidate)
        return candidate

    @staticmethod
    def _append_user_data(parent: etree._Element, code: str, value: str) -> etree._Element:
        return etree.SubElement(parent, "userData", code=str(code), value=str(value))

    def _append_lanelet2_metadata(
        self,
        parent: etree._Element,
        source_kind: str,
        source_id: str,
        tags: Dict[str, str],
    ) -> None:
        """Preserve Lanelet2 metadata without encoding several tags in one string."""
        self._append_user_data(parent, "lanelet2:source_kind", source_kind)
        self._append_user_data(parent, "lanelet2:source_id", source_id)
        for key in sorted(tags):
            self._append_user_data(parent, f"lanelet2:tag:{key}", tags[key])

    # ---------------------------------------------------------------------
    # Multipolygon / object
    # ---------------------------------------------------------------------

    def construct_multipolygons(self) -> None:
        """
        Convert Lanelet2 multipolygons into configured OpenDRIVE objects.

        Area-like objects preserve every usable outer ring as a closed
        cornerRoad outline. Explicitly configured simple objects use a compact
        bounding shape instead.
        """
        road_surfaces = self._build_active_road_surfaces()
        for mp_id, mp in self.multipolygons.items():
            outer_rings = [
                self._clean_outline_ring(self.get_way_points(str(way_id)))
                for way_id in mp.outer_list
            ]
            outer_rings = [ring for ring in outer_rings if len(ring) >= 3]
            inner_rings = [
                self._clean_outline_ring(self.get_way_points(str(way_id)))
                for way_id in getattr(mp, "inner_list", []) or []
            ]
            inner_rings = [ring for ring in inner_rings if len(ring) >= 3]
            all_rings = outer_rings + inner_rings
            all_points = [point for ring in all_rings for point in ring]
            if not all_points:
                continue

            source_subtype = self._multipolygon_source_subtype(mp)
            object_type = self._map_multipolygon_to_object_type(source_subtype)
            simple_types = {
                str(value) for value in self.config.lanelet2_simple_object_types
            }
            is_simple = object_type in simple_types

            centroid = self.centroid(all_points)
            object_surface = self._polygon_from_outer_inner_rings(
                outer_rings, inner_rings
            )
            nearest_road_id, s, t = self.find_nearest_road_st(centroid)
            intersecting_road_id = self._select_primary_object_road(
                centroid, object_surface, road_surfaces
            )
            if intersecting_road_id is not None:
                nearest_road_id = intersecting_road_id
                selected_group = self.road_groups[nearest_road_id]
                s, t = self.project_point_to_polyline(
                    centroid, selected_group.ref_points
                )
            if nearest_road_id is None:
                continue

            group = self.road_groups[nearest_road_id]
            if group.road_xml is None or group.road_xml.getparent() is not self.root:
                continue

            projected_rings: List[List[Tuple[float, float]]] = []
            if not is_simple:
                for ring in all_rings:
                    projected_rings.append(
                        [self.project_point_to_polyline(point, group.ref_points) for point in ring]
                    )

            if projected_rings:
                projected_points = [point for ring in projected_rings for point in ring]
                min_s = min(point[0] for point in projected_points)
                max_s = max(point[0] for point in projected_points)
                min_t = min(point[1] for point in projected_points)
                max_t = max(point[1] for point in projected_points)
                s = max(0.0, min(group.length, (min_s + max_s) * 0.5))
                t = (min_t + max_t) * 0.5
                length = max(0.01, max_s - min_s)
                width = max(0.01, max_t - min_t)
            else:
                length = 0.0
                width = 0.0

            objects_el = group.road_xml.find("objects")
            if objects_el is None:
                objects_el = self._get_or_create_objects_element(group)

            obj_id = f"mp_{mp_id}"
            attributes = {
                "id": obj_id,
                "name": f"lanelet2_{source_subtype}_{mp_id}",
                "s": f"{self._clamp_road_s(s, group.length):.6f}",
                "t": f"{t:.6f}",
                "zOffset": "0.0",
                "orientation": "none",
                "type": object_type,
                "subtype": source_subtype,
                "dynamic": "no",
                "hdg": "0.0",
                "pitch": "0.0",
                "roll": "0.0",
            }

            if is_simple:
                dimensions = self.config.lanelet2_simple_object_dimensions.get(
                    object_type, {}
                )
                attributes["radius"] = str(max(0.01, float(dimensions.get("radius", 0.25))))
                attributes["height"] = str(max(0.0, float(dimensions.get("height", 0.0))))
            else:
                attributes["length"] = f"{length:.6f}"
                attributes["width"] = f"{width:.6f}"
                attributes["height"] = "0.0"

            object_el = etree.SubElement(objects_el, "object", **attributes)

            if projected_rings:
                heading = self.heading_at_s(group.ref_points, s)
                cos_h = math.cos(heading)
                sin_h = math.sin(heading)
                reference_point = self.point_at_s(group.ref_points, s)
                origin = (
                    reference_point[0] - t * sin_h,
                    reference_point[1] + t * cos_h,
                )
                outlines_el = etree.SubElement(object_el, "outlines")
                outlined_rings = [
                    (ring, True) for ring in outer_rings
                ] + [
                    (ring, False) for ring in inner_rings
                ]
                for outline_index, (ring, is_outer) in enumerate(outlined_rings):
                    outline_el = etree.SubElement(
                        outlines_el,
                        "outline",
                        id=str(outline_index),
                        closed="true",
                        outer="true" if is_outer else "false",
                    )
                    for corner_index, (corner_x, corner_y) in enumerate(ring):
                        dx = corner_x - origin[0]
                        dy = corner_y - origin[1]
                        corner_u = dx * cos_h + dy * sin_h
                        corner_v = -dx * sin_h + dy * cos_h
                        etree.SubElement(
                            outline_el,
                            "cornerLocal",
                            id=str(corner_index),
                            u=f"{corner_u:.6f}",
                            v=f"{corner_v:.6f}",
                            z="0.0",
                            height="0.0",
                        )

            self._append_lanelet2_metadata(
                object_el, "multipolygon", str(mp_id), mp.tag_dict
            )
            if projected_rings:
                self._append_object_references(
                    obj_id,
                    object_surface,
                    nearest_road_id,
                    road_surfaces,
                )

            self.multipolygon_to_object[str(mp_id)] = obj_id

    def _select_primary_object_road(self, point, object_surface, road_surfaces):
        """Prefer the nearest road whose actual lane surface overlaps the object."""
        if object_surface is None or object_surface.is_empty:
            return None
        best_road_id = None
        best_abs_t = float("inf")
        object_bounds = object_surface.bounds
        for road_id, group, road_surface, _ in road_surfaces:
            if not self._bounds_overlap(object_bounds, road_surface.bounds):
                continue
            overlap = object_surface.intersection(road_surface)
            if overlap.is_empty or overlap.area <= 1e-4:
                continue
            _, t = self.project_point_to_polyline(point, group.ref_points)
            if abs(t) < best_abs_t:
                best_abs_t = abs(t)
                best_road_id = road_id
        return best_road_id

    def _build_active_road_surfaces(self):
        """Build road and lane polygons used for objectReference assignment."""
        surfaces = []
        for road_id, group in self.road_groups.items():
            if (
                group.is_removed
                or group.road_xml is None
                or group.road_xml.getparent() is not self.root
            ):
                continue
            lane_surfaces = []
            for lanelet in self.iter_group_lanelets(group):
                left = self.get_left_points(lanelet)
                right = self.get_right_points(lanelet)
                if len(left) < 2 or len(right) < 2:
                    continue
                polygon = Polygon(left + list(reversed(right)))
                if not polygon.is_valid:
                    polygon = polygon.buffer(0)
                if polygon.is_empty or polygon.area <= 1e-6:
                    continue
                lane_id = self.lanelet_to_lane.get(str(lanelet.id_))
                lane_surfaces.append((int(lane_id) if lane_id is not None else None, polygon))
            if not lane_surfaces:
                continue
            road_surface = unary_union([polygon for _, polygon in lane_surfaces])
            if not road_surface.is_empty:
                surfaces.append((road_id, group, road_surface, lane_surfaces))
        return surfaces

    @staticmethod
    def _polygon_from_rings(rings: Sequence[Sequence[Point2D]]):
        polygons = []
        for ring in rings:
            if len(ring) < 3:
                continue
            polygon = Polygon(ring)
            if not polygon.is_valid:
                polygon = polygon.buffer(0)
            if not polygon.is_empty and polygon.area > 1e-6:
                polygons.append(polygon)
        return unary_union(polygons) if polygons else None

    @classmethod
    def _polygon_from_outer_inner_rings(
        cls,
        outer_rings: Sequence[Sequence[Point2D]],
        inner_rings: Sequence[Sequence[Point2D]],
    ):
        surface = cls._polygon_from_rings(outer_rings)
        holes = cls._polygon_from_rings(inner_rings)
        if surface is None or surface.is_empty or holes is None or holes.is_empty:
            return surface
        result = surface.difference(holes)
        if not result.is_valid:
            result = result.buffer(0)
        return result

    def _append_object_references(
        self,
        object_id: str,
        object_surface,
        primary_road_id: int,
        road_surfaces,
    ) -> None:
        """Reference one object from every additional road it actually overlaps."""
        if object_surface is None or object_surface.is_empty:
            return
        object_bounds = object_surface.bounds
        representative = object_surface.representative_point()
        point = (float(representative.x), float(representative.y))

        for road_id, group, road_surface, lane_surfaces in road_surfaces:
            if road_id == primary_road_id:
                continue
            road_bounds = road_surface.bounds
            if not self._bounds_overlap(object_bounds, road_bounds):
                continue
            overlap = object_surface.intersection(road_surface)
            if overlap.is_empty or overlap.area <= 1e-4:
                continue

            s, t = self.project_point_to_polyline(point, group.ref_points)
            objects_el = group.road_xml.find("objects")
            if objects_el is None:
                objects_el = self._get_or_create_objects_element(group)
            reference = etree.SubElement(
                objects_el,
                "objectReference",
                id=str(object_id),
                s=f"{self._clamp_road_s(s, group.length):.6f}",
                t=f"{t:.6f}",
                zOffset="0.0",
                orientation="none",
            )

            lane_ids = sorted(
                {
                    lane_id
                    for lane_id, lane_surface in lane_surfaces
                    if lane_id is not None
                    and self._bounds_overlap(object_bounds, lane_surface.bounds)
                    and object_surface.intersection(lane_surface).area > 1e-4
                }
            )
            if lane_ids:
                etree.SubElement(
                    reference,
                    "validity",
                    fromLane=str(lane_ids[0]),
                    toLane=str(lane_ids[-1]),
                )

    @staticmethod
    def _bounds_overlap(a, b) -> bool:
        return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])

    @staticmethod
    def _get_or_create_objects_element(group: RoadGroup) -> etree._Element:
        objects_el = group.road_xml.find("objects")
        if objects_el is not None:
            return objects_el
        objects_el = etree.Element("objects")
        signals_el = group.road_xml.find("signals")
        if signals_el is None:
            group.road_xml.append(objects_el)
        else:
            group.road_xml.insert(group.road_xml.index(signals_el), objects_el)
        return objects_el

    @staticmethod
    def _clean_outline_ring(points: Sequence[Point2D]) -> List[Point2D]:
        ring: List[Point2D] = []
        for point in points:
            normalized = (float(point[0]), float(point[1]))
            if not ring or math.hypot(normalized[0] - ring[-1][0], normalized[1] - ring[-1][1]) > 1e-6:
                ring.append(normalized)
        if len(ring) > 1 and math.hypot(
            ring[0][0] - ring[-1][0], ring[0][1] - ring[-1][1]
        ) <= 1e-6:
            ring.pop()
        return ring

    @staticmethod
    def _multipolygon_source_subtype(mp: Multipolygon) -> str:
        subtype = str(mp.tag_dict.get("subtype", mp.tag_dict.get("type", "multipolygon")))
        normalized = subtype.strip().lower().replace("-", "_").replace(" ", "_")
        if normalized == "stop_area" and mp.tag_dict.get("stop_area_type"):
            stop_type = str(mp.tag_dict["stop_area_type"]).strip().lower()
            normalized = f"stop_area_{stop_type.replace('-', '_').replace(' ', '_')}"
        return normalized or "multipolygon"

    def _map_multipolygon_to_object_type(self, source_subtype: str) -> str:
        mapping = {
            str(key).lower(): str(value)
            for key, value in self.config.lanelet2_object_type_mapping.items()
        }
        return mapping.get(source_subtype.lower(), "none")

    # ---------------------------------------------------------------------
    # XML writing helpers
    # ---------------------------------------------------------------------

    def _create_opendrive_root(self) -> etree._Element:
        root = etree.Element("OpenDRIVE")

        header = etree.SubElement(
            root,
            "header",
            revMajor=str(int(self.config.target_version_major)),
            revMinor=str(int(self.config.target_version_minor)),
            name="Lanelet2_to_OpenDRIVE",
            version="1.00",
            date=time.strftime("%Y-%m-%dT%H:%M:%S"),
            north="0.0",
            south="0.0",
            east="0.0",
            west="0.0",
            vendor="lanelet2opendrive_converter",
        )

        if self.origin_lat is not None and self.origin_lon is not None:
            geo_reference = etree.SubElement(header, "geoReference")
            geo_reference.text = (
                "+proj=tmerc "
                f"+lat_0={self.origin_lat:.12f} +lon_0={self.origin_lon:.12f} "
                f"+k=1 +x_0={self.origin_local_x:.6f} +y_0={self.origin_local_y:.6f} "
                "+datum=WGS84 +units=m"
            )

        return root

    def _write_road_type(self, road_el: etree._Element) -> None:
        type_el = etree.SubElement(road_el, "type", s="0.0", type="town")
        etree.SubElement(type_el, "speed", max="13.8889", unit="m/s")

    def _write_plan_view(self, road_el: etree._Element, ref_points: List[Point2D]) -> None:
        """
        Write OpenDRIVE planView.

        Fitting strategy:
        1. If the reference line is almost straight, write one <line/> geometry.
        2. If the points fit a circular arc well, write one <arc/> geometry.
        3. If the points fit a local cubic lateral polynomial well, write one <paramPoly3/> geometry.
        4. Otherwise, preserve the original polyline as many short <line/> geometries.

        This keeps robust first-version behavior while reducing unnecessary broken
        geometry when a simple fit is reliable.
        """
        plan_view_el = etree.SubElement(road_el, "planView")
        ref_points = self.clean_polyline(ref_points)

        if len(ref_points) < 2:
            return

        if not self.ref_line_fit:
            self._write_plan_view_as_polyline(plan_view_el, ref_points)
            return

        straight = self.fit_straight_line_geometry(ref_points)
        if straight is not None:
            x, y, hdg, length = straight
            geometry_el = etree.SubElement(
                plan_view_el,
                "geometry",
                s="0.000000",
                x=f"{x:.6f}",
                y=f"{y:.6f}",
                hdg=f"{hdg:.12f}",
                length=f"{length:.6f}",
            )
            etree.SubElement(geometry_el, "line")
            return

        arc = self.fit_arc_geometry(ref_points, max_error=self.ref_line_fit_max_error)
        if arc is not None:
            x, y, hdg, length, curvature = arc
            geometry_el = etree.SubElement(
                plan_view_el,
                "geometry",
                s="0.000000",
                x=f"{x:.6f}",
                y=f"{y:.6f}",
                hdg=f"{hdg:.12f}",
                length=f"{length:.6f}",
            )
            etree.SubElement(geometry_el, "arc", curvature=f"{curvature:.12f}")
            return

        param_poly = self.fit_param_poly3_geometry(ref_points, max_error=self.ref_line_fit_max_error)
        if param_poly is not None:
            x, y, hdg, length, coeffs = param_poly
            geometry_el = etree.SubElement(
                plan_view_el,
                "geometry",
                s="0.000000",
                x=f"{x:.6f}",
                y=f"{y:.6f}",
                hdg=f"{hdg:.12f}",
                length=f"{length:.6f}",
            )
            etree.SubElement(
                geometry_el,
                "paramPoly3",
                aU=f"{coeffs['aU']:.12f}",
                bU=f"{coeffs['bU']:.12f}",
                cU=f"{coeffs['cU']:.12f}",
                dU=f"{coeffs['dU']:.12f}",
                aV=f"{coeffs['aV']:.12f}",
                bV=f"{coeffs['bV']:.12f}",
                cV=f"{coeffs['cV']:.12f}",
                dV=f"{coeffs['dV']:.12f}",
                pRange="arcLength",
            )
            return

        self._write_plan_view_as_polyline(plan_view_el, ref_points)

    def _write_plan_view_as_polyline(self, plan_view_el: etree._Element, ref_points: List[Point2D]) -> None:
        """Fallback: write every reference-line segment as a line geometry."""
        s_acc = 0.0
        for p0, p1 in zip(ref_points[:-1], ref_points[1:]):
            seg_len = self.distance(p0, p1)
            if seg_len <= 1e-6:
                continue

            hdg = math.atan2(p1[1] - p0[1], p1[0] - p0[0])

            geometry_el = etree.SubElement(
                plan_view_el,
                "geometry",
                s=f"{s_acc:.6f}",
                x=f"{p0[0]:.6f}",
                y=f"{p0[1]:.6f}",
                hdg=f"{hdg:.12f}",
                length=f"{seg_len:.6f}",
            )
            etree.SubElement(geometry_el, "line")
            s_acc += seg_len

    def _write_lanes(self, road_el: etree._Element, road_group: RoadGroup) -> None:
        lanes_el = etree.SubElement(road_el, "lanes")
        lane_section_el = etree.SubElement(lanes_el, "laneSection", s="0.0")

        center_el = etree.SubElement(lane_section_el, "center")
        center_lane_el = etree.SubElement(center_el, "lane", id="0", type="none", level="false")

        # The current simplified geometry uses the left boundary of the left-most lanelet
        # as OpenDRIVE reference line, so the center roadMark corresponds to that boundary.
        if road_group.road_lanes:
            reference_marking = self.get_boundary_marking(road_group.road_lanes[-1].left_way)
        else:
            reference_marking = "unknown"
        etree.SubElement(
            center_lane_el,
            "roadMark",
            sOffset="0.0",
            type=self.map_boundary_marking_to_roadmark(reference_marking),
            weight="standard",
            color="standard",
            width="0.13",
            laneChange=self.map_boundary_marking_to_lane_change(reference_marking),
        )

        right_el = etree.SubElement(lane_section_el, "right")

        if road_group.longitudinal_road_lanes:
            first_section = road_group.longitudinal_road_lanes[0]
            lane_count = len(first_section)
            for index in range(1, lane_count + 1):
                lane_id = -index
                # road_lanes are right -> left; OpenDRIVE right-side lane -1 is
                # the left-most lanelet, so use reversed section order.
                first_lanelet = list(reversed(first_section))[index - 1]
                lane_type = self._map_lanelet_type(first_lanelet)

                lane_el = etree.SubElement(
                    right_el,
                    "lane",
                    id=str(lane_id),
                    type=lane_type,
                    level="false",
                )
                link_el = etree.SubElement(lane_el, "link")

                right_marking = self.get_boundary_marking(first_lanelet.right_way)
                etree.SubElement(
                    lane_el,
                    "roadMark",
                    sOffset="0.0",
                    type=self.map_boundary_marking_to_roadmark(right_marking),
                    weight="standard",
                    color="standard",
                    width="0.13",
                    laneChange=self.map_boundary_marking_to_lane_change(right_marking),
                )

                width_samples: List[Tuple[float, float]] = []
                s_offset_acc = 0.0
                mapped_lanelets: List[WayRelation] = []
                for section_lanes, section_ref in zip(
                    road_group.longitudinal_road_lanes,
                    road_group.longitudinal_ref_points or [],
                ):
                    section_left_to_right = list(reversed(section_lanes))
                    if index - 1 >= len(section_left_to_right):
                        continue
                    lanelet = section_left_to_right[index - 1]
                    mapped_lanelets.append(lanelet)
                    lanelet_id = str(lanelet.id_)
                    self.lanelet_to_lane_xml[lanelet_id] = lane_el
                    self.lanelet_to_lane_link_xml[lanelet_id] = link_el

                    section_samples = self.compute_lane_width_samples(lanelet, section_ref)
                    for s_local, width in section_samples:
                        width_samples.append((s_offset_acc + s_local, width))
                    s_offset_acc += self.polyline_length(section_ref)

                # Write one constant width record for the whole lane.
                # This intentionally avoids b/c/d polynomial variation, which can
                # create twisted lane meshes in TESS NG when Lanelet2 boundaries are
                # non-parallel or locally irregular.
                constant_width = self.constant_lane_width_from_samples(width_samples, first_lanelet)
                road_group.lane_width_samples[lane_id] = [(0.0, constant_width)]
                self.write_constant_lane_width(lane_el, constant_width)
                self._append_lanelet_provenance(lane_el, mapped_lanelets)

            return

        # Normal non-chain road: road_lanes is right -> left.
        left_to_right = list(reversed(road_group.road_lanes))

        for index, lanelet in enumerate(left_to_right, start=1):
            lane_id = -index
            lane_type = self._map_lanelet_type(lanelet)

            lane_el = etree.SubElement(
                right_el,
                "lane",
                id=str(lane_id),
                type=lane_type,
                level="false",
            )

            lanelet_id = str(lanelet.id_)
            link_el = etree.SubElement(lane_el, "link")
            self.lanelet_to_lane_xml[lanelet_id] = lane_el
            self.lanelet_to_lane_link_xml[lanelet_id] = link_el

            # With the current all-right-side simplification, this lane's roadMark is
            # approximated using the lanelet right boundary. Later this can be refined
            # to distinguish inner/outer marks and roadMark records along s.
            right_marking = self.get_boundary_marking(lanelet.right_way)
            etree.SubElement(
                lane_el,
                "roadMark",
                sOffset="0.0",
                type=self.map_boundary_marking_to_roadmark(right_marking),
                weight="standard",
                color="standard",
                width="0.13",
                laneChange=self.map_boundary_marking_to_lane_change(right_marking),
            )

            width_samples = self.compute_lane_width_samples(lanelet, road_group.ref_points)
            # Write one constant width record for the whole lane.
            # This keeps the generated internal lane boundaries parallel/regular
            # with respect to the reference line and avoids width-polynomial
            # oscillation in downstream simulators.
            constant_width = self.constant_lane_width_from_samples(width_samples, lanelet)
            road_group.lane_width_samples[lane_id] = [(0.0, constant_width)]
            self.write_constant_lane_width(lane_el, constant_width)
            self._append_lanelet_provenance(lane_el, [lanelet])

    def _append_lanelet_provenance(
        self,
        lane_el: etree._Element,
        lanelets: Sequence[WayRelation],
    ) -> None:
        """Record every source lanelet and boundary way represented by one ODR lane."""
        self._append_user_data(lane_el, "lanelet2:source_kind", "lanelet")
        seen = set()
        for lanelet in lanelets:
            value = (
                f"{lanelet.id_}|{lanelet.left_way}|{lanelet.right_way}"
            )
            if value in seen:
                continue
            seen.add(value)
            self._append_user_data(
                lane_el, "lanelet2:lanelet_mapping", value
            )

    def get_boundary_marking(self, way_id: str) -> str:
        """
        Return a compact Lanelet2 boundary marking name for a way.

        Typical Lanelet2 way tags:
            type=virtual
            type=line_thin, subtype=solid
            type=line_thin, subtype=dashed
            type=line_thin, subtype=solid_solid
        """
        way = self.ways.get(str(way_id))
        if way is None:
            return "unknown"

        typ = str(way.tag_dict.get("type", "")).lower()
        subtype = str(way.tag_dict.get("subtype", "")).lower()

        if typ == "virtual":
            return "virtual"

        if typ == "line_thin":
            return subtype or "line_thin"

        if subtype:
            return subtype

        return typ or "unknown"

    def get_lanelet_boundary_markings(self, lanelet: WayRelation) -> Tuple[str, str]:
        """Return (left_marking, right_marking) for a lanelet."""
        return (
            self.get_boundary_marking(lanelet.left_way),
            self.get_boundary_marking(lanelet.right_way),
        )

    def map_boundary_marking_to_roadmark(self, marking: str) -> str:
        """
        Map Lanelet2 boundary marking to a simple OpenDRIVE roadMark type.

        This is intentionally conservative for the first version. Later it can be
        extended to support multi-record roadMark, color, line width, and roadMark
        placement along s.
        """
        marking = (marking or "unknown").lower()

        if marking in {"dashed", "broken"}:
            return "broken"

        if marking in {"solid", "line_thin"}:
            return "solid"

        if marking in {"solid_solid", "double_solid"}:
            return "solid solid"

        if marking in {"virtual", "none"}:
            return "none"

        return "broken"

    def map_boundary_marking_to_lane_change(self, marking: str) -> str:
        """
        Approximate laneChange from Lanelet2 boundary marking.

        broken/dashed -> both
        solid/double solid -> none
        virtual/none -> none
        """
        marking = (marking or "unknown").lower()

        if marking in {"dashed", "broken"}:
            return "both"

        if marking in {"solid", "solid_solid", "double_solid", "virtual", "none"}:
            return "none"

        return "both"

    def _map_lanelet_type(self, lanelet: WayRelation) -> str:
        subtype = lanelet.tag_dict.get("subtype", "").lower()
        location = lanelet.tag_dict.get("location", "").lower()

        if subtype in {"crosswalk", "walkway", "pedestrian"}:
            return "sidewalk"

        if subtype in {"bicycle_lane", "bike_lane"}:
            return "biking"

        if subtype in {"shoulder", "border"}:
            return "shoulder"

        if subtype in {"bus_lane"}:
            return "driving"

        if location in {"nonurban", "urban"}:
            return self.default_lane_type

        return self.default_lane_type

    def finalize(self, file_path_out: str) -> None:
        if self.root is None:
            raise RuntimeError("OpenDRIVE root is not initialized.")

        self._normalize_opendrive_16_xml()
        tree = etree.ElementTree(self.root)
        etree.indent(tree, space="  ")
        tree.write(
            file_path_out,
            pretty_print=True,
            xml_declaration=True,
            encoding="UTF-8",
        )

    def _normalize_opendrive_16_xml(self) -> None:
        """Normalize generated XML to the OpenDRIVE 1.6.1 schema."""
        if self.root is None:
            return

        lane_child_order = {
            "link": 0,
            "width": 1,
            "border": 1,
            "roadMark": 2,
            "material": 3,
            "speed": 4,
            "access": 5,
            "height": 6,
            "rule": 7,
            "include": 8,
            "userData": 8,
            "dataQuality": 8,
        }
        for side in ("left", "right"):
            for lane in self.root.findall(
                f"./road/lanes/laneSection/{side}/lane"
            ):
                self._sort_xml_children(lane, lane_child_order)

        link_child_order = {
            "predecessor": 0,
            "successor": 1,
            "include": 2,
            "userData": 2,
            "dataQuality": 2,
        }
        for link in self.root.findall(
            "./road/lanes/laneSection/*/lane/link"
        ):
            self._sort_xml_children(link, link_child_order)

        for lane_link in self.root.findall("./junction/connection/laneLink"):
            lane_link.attrib.pop("id", None)

        objects_child_order = {
            "object": 0,
            "objectReference": 1,
            "tunnel": 2,
            "bridge": 3,
            "include": 4,
            "userData": 4,
            "dataQuality": 4,
        }
        for objects in self.root.findall("./road/objects"):
            self._sort_xml_children(objects, objects_child_order)

        object_child_order = {
            "repeat": 0,
            "outlines": 1,
            "material": 2,
            "validity": 3,
            "parkingSpace": 4,
            "markings": 5,
            "borders": 6,
            "surface": 7,
            "include": 8,
            "userData": 8,
            "dataQuality": 8,
        }
        for object_el in self.root.findall("./road/objects/object"):
            self._sort_xml_children(object_el, object_child_order)

        signal_child_order = {
            "validity": 0,
            "dependency": 1,
            "reference": 2,
            "positionRoad": 3,
            "positionInertial": 4,
            "include": 5,
            "userData": 5,
            "dataQuality": 5,
        }
        for signal in self.root.findall("./road/signals/signal"):
            self._sort_xml_children(signal, signal_child_order)

        # Smoothing may change the accumulated planView length.
        for road in self.root.findall("./road"):
            geometry_ends = []
            for geometry in road.findall("./planView/geometry"):
                try:
                    geometry_ends.append(
                        float(geometry.get("s", "0.0"))
                        + float(geometry.get("length", "0.0"))
                    )
                except ValueError:
                    continue
            if geometry_ends:
                road.set("length", f"{max(geometry_ends):.6f}")

    @staticmethod
    def _sort_xml_children(parent: etree._Element, order: Dict[str, int]) -> None:
        children = list(parent)
        sorted_children = sorted(
            enumerate(children),
            key=lambda item: (order.get(item[1].tag, 999), item[0]),
        )
        reordered = [child for _, child in sorted_children]
        if reordered != children:
            parent[:] = reordered

    def print_time(self) -> None:
        LOGGER.info("Lanelet2 to OpenDRIVE conversion took %.2f seconds.", self.conv_time)
        LOGGER.info(
            "Geometry fixes: reversed lanelet boundaries=%d, constant-width fallbacks=%d, width clamps=%d, refline simplified roads=%d, removed refline points=%d, smoothed reflines=%d.",
            getattr(self, "boundary_orientation_fix_count", 0),
            getattr(self, "constant_width_fallback_count", 0),
            getattr(self, "constant_width_clamp_count", 0),
            getattr(self, "refline_simplified_count", 0),
            getattr(self, "refline_removed_point_count", 0),
            getattr(self, "refline_smoothed_count", 0),
        )
        print(f"Lanelet2 to OpenDRIVE conversion took {self.conv_time:.2f} seconds.")
        print(
            "Geometry fixes: "
            f"reversed lanelet boundaries={getattr(self, 'boundary_orientation_fix_count', 0)}, "
            f"constant-width fallbacks={getattr(self, 'constant_width_fallback_count', 0)}, "
            f"width clamps={getattr(self, 'constant_width_clamp_count', 0)}, "
            f"refline simplified roads={getattr(self, 'refline_simplified_count', 0)}, "
            f"removed refline points={getattr(self, 'refline_removed_point_count', 0)}, "
            f"smoothed reflines={getattr(self, 'refline_smoothed_count', 0)}."
        )

    # ---------------------------------------------------------------------
    # Fitting helpers
    # ---------------------------------------------------------------------

    def fit_width_records(
        self,
        samples: Sequence[Tuple[float, float]],
        max_error: float = 0.15,
        min_segment_length: float = 1.0,
        min_points: int = 3,
    ) -> List[Tuple[float, float, float, float, float]]:
        """
        Fit lane width samples into OpenDRIVE width records.

        Returns records:
            (sOffset, a, b, c, d)

        First tries one linear fit. If the fit error is too large, recursively
        splits at the largest-error sample and uses piecewise linear records.
        """
        clean = [(float(s), max(float(w), 0.0)) for s, w in samples]
        if not clean:
            return []
        if len(clean) == 1:
            return [(clean[0][0], clean[0][1], 0.0, 0.0, 0.0)]

        clean.sort(key=lambda x: x[0])
        return self._fit_width_records_recursive(clean, max_error, min_segment_length, min_points)

    def _fit_width_records_recursive(
        self,
        samples: List[Tuple[float, float]],
        max_error: float,
        min_segment_length: float,
        min_points: int,
    ) -> List[Tuple[float, float, float, float, float]]:
        s0 = samples[0][0]
        s1 = samples[-1][0]
        if len(samples) < min_points or (s1 - s0) <= min_segment_length:
            a, b = self.fit_linear_width(samples)
            return [(s0, a, b, 0.0, 0.0)]

        a, b = self.fit_linear_width(samples)
        errors = []
        for s, w in samples:
            pred = a + b * (s - s0)
            errors.append(abs(pred - w))

        max_err = max(errors)
        if max_err <= max_error:
            return [(s0, a, b, 0.0, 0.0)]

        split_idx = max(range(1, len(samples) - 1), key=lambda i: errors[i], default=-1)
        if split_idx <= 0 or split_idx >= len(samples) - 1:
            return [(s0, a, b, 0.0, 0.0)]

        left = samples[: split_idx + 1]
        right = samples[split_idx:]
        return (
            self._fit_width_records_recursive(left, max_error, min_segment_length, min_points)
            + self._fit_width_records_recursive(right, max_error, min_segment_length, min_points)
        )

    @staticmethod
    def fit_linear_width(samples: Sequence[Tuple[float, float]]) -> Tuple[float, float]:
        """Fit w = a + b * (s - s0)."""
        if not samples:
            return 3.5, 0.0
        if len(samples) == 1:
            return max(samples[0][1], 0.0), 0.0

        s0 = samples[0][0]
        xs = [s - s0 for s, _ in samples]
        ys = [w for _, w in samples]
        n = len(xs)
        mean_x = sum(xs) / n
        mean_y = sum(ys) / n
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom <= 1e-12:
            return max(mean_y, 0.0), 0.0
        b = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys)) / denom
        a = mean_y - b * mean_x
        return max(a, 0.0), b

    def fit_straight_line_geometry(self, pts: List[Point2D]) -> Optional[Tuple[float, float, float, float]]:
        """Return one line geometry if all points are close to the chord."""
        if len(pts) < 2:
            return None
        p0, p1 = pts[0], pts[-1]
        chord = self.distance(p0, p1)
        length = self.polyline_length(pts)
        if chord <= 1e-6 or length <= 1e-6:
            return None

        max_dev = self.max_distance_to_line(pts, p0, p1)
        # If the chord is close to the polyline and lateral deviation is small, use one line.
        if max_dev <= self.ref_line_fit_max_error and abs(length - chord) <= max(0.5, 0.02 * length):
            hdg = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
            return p0[0], p0[1], hdg, chord
        return None

    def fit_arc_geometry(
        self,
        pts: List[Point2D],
        max_error: float = 0.20,
    ) -> Optional[Tuple[float, float, float, float, float]]:
        """Fit a circular OpenDRIVE <arc/> geometry if reliable."""
        if len(pts) < 4:
            return None

        circle = self.fit_circle(pts)
        if circle is None:
            return None
        cx, cy, r = circle
        if r <= 1e-6 or r > 100000:
            return None

        radial_errors = [abs(math.hypot(x - cx, y - cy) - r) for x, y in pts]
        if max(radial_errors) > max_error:
            return None

        p0 = pts[0]
        p1 = pts[1]
        v0 = (p1[0] - p0[0], p1[1] - p0[1])
        if math.hypot(v0[0], v0[1]) <= 1e-9:
            return None

        rx = p0[0] - cx
        ry = p0[1] - cy
        t_ccw = (-ry, rx)
        dot_ccw = t_ccw[0] * v0[0] + t_ccw[1] * v0[1]
        sign = 1.0 if dot_ccw >= 0 else -1.0
        tangent = (sign * t_ccw[0], sign * t_ccw[1])
        hdg = math.atan2(tangent[1], tangent[0])

        start_ang = math.atan2(pts[0][1] - cy, pts[0][0] - cx)
        end_ang = math.atan2(pts[-1][1] - cy, pts[-1][0] - cx)
        if sign > 0:
            delta = self.normalize_angle_positive(end_ang - start_ang)
        else:
            delta = self.normalize_angle_positive(start_ang - end_ang)
        arc_length = max(r * delta, self.polyline_length(pts))

        # Avoid using arc for tiny curvature changes; a polyline or straight line is safer.
        if delta < math.radians(2.0):
            return None

        curvature = sign / r
        return pts[0][0], pts[0][1], hdg, arc_length, curvature

    def fit_param_poly3_geometry(
        self,
        pts: List[Point2D],
        max_error: float = 0.20,
    ) -> Optional[Tuple[float, float, float, float, Dict[str, float]]]:
        """
        Fit a local cubic lateral polynomial as OpenDRIVE paramPoly3.

        Uses pRange="arcLength" with:
            u(p) = p
            v(p) = aV + bV*p + cV*p^2 + dV*p^3
        in the local frame of the starting point and starting heading.
        """
        if len(pts) < 5:
            return None

        length = self.polyline_length(pts)
        if length <= 1e-6:
            return None

        p0, p1 = pts[0], pts[1]
        hdg = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        ch = math.cos(hdg)
        sh = math.sin(hdg)

        s_vals = [0.0]
        acc = 0.0
        for a, b in zip(pts[:-1], pts[1:]):
            acc += self.distance(a, b)
            s_vals.append(acc)

        v_vals = []
        for x, y in pts:
            dx = x - p0[0]
            dy = y - p0[1]
            # local lateral coordinate
            v = -sh * dx + ch * dy
            v_vals.append(v)

        coeff = self.polyfit_cubic(s_vals, v_vals)
        if coeff is None:
            return None
        a, b, c, d = coeff

        max_err = 0.0
        for ss, vv in zip(s_vals, v_vals):
            pred = a + b * ss + c * ss * ss + d * ss * ss * ss
            max_err = max(max_err, abs(pred - vv))
        if max_err > max_error:
            return None

        coeffs = {
            "aU": 0.0,
            "bU": 1.0,
            "cU": 0.0,
            "dU": 0.0,
            "aV": a,
            "bV": b,
            "cV": c,
            "dV": d,
        }
        return p0[0], p0[1], hdg, length, coeffs

    @staticmethod
    def max_distance_to_line(pts: Sequence[Point2D], p0: Point2D, p1: Point2D) -> float:
        x0, y0 = p0
        x1, y1 = p1
        vx = x1 - x0
        vy = y1 - y0
        norm = math.hypot(vx, vy)
        if norm <= 1e-12:
            return 0.0
        return max(abs(vx * (y - y0) - vy * (x - x0)) / norm for x, y in pts)

    @staticmethod
    def normalize_angle_positive(angle: float) -> float:
        two_pi = 2.0 * math.pi
        while angle < 0.0:
            angle += two_pi
        while angle >= two_pi:
            angle -= two_pi
        return angle

    @staticmethod
    def fit_circle(pts: Sequence[Point2D]) -> Optional[Tuple[float, float, float]]:
        """Least-squares circle fit using normal equations."""
        if len(pts) < 3:
            return None

        # Solve x^2 + y^2 + D*x + E*y + F = 0
        ata = [[0.0] * 3 for _ in range(3)]
        atb = [0.0] * 3
        for x, y in pts:
            row = [x, y, 1.0]
            b = -(x * x + y * y)
            for i in range(3):
                atb[i] += row[i] * b
                for j in range(3):
                    ata[i][j] += row[i] * row[j]

        sol = Lanelet2OpendriveConverter.solve_3x3(ata, atb)
        if sol is None:
            return None
        d, e, f = sol
        cx = -d / 2.0
        cy = -e / 2.0
        r2 = cx * cx + cy * cy - f
        if r2 <= 0.0:
            return None
        return cx, cy, math.sqrt(r2)

    @staticmethod
    def polyfit_cubic(xs: Sequence[float], ys: Sequence[float]) -> Optional[Tuple[float, float, float, float]]:
        """Least-squares cubic polynomial fit y=a+b*x+c*x^2+d*x^3."""
        if len(xs) != len(ys) or len(xs) < 4:
            return None
        ata = [[0.0] * 4 for _ in range(4)]
        atb = [0.0] * 4
        for x, y in zip(xs, ys):
            row = [1.0, x, x * x, x * x * x]
            for i in range(4):
                atb[i] += row[i] * y
                for j in range(4):
                    ata[i][j] += row[i] * row[j]
        sol = Lanelet2OpendriveConverter.solve_linear_system(ata, atb)
        if sol is None:
            return None
        return sol[0], sol[1], sol[2], sol[3]

    @staticmethod
    def solve_3x3(a: List[List[float]], b: List[float]) -> Optional[Tuple[float, float, float]]:
        sol = Lanelet2OpendriveConverter.solve_linear_system(a, b)
        if sol is None or len(sol) != 3:
            return None
        return sol[0], sol[1], sol[2]

    @staticmethod
    def solve_linear_system(a: List[List[float]], b: List[float]) -> Optional[List[float]]:
        """Small Gaussian elimination solver with partial pivoting."""
        n = len(b)
        if n == 0:
            return []
        mat = [list(row) + [float(rhs)] for row, rhs in zip(a, b)]
        for col in range(n):
            pivot = max(range(col, n), key=lambda r: abs(mat[r][col]))
            if abs(mat[pivot][col]) <= 1e-12:
                return None
            if pivot != col:
                mat[col], mat[pivot] = mat[pivot], mat[col]
            div = mat[col][col]
            for j in range(col, n + 1):
                mat[col][j] /= div
            for r in range(n):
                if r == col:
                    continue
                factor = mat[r][col]
                if abs(factor) <= 1e-18:
                    continue
                for j in range(col, n + 1):
                    mat[r][j] -= factor * mat[col][j]
        return [mat[i][n] for i in range(n)]

    # ---------------------------------------------------------------------
    # Geometry utilities
    # ---------------------------------------------------------------------

    def get_node_xy(self, node_id: str) -> Optional[Point2D]:
        node = self.nodes.get(str(node_id))
        if node is None:
            return None

        if node.local_x is not None and node.local_y is not None:
            return float(node.local_x), float(node.local_y)

        try:
            lat = float(node.lat)
            lon = float(node.lon)
        except Exception:
            return None

        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = lat
            self.origin_lon = lon

        return self.latlon_to_xy(lat, lon)

    def latlon_to_xy(self, lat: float, lon: float) -> Point2D:
        """
        Simple local equirectangular projection.

        This is enough for first-version conversion/debugging.
        Later it can be replaced by pyproj using geoReference.
        """
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = lat
            self.origin_lon = lon

        r = 6371000.0
        lat0 = math.radians(self.origin_lat)
        x = r * math.radians(lon - self.origin_lon) * math.cos(lat0)
        y = r * math.radians(lat - self.origin_lat)
        return x, y

    def get_way_points(self, way_id: str) -> List[Point2D]:
        way = self.ways.get(str(way_id))
        if way is None:
            return []

        pts: List[Point2D] = []
        for node_id in way.nodes:
            p = self.get_node_xy(str(node_id))
            if p is not None:
                pts.append(p)

        return self.clean_polyline(pts)

    def prepare_refline_points(self, pts: Sequence[Point2D]) -> List[Point2D]:
        """Prepare a road reference line before writing OpenDRIVE.

        v27 changes the v26 behavior from "simplify every ref_line" to a
        conditional strategy:

        1. Smooth curve-like polylines are preserved almost as-is. This keeps
           real curved road sections visually natural instead of turning them
           into coarse straight chords.
        2. Straight or zigzag-like polylines still go through the v26 short-segment
           cleanup, RDP simplification, and mild smoothing. This removes the
           sub-meter kink pattern observed on roads such as 9 and 70, where a
           large right-side cumulative lane offset can amplify tiny ref_line
           direction jumps into distorted offset boundaries in TESS NG.

        This function is geometry-only: it preserves endpoints and never changes
        road topology, lane ids, junction roles, or lane widths.
        """
        cleaned = self.clean_polyline(pts)
        if not self.refline_simplify_enabled or len(cleaned) < 3:
            return cleaned

        original_count = len(cleaned)
        original_length = self.polyline_length(cleaned)

        # If the polyline looks like a real smooth curve, keep its original node
        # sampling. Do not apply RDP or Laplacian smoothing, because both can make
        # curved road sections look like coarse broken lines after import.
        if self.is_smooth_curve_refline(cleaned):
            return cleaned

        simplified = self.remove_short_refline_vertices(
            cleaned,
            min_segment_length=self.refline_min_segment_length,
            turn_threshold_deg=self.refline_short_turn_threshold_deg,
        )
        simplified = self.rdp_simplify_polyline(simplified, tolerance=self.refline_rdp_tolerance)
        simplified = self.clean_polyline(simplified)

        # Do not accept an over-aggressive simplification that collapses the road
        # or changes total length too much. In that case, keep the cleaned input.
        if len(simplified) < 2:
            simplified = cleaned
        simplified_length = self.polyline_length(simplified)
        if original_length > 1e-6 and abs(simplified_length - original_length) > max(2.0, 0.20 * original_length):
            simplified = cleaned

        smoothed = self.smooth_refline_polyline(
            simplified,
            iterations=self.refline_smooth_iterations,
            alpha=self.refline_smooth_alpha,
        )
        smoothed = self.clean_polyline(smoothed)
        if len(smoothed) < 2:
            smoothed = simplified

        if len(smoothed) != original_count:
            self.refline_simplified_count += 1
            self.refline_removed_point_count += max(0, original_count - len(smoothed))
        elif len(smoothed) >= 3 and any(self.distance(a, b) > 1e-6 for a, b in zip(smoothed, simplified)):
            self.refline_smoothed_count += 1

        return smoothed

    def is_smooth_curve_refline(self, pts: Sequence[Point2D]) -> bool:
        """Return True when a ref_line looks like a real smooth curve.

        The goal is to avoid simplifying genuine curves. A genuine curve should
        have mostly same-signed local turns, no local short-segment kink, and no
        large heading jump at a tiny segment. Roads 9/70-like zigzags should fail
        this test and still be simplified.
        """
        pts = self.clean_polyline(pts)
        if len(pts) < 5:
            return False

        seg_lengths = [self.distance(a, b) for a, b in zip(pts[:-1], pts[1:])]
        valid_lengths = [l for l in seg_lengths if l > 1e-6]
        if len(valid_lengths) < 4:
            return False

        turns: List[float] = []
        short_kink_count = 0
        for i in range(1, len(pts) - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            v1 = (b[0] - a[0], b[1] - a[1])
            v2 = (c[0] - b[0], c[1] - b[1])
            n1 = math.hypot(v1[0], v1[1])
            n2 = math.hypot(v2[0], v2[1])
            if n1 <= 1e-9 or n2 <= 1e-9:
                continue
            cross = v1[0] * v2[1] - v1[1] * v2[0]
            dot = v1[0] * v2[0] + v1[1] * v2[1]
            turn = math.atan2(cross, dot)
            turns.append(turn)

            # A real curve can have many small turns, but should not contain a
            # sub-meter segment coupled with a large local direction jump.
            if min(n1, n2) < self.refline_min_segment_length and abs(turn) > math.radians(12.0):
                short_kink_count += 1

        if not turns or short_kink_count > 0:
            return False

        abs_turns = [abs(t) for t in turns]
        total_abs_turn = sum(abs_turns)
        max_abs_turn = max(abs_turns)

        # Tiny total curvature is essentially straight; simplify it.
        if total_abs_turn < math.radians(12.0):
            return False

        # Very sharp local turns are not a smooth curve; simplify/clean them.
        if max_abs_turn > math.radians(25.0):
            return False

        signed = [1 if t > math.radians(1.0) else -1 if t < -math.radians(1.0) else 0 for t in turns]
        pos = sum(1 for s in signed if s > 0)
        neg = sum(1 for s in signed if s < 0)
        nonzero = pos + neg
        if nonzero == 0:
            return False

        # Most turns should bend in the same direction. Alternating signs indicate
        # zigzag/noise rather than a smooth curve.
        same_sign_ratio = max(pos, neg) / nonzero
        if same_sign_ratio < 0.75:
            return False

        # A long enough polyline with consistently signed moderate curvature is
        # treated as a smooth curve and left at normal/original sampling density.
        return True

    def remove_short_refline_vertices(
        self,
        pts: Sequence[Point2D],
        min_segment_length: float,
        turn_threshold_deg: float,
        max_passes: int = 8,
    ) -> List[Point2D]:
        """Remove local short-segment vertices while preserving endpoints.

        A vertex is removed when it participates in very short adjacent segments
        or when a short segment forms a local direction jump. This is more
        conservative than global downsampling: it keeps long geometry and removes
        only the point-level jitter that is dangerous for offset boundaries.
        """
        result = self.clean_polyline(pts)
        if len(result) < 3:
            return result

        min_segment_length = max(float(min_segment_length), 0.05)
        turn_threshold = math.radians(max(float(turn_threshold_deg), 0.0))

        for _ in range(max_passes):
            if len(result) < 3:
                break
            changed = False
            new_pts: List[Point2D] = [result[0]]

            for i in range(1, len(result) - 1):
                prev_pt = new_pts[-1]
                cur_pt = result[i]
                next_pt = result[i + 1]

                len_prev = self.distance(prev_pt, cur_pt)
                len_next = self.distance(cur_pt, next_pt)
                len_bridge = self.distance(prev_pt, next_pt)
                turn = self.turn_angle_abs(prev_pt, cur_pt, next_pt)

                # Remove isolated tiny points and points that create a short local
                # kink. Keep points when the bridge would collapse to near zero.
                remove_tiny = len_prev < 0.50 * min_segment_length or len_next < 0.50 * min_segment_length
                remove_short_turn = (
                    min(len_prev, len_next) < min_segment_length
                    and turn > turn_threshold
                    and len_bridge > 1e-6
                )
                remove_redundant_collinear = (
                    min(len_prev, len_next) < min_segment_length
                    and turn < math.radians(3.0)
                    and len_bridge > max(len_prev, len_next)
                )

                if remove_tiny or remove_short_turn or remove_redundant_collinear:
                    changed = True
                    continue
                new_pts.append(cur_pt)

            new_pts.append(result[-1])
            result = self.clean_polyline(new_pts)
            if not changed:
                break

        return result

    def rdp_simplify_polyline(self, pts: Sequence[Point2D], tolerance: float) -> List[Point2D]:
        """Douglas-Peucker simplification preserving endpoints."""
        pts = self.clean_polyline(pts)
        if len(pts) < 3 or tolerance <= 0.0:
            return pts

        def perpendicular_distance(p: Point2D, a: Point2D, b: Point2D) -> float:
            ax, ay = a
            bx, by = b
            px, py = p
            vx = bx - ax
            vy = by - ay
            denom = vx * vx + vy * vy
            if denom <= 1e-12:
                return self.distance(p, a)
            u = ((px - ax) * vx + (py - ay) * vy) / denom
            u = max(0.0, min(1.0, u))
            q = (ax + u * vx, ay + u * vy)
            return self.distance(p, q)

        def recurse(seq: List[Point2D]) -> List[Point2D]:
            if len(seq) < 3:
                return seq
            a, b = seq[0], seq[-1]
            max_idx = -1
            max_dist = -1.0
            for idx in range(1, len(seq) - 1):
                dist = perpendicular_distance(seq[idx], a, b)
                if dist > max_dist:
                    max_dist = dist
                    max_idx = idx
            if max_dist <= tolerance or max_idx <= 0:
                return [a, b]
            left = recurse(seq[: max_idx + 1])
            right = recurse(seq[max_idx:])
            return left[:-1] + right

        return self.clean_polyline(recurse(list(pts)))

    def smooth_refline_polyline(
        self,
        pts: Sequence[Point2D],
        iterations: int = 2,
        alpha: float = 0.25,
    ) -> List[Point2D]:
        """Laplacian-like smoothing with fixed endpoints.

        The smoothing is intentionally mild. It only moves interior points toward
        the midpoint of their neighbors, which reduces abrupt heading jumps while
        preserving the road's start/end locations for topology continuity.
        """
        result = self.clean_polyline(pts)
        if len(result) < 4 or iterations <= 0 or alpha <= 0.0:
            return result

        alpha = max(0.0, min(float(alpha), 0.5))
        for _ in range(iterations):
            new_pts = [result[0]]
            for i in range(1, len(result) - 1):
                prev_pt = result[i - 1]
                cur_pt = result[i]
                next_pt = result[i + 1]
                target = ((prev_pt[0] + next_pt[0]) * 0.5, (prev_pt[1] + next_pt[1]) * 0.5)
                new_pts.append(
                    (
                        cur_pt[0] * (1.0 - alpha) + target[0] * alpha,
                        cur_pt[1] * (1.0 - alpha) + target[1] * alpha,
                    )
                )
            new_pts.append(result[-1])
            result = self.clean_polyline(new_pts)
        return result

    @staticmethod
    def turn_angle_abs(a: Point2D, b: Point2D, c: Point2D) -> float:
        """Absolute heading change at b in radians."""
        v1 = (b[0] - a[0], b[1] - a[1])
        v2 = (c[0] - b[0], c[1] - b[1])
        n1 = math.hypot(v1[0], v1[1])
        n2 = math.hypot(v2[0], v2[1])
        if n1 <= 1e-12 or n2 <= 1e-12:
            return 0.0
        dot = max(-1.0, min(1.0, (v1[0] * v2[0] + v1[1] * v2[1]) / (n1 * n2)))
        return abs(math.acos(dot))

    def get_left_points(self, lanelet: WayRelation) -> List[Point2D]:
        left_pts, _ = self.get_oriented_lanelet_boundaries(lanelet)
        return list(left_pts)

    def get_right_points(self, lanelet: WayRelation) -> List[Point2D]:
        _, right_pts = self.get_oriented_lanelet_boundaries(lanelet)
        return list(right_pts)

    def get_raw_lanelet_boundaries(self, lanelet: WayRelation) -> Tuple[List[Point2D], List[Point2D]]:
        """Return raw left/right boundary points, with right aligned to left direction.

        This helper intentionally does not call get_left_points/get_right_points,
        because those functions use the orientation-fix cache. Keeping this raw
        helper non-recursive prevents orientation detection from calling itself.
        """
        left_pts = self.get_way_points(str(lanelet.left_way))
        right_pts = self.get_way_points(str(lanelet.right_way))

        if left_pts and right_pts:
            # Align right boundary direction with left boundary direction.
            if self.distance(left_pts[0], right_pts[-1]) < self.distance(left_pts[0], right_pts[0]):
                right_pts = list(reversed(right_pts))

        return self.clean_polyline(left_pts), self.clean_polyline(right_pts)

    def get_oriented_lanelet_boundaries(self, lanelet: WayRelation) -> Tuple[List[Point2D], List[Point2D]]:
        """Return lanelet boundaries with occasional full boundary reversal fixed.

        Lanelet2 maps can contain a lanelet whose left/right boundary point order
        is opposite to the local driving direction, while predecessor and successor
        lanelets are ordered normally. If such a lanelet is used directly as
        OpenDRIVE geometry, ref_line/width sampling can flip locally.

        This function compares endpoint continuity against predecessor and
        successor centerlines. If the reversed current centerline connects more
        naturally to both neighbors than the original order, both left and right
        boundaries are reversed. Only conversion-time geometry is changed; original
        Lanelet2 topology is not modified.
        """
        lanelet_id = str(lanelet.id_)
        cached = self._oriented_lanelet_boundary_cache.get(lanelet_id)
        if cached is not None:
            return cached

        left_pts, right_pts = self.get_raw_lanelet_boundaries(lanelet)
        if len(left_pts) < 2 or len(right_pts) < 2:
            self._oriented_lanelet_boundary_cache[lanelet_id] = (left_pts, right_pts)
            return left_pts, right_pts

        center = self.centerline_from_boundaries(left_pts, right_pts, n_samples=5)
        if len(center) < 2:
            self._oriented_lanelet_boundary_cache[lanelet_id] = (left_pts, right_pts)
            return left_pts, right_pts

        normal_score = 0.0
        reversed_score = 0.0
        evidence = 0

        for pred_id in lanelet.predecessors:
            pred = self.lanelets.get(str(pred_id))
            if pred is None:
                continue
            pred_center = self.raw_lanelet_centerline(pred, n_samples=5)
            if len(pred_center) < 2:
                continue
            # predecessor end should connect to current start
            normal_score += self.distance(pred_center[-1], center[0])
            reversed_score += self.distance(pred_center[-1], center[-1])
            evidence += 1

        for succ_id in lanelet.successors:
            succ = self.lanelets.get(str(succ_id))
            if succ is None:
                continue
            succ_center = self.raw_lanelet_centerline(succ, n_samples=5)
            if len(succ_center) < 2:
                continue
            # current end should connect to successor start
            normal_score += self.distance(center[-1], succ_center[0])
            reversed_score += self.distance(center[0], succ_center[0])
            evidence += 1

        # Use a margin to avoid flipping ambiguous short lanelets. The target case
        # is a clear direction突变: reversed endpoints are much closer to neighbors.
        if evidence > 0 and reversed_score + 0.50 < normal_score:
            left_pts = list(reversed(left_pts))
            right_pts = list(reversed(right_pts))
            self.boundary_orientation_fix_count += 1

        self._oriented_lanelet_boundary_cache[lanelet_id] = (left_pts, right_pts)
        return left_pts, right_pts

    def raw_lanelet_centerline(self, lanelet: WayRelation, n_samples: int = 5) -> List[Point2D]:
        left_pts, right_pts = self.get_raw_lanelet_boundaries(lanelet)
        return self.centerline_from_boundaries(left_pts, right_pts, n_samples=n_samples)

    def centerline_from_boundaries(
        self,
        left_pts: Sequence[Point2D],
        right_pts: Sequence[Point2D],
        n_samples: int = 20,
    ) -> List[Point2D]:
        if len(left_pts) < 2 or len(right_pts) < 2:
            return []

        left_len = self.polyline_length(left_pts)
        right_len = self.polyline_length(right_pts)
        if left_len <= 1e-6 or right_len <= 1e-6:
            return []

        center: List[Point2D] = []
        for i in range(max(n_samples, 2)):
            ratio = i / max(n_samples - 1, 1)
            pl = self.point_at_s(left_pts, ratio * left_len)
            pr = self.point_at_s(right_pts, ratio * right_len)
            center.append(((pl[0] + pr[0]) * 0.5, (pl[1] + pr[1]) * 0.5))

        return self.clean_polyline(center)

    def get_center_points(self, lanelet: WayRelation, n_samples: int = 20) -> List[Point2D]:
        left_pts, right_pts = self.get_oriented_lanelet_boundaries(lanelet)
        return self.centerline_from_boundaries(left_pts, right_pts, n_samples=n_samples)

    def compute_lane_width_samples(
        self,
        lanelet: WayRelation,
        ref_points: List[Point2D],
    ) -> List[Tuple[float, float]]:
        """Return width samples used only to derive a robust constant lane width.

        In v25 the final OpenDRIVE <lane><width> is always written as one constant
        record. We still sample the Lanelet2 lanelet width here so the constant can
        use the lanelet's median width rather than a fixed 3.5 m everywhere.
        """
        left_pts, right_pts = self.get_oriented_lanelet_boundaries(lanelet)

        if len(left_pts) < 2 or len(right_pts) < 2:
            return []

        ref_len = self.polyline_length(ref_points)
        left_len = self.polyline_length(left_pts)
        right_len = self.polyline_length(right_pts)

        if ref_len <= 1e-6 or left_len <= 1e-6 or right_len <= 1e-6:
            return []

        n_steps = max(int(ref_len / max(self.step_size, 0.5)), 1)

        samples: List[Tuple[float, float]] = []
        for i in range(n_steps + 1):
            ratio = i / n_steps
            s_ref = ratio * ref_len

            # Because the output width is constant, using median robustly absorbs
            # local non-parallel boundary artifacts. Extreme outliers are later
            # clamped by constant_lane_width_from_samples().
            pl = self.point_at_s(left_pts, ratio * left_len)
            pr = self.point_at_s(right_pts, ratio * right_len)
            width = self.distance(pl, pr)

            if math.isfinite(width) and width > 1e-6:
                samples.append((s_ref, width))

        return samples

    def default_width_bounds_for_lanelet(self, lanelet: WayRelation) -> Tuple[float, float, float]:
        """Return (default, min, max) width by lane type.

        The bounds are intentionally conservative for simulator import stability.
        They prevent non-parallel Lanelet2 boundaries from creating very wide or
        near-zero OpenDRIVE lanes that TESS NG may render as twisted meshes.
        """
        subtype = lanelet.tag_dict.get("subtype", "").lower()
        if subtype == "border":
            # The current per-lane road model writes this as shoulder so that
            # downstream parsers retain the isolated lane, but its original
            # narrow border geometry must not be clamped to shoulder widths.
            return 0.5, 0.1, 2.0
        lane_type = self._map_lanelet_type(lanelet)
        if lane_type == "sidewalk":
            return 2.0, 1.0, 4.0
        if lane_type == "biking":
            return 1.8, 1.0, 3.0
        if lane_type == "shoulder":
            return 2.5, 1.5, 4.0
        return 3.5, 2.8, 4.2

    def constant_lane_width_from_samples(
        self,
        samples: Sequence[Tuple[float, float]],
        lanelet: WayRelation,
    ) -> float:
        """Use median sampled width, then clamp to a reasonable constant value."""
        default_width, min_width, max_width = self.default_width_bounds_for_lanelet(lanelet)
        widths = sorted(
            float(width)
            for _, width in samples
            if math.isfinite(float(width)) and float(width) > 1e-6
        )
        if not widths:
            self.constant_width_fallback_count += 1
            return default_width

        n = len(widths)
        if n % 2 == 1:
            median_width = widths[n // 2]
        else:
            median_width = 0.5 * (widths[n // 2 - 1] + widths[n // 2])

        clamped = max(min_width, min(max_width, median_width))
        if abs(clamped - median_width) > 1e-6:
            self.constant_width_clamp_count += 1
        return clamped

    @staticmethod
    def write_constant_lane_width(lane_el: etree._Element, width: float) -> None:
        etree.SubElement(
            lane_el,
            "width",
            sOffset="0.0",
            a=f"{max(float(width), 0.0):.6f}",
            b="0.0",
            c="0.0",
            d="0.0",
        )

    def is_same_direction(self, lanelet_a: WayRelation, lanelet_b: WayRelation) -> bool:
        ca = self.get_center_points(lanelet_a, n_samples=5)
        cb = self.get_center_points(lanelet_b, n_samples=5)

        if len(ca) < 2 or len(cb) < 2:
            return False

        va = (ca[-1][0] - ca[0][0], ca[-1][1] - ca[0][1])
        vb = (cb[-1][0] - cb[0][0], cb[-1][1] - cb[0][1])

        na = math.hypot(va[0], va[1])
        nb = math.hypot(vb[0], vb[1])

        if na <= 1e-6 or nb <= 1e-6:
            return False

        cos_val = (va[0] * vb[0] + va[1] * vb[1]) / (na * nb)
        return cos_val > 0.0

    @staticmethod
    def clean_polyline(pts: Sequence[Point2D], eps: float = 1e-6) -> List[Point2D]:
        result: List[Point2D] = []
        for p in pts:
            if not result:
                result.append((float(p[0]), float(p[1])))
            else:
                if math.hypot(result[-1][0] - p[0], result[-1][1] - p[1]) > eps:
                    result.append((float(p[0]), float(p[1])))
        return result

    @staticmethod
    def distance(p1: Point2D, p2: Point2D) -> float:
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

    def polyline_length(self, pts: Sequence[Point2D]) -> float:
        if len(pts) < 2:
            return 0.0
        return sum(self.distance(pts[i], pts[i + 1]) for i in range(len(pts) - 1))

    def point_at_s(self, pts: Sequence[Point2D], s: float) -> Point2D:
        if not pts:
            return 0.0, 0.0

        if s <= 0:
            return pts[0]

        total = self.polyline_length(pts)
        if s >= total:
            return pts[-1]

        acc = 0.0
        for p0, p1 in zip(pts[:-1], pts[1:]):
            seg_len = self.distance(p0, p1)

            if seg_len <= 1e-9:
                continue

            if acc + seg_len >= s:
                ratio = (s - acc) / seg_len
                return (
                    p0[0] + (p1[0] - p0[0]) * ratio,
                    p0[1] + (p1[1] - p0[1]) * ratio,
                )

            acc += seg_len

        return pts[-1]

    def polyline_midpoint(self, pts: Sequence[Point2D]) -> Point2D:
        return self.point_at_s(pts, self.polyline_length(pts) * 0.5)

    @staticmethod
    def centroid(pts: Sequence[Point2D]) -> Point2D:
        if not pts:
            return 0.0, 0.0
        x = sum(p[0] for p in pts) / len(pts)
        y = sum(p[1] for p in pts) / len(pts)
        return x, y

    def project_point_to_polyline(self, point: Point2D, pts: Sequence[Point2D]) -> Tuple[float, float]:
        """
        Project a point to a polyline.

        Return:
            s: longitudinal position along polyline
            t: signed lateral offset. Positive means left of segment direction.
        """
        if len(pts) < 2:
            return 0.0, 0.0

        best_dist = float("inf")
        best_s = 0.0
        best_t = 0.0
        acc = 0.0

        px, py = point

        for p0, p1 in zip(pts[:-1], pts[1:]):
            x0, y0 = p0
            x1, y1 = p1
            vx = x1 - x0
            vy = y1 - y0
            seg_len2 = vx * vx + vy * vy

            if seg_len2 <= 1e-12:
                continue

            u = ((px - x0) * vx + (py - y0) * vy) / seg_len2
            u_clamped = max(0.0, min(1.0, u))

            qx = x0 + u_clamped * vx
            qy = y0 + u_clamped * vy

            dx = px - qx
            dy = py - qy
            dist = math.hypot(dx, dy)

            seg_len = math.sqrt(seg_len2)
            cross = vx * (py - y0) - vy * (px - x0)
            signed_t = cross / seg_len

            if dist < best_dist:
                best_dist = dist
                best_s = acc + u_clamped * seg_len
                best_t = signed_t

            acc += seg_len

        return best_s, best_t

    def find_nearest_road_st(self, point: Point2D) -> Tuple[Optional[int], float, float]:
        best_road_id = None
        best_s = 0.0
        best_t = 0.0
        best_abs_t = float("inf")

        for road_id, group in self.road_groups.items():
            if (
                group.is_removed
                or group.road_xml is None
                or group.road_xml.getparent() is not self.root
                or len(group.ref_points) < 2
            ):
                continue
            s, t = self.project_point_to_polyline(point, group.ref_points)
            if abs(t) < best_abs_t:
                best_abs_t = abs(t)
                best_road_id = road_id
                best_s = s
                best_t = t

        return best_road_id, best_s, best_t

    @staticmethod
    def heading_at_s(pts: Sequence[Point2D], s: float) -> float:
        """Return the heading of the non-degenerate polyline segment at s."""
        if len(pts) < 2:
            return 0.0
        remaining = max(0.0, float(s))
        last_heading = 0.0
        for p0, p1 in zip(pts[:-1], pts[1:]):
            dx = p1[0] - p0[0]
            dy = p1[1] - p0[1]
            segment_length = math.hypot(dx, dy)
            if segment_length <= 1e-9:
                continue
            last_heading = math.atan2(dy, dx)
            if remaining <= segment_length:
                return last_heading
            remaining -= segment_length
        return last_heading
