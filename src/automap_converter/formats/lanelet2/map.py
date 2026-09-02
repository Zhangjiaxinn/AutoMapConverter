from typing import Dict, List, Optional, Any

from lxml import etree  # type: ignore

from automap_converter.formats.lanelet2.utils import create_mgrs_code


class Node:
    """
    OSM Node.
    """

    def __init__(
        self,
        id_,
        lat,
        lon,
        ele: float = 0.0,
        autoware: bool = False,
        local_x: Optional[float] = None,
        local_y: Optional[float] = None,
    ):
        """
        Initialization of Node

        :param lat: Latitude geo position information
        :param lon: Longitude geo position information
        :param ele: Elevation (height information)
        :param autoware: Boolean indicating whether the map is autoware-compatible.
        :param local_x: local x-position instead of latitude/longitude (lon/lat values have no meaning)
        :param local_y: local y-position instead of latitude/longitude (lon/lat values have no meaning)
        """
        self.id_: str = str(id_)
        self.lat: str = str(lat)
        self.lon: str = str(lon)
        self.ele: str = str(ele)
        self.autoware: bool = autoware
        self.local_x: Optional[float] = local_x
        self.local_y: Optional[float] = local_y
        self.mgrs_code: Optional[str] = None

        if self.autoware:
            self.mgrs_code = create_mgrs_code(lat, lon)

    def serialize_to_xml(self) -> etree:
        """
        Serializes the Node object to the xml format
        """
        node = etree.Element("node")
        node.set("id", self.id_)
        node.set("action", "modify")
        node.set("visible", "true")
        node.set("version", "1")
        node.set("lat", self.lat)
        node.set("lon", self.lon)
        if self.local_x is not None and self.local_y is not None:
            local_x = etree.SubElement(node, "tag")
            local_x.set("k", "local_x")
            local_x.set("v", str(self.local_x))
            local_y = etree.SubElement(node, "tag")
            local_y.set("k", "local_y")
            local_y.set("v", str(self.local_y))
            node.append(local_x)
            node.append(local_y)
        if (self.ele != "0.0" and self.ele != "0") or self.autoware:
            ele = etree.SubElement(node, "tag")
            ele.set("k", "ele")
            ele.set("v", self.ele)
            node.append(ele)
        if self.mgrs_code:
            mgrs_code = etree.SubElement(node, "tag")
            mgrs_code.set("k", "mgrs_code")
            mgrs_code.set("v", self.mgrs_code)
            node.append(mgrs_code)

        return node


class Way:
    """
    OSM Way
    """

    def __init__(self, id_, nodes: list, tag_dict: Optional[Dict[str, str]] = None):
        """
        Initialization of Way

        :param id_: ID of the way
        :param nodes: list of the way node IDs
        :param tag_dict: tag dictionary of the way
        """
        self.id_ = str(id_)
        self.nodes = [str(node) for node in nodes]
        self.tag_dict = tag_dict if tag_dict is not None else {}

    def serialize_to_xml(self):
        """
        Serializes the Way object to the xml format
        """
        way = etree.Element("way")
        way.set("id", self.id_)
        way.set("action", "modify")
        way.set("visible", "true")
        way.set("version", "1")
        for node in self.nodes:
            xml_node = etree.SubElement(way, "nd")
            xml_node.set("ref", node)
        for tag_key, tag_value in self.tag_dict.items():
            xml_node = etree.SubElement(way, "tag")
            xml_node.set("k", tag_key)
            xml_node.set("v", tag_value)

        return way


class WayRelation:
    """
    Relation for a lanelet with a left and a right way
    """

    def __init__(
        self,
        id_,
        left_way,
        right_way,
        tag_dict: Optional[Dict[str, str]] = None,
        regulatory_elements: Optional[List[str]] = None,
    ):
        """
        Initialization of WayRelation

        :param id_: ID of the WayRelation
        :param left_way: ID of the left_way of WayRelation
        :param right_way: ID of the right_way of WayRelation
        :param tag_dict: tag dictionary of WayRelation
        :param regulatory_elements: regulatory elements of WayRelation
        """
        #  add a spot for traffic signs especially speed limit
        self.id_ = str(id_)
        self.left_way = str(left_way)
        self.right_way = str(right_way)
        self.tag_dict = tag_dict if tag_dict is not None else {}
        self.regulatory_elements = regulatory_elements if regulatory_elements is not None else []

        # 显式拓扑字段
        self.predecessors: List[str] = []
        self.successors: List[str] = []
        # routing graph 可变道关系
        self.left_neighbors: List[str] = []
        self.right_neighbors: List[str] = []
        # routing graph 相邻但不一定可变道
        self.adjacent_left: List[str] = []
        self.adjacent_right: List[str] = []
        # 拓扑来源，便于调试
        self.topology_source: Dict[str, str] = {
            "predecessors": "",
            "successors": "",
            "left_neighbors": "",
            "right_neighbors": "",
            "adjacent_left": "",
            "adjacent_right": "",
        }

    def serialize_to_xml(self) -> etree.Element:
        """
        Serializes the WayRelation object to the xml format
        """
        rel = etree.Element("relation")
        rel.set("id", self.id_)
        rel.set("action", "modify")
        rel.set("visible", "true")
        rel.set("version", "1")
        right_way = etree.SubElement(rel, "member")
        right_way.set("type", "way")
        right_way.set("ref", self.right_way)
        right_way.set("role", "right")
        left_way = etree.SubElement(rel, "member")
        left_way.set("type", "way")
        left_way.set("ref", self.left_way)
        left_way.set("role", "left")
        for regulatory_element in self.regulatory_elements:
            xml_node = etree.SubElement(rel, "member")
            xml_node.set("type", "relation")
            xml_node.set("ref", str(regulatory_element))
            xml_node.set("role", "regulatory_element")
        for tag_key, tag_value in self.tag_dict.items():
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", tag_key)
            xml_node.set("v", tag_value)

        if self.predecessors:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:predecessors")
            xml_node.set("v", ",".join(self.predecessors))

        if self.successors:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:successors")
            xml_node.set("v", ",".join(self.successors))

        if self.left_neighbors:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:left_neighbors")
            xml_node.set("v", ",".join(self.left_neighbors))

        if self.right_neighbors:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:right_neighbors")
            xml_node.set("v", ",".join(self.right_neighbors))

        if self.adjacent_left:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:adjacent_left")
            xml_node.set("v", ",".join(self.adjacent_left))

        if self.adjacent_right:
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", "lanelet2:adjacent_right")
            xml_node.set("v", ",".join(self.adjacent_right))

        return rel


class Multipolygon:
    """
    OSM Multipolygon
    """

    def __init__(
        self,
        id_,
        outer_list: List[str],
        tag_dict: Optional[Dict[str, str]],
        inner_list: Optional[List[str]] = None,
    ):
        """
        Initialization of a Multipolygon

        :param id_: ID of the Multipolygon
        :param outer_list: List of ways that make the Multipolygon
        :param inner_list: List of ways that form holes in the Multipolygon
        :param tag_dict: tag dictionary of the Multipolygon
        """
        self.id_ = str(id_)
        self.outer_list = outer_list
        self.inner_list = inner_list if inner_list is not None else []
        self.tag_dict = tag_dict if tag_dict is not None else {}

    def serialize_to_xml(self) -> etree.Element:
        """
        Serializes the Multipolygon object to the xml format
        """
        multipolygon = etree.Element("relation")
        multipolygon.set("id", self.id_)
        multipolygon.set("action", "modify")
        multipolygon.set("visible", "true")
        for outer in self.outer_list:
            way = etree.SubElement(multipolygon, "member")
            way.set("type", "way")
            way.set("ref", outer)
            way.set("role", "outer")
        for inner in self.inner_list:
            way = etree.SubElement(multipolygon, "member")
            way.set("type", "way")
            way.set("ref", inner)
            way.set("role", "inner")
        for tag_key, tag_value in self.tag_dict.items():
            xml_node = etree.SubElement(multipolygon, "tag")
            xml_node.set("k", tag_key)
            xml_node.set("v", tag_value)
        if "type" not in self.tag_dict:
            xml_node = etree.SubElement(multipolygon, "tag")
            xml_node.set("k", "type")
            xml_node.set("v", "multipolygon")

        return multipolygon


# Class that as a subtype contains TrafficLight, TrafficSign and SpeedLimit
class RegulatoryElement:
    """Relation for a regulatory element (traffic light, traffic sign, speed limit)"""

    def __init__(
        self,
        id_,
        refers: Optional[list] = None,
        yield_ways: Optional[list] = None,
        right_of_ways: Optional[list] = None,
        tag_dict: Optional[Dict[str, str]] = None,
        ref_line: Optional[list] = None,
    ):
        """
        Initialization of RegulatoryElement

        :param id_: ID of the RegulatoryElement
        :param refers: list of the way IDs that the relation refers to
        :param yield_ways: list of the yield way IDs that the relation contains
        :param right_of_ways: list of the right of way IDs that the relation contains
        :param tag_dict: tag dictionary of the RegulatoryElement
        :param ref_line: list of the ref line IDs of the RegulatoryElement
        """
        self.id_ = str(id_)
        self.refers = [str(i) for i in refers] if refers is not None else ()
        self.yield_ways = [str(i) for i in yield_ways] if yield_ways is not None else ()
        self.right_of_ways = [str(i) for i in right_of_ways] if right_of_ways is not None else ()
        self.ref_line = [str(i) for i in ref_line] if ref_line is not None else []
        self.tag_dict = tag_dict if tag_dict is not None else {}

    def serialize_to_xml(self) -> etree.Element:
        """
        Serializes the RegulatoryElement object to the xml format
        """
        rel = etree.Element("relation")
        rel.set("id", self.id_)
        rel.set("action", "modify")
        rel.set("version", "1")
        rel.set("visible", "true")
        for r in self.refers:
            right_way = etree.SubElement(rel, "member")
            right_way.set("type", "way")
            right_way.set("ref", r)
            right_way.set("role", "refers")
        for y in self.yield_ways:
            right_way = etree.SubElement(rel, "member")
            right_way.set("type", "relation")
            right_way.set("ref", y)
            right_way.set("role", "yield")
        for r in self.right_of_ways:
            right_way = etree.SubElement(rel, "member")
            right_way.set("type", "relation")
            right_way.set("ref", r)
            right_way.set("role", "right_of_way")
        for r in self.ref_line:
            right_way = etree.SubElement(rel, "member")
            right_way.set("type", "way")
            right_way.set("ref", r)
            right_way.set("role", "ref_line")
        for tag_key, tag_value in self.tag_dict.items():
            xml_node = etree.SubElement(rel, "tag")
            xml_node.set("k", tag_key)
            xml_node.set("v", tag_value)
        return rel


class OSMLanelet:
    """
    Basic OSM representation.
    """

    def __init__(self):
        """Initialization of the OSMLanelet"""
        self.nodes = {}
        self.ways = {}
        self.way_relations = {}
        self.multipolygons = {}
        self.regulatory_elements = {}
        self.right_of_way_relations = {}
        self.speed_limit_relations = {}
        self.traffic_light_relations = {}
        self.speed_limit_signs = {}  # although that right_of_way_relations,traffic_light_relations and   #
        # speed_limit_relations  # are all stored inside regulatory_elements, there are separate getters for them as
        # it is easier to understand  # the code.

    def add_node(self, node: Node):
        """
        Add a new node to the OSM.

        :param node: Node to be added.
        """
        self.nodes[node.id_] = node

    def add_way(self, way: Way):
        """
        Maps a way id with the way object and inserts it into the OSM object.

        :param way: Way that will be added to the OSM object
        """
        self.ways[way.id_] = way

    def add_way_relation(self, way_relation: WayRelation):
        """
        Maps a way relation id with the way relation object and inserts it into the OSM object.

        :param way_relation: WayRelation that will be added to the OSM object
        """
        self.way_relations[way_relation.id_] = way_relation

    def add_multipolygon(self, multipolygon: Multipolygon):
        """
        Maps a multipolygon id with the multipolygon object and inserts it into the OSM object.

        :param multipolygon: Multipolygon that will be added to the OSM object
        """
        self.multipolygons[multipolygon.id_] = multipolygon

    def add_regulatory_element(self, regulatory_element: RegulatoryElement):
        """
        Maps a right of way relation id with the regulatory element object and inserts it into the OSM object.

        :param regulatory_element: RegulatoryElement that will be added to the OSM object
        """
        if regulatory_element.tag_dict["subtype"] == "right_of_way":
            self.right_of_way_relations[regulatory_element.id_] = regulatory_element

        if regulatory_element.tag_dict["subtype"] == "speed_limit":
            self.speed_limit_relations[regulatory_element.id_] = regulatory_element

        if regulatory_element.tag_dict["subtype"] == "traffic_light":
            self.traffic_light_relations[regulatory_element.id_] = regulatory_element

        self.regulatory_elements[regulatory_element.id_] = regulatory_element

    def add_speed_limit_sign(self, speed_limit_id: str, speed_limit_speed: str, traffic_sign_id):
        """
        Maps a speed limit id with the speed limit speed and the traffic sign id and inserts it into the OSM object.

        :param speed_limit_id: ID of speed limit sign
        :param speed_limit_speed: Speed limit [m/s]
        :param traffic_sign_id: ID of traffic sign
        """
        self.speed_limit_signs[speed_limit_id] = (speed_limit_speed, traffic_sign_id)

    def find_way_by_id(self, way_id: str) -> Optional[Way]:
        """
        Finds a way corresponding to its id.

        :param way_id: id of the way
        :return: way with the corresponding id
        """
        return self.ways.get(way_id)

    def find_regulatory_element_by_id(self, right_id: str) -> Optional[RegulatoryElement]:
        """
        Finds a right of way relation corresponding to its id.

        :param right_id: id of the right of way relation
        :return: RegulatoryElement with the corresponding id
        """
        return self.regulatory_elements.get(right_id)

    def find_right_of_way_by_id(self, right_id: str) -> Optional[RegulatoryElement]:
        """
        Finds a right of way relation corresponding to its id.

        :param right_id: id of the right of way relation
        :return: RegulatoryElement with the corresponding id
        """
        return self.regulatory_elements.get(right_id)

    def find_node_by_id(self, node_id: str) -> Optional[Node]:
        """
        Finds a node corresponding to its id.

        :param node_id: id of the node
        :return: Node with the corresponding id
        """
        return self.nodes.get(node_id)

    def find_way_rel_by_id(self, way_rel_id: str) -> Optional[WayRelation]:
        """
        Finds a way relation corresponding to its id

        :param way_rel_id: Id to the way relation
        :return: WayRelation with the corresponding id
        """
        return self.way_relations.get(way_rel_id)

    def serialize_to_xml(self) -> etree.Element:
        """
        Serialize the OSM to an XML document.
        """
        osm = etree.Element("osm")
        osm.set("version", "0.6")
        osm.set("upload", "true")
        osm.set("generator", "AutoMapConverter")

        for node in self.nodes.values():
            osm.append(node.serialize_to_xml())

        for way in self.ways.values():
            osm.append(way.serialize_to_xml())

        for way_relation in self.way_relations.values():
            osm.append(way_relation.serialize_to_xml())

        for multipolygon in self.multipolygons.values():
            osm.append(multipolygon.serialize_to_xml())

        for regulatory_element in self.regulatory_elements.values():
            osm.append(regulatory_element.serialize_to_xml())

        return osm
    
    def clear_way_relation_topology(self) -> None:
        for way_relation in self.way_relations.values():
            way_relation.predecessors = []
            way_relation.successors = []
            way_relation.left_neighbors = []
            way_relation.right_neighbors = []
            way_relation.adjacent_left = []
            way_relation.adjacent_right = []
            way_relation.topology_source = {
                "predecessors": "",
                "successors": "",
                "left_neighbors": "",
                "right_neighbors": "",
                "adjacent_left": "",
                "adjacent_right": "",
            }

    def _append_unique_relation_id(self, target_list: List[str], relation_id: str) -> None:
        relation_id = str(relation_id)
        if relation_id not in target_list:
            target_list.append(relation_id)

    def set_way_relation_topology(
        self,
        way_rel_id: str,
        predecessors: Optional[List[str]] = None,
        successors: Optional[List[str]] = None,
        left_neighbors: Optional[List[str]] = None,
        right_neighbors: Optional[List[str]] = None,
        adjacent_left: Optional[List[str]] = None,
        adjacent_right: Optional[List[str]] = None,
        source: str = "",
        overwrite: bool = False,
    ) -> None:
        way_rel = self.find_way_rel_by_id(way_rel_id)
        if way_rel is None:
            return

        def _apply(dst_attr: str, values: Optional[List[str]], source_key: str) -> None:
            if values is None:
                return
            cleaned = [str(x) for x in values if str(x) != str(way_rel_id)]
            cleaned = sorted(set(cleaned))
            dst = getattr(way_rel, dst_attr)

            if overwrite:
                setattr(way_rel, dst_attr, cleaned)
            else:
                for item in cleaned:
                    self._append_unique_relation_id(dst, item)

            if source:
                way_rel.topology_source[source_key] = source

        _apply("predecessors", predecessors, "predecessors")
        _apply("successors", successors, "successors")
        _apply("left_neighbors", left_neighbors, "left_neighbors")
        _apply("right_neighbors", right_neighbors, "right_neighbors")
        _apply("adjacent_left", adjacent_left, "adjacent_left")
        _apply("adjacent_right", adjacent_right, "adjacent_right")

    def sort_way_relation_topology(self) -> None:
        for way_relation in self.way_relations.values():
            way_relation.predecessors = sorted(set(way_relation.predecessors), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))
            way_relation.successors = sorted(set(way_relation.successors), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))
            way_relation.left_neighbors = sorted(set(way_relation.left_neighbors), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))
            way_relation.right_neighbors = sorted(set(way_relation.right_neighbors), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))
            way_relation.adjacent_left = sorted(set(way_relation.adjacent_left), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))
            way_relation.adjacent_right = sorted(set(way_relation.adjacent_right), key=lambda x: int(x) if str(x).lstrip("-").isdigit() else str(x))

def _lanelet2_enum_value(enum_container: Any, name: str) -> Any:
    if not hasattr(enum_container, name):
        available = [x for x in dir(enum_container) if not x.startswith("_")]
        raise ValueError(f"Enum value '{name}' not found. Available: {available}")
    return getattr(enum_container, name)


def _safe_native_lanelet_id(native_lanelet: Any) -> Optional[str]:
    if native_lanelet is None:
        return None
    lanelet_id = getattr(native_lanelet, "id", None)
    if lanelet_id is None:
        return None
    return str(lanelet_id)


def _safe_optional_lanelet_id(optional_lanelet: Any) -> Optional[str]:
    if optional_lanelet is None:
        return None

    lanelet_id = _safe_native_lanelet_id(optional_lanelet)
    if lanelet_id is not None:
        return lanelet_id

    getter = getattr(optional_lanelet, "get", None)
    if callable(getter):
        try:
            value = getter()
            return _safe_native_lanelet_id(value)
        except Exception:
            return None
    return None


def _safe_lanelet_list_ids(native_lanelet_list: Any) -> List[str]:
    result: List[str] = []
    if native_lanelet_list is None:
        return result

    try:
        for item in native_lanelet_list:
            item_id = _safe_native_lanelet_id(item)
            if item_id is not None and item_id not in result:
                result.append(item_id)
    except TypeError:
        single_id = _safe_native_lanelet_id(native_lanelet_list)
        if single_id is not None:
            result.append(single_id)
    return result


def enrich_topology_from_official_routing_graph(
    osm_lanelet: "OSMLanelet",
    osm_file_path: str,
    origin_lat: float,
    origin_lon: float,
    location: str = "Germany",
    participant: str = "Vehicle",
    overwrite: bool = True,
    fill_left_right: bool = True,
    fill_adjacent_left_right: bool = True,
) -> Dict[str, Dict[str, List[str]]]:
    try:
        import lanelet2
    except ImportError as exc:
        raise ImportError(
            "Official lanelet2 python package is not available."
        ) from exc

    origin = lanelet2.io.Origin(float(origin_lat), float(origin_lon))
    native_map = lanelet2.io.load(str(osm_file_path), origin)

    location_enum = _lanelet2_enum_value(lanelet2.traffic_rules.Locations, location)
    participant_enum = _lanelet2_enum_value(lanelet2.traffic_rules.Participants, participant)
    traffic_rules = lanelet2.traffic_rules.create(location_enum, participant_enum)

    routing_graph = lanelet2.routing.RoutingGraph(native_map, traffic_rules)

    if overwrite:
        osm_lanelet.clear_way_relation_topology()

    topology_by_lanelet_id: Dict[str, Dict[str, List[str]]] = {}

    for native_lanelet in native_map.laneletLayer:
        lanelet_id = _safe_native_lanelet_id(native_lanelet)
        if lanelet_id is None:
            continue

        custom_way_rel = osm_lanelet.find_way_rel_by_id(lanelet_id)
        if custom_way_rel is None:
            continue

        successors = _safe_lanelet_list_ids(routing_graph.following(native_lanelet))
        predecessors = _safe_lanelet_list_ids(routing_graph.previous(native_lanelet))

        left_neighbors: List[str] = []
        right_neighbors: List[str] = []
        if fill_left_right:
            left_id = _safe_optional_lanelet_id(routing_graph.left(native_lanelet))
            right_id = _safe_optional_lanelet_id(routing_graph.right(native_lanelet))
            if left_id is not None:
                left_neighbors.append(left_id)
            if right_id is not None:
                right_neighbors.append(right_id)

        adjacent_left: List[str] = []
        adjacent_right: List[str] = []
        if fill_adjacent_left_right:
            adj_left_id = _safe_optional_lanelet_id(routing_graph.adjacentLeft(native_lanelet))
            adj_right_id = _safe_optional_lanelet_id(routing_graph.adjacentRight(native_lanelet))
            if adj_left_id is not None:
                adjacent_left.append(adj_left_id)
            if adj_right_id is not None:
                adjacent_right.append(adj_right_id)

        valid_ids = set(osm_lanelet.way_relations.keys())
        successors = [x for x in successors if x in valid_ids]
        predecessors = [x for x in predecessors if x in valid_ids]
        left_neighbors = [x for x in left_neighbors if x in valid_ids]
        right_neighbors = [x for x in right_neighbors if x in valid_ids]
        adjacent_left = [x for x in adjacent_left if x in valid_ids]
        adjacent_right = [x for x in adjacent_right if x in valid_ids]

        osm_lanelet.set_way_relation_topology(
            way_rel_id=lanelet_id,
            predecessors=predecessors,
            successors=successors,
            left_neighbors=left_neighbors,
            right_neighbors=right_neighbors,
            adjacent_left=adjacent_left,
            adjacent_right=adjacent_right,
            source="official_routing_graph",
            overwrite=overwrite,
        )

        topology_by_lanelet_id[lanelet_id] = {
            "predecessors": predecessors,
            "successors": successors,
            "left_neighbors": left_neighbors,
            "right_neighbors": right_neighbors,
            "adjacent_left": adjacent_left,
            "adjacent_right": adjacent_right,
        }

    osm_lanelet.sort_way_relation_topology()
    return topology_by_lanelet_id
