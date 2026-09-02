from typing import Dict, List, Optional, Union
from lxml import etree  # type: ignore


class Taggable:
    """
    Base class for OSM elements that can contain tags.
    """

    def __init__(self, tag_dict: Optional[Dict[str, str]] = None):
        self.tag_dict: Dict[str, str] = tag_dict if tag_dict is not None else {}

    def _append_tags_to_xml(self, element: etree._Element) -> None:
        """
        Append all tags in tag_dict to the given xml element.
        """
        for tag_key, tag_value in self.tag_dict.items():
            xml_tag = etree.SubElement(element, "tag")
            xml_tag.set("k", str(tag_key))
            xml_tag.set("v", str(tag_value))


class Node(Taggable):
    """
    OSM Node (point element).
    """

    def __init__(
        self,
        id_,
        lat: Union[str, float],
        lon: Union[str, float],
        ele: Optional[Union[str, float]] = None,
        tag_dict: Optional[Dict[str, str]] = None,
    ):
        """
        Initialization of Node.

        :param id_: Node ID
        :param lat: Latitude
        :param lon: Longitude
        :param ele: Elevation (optional)
        :param tag_dict: Node tags
        """
        super().__init__(tag_dict)
        self.id_: str = str(id_)
        self.lat: str = str(lat)
        self.lon: str = str(lon)
        self.ele: Optional[str] = str(ele) if ele is not None else None

    def serialize_to_xml(self) -> etree._Element:
        """
        Serialize Node to XML.
        """
        node = etree.Element("node")
        node.set("id", self.id_)
        node.set("visible", "true")
        node.set("version", "1")
        node.set("lat", self.lat)
        node.set("lon", self.lon)

        if self.ele is not None:
            ele_tag = etree.SubElement(node, "tag")
            ele_tag.set("k", "ele")
            ele_tag.set("v", self.ele)

        self._append_tags_to_xml(node)
        return node


class Way(Taggable):
    """
    OSM Way.
    """

    def __init__(
        self,
        id_,
        nodes: List[Union[str, int]],
        tag_dict: Optional[Dict[str, str]] = None,
    ):
        """
        Initialization of Way.

        :param id_: Way ID
        :param nodes: Ordered list of node IDs
        :param tag_dict: Way tags
        """
        super().__init__(tag_dict)
        self.id_: str = str(id_)
        self.nodes: List[str] = [str(node_id) for node_id in nodes]

    def serialize_to_xml(self) -> etree._Element:
        """
        Serialize Way to XML.
        """
        way = etree.Element("way")
        way.set("id", self.id_)
        way.set("visible", "true")
        way.set("version", "1")

        for node_id in self.nodes:
            nd = etree.SubElement(way, "nd")
            nd.set("ref", node_id)

        self._append_tags_to_xml(way)
        return way


class RelationMember:
    """
    OSM Relation member.
    """

    def __init__(self, member_type: str, ref, role: str = ""):
        """
        Initialization of RelationMember.

        :param member_type: 'node', 'way', or 'relation'
        :param ref: Referenced element ID
        :param role: Role name in relation
        """
        if member_type not in {"node", "way", "relation"}:
            raise ValueError("member_type must be one of: 'node', 'way', 'relation'")

        self.member_type: str = member_type
        self.ref: str = str(ref)
        self.role: str = str(role)

    def serialize_to_xml(self) -> etree._Element:
        """
        Serialize RelationMember to XML.
        """
        member = etree.Element("member")
        member.set("type", self.member_type)
        member.set("ref", self.ref)
        member.set("role", self.role)
        return member


class Relation(Taggable):
    """
    OSM Relation.
    """

    def __init__(
        self,
        id_,
        members: Optional[List[RelationMember]] = None,
        tag_dict: Optional[Dict[str, str]] = None,
    ):
        """
        Initialization of Relation.

        :param id_: Relation ID
        :param members: List of RelationMember
        :param tag_dict: Relation tags
        """
        super().__init__(tag_dict)
        self.id_: str = str(id_)
        self.members: List[RelationMember] = members if members is not None else []

    def add_member(self, member: RelationMember) -> None:
        """
        Add a relation member.
        """
        self.members.append(member)

    def serialize_to_xml(self) -> etree._Element:
        """
        Serialize Relation to XML.
        """
        relation = etree.Element("relation")
        relation.set("id", self.id_)
        relation.set("visible", "true")
        relation.set("version", "1")

        for member in self.members:
            relation.append(member.serialize_to_xml())

        self._append_tags_to_xml(relation)
        return relation


class OSM:
    """
    Basic OSM representation containing nodes, ways, and relations.
    """

    def __init__(self):
        """
        Initialization of OSM container.
        """
        self.nodes: Dict[str, Node] = {}
        self.ways: Dict[str, Way] = {}
        self.relations: Dict[str, Relation] = {}

    def add_node(self, node: Node) -> None:
        """
        Add a node to OSM.
        """
        self.nodes[node.id_] = node

    def add_way(self, way: Way) -> None:
        """
        Add a way to OSM.
        """
        self.ways[way.id_] = way

    def add_relation(self, relation: Relation) -> None:
        """
        Add a relation to OSM.
        """
        self.relations[relation.id_] = relation

    def find_node_by_id(self, node_id: Union[str, int]) -> Optional[Node]:
        """
        Find a node by ID.
        """
        return self.nodes.get(str(node_id))

    def find_way_by_id(self, way_id: Union[str, int]) -> Optional[Way]:
        """
        Find a way by ID.
        """
        return self.ways.get(str(way_id))

    def find_relation_by_id(self, relation_id: Union[str, int]) -> Optional[Relation]:
        """
        Find a relation by ID.
        """
        return self.relations.get(str(relation_id))

    def remove_node_by_id(self, node_id: Union[str, int]) -> None:
        """
        Remove a node by ID if it exists.
        """
        self.nodes.pop(str(node_id), None)

    def remove_way_by_id(self, way_id: Union[str, int]) -> None:
        """
        Remove a way by ID if it exists.
        """
        self.ways.pop(str(way_id), None)

    def remove_relation_by_id(self, relation_id: Union[str, int]) -> None:
        """
        Remove a relation by ID if it exists.
        """
        self.relations.pop(str(relation_id), None)

    def serialize_to_xml(self) -> etree._Element:
        """
        Serialize the OSM object to an XML root element.
        """
        osm = etree.Element("osm")
        osm.set("version", "0.6")
        osm.set("generator", "custom-python-osm")

        for node in self.nodes.values():
            osm.append(node.serialize_to_xml())

        for way in self.ways.values():
            osm.append(way.serialize_to_xml())

        for relation in self.relations.values():
            osm.append(relation.serialize_to_xml())

        return osm

    def to_xml_bytes(
        self,
        pretty_print: bool = True,
        xml_declaration: bool = True,
        encoding: str = "UTF-8",
    ) -> bytes:
        """
        Convert OSM object to XML bytes.
        """
        root = self.serialize_to_xml()
        return etree.tostring(
            root,
            pretty_print=pretty_print,
            xml_declaration=xml_declaration,
            encoding=encoding,
        )

    def write_xml(
        self,
        file_path: str,
        pretty_print: bool = True,
        xml_declaration: bool = True,
        encoding: str = "UTF-8",
    ) -> None:
        """
        Write OSM object to an XML/OSM file.
        """
        xml_bytes = self.to_xml_bytes(
            pretty_print=pretty_print,
            xml_declaration=xml_declaration,
            encoding=encoding,
        )
        with open(file_path, "wb") as f:
            f.write(xml_bytes)


if __name__ == "__main__":

    osm = OSM()

    # 1. Add nodes
    n1 = Node(id_=-1, lat=39.9042, lon=116.4074, tag_dict={"name": "point_a"})
    n2 = Node(id_=-2, lat=39.9043, lon=116.4075)
    n3 = Node(id_=-3, lat=39.9044, lon=116.4076)

    osm.add_node(n1)
    osm.add_node(n2)
    osm.add_node(n3)

    # 2. Add way
    w1 = Way(
        id_=-10,
        nodes=[-1, -2, -3],
        tag_dict={"highway": "residential", "name": "test_way"},
    )
    osm.add_way(w1)

    # 3. Add relation
    rel = Relation(
        id_=-100,
        tag_dict={"type": "route", "route": "road", "name": "test_relation"},
    )
    rel.add_member(RelationMember("way", -10, ""))
    rel.add_member(RelationMember("node", -1, "via"))
    osm.add_relation(rel)

    # 4. Write file
    osm.write_xml("example.osm")

    print("OSM XML file written to example.osm")