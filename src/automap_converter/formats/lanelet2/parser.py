from commonroad.scenario.traffic_sign import TrafficSignIDGermany  # type: ignore
from lxml import etree  # type: ignore

from automap_converter.core.config.lanelet2_config import lanelet2_config
from automap_converter.formats.lanelet2.map import (
    Multipolygon,
    Node,
    OSMLanelet,
    RegulatoryElement,
    Way,
    WayRelation,
)


class Lanelet2Parser:
    """
    Parser for OSM documents.
    Only extracts relevant information for conversion to Lanelet.
    """

    def __init__(self, xml_doc: etree.Element, config: lanelet2_config = lanelet2_config):
        """
        Inits Lanelet2Parser

        :param xml_doc: XML tree.
        :param config: Lanelet2 conversion parameters.
        """
        self.xml = xml_doc
        self.config = config

    def parse(self):
        """
        Parses the nodes, way relations, reg_element relations#解析
        """
        osm = OSMLanelet()
        for node in self.xml.xpath("//node[@lat and @lon and @id]"):
            node_tags = {
                tag.get("k"): tag.get("v")
                for tag in node.findall("./tag")
                if tag.get("k") and tag.get("v") is not None
            }
            local_x = node_tags.get("local_x")
            local_y = node_tags.get("local_y")
            osm.add_node(
                Node(
                    node.get("id"),
                    node.get("lat"),
                    node.get("lon"),
                    node_tags.get("ele", 0.0),
                    autoware=self.config.autoware,
                    local_x=float(local_x) if local_x is not None else None,
                    local_y=float(local_y) if local_y is not None else None,
                )
            )

        for way in self.xml.xpath("//way[@id]"):
            node_ids = [nd.get("ref") for nd in way.xpath("./nd")]
            tag_dict = {
                tag.get("k"): tag.get("v")
                for tag in way.xpath("./tag[@k and @v]")
                if tag.get("k") in self.config.allowed_tags
            }

            osm.add_way(Way(way.get("id"), node_ids, tag_dict))

        for way_rel in self.xml.xpath("//relation/tag[@v='lanelet' and @k='type']/.."):
            try:
                left_way = way_rel.xpath("./member[@type='way' and @role='left']/@ref")[0]
                right_way = way_rel.xpath("./member[@type='way' and @role='right']/@ref")[0]
                tag_dict = {
                    tag.get("k"): tag.get("v")
                    for tag in way_rel.xpath("./tag[@k and @v]")
                    if tag.get("k") in self.config.allowed_tags
                }
                regulatory_elements = list(
                    way_rel.xpath("./member[@role='regulatory_element']/@ref")
                )
                osm.add_way_relation(
                    WayRelation(
                        way_rel.get("id"), left_way, right_way, tag_dict, regulatory_elements
                    )
                )
            except IndexError:
                print(
                    f"Lanelet relation {way_rel.attrib.get('id')} has either no left or no right way! "
                    f"Please check your data! Discarding this lanelet relation."
                )
        for multipolygon in self.xml.xpath("//relation/tag[@v='multipolygon' and @k='type']/.."):
            outer_list = list()
            for outer in multipolygon.xpath("./member[@type='way' and @role='outer']/@ref"):
                outer_list.append(outer)
            inner_list = list()
            for inner in multipolygon.xpath("./member[@type='way' and @role='inner']/@ref"):
                inner_list.append(inner)
            tag_dict = {
                tag.get("k"): tag.get("v")
                for tag in multipolygon.xpath("./tag[@k and @v]")
                if tag.get("k") in self.config.allowed_tags
            }
            osm.add_multipolygon(
                Multipolygon(
                    multipolygon.get("id"),
                    outer_list,
                    tag_dict,
                    inner_list=inner_list,
                )
            )

        for reg_element_rel in self.xml.xpath(
            "//relation/tag[@v='regulatory_element' and @k='type']/.."
        ):
            reg_id = reg_element_rel.get("id")
            if not reg_id:
                continue
            tags = {
                tag.get("k"): tag.get("v")
                for tag in reg_element_rel.xpath("./tag[@k and @v]")
            }
            members = reg_element_rel.findall("member")

            def references(role):
                return [
                    member.get("ref")
                    for member in members
                    if member.get("role") == role and member.get("ref")
                ]

            osm.add_regulatory_element(
                RegulatoryElement(
                    reg_id,
                    refers=references("refers"),
                    yield_ways=references("yield"),
                    right_of_ways=references("right_of_way"),
                    tag_dict=tags,
                    ref_line=references("ref_line"),
                )
            )
            if tags.get("subtype") == "speed_limit" and tags.get("sign_type"):
                osm.add_speed_limit_sign(
                    reg_id,
                    tags["sign_type"],
                    TrafficSignIDGermany.MAX_SPEED,
                )

        return osm
