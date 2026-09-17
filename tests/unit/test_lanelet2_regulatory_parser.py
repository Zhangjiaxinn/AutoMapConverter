from lxml import etree

from automap_converter.formats.lanelet2.parser import Lanelet2Parser


def test_parser_preserves_regulatory_subtypes_and_roles():
    root = etree.fromstring(
        b"""<osm>
        <relation id="1">
          <member type="way" role="refers" ref="11"/>
          <member type="way" role="ref_line" ref="12"/>
          <tag k="type" v="regulatory_element"/>
          <tag k="subtype" v="traffic_sign"/>
        </relation>
        <relation id="2">
          <member type="relation" role="yield" ref="21"/>
          <member type="relation" role="right_of_way" ref="22"/>
          <tag k="type" v="regulatory_element"/>
          <tag k="subtype" v="right_of_way"/>
        </relation>
        <relation id="3">
          <member type="way" role="refers" ref="31"/>
          <tag k="type" v="regulatory_element"/>
          <tag k="subtype" v="bus_stop_area"/>
        </relation>
        </osm>"""
    )

    parsed = Lanelet2Parser(root).parse()

    assert set(parsed.regulatory_elements) == {"1", "2", "3"}
    assert parsed.regulatory_elements["1"].refers == ["11"]
    assert parsed.regulatory_elements["1"].ref_line == ["12"]
    assert parsed.regulatory_elements["2"].yield_ways == ["21"]
    assert parsed.regulatory_elements["2"].right_of_ways == ["22"]
    assert parsed.regulatory_elements["3"].tag_dict["subtype"] == "bus_stop_area"
