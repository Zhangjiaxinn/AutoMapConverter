from itertools import pairwise

import pytest
from lxml import etree

from automap_converter.conversion.lanelet2_to_opendrive.converter import Lanelet2OpendriveConverter
from automap_converter.core.config.opendrive_config import open_drive_config
from automap_converter.formats.lanelet2.shared_geometry import consolidate_shared_geometry
from automap_converter.formats.opendrive.parser.elements.roadLanes import Lane
from automap_converter.validation.diagnostics._roundtrip_contract import (
    WIDTH_FRACTIONS,
    section_width,
)
from automap_converter.validation.diagnostics.native_semantics import opendrive_speed_consistency
from automap_converter.validation.diagnostics.roundtrip_diagnostics import Feature, extract_features


def test_width_uses_one_physical_section_even_when_target_centerline_rotates():
    source = Feature(
        "a",
        [(0, 0), (10, 0)],
        3.5,
        "road",
        "yes",
        left=[(0, 1.75), (10, 1.75)],
        right=[(0, -1.75), (10, -1.75)],
    )
    target = Feature(
        "b", [(0, -4), (10, 4)], 3.5, "road", "yes", left=source.left, right=source.right
    )
    assert WIDTH_FRACTIONS == (0.25, 0.35, 0.5, 0.65, 0.75)
    assert section_width(target, 0.5) > 4
    assert section_width(target, 0.5, reference=source) == pytest.approx(3.5)


def test_supported_bike_and_bus_types_are_not_filtered():
    assert {"biking", "bus"} <= set(open_drive_config.filter_types)
    assert {"biking", "bus"} <= set(Lane.laneTypes)


def test_gradual_widths_are_continuous_positive_and_not_clipped():
    converter = object.__new__(Lanelet2OpendriveConverter)
    lane = etree.Element("lane")
    converter.write_variable_lane_width(lane, [(0, 3), (5, 5), (10, 4)], None)
    widths = lane.findall("width")
    assert max(float(w.get("a")) for w in widths) == 5
    for first, second in pairwise(widths):
        ds = float(second.get("sOffset")) - float(first.get("sOffset"))
        assert float(first.get("a")) + float(first.get("b")) * ds == pytest.approx(
            float(second.get("a"))
        )


def test_native_speed_check_honors_validity_and_detects_unit_error(tmp_path):
    path = tmp_path / "speed.xodr"
    path.write_text("""<OpenDRIVE><road id="1" length="20"><lanes><laneSection s="0"><right>
    <lane id="-1"><speed sOffset="0" max="40" unit="mph"/></lane>
    <lane id="-2"><speed sOffset="0" max="10" unit="m/s"/></lane>
    </right></laneSection></lanes><signals><signal id="s" s="1" type="274" value="17.8816" unit="km/h">
    <validity fromLane="-1" toLane="-1"/></signal></signals></road></OpenDRIVE>""")
    result = opendrive_speed_consistency(path)
    assert result["status"] == "FAIL" and len(result["records"]) == 1
    root = etree.parse(str(path))
    root.find(".//signal").set("unit", "m/s")
    root.write(str(path))
    assert opendrive_speed_consistency(path)["status"] == "PASS"


def duplicate_edges(height=0):
    root = etree.Element("osm", version="0.6")
    for n, (x, y, z) in enumerate(
        (
            (0, 0, 0),
            (10, 0, 0),
            (0, 0, height),
            (10, 0, height),
            (0, 3, 0),
            (10, 3, 0),
            (0, -3, height),
            (10, -3, height),
        ),
        1,
    ):
        node = etree.SubElement(root, "node", id=str(n), lon=str(x / 111320), lat=str(y / 111320))
        for k, v in (("local_x", x), ("local_y", y), ("ele", z)):
            etree.SubElement(node, "tag", k=k, v=str(v))
    for key, refs in ((10, (1, 2)), (11, (3, 4)), (12, (5, 6)), (13, (7, 8))):
        way = etree.SubElement(root, "way", id=str(key))
        for ref in refs:
            etree.SubElement(way, "nd", ref=str(ref))
        etree.SubElement(way, "tag", k="type", v="virtual")
    for key, left, right in ((20, 12, 10), (21, 11, 13)):
        relation = etree.SubElement(root, "relation", id=str(key))
        etree.SubElement(relation, "tag", k="type", v="lanelet")
        for role, ref in (("left", left), ("right", right)):
            etree.SubElement(relation, "member", type="way", role=role, ref=str(ref))
    return root


def test_actual_coincident_boundaries_share_native_id_but_overpasses_do_not():
    root = duplicate_edges()
    assert consolidate_shared_geometry(root)["shared_ways"] == 1
    assert root.find("relation[@id='20']/member[@role='right']").get("ref") == root.find(
        "relation[@id='21']/member[@role='left']"
    ).get("ref")
    assert consolidate_shared_geometry(duplicate_edges(height=5))["shared_ways"] == 0


def test_private_topology_cannot_forge_a_native_link(tmp_path):
    root = duplicate_edges(height=0)
    etree.SubElement(root.find("relation[@id='20']"), "tag", k="automap:topology:right", v="21")
    path = tmp_path / "lanes.osm"
    etree.ElementTree(root).write(str(path))
    features = extract_features(path, "lanelet2")
    assert not features.features[0].relations["right"]
