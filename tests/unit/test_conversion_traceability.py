import json
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree
from pyproj import Transformer

from automap_converter.core.traceability import (
    LINEAGE,
    annotate_conversion,
    document_id,
    extension,
    read_frame,
    transform_points,
)


def lanelet_xml(relation_id="100", extra=""):
    return f'''<osm version="0.6">
    <node id="1" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="2"/></node>
    <node id="2" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="2"/></node>
    <node id="3" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="-2"/></node>
    <node id="4" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="-2"/></node>
    <way id="11"><nd ref="1"/><nd ref="2"/></way>
    <way id="12"><nd ref="3"/><nd ref="4"/></way>
    <relation id="{relation_id}"><member type="way" ref="11" role="left"/>
    <member type="way" ref="12" role="right"/><tag k="type" v="lanelet"/>{extra}</relation></osm>'''


def test_two_leg_lineage_and_derived_vertices_preserve_root_identity(tmp_path):
    a, b, c = [tmp_path / p for p in ("a.osm", "b.xodr", "c.osm")]
    a.write_text(lanelet_xml())
    original = a.read_bytes()
    b.write_text("""<OpenDRIVE><header/><road id="7" length="10"><planView>
    <geometry x="0" y="0" s="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes><laneSection s="0"><right><lane id="-1" type="driving">
    <userData code="lanelet2:lanelet_mapping" value="100|11|12"/>
    </lane></right></laneSection></lanes></road></OpenDRIVE>""")
    annotate_conversion(a, b, "lanelet2", "opendrive")
    c.write_text(
        lanelet_xml(
            "300",
            """<tag k="source:opendrive:road" v="7"/>
    <tag k="source:opendrive:lane_section" v="0"/><tag k="source:opendrive:lane" v="-1"/>""",
        )
    )
    audit = annotate_conversion(b, c, "opendrive", "lanelet2")
    root = etree.parse(str(c)).getroot()
    trace = extension(root.find("relation"), LINEAGE)
    assert {"format": "lanelet2", "document": document_id(a), "key": "100"} in trace["ancestors"]
    assert {"format": "opendrive", "document": document_id(b), "key": "7|0|-1"} in trace[
        "ancestors"
    ]
    assert extension(root.find("node"), LINEAGE)["ancestors"]
    assert a.read_bytes() == original
    assert audit["mapped_source_lanes_or_roads"] == 1
    assert c.with_name("c_conversion_audit.json").is_file()
    assert set(trace["ancestors"][0]) == {"format", "document", "key"}


def test_namespaced_opendrive_input_has_native_lane_correspondence(tmp_path):
    source, target = tmp_path / "source.xodr", tmp_path / "target.osm"
    source.write_text("""<OpenDRIVE xmlns="http://www.opendrive.org"><header/>
    <road id="5"><lanes><laneSection s="0"><right><lane id="-2" type="driving"/>
    </right></laneSection></lanes></road></OpenDRIVE>""")
    target.write_text(
        lanelet_xml(
            "20",
            """<tag k="source:opendrive:road" v="5"/>
    <tag k="source:opendrive:lane_section" v="0"/><tag k="source:opendrive:lane" v="-2"/>""",
        )
    )
    report = annotate_conversion(source, target, "opendrive", "lanelet2")
    assert report["mapped_source_lanes_or_roads"] == 1


def test_unknown_sources_are_reported_not_invented(tmp_path):
    source, target = tmp_path / "a.osm", tmp_path / "b.osm"
    source.write_text(lanelet_xml())
    target.write_text(
        '<osm><node id="1" lat="0" lon="0"/><node id="2" lat="0" lon="1"/>'
        '<way id="9"><nd ref="1"/><nd ref="2"/><tag k="highway" v="road"/></way></osm>'
    )
    report = annotate_conversion(source, target, "lanelet2", "osm")
    assert report["mapped_source_lanes_or_roads"] == 0
    assert "100" in report["unmapped_source_keys"]


def test_declared_coordinate_transforms_do_not_fit_geometry():
    geographic = {"kind": "geographic", "crs": "EPSG:4326"}
    metric = {"kind": "projected", "crs": "EPSG:3857"}
    lonlat = [(11.6, 48.2), (11.6001, 48.2001)]
    xy = transform_points(lonlat, geographic, metric)
    inverse = transform_points(xy, metric, geographic)
    assert inverse[0] == pytest.approx(lonlat[0])
    assert xy[0] == pytest.approx(
        Transformer.from_crs(4326, 3857, always_xy=True).transform(*lonlat[0])
    )


def test_engineering_frame_encoding_is_reversible_but_not_a_world_location():
    engineering = {"kind": "engineering", "id": "local"}
    encoded = {
        "kind": "geographic",
        "crs": "EPSG:4326",
        "encoding_crs": "EPSG:3857",
        "engineering_frame": engineering,
    }
    xy = [(10.0, 20.0)]
    lonlat = transform_points(xy, engineering, encoded)
    assert transform_points(lonlat, encoded, engineering)[0] == pytest.approx(xy[0])


def test_inconsistent_xy_and_geographic_encodings_are_visible(tmp_path):
    source, target = tmp_path / "a.osm", tmp_path / "b.osm"
    source.write_text(
        '<osm><node id="1" lat="48" lon="11"/><node id="2" lat="48" lon="11.001"/>'
        '<way id="1"><nd ref="1"/><nd ref="2"/><tag k="highway" v="road"/></way></osm>'
    )
    target.write_text(lanelet_xml("100", '<tag k="source:osm:ways" v="1"/>'))
    audit = annotate_conversion(source, target, "osm", "lanelet2")
    assert audit["coordinate_checks"]["status"] == "FAIL"
    frame = read_frame(etree.parse(str(target)).getroot(), "lanelet2")
    assert frame["crs"]
    assert (
        json.loads(target.with_name("b_conversion_audit.json").read_text())["coordinate_checks"][
            "status"
        ]
        == "FAIL"
    )


def test_generated_traffic_light_local_xy_matches_geographic_encoding():
    from automap_converter.conversion.commonroad_to_lanelet2.converter import CR2LaneletConverter
    from automap_converter.core.config.lanelet2_config import Lanelet2Config
    from automap_converter.formats.lanelet2.map import OSMLanelet

    config = Lanelet2Config()
    config.autoware = False
    config.use_local_coordinates = True
    converter = CR2LaneletConverter(config)
    converter.osm = OSMLanelet()
    converter.transformer = Transformer.from_crs("EPSG:3857", "EPSG:4258")
    converter._convert_traffic_light(
        SimpleNamespace(position=np.array([100.0, 200.0]), color=[SimpleNamespace(value="red")])
    )
    inverse = Transformer.from_crs("EPSG:4258", "EPSG:3857", always_xy=True)
    for node in converter.osm.nodes.values():
        xy = inverse.transform(float(node.lon), float(node.lat))
        assert xy == pytest.approx((node.local_x, node.local_y), abs=1e-6)


def test_explicit_lanelet_centerline_is_measured_when_present(tmp_path):
    from automap_converter.validation.diagnostics.roundtrip_diagnostics import _extract_lanelet2

    path = tmp_path / "centerline.osm"
    text = lanelet_xml().replace(
        '<relation id="100">',
        """
      <node id="5" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="0.25"/></node>
      <node id="6" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="0.25"/></node>
      <way id="13"><nd ref="5"/><nd ref="6"/></way>
      <relation id="100"><member type="way" role="centerline" ref="13"/>""",
    )
    path.write_text(text)
    assert _extract_lanelet2(path).features[0].points == [(0.0, 0.25), (10.0, 0.25)]
