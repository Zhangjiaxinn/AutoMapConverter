from pathlib import Path

import networkx as nx
import pytest
from lxml import etree

from automap_converter.core.traceability import append_tag, extension, set_extension, tags
from automap_converter.formats.osm.parser import extract_speedlimit
from automap_converter.formats.osm.road_semantics import length_m, road_profile
from automap_converter.validation.diagnostics._roundtrip_contract import _relation_path
from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    _extract_osm,
    _speed_value,
)


@pytest.mark.parametrize(
    "value,expected",
    [
        ("3.5", 3.5),
        ("3.5 m", 3.5),
        ("10 ft", 3.048),
        ("6'6\"", 1.9812),
        ("unknown", None),
        ("0", None),
    ],
)
def test_native_width_units(value, expected):
    result = length_m(value)
    assert result is None if expected is None else result == pytest.approx(expected)


def test_width_precedence_and_same_import_defaults():
    measured = road_profile({"highway": "primary", "lanes": "2", "width": "8", "est_width": "7"})
    assert measured["width_m"] == 8
    assert not measured["estimated"]
    individual = road_profile({"highway": "primary", "lanes": "2", "width:lanes": "3|4"})
    assert individual["lane_widths_m"] == [3, 4]
    assert individual["width_m"] == 7
    estimated = road_profile({"highway": "primary"})
    assert estimated["estimated"] and estimated["width_m"] == 7
    assert road_profile({"highway": "footway"})["width_m"] == 2


def test_reverse_oneway_counts_are_relative_to_osm_way_order():
    profile = road_profile({"highway": "primary", "lanes": "2", "oneway": "-1"})
    assert (profile["forward"], profile["backward"], profile["oneway"]) == (0, 2, "-1")


def test_scientific_notation_and_mph_are_normalized_without_changing_speed():
    assert _speed_value("4.000000000e+1", "mph") == "17.8816 m/s"
    assert extract_speedlimit("40 mph")[0] == pytest.approx(17.8816)


def test_chunked_extensions_are_ordinary_bounded_osm_tags_and_update_cleanly():
    element = etree.Element("relation")
    value = {"description": "road" * 500}
    set_extension(element, "automap:example", value)
    assert all(len(t.get("v").encode()) <= 255 for t in element)
    assert extension(element, "automap:example") == value
    set_extension(element, "automap:example", {"small": True})
    assert len(element) == 1 and extension(element, "automap:example") == {"small": True}
    append_tag(element, "description", "\u9053\u8def" * 100)
    assert tags(element)["description"] == "\u9053\u8def" * 100
    assert element.find("tag[@k='description']") is not None
    with pytest.raises(ValueError, match="255 characters"):
        append_tag(element, "name", "x" * 256)


def test_directed_subdivision_paths_do_not_route_through_other_source_roads():
    graph = nx.MultiDiGraph()
    graph.add_edge("a", "connector", key="successor")
    graph.add_edge("connector", "b", key="successor")
    path, _ = _relation_path(["a"], ["b"], "successor", graph, {}, {"a", "b"})
    assert path == ["a", "connector", "b"]
    assert not _relation_path(["a"], ["b"], "successor", graph, {}, {"a", "b", "connector"})[0]
    assert not _relation_path(["b"], ["a"], "successor", graph, {}, {"a", "b"})[0]
    graph.remove_edge("connector", "b", key="successor")
    assert not _relation_path(["a"], ["b"], "successor", graph, {}, {"a", "b"})[0]


def test_osm_uses_road_ways_and_native_width_not_lane_boundaries(tmp_path: Path):
    source = tmp_path / "roads.osm"
    source.write_text("""<osm version="0.6">
    <node id="1" lat="48" lon="11"/><node id="2" lat="48" lon="11.001"/>
    <node id="3" lat="48.001" lon="11.001"/>
    <way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/>
      <tag k="lanes" v="4"/><tag k="width" v="14"/></way>
    <way id="20"><nd ref="2"/><nd ref="3"/><tag k="highway" v="footway"/></way>
    </osm>""")
    features = _extract_osm(source)
    assert len(features.features) == 2
    assert features.features[0].width == 14
    assert not features.features[0].left
    assert features.features[1].measurements["estimated"]
    assert features.topology_edges == {("10", "20"), ("20", "10")}
