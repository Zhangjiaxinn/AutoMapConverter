from pathlib import Path

import pytest
from lxml import etree
from pyproj import Transformer

from automap_converter.core.config.general_config import general_config
from automap_converter.core.traceability import tags, write_frame
from automap_converter.validation.diagnostics._roundtrip_contract import compare
from automap_converter.validation.diagnostics._workflow_runtime import (
    convert_lanelet2_to_osm,
    convert_osm_to_lanelet2,
)
from automap_converter.validation.diagnostics.roundtrip_diagnostics import extract_features


def make_two_lanes(path: Path):
    root = etree.Element("osm", version="0.6")
    inverse = Transformer.from_crs(3857, 4326, always_xy=True)
    ids = []
    for y in (3.5, 0, -0.5):
        refs = []
        for x in (0, 20):
            nid = str(len(ids) + 1)
            lon, lat = inverse.transform(x, y)
            n = etree.SubElement(root, "node", id=nid, lat=str(lat), lon=str(lon))
            etree.SubElement(n, "tag", k="local_x", v=str(x))
            etree.SubElement(n, "tag", k="local_y", v=str(y))
            refs.append(nid)
            ids.append(nid)
        w = etree.SubElement(root, "way", id=str(10 + len(ids)))
        for nid in refs:
            etree.SubElement(w, "nd", ref=nid)
    for key, left, right, subtype in (("100", "12", "14", "road"), ("101", "14", "16", "shoulder")):
        r = etree.SubElement(root, "relation", id=key)
        for role, ref in (("left", left), ("right", right)):
            etree.SubElement(r, "member", type="way", ref=ref, role=role)
        for k, v in (("type", "lanelet"), ("subtype", subtype)):
            etree.SubElement(r, "tag", k=k, v=v)
    write_frame(
        root, "lanelet2", {"kind": "projected", "crs": "EPSG:3857", "units": "m", "axis": "x,y"}
    )
    etree.ElementTree(root).write(str(path))


def test_default_condensation_uses_public_roads_not_reconstruction_profiles(tmp_path):
    a, b, c = [tmp_path / name for name in ("source.osm", "roads.osm", "lanes.osm")]
    make_two_lanes(a)
    convert_lanelet2_to_osm(a, b, tmp_path / "first")
    root = etree.parse(str(b)).getroot()
    roads = [w for w in root.findall("way") if tags(w).get("highway")]
    assert len(roads) == 1
    assert tags(roads[0])["lanes"] == "1"  # A shoulder is not a traffic lane.
    assert float(tags(roads[0])["width"]) == pytest.approx(3.5)
    assert not any(tags(r).get("type", "").startswith("automap:") for r in root.findall("relation"))
    elements = {(e.tag, e.get("id")) for e in root if e.get("id")}
    for element in root:
        for member in element.findall("member"):
            assert (member.get("type"), member.get("ref")) in elements
    assert all(len(t.get("v").encode()) <= 255 for t in root.findall(".//tag"))
    old_projection = general_config.proj_string_cr
    unavailable_source = tmp_path / "not_supplied_to_reverse_conversion.osm"
    a.rename(unavailable_source)
    convert_osm_to_lanelet2(b, c, tmp_path / "second")
    unavailable_source.rename(a)
    assert general_config.proj_string_cr == old_projection
    result = compare(extract_features(a, "lanelet2"), extract_features(c, "lanelet2"))
    # A road alone does not secretly restore the omitted shoulder. Keep that
    # source feature in the evaluation denominator and expose the actual loss.
    assert len(extract_features(c, "lanelet2").features) == 1
    assert result["result"] == "FAIL"
    assert "101" in result["evidence"]["unmapped_source_keys"]


def test_actual_osm_geometry_mutation_is_not_restored_from_hidden_source_cache(tmp_path):
    a, b, c = [tmp_path / name for name in ("source.osm", "roads.osm", "lanes.osm")]
    make_two_lanes(a)
    convert_lanelet2_to_osm(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    for node in tree.getroot().findall("node"):
        node.set("lat", str(float(node.get("lat")) + 0.00004))
    tree.write(str(b))
    convert_osm_to_lanelet2(b, c, tmp_path / "second")
    result = compare(extract_features(a, "lanelet2"), extract_features(c, "lanelet2"))
    assert result["metrics"][0]["status"] == "FAIL"
    assert result["metrics"][0]["value"] > 3


def test_ancestral_profile_is_not_an_input_to_default_conversion(tmp_path):
    a, b, c = [tmp_path / name for name in ("source.osm", "roads.osm", "lanes.osm")]
    make_two_lanes(a)
    convert_lanelet2_to_osm(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    profile = etree.SubElement(tree.getroot(), "relation", id="-99999")
    etree.SubElement(profile, "tag", k="type", v="automap:lane_profile")
    etree.SubElement(profile, "tag", k="widths", v="nan," * 8 + "nan")
    tree.write(str(b))
    convert_osm_to_lanelet2(b, c, tmp_path / "second")
    assert len(extract_features(c, "lanelet2").features) == 1


def test_bad_ancestral_speed_and_topology_do_not_override_native_lanelet_fields(tmp_path):
    from automap_converter.api.interface import _enrich_lanelet2_topology, _parse_lanelet2
    from automap_converter.core.config.lanelet2_config import lanelet2_config

    a = tmp_path / "source.osm"
    make_two_lanes(a)
    tree = etree.parse(str(a))
    relation = tree.getroot().find("relation[@id='100']")
    for key, value in (
        ("automap:od_lane_type", '"bus"'),
        ("automap:od_speeds", '[{"max":"999","unit":"mph"}]'),
        ("automap:topology:successor", "101"),
    ):
        etree.SubElement(relation, "tag", k=key, v=value)
    tree.write(str(a))
    model = _parse_lanelet2(a, lanelet2_config)
    _enrich_lanelet2_topology(model, a, lanelet2_config, enabled=False)
    assert not model.way_relations["100"].successors
    assert not any(k.startswith("automap:") for k in model.way_relations["100"].tag_dict)


def test_public_unequal_lane_widths_are_not_overwritten_by_total_width():
    from automap_converter.formats.osm.road_semantics import road_profile

    profile = road_profile(
        {
            "highway": "residential",
            "lanes": "2",
            "oneway": "yes",
            "width": "7",
            "width:lanes": "3|4",
        }
    )
    assert profile["lane_widths_m"] == [3, 4]
    profile = road_profile(
        {"highway": "residential", "lanes": "2", "width": "8", "width:lanes": "3|4"}
    )
    assert profile["lane_widths_m"] == [3, 4]
    assert profile["unallocated_width_m"] == 1
    assert profile["assumptions"]
