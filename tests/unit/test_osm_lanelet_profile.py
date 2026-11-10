from dataclasses import replace

import pytest
from lxml import etree
from test_osm_lane_profiles import make_two_lanes

from automap_converter.api.settings import (
    current_runtime_settings,
    load_runtime_settings,
    use_runtime_settings,
)
from automap_converter.core.traceability import tags
from automap_converter.formats.osm.lanelet_profile import declared_lanes
from automap_converter.validation.diagnostics._roundtrip_contract import compare
from automap_converter.validation.diagnostics._workflow_runtime import (
    convert_lanelet2_to_osm,
    convert_osm_to_lanelet2,
)
from automap_converter.validation.diagnostics.roundtrip_diagnostics import extract_features


def export_profile(source, target, folder):
    with use_runtime_settings(replace(current_runtime_settings(), osm_profile="lanelet2-v1")):
        convert_lanelet2_to_osm(source, target, folder)


def geometry(path):
    return sorted(
        (f.kind, tuple(tuple(round(v, 5) for v in p) for p in f.points), round(f.width, 5))
        for f in extract_features(path, "lanelet2").features
    )


def test_profile_has_native_roles_and_no_source_reconstruction_payload(tmp_path):
    source, middle, target = [tmp_path / n for n in ("source.osm", "middle.osm", "target.osm")]
    make_two_lanes(source)
    export_profile(source, middle, tmp_path / "first")
    root = etree.parse(str(middle)).getroot()
    selected = declared_lanes(root)
    assert selected
    assert len(root.xpath("relation[tag[@k='type' and @v='lanelet']]")) == 2
    assert not root.xpath(
        ".//tag[@k='automap:osm_road_model' or @k='automap:od_speeds' or @k='endpoint_start']"
    )
    assert not root.xpath(
        "relation[tag[@v='automap:lane_profile' or @v='automap:road_relative_geometry']]"
    )
    unavailable = source.with_suffix(".unavailable")
    source.rename(unavailable)
    convert_osm_to_lanelet2(middle, target, tmp_path / "second")
    source = unavailable
    result = compare(extract_features(source, "lanelet2"), extract_features(target, "lanelet2"))
    assert result["result"] == "PASS"
    features = extract_features(target, "lanelet2").features
    assert sorted(f.kind for f in features) == ["road", "shoulder"]


def test_profile_provenance_removal_and_boundary_mutation(tmp_path):
    a, b, c, d = [tmp_path / n for n in ("a.osm", "b.osm", "c.osm", "d.osm")]
    make_two_lanes(a)
    export_profile(a, b, tmp_path / "first")
    convert_osm_to_lanelet2(b, c, tmp_path / "second")
    tree = etree.parse(str(b))
    root = tree.getroot()
    for tag in list(root.findall(".//tag")):
        key = tag.get("k", "")
        if key.startswith(("source:", "lanelet2:", "automap:chunk", "automap:lineage")):
            tag.getparent().remove(tag)
    tree.write(str(b))
    convert_osm_to_lanelet2(b, d, tmp_path / "no_provenance")
    assert geometry(c) == geometry(d)
    selected = declared_lanes(root)
    for kind, key in selected:
        if kind == "node":
            node = root.find(f"node[@id='{key}']")
            node.set("lat", str(float(node.get("lat")) + 0.00005))
    tree.write(str(b))
    with pytest.raises(ValueError, match="geometry conflicts"):
        convert_osm_to_lanelet2(b, d, tmp_path / "inconsistent")
    for node in root.findall("node"):
        if ("node", node.get("id")) not in selected:
            node.set("lat", str(float(node.get("lat")) + 0.00005))
    tree.write(str(b))
    convert_osm_to_lanelet2(b, d, tmp_path / "moved")
    assert geometry(c) != geometry(d)


def test_bad_profile_is_rejected_not_silently_restored(tmp_path):
    a, b = tmp_path / "a.osm", tmp_path / "b.osm"
    make_two_lanes(a)
    export_profile(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    declaration = next(
        r
        for r in tree.getroot().findall("relation")
        if tags(r).get("type") == "automap:map_profile"
    )
    declaration.find("tag[@k='automap:profile']").set("v", "unknown-v999")
    tree.write(str(b))
    with pytest.raises(ValueError, match="Unsupported OSM detail profile"):
        convert_osm_to_lanelet2(b, tmp_path / "c.osm", tmp_path / "bad")


def test_public_width_conflict_is_rejected_not_overridden(tmp_path):
    a, b = tmp_path / "a.osm", tmp_path / "b.osm"
    make_two_lanes(a)
    export_profile(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    way = next(w for w in tree.getroot().findall("way") if tags(w).get("highway"))
    way.find("tag[@k='width']").set("v", "9")
    tree.write(str(b))
    with pytest.raises(ValueError, match="width conflicts"):
        convert_osm_to_lanelet2(b, tmp_path / "c.osm", tmp_path / "bad")


def test_active_public_speed_conflict_is_rejected(tmp_path):
    a, b = tmp_path / "a.osm", tmp_path / "b.osm"
    make_two_lanes(a)
    tree = etree.parse(str(a))
    root = tree.getroot()
    rule = etree.SubElement(root, "relation", id="500")
    for k, v in (
        ("type", "regulatory_element"),
        ("subtype", "speed_limit"),
        ("sign_type", "50 km/h"),
    ):
        etree.SubElement(rule, "tag", k=k, v=v)
    etree.SubElement(
        root.find("relation[@id='100']"),
        "member",
        type="relation",
        ref="500",
        role="regulatory_element",
    )
    tree.write(str(a))
    export_profile(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    speed = tree.getroot().find(".//way/tag[@k='maxspeed:lanes']")
    assert speed is not None
    speed.set("v", "30")
    tree.write(str(b))
    with pytest.raises(ValueError, match="speed conflicts"):
        convert_osm_to_lanelet2(b, tmp_path / "c.osm", tmp_path / "bad")


@pytest.mark.parametrize("problem", ["dangling", "duplicate_tag", "duplicate_id"])
def test_invalid_native_objects_in_profile_are_rejected(tmp_path, problem):
    a, b = tmp_path / "a.osm", tmp_path / "b.osm"
    make_two_lanes(a)
    export_profile(a, b, tmp_path / "first")
    tree = etree.parse(str(b))
    root = tree.getroot()
    lane = root.xpath("relation[tag[@k='type' and @v='lanelet']]")[0]
    if problem == "dangling":
        lane.find("member[@role='left']").set("ref", "999999999")
    elif problem == "duplicate_tag":
        etree.SubElement(lane, "tag", k="type", v="lanelet")
    else:
        etree.SubElement(root, "relation", id=lane.get("id"))
    tree.write(str(b))
    with pytest.raises(ValueError):
        convert_osm_to_lanelet2(b, tmp_path / "c.osm", tmp_path / "bad")


def test_profile_is_an_explicit_configuration_not_the_public_default(tmp_path):
    assert current_runtime_settings().osm_profile == "public"
    config = tmp_path / "profile.yaml"
    config.write_text("conversion:\n  osm_profile: lanelet2-v1\n")
    assert load_runtime_settings(config).osm_profile == "lanelet2-v1"
    config.write_text("conversion:\n  osm_profile: unknown\n")
    with pytest.raises(ValueError, match="osm_profile"):
        load_runtime_settings(config)
