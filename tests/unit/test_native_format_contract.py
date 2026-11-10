from types import SimpleNamespace

import pytest
from lxml import etree
from pyproj import Transformer

from automap_converter.core.traceability import tags, write_frame
from automap_converter.formats.osm.connectivity import apply_connectivity
from automap_converter.validation.diagnostics._roundtrip_contract import compare
from automap_converter.validation.diagnostics._workflow_runtime import convert_osm_to_lanelet2
from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    Feature,
    FeatureSet,
    extract_features,
)


def public_osm(path, widths="3|4", speeds="30|50", poison=False):
    root = etree.fromstring(f'''<osm version="0.6">
    <node id="1" lat="48" lon="11"/><node id="2" lat="48" lon="11.001"/>
    <way id="10"><nd ref="1"/><nd ref="2"/>
    <tag k="highway" v="residential"/><tag k="oneway" v="yes"/>
    <tag k="lanes" v="2"/><tag k="width" v="{sum(map(float, widths.split("|")))}"/>
    <tag k="width:lanes" v="{widths}"/><tag k="maxspeed:lanes" v="{speeds}"/>
    </way></osm>''')
    if poison:
        way = root.find("way")
        for key, value in (
            ("automap:od_speeds", '[{"max":"999","unit":"mph"}]'),
            ("endpoint_start", "wrong-source-node"),
            ("automap:lineage", '{"ancestors":null}'),
            ("automap:chunks:automap:lineage", "not-a-number"),
            ("automap:chunks:width", "1"),
            ("automap:chunk:width:0000", "999"),
        ):
            etree.SubElement(way, "tag", k=key, v=value)
        relation = etree.SubElement(root, "relation", id="100")
        etree.SubElement(relation, "tag", k="type", v="automap:road_relative_geometry")
        etree.SubElement(relation, "tag", k="stations", v="malformed-ancestral-data")
    etree.ElementTree(root).write(str(path))


def measured_native(path):
    features = extract_features(path, "lanelet2").features
    return sorted(
        (
            f.kind,
            tuple(tuple(round(c, 6) for c in p) for p in f.points),
            round(f.width, 5),
            str(f.attributes),
            tuple(sorted((k, len(v)) for k, v in f.relations.items())),
        )
        for f in features
    )


def test_public_lane_widths_speeds_and_poisoned_metadata_are_independent(tmp_path):
    a, b = tmp_path / "plain.osm", tmp_path / "poison.osm"
    public_osm(a)
    public_osm(b, poison=True)
    first, second = tmp_path / "a.osm", tmp_path / "b.osm"
    convert_osm_to_lanelet2(a, first, tmp_path / "first")
    convert_osm_to_lanelet2(b, second, tmp_path / "second")
    assert measured_native(first) == measured_native(second)
    fs = extract_features(first, "lanelet2").features
    assert sorted(f.width for f in fs) == pytest.approx([3, 4], abs=0.02)
    assert sorted(f.attributes["rule:speed_limit"][0] for f in fs) == ["13.8889 m/s", "8.3333 m/s"]


def test_actual_public_tag_mutation_changes_output_not_ancestral_speed(tmp_path):
    source, target = tmp_path / "source.osm", tmp_path / "target.osm"
    public_osm(source, widths="3|5", speeds="50|70", poison=True)
    convert_osm_to_lanelet2(source, target, tmp_path / "run")
    features = extract_features(target, "lanelet2").features
    assert sorted(f.width for f in features) == pytest.approx([3, 5], abs=0.02)
    assert sorted(f.attributes["rule:speed_limit"][0] for f in features) == [
        "13.8889 m/s",
        "19.4444 m/s",
    ]


class LaneStub:
    def __init__(self):
        self.forward = True
        self.successors = set()
        self.predecessors = set()


def test_documented_connectivity_overrides_default_lane_pairing(tmp_path):
    path = tmp_path / "connectivity.osm"
    path.write_text("""<osm><relation id="1"><member type="way" ref="10" role="from"/>
    <member type="node" ref="2" role="via"/><member type="way" ref="20" role="to"/>
    <tag k="type" v="connectivity"/><tag k="connectivity" v="1:2|2:1"/></relation></osm>""")
    left, right = [LaneStub(), LaneStub()], [LaneStub(), LaneStub()]
    for lane in left:
        lane.successors.update(right)
    for lane in right:
        lane.predecessors.update(left)
    graph = SimpleNamespace(
        edges=[
            SimpleNamespace(
                osm_source_way_ids={"10"},
                node1=SimpleNamespace(id=1),
                node2=SimpleNamespace(id=2),
                lanes=left,
            ),
            SimpleNamespace(
                osm_source_way_ids={"20"},
                node1=SimpleNamespace(id=2),
                node2=SimpleNamespace(id=3),
                lanes=right,
            ),
        ]
    )
    assert apply_connectivity(graph, path)[0]["status"] == "PASS"
    assert left[0].successors == {right[1]}
    assert left[1].successors == {right[0]}
    assert right[0].predecessors == {left[1]}


def road_feature(key, points, width=7):
    return Feature(
        key,
        points,
        width,
        "residential",
        "no",
        ancestors=[{"format": "osm", "document": "A", "key": "source"}],
    )


def test_bidirectional_osm_subdivisions_are_a_physical_chain_not_a_cycle():
    frame = {"kind": "engineering", "id": "test", "units": "m"}
    source = FeatureSet(
        "osm", [road_feature("source", [(0, 0), (20, 0)])], frame=frame, document="A"
    )
    target = FeatureSet(
        "osm",
        [road_feature("x", [(10, 0), (0, 0)]), road_feature("y", [(20, 0), (10, 0)])],
        {("x", "y"), ("y", "x")},
        frame=frame,
        document="B",
    )
    report = compare(source, target)
    assert report["metrics"][0]["pass_rate"] == 1
    assert report["metrics"][1]["pass_rate"] == 1
    target.features[1].width = 8
    assert compare(source, target)["metrics"][1]["status"] == "FAIL"


def test_generated_connector_has_origins_but_is_not_the_source_road(tmp_path):
    frame = {"kind": "engineering", "id": "test", "units": "m"}
    source = FeatureSet(
        "osm", [road_feature("source", [(0, 0), (10, 0)])], frame=frame, document="A"
    )
    target = road_feature("junction", [(0, 0), (10, 0)])
    target.ancestors[0]["role"] = "generated_connector"
    report = compare(source, FeatureSet("osm", [target], frame=frame, document="B"))
    assert report["metrics"][0]["pass_rate"] == 0


def test_native_boundary_sections_reuse_objects_and_keep_endpoint_identity(tmp_path):
    from automap_converter.conversion.opendrive_to_commonroad.native_boundaries import (
        materialize_boundaries,
    )

    source = tmp_path / "road.xodr"
    source.write_text("""<OpenDRIVE><header revMajor="1" revMinor="6" name="test" version="1" date="" north="0" south="0" east="0" west="0"/>
    <road id="1" name="test" length="10" junction="-1"><planView><geometry s="0" x="0" y="0" hdg="0" length="10"><line/></geometry></planView>
    <lanes><laneSection s="0"><center><lane id="0" type="none" level="false"/></center><right>
    <lane id="-1" type="driving" level="false"><width sOffset="0" a="3" b="0" c="0" d="0"/></lane>
    <lane id="-2" type="driving" level="false"><width sOffset="0" a="4" b="0" c="0" d="0"/></lane>
    </right></laneSection></lanes></road></OpenDRIVE>""")
    root = etree.Element("osm", version="0.6")
    inverse = Transformer.from_crs(3857, 4326, always_xy=True)
    for nid, (x, y) in enumerate(((0, 0), (10, 0), (0, -3), (10, -3), (0, -7), (10, -7)), 1):
        lon, lat = inverse.transform(x, y)
        node = etree.SubElement(root, "node", id=str(nid), lon=str(lon), lat=str(lat))
        for k, v in (("local_x", x), ("local_y", y)):
            etree.SubElement(node, "tag", k=k, v=str(v))
    for wid, refs in ((10, (1, 2)), (11, (3, 4)), (12, (5, 6))):
        way = etree.SubElement(root, "way", id=str(wid))
        for ref in refs:
            etree.SubElement(way, "nd", ref=str(ref))
        etree.SubElement(way, "tag", k="type", v="line_thin")
        etree.SubElement(way, "tag", k="subtype", v="dashed")
    for rid, left, right, lane in ((100, 10, 11, -1), (101, 11, 12, -2)):
        relation = etree.SubElement(root, "relation", id=str(rid))
        for role, ref in (("left", left), ("right", right)):
            etree.SubElement(relation, "member", type="way", role=role, ref=str(ref))
        etree.SubElement(relation, "tag", k="type", v="lanelet")
        etree.SubElement(
            relation, "tag", k="source:opendrive:lane_locations_detailed", v=f"1,0,{lane}"
        )
    write_frame(
        root, "lanelet2", {"kind": "projected", "crs": "EPSG:3857", "units": "m", "axis": "x,y"}
    )
    target = tmp_path / "lanes.osm"
    etree.ElementTree(root).write(str(target))
    assert materialize_boundaries(source, target, "EPSG:3857")["native_lane_geometries"] == 2
    result = etree.parse(str(target)).getroot()
    assert len(result.xpath("relation[tag[@k='type' and @v='lanelet']]")) == 6
    left = result.find("relation[@id='100']/member[@role='left']").get("ref")
    assert result.find(f"way[@id='{left}']/nd").get("ref") == "1"
    assert len(extract_features(target, "lanelet2").features) == 6
    assert all(
        tags(w).get("type") == "line_thin"
        for w in result.findall("way")
        if tags(w).get("subtype") == "dashed"
    )
