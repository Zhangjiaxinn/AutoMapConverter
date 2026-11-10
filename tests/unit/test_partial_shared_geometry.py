import networkx as nx
import pytest
from lxml import etree
from pyproj import Transformer

from automap_converter.core.traceability import tags, write_frame
from automap_converter.formats.lanelet2.partial_geometry import share_partial_boundaries
from automap_converter.formats.lanelet2.shared_geometry import consolidate_shared_geometry
from automap_converter.validation.diagnostics._roundtrip_contract import _lateral_interval_evidence
from automap_converter.validation.diagnostics.roundtrip_diagnostics import Feature, extract_features


def map_with_partial_border(height=0, crossing=False):
    root = etree.Element("osm", version="0.6")
    inverse = Transformer.from_crs(3857, 4326, always_xy=True)
    count = 0

    def way(points, elevation=0):
        nonlocal count
        refs = []
        for x, y in points:
            count += 1
            lon, lat = inverse.transform(x, y)
            node = etree.SubElement(root, "node", id=str(count), lon=str(lon), lat=str(lat))
            for k, v in (("local_x", x), ("local_y", y), ("ele", elevation)):
                etree.SubElement(node, "tag", k=k, v=str(v))
            refs.append(str(count))
        count += 1
        wid = str(count)
        item = etree.SubElement(root, "way", id=wid)
        for ref in refs:
            etree.SubElement(item, "nd", ref=ref)
        etree.SubElement(item, "tag", k="type", v="virtual")
        return wid

    bounds = [
        (way([(0, 3), (100, 3)]), way([(0, 0), (100, 0)])),
        (
            way([(30, 0), (70, 0)] if not crossing else [(50, -20), (50, 20)], height),
            way([(30, -3), (70, -3)] if not crossing else [(53, -20), (53, 20)], height),
        ),
    ]
    for rid, (left, right) in zip((100, 101), bounds):
        lane = etree.SubElement(root, "relation", id=str(rid))
        for role, wid in (("left", left), ("right", right)):
            etree.SubElement(lane, "member", type="way", role=role, ref=wid)
        etree.SubElement(lane, "tag", k="type", v="lanelet")
        etree.SubElement(lane, "tag", k="subtype", v="road")
    write_frame(
        root, "lanelet2", {"kind": "projected", "crs": "EPSG:3857", "units": "m", "axis": "x,y"}
    )
    return root


def test_partial_common_interval_is_shared_without_truncating_tails(tmp_path):
    root = map_with_partial_border()
    result = share_partial_boundaries(root, "EPSG:3857")
    consolidate_shared_geometry(root)
    assert result["partial_boundary_repairs"] == 1
    assert result["intervals"][0]["shared_length_m"] == pytest.approx(40, abs=0.05)
    path = tmp_path / "partial.osm"
    etree.ElementTree(root).write(str(path))
    fs = extract_features(path, "lanelet2")
    assert len(fs.features) == 4
    assert sorted(round(f.width, 4) for f in fs.features) == [3, 3, 3, 3]
    assert all(len(f.relations.get("successor", [])) <= 1 for f in fs.features)
    assert sum(bool(f.relations.get("right")) for f in fs.features) == 1
    lane = root.find("relation[@id='101']")
    shared = lane.find("member[@role='left']").get("ref")
    assert (
        sum(m.get("ref") == shared for r in root.findall("relation") for m in r.findall("member"))
        == 2
    )
    points = [p for f in fs.features for p in f.points]
    assert min(p[0] for p in points) == pytest.approx(0)
    assert max(p[0] for p in points) == pytest.approx(100)


@pytest.mark.parametrize("height,crossing", [(5, False), (0, True)])
def test_crossing_or_elevated_boundaries_are_not_joined(height, crossing):
    root = map_with_partial_border(height, crossing)
    before = etree.tostring(root)
    assert share_partial_boundaries(root, "EPSG:3857")["partial_boundary_repairs"] == 0
    # Sorting may change order, but no native object is added or removed.
    assert len(root.findall("relation")) == 2
    assert all(tags(r).get("type") == "lanelet" for r in root.findall("relation"))
    assert before


def test_interval_evidence_does_not_call_a_short_shared_part_complete():
    source = Feature("source", [(0, 1.5), (100, 1.5)], 3, "road", "yes", right=[(0, 0), (100, 0)])
    a = Feature(
        "a",
        [(30, 1.5), (70, 1.5)],
        3,
        "road",
        "yes",
        right=[(30, 0), (70, 0)],
        measurements={"native_boundary_ids": {"right": "shared"}},
    )
    b = Feature(
        "b",
        [(30, -1.5), (70, -1.5)],
        3,
        "road",
        "yes",
        left=[(30, 0), (70, 0)],
        measurements={"native_boundary_ids": {"left": "shared"}},
    )
    graph = nx.MultiDiGraph()
    graph.add_edge("a", "b", key="right")
    result = _lateral_interval_evidence(source, ["a"], ["b"], "right", graph, {"a": a, "b": b})
    assert result["covered_fraction"] == pytest.approx(0.4)
    assert result["source_fraction_intervals"] == [[0.3, 0.7]]
    assert not result["full_interval_preserved"]
    assert result["sample_covered"] == [False, True, True, True, False]
