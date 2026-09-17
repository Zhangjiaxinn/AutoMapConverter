from collections import Counter

from lxml import etree

from automap_converter.validation.diagnostics._semantic_engine import (
    OsmStats,
    _collect_opendrive_stats,
    _opendrive_center_lane_conformance_check,
    _merge_status_counts,
    _result_from_counts,
    _lanelet_detail,
    _match_highways_to_lanelets,
    _target_source_key_adjacency_edges,
)
from automap_converter.validation.diagnostics.lanelet2_opendrive_diagnostics import (
    Lanelet2OpenDriveDiagnostics,
    SourceStats,
    TargetStats,
)
from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    _extract_lanelet2,
)


def test_reversed_shared_boundary_uses_lanelet_travel_direction():
    stats = OsmStats()
    stats.way_nodes = {
        "shared": ["1", "2"],
        "outer": ["4", "3"],
    }
    stats.node_xy = {
        "1": (0.0, 0.0),
        "2": (10.0, 0.0),
        "3": (0.0, 3.0),
        "4": (10.0, 3.0),
    }
    relation = etree.fromstring(
        b'<relation id="5"><member role="left" ref="shared"/>'
        b'<member role="right" ref="outer"/></relation>'
    )

    detail = _lanelet_detail(relation, stats, Counter({"shared": 2, "outer": 1}))

    assert detail["reversed_boundary"] == "left"
    assert detail["center_start_xy"] == (10.0, 1.5)
    assert detail["center_end_xy"] == (0.0, 1.5)
    assert detail["start_key"] == "2|4"
    assert detail["end_key"] == "1|3"


def test_target_metadata_accepts_both_regulatory_and_way_provenance():
    signal = etree.fromstring(
        b'<signal><userData code="lanelet2:source_kind" value="regulatory_element"/>'
        b'<userData code="lanelet2:source_id" value="31"/>'
        b'<userData code="lanelet2:source_way_id" value="11"/>'
        b'<userData code="lanelet2:regulatory_element" value="32"/></signal>'
    )
    stats = TargetStats()

    Lanelet2OpenDriveDiagnostics._collect_target_source_metadata(signal, stats)

    assert stats.mapped_regulatory_ids == {"31", "32"}
    assert stats.mapped_way_ids == {"11"}


def test_unknown_regulatory_signal_type_remains_visible_as_warning():
    diagnostics = Lanelet2OpenDriveDiagnostics("source.osm", "target.xodr")
    diagnostics.source_stats = SourceStats(
        regulatory_ids={"31"}, convertible_regulatory_ids={"31"}
    )
    diagnostics.target_stats = TargetStats(
        mapped_regulatory_ids={"31"},
        unresolved_regulatory_signal_ids={"31"},
    )

    diagnostics._compare_source_regulatory_elements()

    assert [(item.category, item.status) for item in diagnostics.results] == [
        ("5-regulatory-elements", "PASS"),
        ("5b-regulatory-signal-type", "WARN"),
    ]


def test_roundtrip_features_follow_reversed_shared_boundary(tmp_path):
    source = tmp_path / "shared.osm"
    source.write_text(
        """<osm>
        <node id="1" lon="0" lat="0"/><node id="2" lon="10" lat="0"/>
        <node id="3" lon="0" lat="3"/><node id="4" lon="10" lat="3"/>
        <node id="5" lon="0" lat="-3"/><node id="6" lon="10" lat="-3"/>
        <way id="shared"><nd ref="1"/><nd ref="2"/></way>
        <way id="positive"><nd ref="4"/><nd ref="3"/></way>
        <way id="negative"><nd ref="5"/><nd ref="6"/></way>
        <relation id="positive_lane">
          <member role="left" ref="shared"/><member role="right" ref="positive"/>
          <tag k="type" v="lanelet"/>
        </relation>
        <relation id="negative_lane">
          <member role="left" ref="negative"/><member role="right" ref="shared"/>
          <tag k="type" v="lanelet"/>
        </relation>
        </osm>""",
        encoding="utf-8",
    )

    features = _extract_lanelet2(source).features
    positive = next(feature for feature in features if feature.key == "positive_lane")

    assert positive.points[0] == (10.0, 1.5)
    assert positive.points[-1] == (0.0, 1.5)


def test_topology_accepts_small_gap_only_when_boundary_node_is_shared():
    source = {
        "lanelet_id": "1",
        "source_opendrive_keys": ["road_a|0|-1"],
        "end_key": "10|11",
        "center_end_xy": (0.0, 0.0),
    }
    connected = {
        "lanelet_id": "2",
        "source_opendrive_keys": ["road_b|0|-1"],
        "start_key": "10|12",
        "center_start_xy": (0.8, 0.0),
    }
    disconnected = {
        "lanelet_id": "3",
        "source_opendrive_keys": ["road_c|0|-1"],
        "start_key": "20|21",
        "center_start_xy": (0.8, 0.0),
    }

    edges = _target_source_key_adjacency_edges(
        [source, connected, disconnected]
    )

    assert "road_a|0|-1->road_b|0|-1" in edges
    assert "road_a|0|-1->road_c|0|-1" not in edges


def test_topology_accepts_split_tip_with_shared_boundary_node():
    source = {
        "lanelet_id": "1",
        "source_opendrive_keys": ["road_a|0|-1"],
        "end_key": "10|11",
        "center_end_xy": (0.0, 0.0),
    }
    split = {
        "lanelet_id": "2",
        "source_opendrive_keys": ["road_b|0|-2"],
        "start_key": "11|12",
        "center_start_xy": (3.0, 0.0),
    }

    edges = _target_source_key_adjacency_edges([source, split])

    assert "road_a|0|-1->road_b|0|-2" in edges


def test_highway_mapping_checks_long_way_geometry_after_midpoint_miss():
    source = OsmStats()
    source.node_xy = {"a": (0.0, 0.0), "b": (100.0, 0.0)}
    source.highway_details = [
        {
            "way_id": "1",
            "highway": "unclassified",
            "nodes": ["a", "b"],
            "mid_xy": (50.0, 0.0),
        }
    ]
    target = OsmStats()
    target.lanelet_details = [
        {
            "lanelet_id": "2",
            "center_mid_xy": (90.0, 0.0),
            "issues": [],
        }
    ]

    matches, unmatched = _match_highways_to_lanelets(source, target)

    assert not unmatched
    assert matches[0]["mapping_status"] == "geometry_polyline_match"
    assert matches[0]["distance_m"] == 0.0


def test_junction_topology_matches_converter_contact_point_direction():
    root = etree.fromstring(
        b"""<OpenDRIVE>
        <road id="1" length="20" junction="7"><link>
          <successor elementType="junction" elementId="7"/>
        </link><lanes>
          <laneSection s="0"><left><lane id="1" type="driving"/></left>
            <right><lane id="-1" type="driving"/></right></laneSection>
          <laneSection s="10"><left><lane id="1" type="driving"/></left>
            <right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road>
        <road id="2" length="20" junction="7"><lanes>
          <laneSection s="0"><left><lane id="1" type="driving"/></left>
            <right><lane id="-1" type="driving"/></right></laneSection>
          <laneSection s="10"><left><lane id="1" type="driving"/></left>
            <right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road>
        <junction id="7">
          <connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start">
            <laneLink from="-1" to="-1"/><laneLink from="1" to="1"/>
          </connection>
          <connection id="2" incomingRoad="1" connectingRoad="2" contactPoint="end">
            <laneLink from="-1" to="-1"/><laneLink from="1" to="1"/>
          </connection>
        </junction>
        </OpenDRIVE>"""
    )

    edges = _collect_opendrive_stats(root).topology_edges

    assert "1|1|-1->2|0|-1" in edges
    assert "2|0|1->1|1|1" in edges
    assert "2|1|-1->1|1|-1" in edges
    assert "1|1|1->2|1|1" in edges


def test_junction_topology_inverts_for_left_hand_traffic():
    root = etree.fromstring(
        b"""<OpenDRIVE>
        <road id="1" rule="LHT" length="10" junction="7"><lanes>
          <laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road>
        <road id="2" length="10" junction="7"><lanes>
          <laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road>
        <junction id="7"><connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start">
          <laneLink from="-1" to="-1"/>
        </connection></junction>
        </OpenDRIVE>"""
    )

    edges = _collect_opendrive_stats(root).topology_edges

    assert "2|0|-1->1|0|-1" in edges


def test_center_lane_is_reported_but_not_counted_as_driving_inventory():
    root = etree.fromstring(
        b"""<OpenDRIVE><road id="1" length="10" junction="-1"><lanes>
        <laneSection s="0"><center><lane id="0" type="driving"/></center>
        <right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road></OpenDRIVE>"""
    )

    stats = _collect_opendrive_stats(root)
    check = _opendrive_center_lane_conformance_check(stats)

    assert stats.driving_lanes == 1
    assert stats.driving_lane_keys == {"1|0|-1"}
    assert check.status == "WARN"
    assert check.details[0]["location"].endswith("center/lane[@id='0']")


def test_final_result_includes_target_loader_failure():
    stage2 = {"pass": 5, "warn": 0, "fail": 0, "review": 0, "skip": 0}
    acceptance = {"pass": 0, "warn": 0, "fail": 1, "review": 0, "skip": 1}

    result = _result_from_counts(_merge_status_counts(stage2, acceptance))

    assert result == "FAIL"


def test_junction_lane_link_must_exist_at_road_contact_section():
    root = etree.fromstring(
        b"""<OpenDRIVE>
        <road id="1" length="20" junction="7"><link>
          <predecessor elementType="junction" elementId="7"/>
        </link><lanes>
          <laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection>
          <laneSection s="10"><right><lane id="-2" type="driving"/></right></laneSection>
        </lanes></road>
        <road id="2" length="10" junction="7"><lanes>
          <laneSection s="0"><right><lane id="-1" type="driving"/></right></laneSection>
        </lanes></road>
        <junction id="7"><connection id="1" incomingRoad="1" connectingRoad="2" contactPoint="start">
          <laneLink from="-2" to="-1"/>
        </connection></junction>
        </OpenDRIVE>"""
    )

    stats = _collect_opendrive_stats(root)

    assert not stats.topology_edges
    assert "absent at incomingRoad 1 junction contact section" in stats.invalid_lane_link_details[0]["issues"][0]
