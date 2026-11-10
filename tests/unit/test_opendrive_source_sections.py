from types import SimpleNamespace

import pytest
from lxml import etree

from automap_converter.conversion.lanelet2_to_opendrive.converter import Lanelet2OpendriveConverter


def converter():
    return object.__new__(Lanelet2OpendriveConverter)


def group():
    a, b = SimpleNamespace(id_="first"), SimpleNamespace(id_="second")
    return SimpleNamespace(
        longitudinal_road_lanes=[[a], [b]],
        longitudinal_ref_points=[[(0.0, 0.0), (10.0, 0.0)], [(10.0, 0.0), (20.0, 0.0)]],
        length=20.0,
        road_lanes=[a],
    )


def test_merged_source_sections_follow_the_road_split_and_keep_exact_endpoints():
    instance = converter()
    front, back = instance._partition_longitudinal_sources(group(), 12.0)
    assert [[lane.id_ for lane in lanes] for lanes in front[0]] == [["first"], ["second"]]
    assert [[lane.id_ for lane in lanes] for lanes in back[0]] == [["second"]]
    assert front[1][-1][-1] == (12.0, 0.0)
    assert back[1][0] == [(12.0, 0.0), (20.0, 0.0)]


def test_split_at_a_section_boundary_does_not_inherit_the_first_section():
    instance = converter()
    front, back = instance._partition_longitudinal_sources(group(), 10.0)
    instance._set_longitudinal_sources(group_front := group(), front)
    instance._set_longitudinal_sources(group_back := group(), back)
    assert instance._source_lanelet_ids(group_front) == ["first"]
    assert instance._source_lanelet_ids(group_back) == ["second"]
    assert group_back.road_lanes[0].id_ == "second"


def test_back_only_source_keeps_live_xml_instead_of_a_detached_front_lane():
    instance = converter()
    front, back = instance._partition_longitudinal_sources(group(), 10.0)
    instance._set_longitudinal_sources(group_front := group(), front)
    instance._set_longitudinal_sources(group_back := group(), back)
    group_back.od_road_id = 9
    instance.lanelet_to_road = {"first": 1, "second": 1}
    instance.lanelet_to_lane = {"first": -1, "second": -1}
    live = etree.Element("lane")
    instance.lanelet_to_lane_xml = {"second": live}
    assert instance._retain_split_back_mapping(group_front, group_back, "second")
    assert instance.lanelet_to_road["second"] == 9
    assert instance.lanelet_to_lane_xml["second"] is live
    assert not instance._retain_split_back_mapping(group_front, group_back, "first")


@pytest.mark.parametrize(
    "points, expected",
    [
        ([(10.0, 0.0), (20.0, 0.0)], True),
        ([(10.1, 0.0), (20.0, 0.0)], True),
        ([(12.0, 0.0), (20.0, 0.0)], False),
        ([(10.0, 0.0), (1.0, 0.0)], False),
        ([(10.0, 0.0), (10.0, 10.0)], False),
    ],
)
def test_connector_merge_does_not_bridge_gaps_or_reverse_a_directed_lane(points, expected):
    first = SimpleNamespace(ref_points=[(0.0, 0.0), (10.0, 0.0)])
    second = SimpleNamespace(ref_points=points)
    assert converter().are_road_groups_direction_continuous(first, second) is expected
