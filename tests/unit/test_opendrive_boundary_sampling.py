import pytest
from lxml import etree
from shapely.geometry import LineString, Point

from automap_converter.conversion.lanelet2_to_opendrive.boundary_sampling import local_boundary_hit
from automap_converter.conversion.lanelet2_to_opendrive.converter import Lanelet2OpendriveConverter


def test_remote_return_leg_is_not_misread_as_the_width_of_a_skew_port():
    boundary = LineString([(1, -3), (10, -3), (23, 10), (10, 23), (-1, 23)])
    normal = LineString([(0, -200), (0, 200)])
    assert boundary.intersection(normal).y == 23
    assert local_boundary_hit(boundary, normal, Point(0, 0)) is None
    assert local_boundary_hit(boundary, LineString([(5, -200), (5, 200)]), Point(5, 0)).y == -3


def test_genuinely_wide_lanes_and_small_port_roundoff_are_retained():
    normal = LineString([(0, -200), (0, 200)])
    assert local_boundary_hit(LineString([(-10, -60), (30, -60)]), normal, Point(0, 0)).y == -60
    assert local_boundary_hit(LineString([(-1e-7, -3), (10, -3)]), normal, Point(0, 0)).y == -3


def test_missing_port_sample_uses_a_valid_neighbour_not_a_remote_intersection():
    converter = object.__new__(Lanelet2OpendriveConverter)
    converter.step_size = 0.5
    left = [(0, 0), (10, 0)]
    right = [(1, -3), (10, -3), (23, 10), (10, 23), (-1, 23)]
    converter.get_oriented_lanelet_boundaries = lambda _: (left, right)
    samples = converter.compute_lane_width_samples(object(), left)
    assert samples[0][0] > 0
    assert max(width for _, width in samples) == pytest.approx(3)
    lane = etree.Element("lane")
    converter.write_variable_lane_width(lane, samples, object())
    assert float(lane.find("width").get("a")) == pytest.approx(3)
