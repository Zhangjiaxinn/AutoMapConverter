from itertools import pairwise
from types import SimpleNamespace

import numpy as np
import pytest
from lxml import etree
from scipy.integrate import quad
from shapely.geometry import LineString, Point, Polygon

from automap_converter.conversion.lanelet2_to_opendrive.reference_geometry import (
    SmoothReference,
    smooth_generated_roads,
)


@pytest.mark.parametrize(
    "points",
    [
        [(0, 0), (4, 0), (8, 2), (12, 6)],
        [(1e7, 2e7), (1e7 + 100, 2e7), (1e7 + 102, 2e7 + 3)],
        [(0, 0), (10, 0), (10, 10)],
        [(0, 0), (2, 0), (2, 0), (4, 1)],
    ],
)
def test_bounded_curve_has_common_tangents_and_real_arc_lengths(points):
    curve = SmoothReference(points)
    source = LineString(points)
    for index, coefficient in enumerate(curve.coefficients):
        samples = [curve.at_parameter(index, p)[0] for p in np.linspace(0, 1, 101)]
        assert max(Point(p).distance(source) for p in samples) <= 0.050001
        if index:
            previous_point, previous_tangent = curve.at_parameter(index - 1, 1.0)
            point, tangent = curve.at_parameter(index, 0.0)
            np.testing.assert_allclose(previous_point, point, atol=1e-8, rtol=0)
            np.testing.assert_allclose(previous_tangent, tangent, atol=1e-9)
        length = quad(
            lambda p, coefficient=coefficient: float(
                np.linalg.norm((3 * coefficient[0] * p + 2 * coefficient[1]) * p + coefficient[2])
            ),
            0,
            1,
            epsabs=1e-10,
        )[0]
        assert curve.stations[index + 1] - curve.stations[index] == pytest.approx(length, abs=1e-8)
    np.testing.assert_allclose(curve.at(0)[0], points[0], atol=1e-8, rtol=0)
    np.testing.assert_allclose(curve.at(curve.length)[0], points[-1], atol=1e-8, rtol=0)
    for old in np.linspace(0, curve.old_length, 70):
        assert curve.new_to_old(curve.old_to_new(old)) == pytest.approx(old, abs=2e-5)


def test_invalid_vertices_or_exact_reversal_are_not_silently_fitted():
    with pytest.raises(ValueError):
        SmoothReference([(0, 0), (1, 0), (0, 0)])
    with pytest.raises(ValueError):
        SmoothReference([(0, 0), (float("nan"), 1)])
    with pytest.raises(ValueError):
        SmoothReference([(0, 0), (1, 0)], max_error=0)


def add_road(root, key, points):
    road = etree.SubElement(
        root, "road", id=str(key), junction="-1", length=str(LineString(points).length)
    )
    plan = etree.SubElement(road, "planView")
    station = 0.0
    for first, second in pairwise(points):
        delta = np.subtract(second, first)
        length = float(np.linalg.norm(delta))
        geometry = etree.SubElement(
            plan,
            "geometry",
            s=str(station),
            x=str(first[0]),
            y=str(first[1]),
            hdg=str(np.arctan2(delta[1], delta[0])),
            length=str(length),
        )
        etree.SubElement(geometry, "line")
        station += length
    lanes = etree.SubElement(road, "lanes")
    etree.SubElement(lanes, "laneOffset", s="0", a="1.5", b="0", c="0", d="0")
    section = etree.SubElement(lanes, "laneSection", s="0")
    center = etree.SubElement(section, "center")
    etree.SubElement(center, "lane", id="0", type="none")
    right = etree.SubElement(section, "right")
    lane = etree.SubElement(right, "lane", id="-1", type="driving")
    etree.SubElement(lane, "width", sOffset="0", a="3", b="0", c="0", d="0")
    etree.SubElement(lane, "speed", sOffset="1", max="40", unit="mph")
    etree.SubElement(lane, "userData", code="source", value="kept")
    return road


def polynomial(elements, station, key="sOffset"):
    record = [e for e in elements if float(e.get(key)) <= station + 1e-9][-1]
    delta = station - float(record.get(key))
    return sum(
        float(record.get(name, "0")) * delta**p for p, name in enumerate(("a", "b", "c", "d"))
    )


def test_straight_road_is_byte_identical():
    root = etree.Element("OpenDRIVE")
    add_road(root, 1, [(0, 0), (10, 0)])
    before = etree.tostring(root)
    report = smooth_generated_roads(root, {}, lambda _: None)
    assert report["roads_smoothed"] == 0
    assert etree.tostring(root) == before


def test_sharp_turn_adapts_only_with_an_explicit_geometry_error_bound():
    root = etree.Element("OpenDRIVE")
    points = [(0, 0), (10, 0), (10, 10)]
    road = add_road(root, 1, points)
    report = smooth_generated_roads(root, {}, lambda _: None)
    assert report["max_reference_deviation_m"] == 0.2
    assert report["adaptive_roads"] == 1
    source = LineString(points)
    for geometry in road.findall("planView/geometry"):
        poly = geometry.find("paramPoly3")
        if poly is None:
            continue
        for p in np.linspace(0, 1, 31):
            u = sum(float(poly.get(k)) * p**i for i, k in enumerate(["aU", "bU", "cU", "dU"]))
            v = sum(float(poly.get(k)) * p**i for i, k in enumerate(["aV", "bV", "cV", "dV"]))
            h = float(geometry.get("hdg"))
            point = Point(
                float(geometry.get("x")) + u * np.cos(h) - v * np.sin(h),
                float(geometry.get("y")) + u * np.sin(h) + v * np.cos(h),
            )
            assert point.distance(source) <= 0.200001


def test_connected_roads_keep_native_links_and_common_endpoint_tangent():
    root = etree.Element("OpenDRIVE")
    a = add_road(root, 1, [(0, 0), (10, 0)])
    b = add_road(root, 2, [(10, 0), (20, 2)])
    link = etree.SubElement(a, "link")
    etree.SubElement(link, "successor", elementType="road", elementId="2", contactPoint="start")
    link = etree.SubElement(b, "link")
    etree.SubElement(link, "predecessor", elementType="road", elementId="1", contactPoint="end")
    before = [etree.tostring(r.find("link")) for r in (a, b)]
    report = smooth_generated_roads(root, {}, lambda _: None)
    assert report["roads_smoothed"] == 2
    assert [etree.tostring(r.find("link")) for r in (a, b)] == before
    end = a.findall("planView/geometry")[-1]
    poly = end.find("paramPoly3")
    dx = sum(k * float(poly.get(name)) for k, name in enumerate(("aU", "bU", "cU", "dU")))
    dy = sum(k * float(poly.get(name)) for k, name in enumerate(("aV", "bV", "cV", "dV")))
    heading = float(end.get("hdg")) + np.arctan2(dy, dx)
    assert heading == pytest.approx(float(b.find("planView/geometry").get("hdg")), abs=1e-10)
    for road in (a, b):
        assert road.find(".//speed").get("unit") == "mph"
        assert road.find(".//speed").get("max") == "40"
        assert road.find(".//userData").get("value") == "kept"


def test_gradual_width_is_resampled_on_new_curve_normal_from_source_boundaries():
    root = etree.Element("OpenDRIVE")
    points = [(0, 0), (5, 0), (10, 1)]
    road = add_road(root, 1, points)
    left = [(0, 1.5), (5, 2), (10, 3)]
    right = [(0, -1.5), (5, -2), (10, -2)]
    group = SimpleNamespace(
        longitudinal_road_lanes=None,
        road_lanes=[object()],
        longitudinal_ref_points=None,
        ref_points=points,
        length=LineString(points).length,
    )
    smooth_generated_roads(root, {1: group}, lambda _: (left, right))
    curve = SmoothReference(points)
    polygon = Polygon(left + list(reversed(right)))
    for ratio in [0.25, 0.35, 0.5, 0.65, 0.75]:
        station = ratio * curve.length
        point, tangent = curve.at(station)
        normal = np.array([-tangent[1], tangent[0]])
        measured = polygon.intersection(
            LineString([point - 30 * normal, point + 30 * normal])
        ).length
        actual = polynomial(road.findall('.//lane[@id="-1"]/width'), station)
        assert actual == pytest.approx(measured, abs=0.025)
    assert max(float(w.get("a")) for w in road.findall(".//width")) > 4
    assert all(float(w.get("a")) > 0 for w in road.findall(".//width"))


def test_lateral_signal_at_road_endpoint_is_not_moved_by_joint_tangent_fitting():
    root = etree.Element("OpenDRIVE")
    a = add_road(root, 1, [(0, 0), (10, 0)])
    b = add_road(root, 2, [(10, 0), (20, 2)])
    link = etree.SubElement(a, "link")
    etree.SubElement(link, "successor", elementType="road", elementId="2", contactPoint="start")
    signals = etree.SubElement(b, "signals")
    etree.SubElement(signals, "signal", s="0", t="1")
    original = etree.tostring(signals)
    smooth_generated_roads(root, {}, lambda _: None)
    assert etree.tostring(signals) == original
    assert float(b.find("planView/geometry").get("hdg")) == pytest.approx(np.arctan2(2, 10))


def test_signal_object_and_corner_keep_world_positions_after_station_change():
    root = etree.Element("OpenDRIVE")
    points = [(0, 0), (4, 0), (8, 2)]
    road = add_road(root, 1, points)
    objects = etree.SubElement(road, "objects")
    obj = etree.SubElement(objects, "object", s="2", t="1", hdg=".2")
    outlines = etree.SubElement(obj, "outlines")
    outline = etree.SubElement(outlines, "outline")
    corner = etree.SubElement(outline, "cornerRoad", s="3", t="2")
    signals = etree.SubElement(road, "signals")
    signal = etree.SubElement(signals, "signal", s="3.5", t="-2", hOffset=".1")
    report = smooth_generated_roads(root, {}, lambda _: None)
    curve = SmoothReference(
        points,
        0.2 if report["adaptive_roads"] else 0.05,
        round_corners=bool(report["rounded_roads"]),
    )
    for element, expected in [(obj, (2, 1)), (corner, (3, 2)), (signal, (3.5, -2))]:
        point, tangent = curve.at(float(element.get("s")))
        actual = point + float(element.get("t")) * np.array([-tangent[1], tangent[0]])
        np.testing.assert_allclose(actual, expected, atol=1e-6)
    point, tangent = curve.at(float(obj.get("s")))
    assert float(obj.get("hdg")) + np.arctan2(tangent[1], tangent[0]) == pytest.approx(
        0.2, abs=1e-8
    )
