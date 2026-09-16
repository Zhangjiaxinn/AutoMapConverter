from types import SimpleNamespace

import numpy as np
from lxml import etree

from automap_converter.conversion.osm_to_commonroad.converter_modules.graph_operations.road_graph._graph_functions import (
    set_points,
)
from automap_converter.conversion.osm_to_commonroad.converter_modules.graph_operations.traffic_sign_parser import (
    TrafficSignParser,
)
from automap_converter.conversion.osm_to_commonroad.support.geometry import (
    create_parallels,
    create_tilted_parallels,
    filter_points,
    get_inner_bezier_point,
)
from automap_converter.core.config.osm_config import osm_config
from automap_converter.formats.opendrive.parser.elements.geometry import Arc
from automap_converter.formats.opendrive.parser.parser import (
    _normalize_textual_map_ids,
    parse_opendrive_junction,
    parse_opendrive_road_geometry,
)
from automap_converter.formats.osm.parser import create_graph
from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion.network import (
    _incoming_junction_lane_section_index,
)
from automap_converter.validation.diagnostics.roundtrip_diagnostics import _read_osm_nodes
from automap_converter.validation.diagnostics.source_map_precheck import _load_opendrive_xml


def test_direct_junction_linked_road_is_parsed() -> None:
    opendrive = SimpleNamespace(junctions=[])
    junction = etree.fromstring(
        b'<junction id="4"><connection id="0" incomingRoad="1" '
        b'linkedRoad="2" contactPoint="start"><laneLink from="-1" to="-1"/>'
        b'</connection></junction>'
    )

    parse_opendrive_junction(opendrive, junction)

    assert opendrive.junctions[0].connections[0].connectingRoad == 2


def test_textual_opendrive_ids_are_mapped_with_references() -> None:
    root = etree.ElementTree(
        etree.fromstring(
            b'<OpenDRIVE><road id="Kalle" junction="Junction4"><link>'
            b'<successor elementType="road" elementId="2Kalle3"/>'
            b'</link></road><road id="2Kalle3" junction="-1"/>'
            b'<junction id="Junction4"><connection incomingRoad="Kalle" '
            b'connectingRoad="2Kalle3"/></junction></OpenDRIVE>'
        )
    )

    _normalize_textual_map_ids(root)

    roads = root.findall("road")
    junction = root.find("junction")
    assert roads[0].get("id") == junction.find("connection").get("incomingRoad")
    assert roads[1].get("id") == roads[0].find("link/successor").get("elementId")
    assert roads[1].get("id") == junction.find("connection").get("connectingRoad")
    assert roads[0].get("junction") == junction.get("id")


def test_zero_curvature_arc_has_line_geometry() -> None:
    arc = Arc(np.array([3.0, 4.0]), 0.0, 10.0, 0.0)

    point, tangent, curvature = arc.calc_position(2.0)

    np.testing.assert_allclose(point, [5.0, 4.0])
    assert tangent == 0.0
    assert curvature == 0.0


def test_constant_curvature_spiral_uses_arc() -> None:
    calls = []
    road = SimpleNamespace(plan_view=SimpleNamespace(add_arc=lambda *args: calls.append(args)))
    geometry = etree.fromstring(
        b'<geometry x="0" y="0" hdg="0" length="5">'
        b'<spiral curvStart="0.2" curvEnd="0.2"/></geometry>'
    )

    parse_opendrive_road_geometry(road, geometry, {"x": 0, "y": 0, "hdg": 0})

    assert calls[0][3] == 0.2


def test_short_speed_sign_does_not_raise() -> None:
    assert TrafficSignParser({"traffic_sign": "DE:274.1"}).parse_traffic_sign() == []


def test_zero_length_lane_tangent_uses_finite_straight_link() -> None:
    predecessor = SimpleNamespace(waypoints=[np.array([0.0, 0.0]), np.array([0.0, 0.0])])
    successor = SimpleNamespace(waypoints=[np.array([2.0, 0.0]), np.array([3.0, 0.0])])

    points = set_points(predecessor, successor)

    assert len(points) >= 3
    assert np.isfinite(points).all()
    np.testing.assert_allclose(points[0], [0.0, 0.0])
    np.testing.assert_allclose(points[-1], [2.0, 0.0])


def test_duplicate_waypoint_parallel_uses_next_nonzero_segment() -> None:
    points = [np.array([0.0, 0.0]), np.array([0.0, 0.0]), np.array([2.0, 0.0])]

    for left, right in (create_parallels(points, 1.0), create_tilted_parallels(points, 1.0, 2.0)):
        assert np.isfinite(left).all()
        assert np.isfinite(right).all()
        np.testing.assert_allclose(left[0], [0.0, 1.0])


def test_duplicate_bezier_control_points_remain_finite() -> None:
    origin = np.array([0.0, 0.0])
    target = np.array([2.0, 0.0])

    np.testing.assert_allclose(get_inner_bezier_point(origin, origin, target, 0.25), [0.5, 0.0])
    np.testing.assert_allclose(get_inner_bezier_point(origin, target, target, 0.25), target)


def test_namespaced_opendrive_root_passes_source_precheck(tmp_path) -> None:
    source = tmp_path / "namespaced.xodr"
    source.write_text('<OpenDRIVE xmlns="http://www.opendrive.org"/>', encoding="utf-8")

    assert _load_opendrive_xml(source, "source-opendrive-xml")["status"] == "PASS"


def test_roundtrip_node_reader_can_match_local_source_coordinates() -> None:
    root = etree.fromstring(
        b'<osm><node id="1" lat="52" lon="13">'
        b'<tag k="local_x" v="100"/><tag k="local_y" v="200"/>'
        b'</node><node id="2" lat="52" lon="14"/></osm>'
    )

    nodes, geographic = _read_osm_nodes(root, prefer_geographic=False)

    assert nodes == {"1": (100.0, 200.0)}
    assert geographic is False


def test_service_only_osm_map_is_not_discarded(tmp_path) -> None:
    source = tmp_path / "service.osm"
    source.write_text(
        '<osm version="0.6">'
        '<node id="1" lat="0" lon="0"/>'
        '<node id="2" lat="0" lon="0.001"/>'
        '<way id="3"><nd ref="1"/><nd ref="2"/>'
        '<tag k="highway" v="service"/></way></osm>',
        encoding="utf-8",
    )

    assert osm_config.ACCEPTED_HIGHWAYS_MAINLAYER["service"] is True
    assert len(create_graph(str(source)).edges) > 0


def test_long_straight_parallel_lines_filter_to_three_points() -> None:
    x = np.linspace(0.0, 3000.0, 3001)
    lines = [
        [np.array([value, offset]) for value in x]
        for offset in (0.0, 3.0)
    ]

    filtered = filter_points(lines, 0.01)

    assert all(len(line) == 3 for line in filtered)
    np.testing.assert_allclose(filtered[0][0], lines[0][0])
    np.testing.assert_allclose(filtered[0][-1], lines[0][-1])


def test_long_line_filter_accepts_mixed_2d_and_3d_points() -> None:
    line = [np.array([float(index), 0.0]) for index in range(300)]
    line[150] = np.array([150.0, 0.0, 0.0])

    filtered = filter_points([line], 0.01)

    assert len(filtered[0]) >= 3


def test_long_parallel_lines_keep_shared_bend() -> None:
    straight = [np.array([float(index), 0.0]) for index in range(300)]
    bent = [np.array([float(index), 3.0]) for index in range(300)]
    bent[150] = np.array([150.0, 13.0])

    filtered = filter_points([straight, bent], 0.1)

    assert any(np.allclose(point, bent[150]) for point in filtered[1])
    assert len(filtered[0]) == len(filtered[1])


def test_incoming_junction_section_follows_road_link_contact_end() -> None:
    lanes = SimpleNamespace(get_last_lane_section_idx=lambda: 3)
    junction_link = SimpleNamespace(elementType="junction", element_id=7)
    successor_road = SimpleNamespace(
        link=SimpleNamespace(successor=junction_link, predecessor=None),
        lanes=lanes,
    )
    predecessor_road = SimpleNamespace(
        link=SimpleNamespace(successor=None, predecessor=junction_link),
        lanes=lanes,
    )

    assert _incoming_junction_lane_section_index(successor_road, 7, 0) == 3
    assert _incoming_junction_lane_section_index(predecessor_road, 7, 3) == 0
