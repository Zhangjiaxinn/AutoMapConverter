from types import SimpleNamespace

from commonroad.scenario.traffic_sign import TrafficSignIDZamunda

from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion import utils
from automap_converter.conversion.osm_to_commonroad.converter_modules.cr_operations.cleanup import (
    remove_unconnected_lanes,
)
from automap_converter.conversion.osm_to_commonroad.converter_modules.graph_operations import (
    lane_linker,
)
from automap_converter.conversion.osm_to_commonroad.converter_modules.graph_operations.lane_linker import (
    CombinedEdge,
    get_forbidden_turns,
    rotate_restrictions,
)


def test_unknown_opendrive_signal_country_uses_generic_sign_catalog() -> None:
    assert utils.get_traffic_sign_enum_from_country("STP") is TrafficSignIDZamunda


def test_empty_lanelet_network_is_safe_to_clean() -> None:
    scenario = SimpleNamespace(lanelet_network=SimpleNamespace(lanelets=[]))

    remove_unconnected_lanes(scenario)


def test_combined_edge_turn_restrictions_use_incoming_edge() -> None:
    junction = object()
    edge1 = SimpleNamespace(
        node1=junction,
        node2=object(),
        forward_restrictions=set(),
        backward_restrictions=set(),
    )
    edge2 = SimpleNamespace(
        node1=object(),
        node2=junction,
        forward_restrictions={"no_straight_on"},
        backward_restrictions=set(),
    )
    combined = CombinedEdge(edge1, edge2, junction)

    assert get_forbidden_turns(combined, junction)["through"] is True

    rotate_restrictions(combined, junction, "left")

    assert edge2.forward_restrictions == {"no_left_turn"}


def test_third_degree_linking_uses_combined_logical_edge_count(monkeypatch) -> None:
    node = SimpleNamespace(get_degree=lambda: 4)
    edges = [object(), object(), object()]

    monkeypatch.setattr(lane_linker, "get_incomings_outgoings", lambda edge, node: ([], []))
    monkeypatch.setattr(lane_linker, "get_turnlane_usefull", lambda *args: (False, None))
    monkeypatch.setattr(lane_linker, "set_turnlane_borders", lambda *args: (-1, 0, -1, 0))
    monkeypatch.setattr(lane_linker, "linkleft_interval", lambda *args: None)
    monkeypatch.setattr(lane_linker, "link_right_interval", lambda *args: None)
    monkeypatch.setattr(lane_linker, "link_skipped_lanes", lambda node: None)

    lane_linker.link_third_degree(node, edges)
