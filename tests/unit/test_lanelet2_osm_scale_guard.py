from automap_converter.conversion.lanelet2_to_osm.converter import Lanelet2OSMConverter
from automap_converter.formats.lanelet2.map import Node, OSMLanelet, Way


def test_endpoint_extension_skips_oversized_draft_network() -> None:
    converter = Lanelet2OSMConverter(endpoint_extension_max_draft_ways=1)

    extended = converter._extend_dangling_draft_way_endpoints_by_true_intersection(
        [object(), object()]
    )

    assert extended == 0
    assert converter.endpoint_extension_skip_reason == "draft_way_count_exceeds_limit"


def test_local_xy_without_georeference_produces_non_degenerate_osm_coordinates() -> None:
    source = OSMLanelet()
    source.add_node(Node("1", "", "", local_x=100.0, local_y=200.0))
    source.add_node(Node("2", "", "", local_x=110.0, local_y=200.0))
    source.add_way(Way("10", ["1", "2"], {}))
    converter = Lanelet2OSMConverter()
    converter._source = source
    converter._initialize_xy_projection_origin()

    geometry = converter._build_way_geometry(source.ways["10"])
    node = converter._make_osm_node_from_source(source.nodes["2"])

    assert geometry["xy"] == [(100.0, 200.0), (110.0, 200.0)]
    assert geometry["ll"][0] != geometry["ll"][1]
    assert geometry["ll"][0] != (0.0, 0.0)
    assert float(node.lon) == round(110.0 / 111320.0, 8)


def test_source_way_midpoint_prefers_valid_wgs84_over_absolute_local_xy() -> None:
    source = OSMLanelet()
    source.add_node(
        Node("1", 37.0, -122.0, local_x=-13_580_000.0, local_y=4_439_000.0)
    )
    source.add_node(
        Node("2", 37.2, -121.8, local_x=-13_570_000.0, local_y=4_449_000.0)
    )
    source.add_way(Way("10", ["1", "2"], {"type": "traffic_light"}))
    converter = Lanelet2OSMConverter()
    converter._source = source
    converter._initialize_xy_projection_origin()

    assert converter._source_way_midpoint_latlon("10") == (37.1, -121.9)
