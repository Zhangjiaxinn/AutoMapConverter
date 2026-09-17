from automap_converter.formats.lanelet2.osm_model import OSM, Node, Relation, Way


def test_osm_elements_are_serialized_by_type_and_numeric_id() -> None:
    osm = OSM()
    osm.add_node(Node(id_=-1, lat=0, lon=0))
    osm.add_node(Node(id_=-10, lat=0, lon=0))
    osm.add_node(Node(id_=2, lat=0, lon=0))
    osm.add_way(Way(id_=-2, nodes=[-10, -1]))
    osm.add_way(Way(id_=-20, nodes=[-10, -1]))
    osm.add_relation(Relation(id_=-3))
    osm.add_relation(Relation(id_=-30))

    elements = list(osm.serialize_to_xml())

    assert [(element.tag, element.get("id")) for element in elements] == [
        ("node", "-1"),
        ("node", "-10"),
        ("node", "2"),
        ("way", "-2"),
        ("way", "-20"),
        ("relation", "-3"),
        ("relation", "-30"),
    ]
