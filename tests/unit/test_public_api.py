import pytest

from automap_converter.api.converter import MapFormat, conversion_name


def test_supported_conversion_names() -> None:
    assert conversion_name("lanelet2", "opendrive") == "lanelet2_to_opendrive"
    assert conversion_name("opendrive", "lanelet2") == "opendrive_to_lanelet2"
    assert conversion_name("osm", "lanelet2") == "osm_to_lanelet2"
    assert conversion_name("lanelet2", "osm") == "lanelet2_to_osm"
    assert conversion_name("osm", "opendrive") == "osm_to_opendrive"
    assert conversion_name("opendrive", "osm") == "opendrive_to_osm"
    assert conversion_name("lanelet2", "raster") == "lanelet2_to_raster"


def test_unsupported_conversion_is_explicit() -> None:
    with pytest.raises(ValueError, match="Unsupported conversion"):
        conversion_name("lanelet2", "lanelet2")


def test_conversion_name_accepts_map_format_members() -> None:
    assert conversion_name(MapFormat.OPENDRIVE, MapFormat.OSM) == "opendrive_to_osm"
