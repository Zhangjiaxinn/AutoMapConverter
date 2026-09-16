from pathlib import Path

from automap_converter.api.settings import load_runtime_settings


def test_custom_runtime_profile_controls_supported_settings(tmp_path: Path) -> None:
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        """\
conversion:
  lanelet2_routing_enrichment: false
  osm_extract_sublayer: false
  opendrive_version: "1.5"
stage1:
  launch_viewer: true
diagnostics:
  target_conformance: false
  topology: true
  source_element_mapping: false
""",
        encoding="utf-8",
    )

    settings = load_runtime_settings(profile)

    assert settings.lanelet2_routing_enrichment is False
    assert settings.osm_extract_sublayer is False
    assert settings.opendrive_version == "1.5"
    assert settings.raster_resolution_m == 0.5
    assert settings.raster_padding_m == 5.0
    assert settings.raster_supersampling == 4
    assert settings.launch_viewer is True
    assert settings.diagnose_target_conformance is False
    assert settings.diagnose_topology is True
    assert settings.diagnose_source_element_mapping is False
