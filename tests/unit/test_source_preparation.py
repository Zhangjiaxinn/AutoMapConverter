from pathlib import Path

from automap_converter.validation.diagnostics.source_map_precheck import (
    detect_repairable_source_issues,
    prepare_source_if_needed,
)


def test_source_repair_writes_a_copy_only_when_a_change_is_needed(tmp_path: Path) -> None:
    source = tmp_path / "source.osm"
    source.write_text(
        "<osm version='0.6'><node id='1' lat='0.0' lon='0.0'/></osm>",
        encoding="utf-8",
    )
    prepared = tmp_path / "source_prepared.osm"

    candidates = detect_repairable_source_issues("lanelet2_to_opendrive", source)

    effective_source, records = prepare_source_if_needed(
        "lanelet2_to_opendrive",
        source,
        prepared,
    )

    assert effective_source == prepared.resolve()
    assert prepared.is_file()
    assert "version=\"1\"" in prepared.read_text(encoding="utf-8")
    assert candidates
    assert all(record["category"] == "source-repair-candidate" for record in candidates)
    assert any(record["category"] == "source-repair" for record in records)


def test_source_repair_does_not_create_a_copy_when_no_change_is_needed(tmp_path: Path) -> None:
    source = tmp_path / "source.osm"
    source.write_text(
        "<osm version='0.6'><node id='1' version='1' lat='0.0' lon='0.0'/></osm>",
        encoding="utf-8",
    )
    prepared = tmp_path / "source_prepared.osm"

    candidates = detect_repairable_source_issues("lanelet2_to_opendrive", source)

    effective_source, records = prepare_source_if_needed(
        "lanelet2_to_opendrive",
        source,
        prepared,
    )

    assert effective_source == source.resolve()
    assert candidates == []
    assert records == []
    assert not prepared.exists()
