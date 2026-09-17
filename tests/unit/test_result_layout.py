import json
from pathlib import Path

from automap_converter.api.result_layout import (
    allocate_batch_run,
    allocate_result_run,
    write_result_manifest,
)


def test_result_runs_are_numbered_per_conversion_and_map(tmp_path: Path) -> None:
    source = tmp_path / "basic_intersection_area.osm"
    source.write_text("<osm version='0.6'/>", encoding="utf-8")

    first = allocate_result_run(tmp_path / "results", "lanelet2_to_opendrive", source, ".xodr")
    second = allocate_result_run(tmp_path / "results", "lanelet2_to_opendrive", source, ".xodr")

    assert first.directory.name == "run_001"
    assert second.directory.name == "run_002"
    assert first.target.name == "basic_intersection_area.xodr"
    assert first.directory.parent == second.directory.parent


def test_manifest_and_batch_directory_are_traceable(tmp_path: Path) -> None:
    source = tmp_path / "sample.osm"
    source.write_text("<osm version='0.6'/>", encoding="utf-8")
    result_run = allocate_result_run(tmp_path / "results", "lanelet2_to_osm", source, ".osm")

    manifest = write_result_manifest(result_run, None, result="PASS")
    payload = json.loads(manifest.read_text(encoding="utf-8"))
    first_batch = allocate_batch_run(tmp_path / "results", "lanelet2_to_osm")
    second_batch = allocate_batch_run(tmp_path / "results", "lanelet2_to_osm")

    assert payload["conversion"] == "lanelet2_to_osm"
    assert payload["result"] == "PASS"
    assert first_batch.name == "batch_001"
    assert second_batch.name == "batch_002"


def test_manifest_records_run_local_prepared_source(tmp_path: Path) -> None:
    source = tmp_path / "sample.osm"
    source.write_text("<osm version='0.6'/>", encoding="utf-8")
    result_run = allocate_result_run(tmp_path / "results", "lanelet2_to_osm", source, ".osm")
    report = result_run.directory / "sample_diagnostics.txt"
    report.write_text("diagnostics\n", encoding="utf-8")
    prepared = result_run.directory / "sample_prepared.osm"
    report.with_suffix(".json").write_text(
        json.dumps({"result": "PASS", "prepared_source": str(prepared)}),
        encoding="utf-8",
    )

    manifest = write_result_manifest(result_run, report)
    payload = json.loads(manifest.read_text(encoding="utf-8"))

    assert payload["prepared_source"] == str(prepared)
