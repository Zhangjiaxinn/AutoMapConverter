import json
from pathlib import Path

from automap_converter.validation.diagnostics._workflow_runtime import (
    ConversionSpec,
    Stage1Check,
    run_single,
)


def _spec(tmp_path: Path, convert):
    return ConversionSpec(
        name="test_conversion",
        source_dir=tmp_path,
        output_root=tmp_path,
        source_suffix=".osm",
        target_suffix=".osm",
        convert=convert,
        validate=lambda _: [
            Stage1Check(category="target", status="PASS", summary="Target created.")
        ],
        validate_source=lambda _conversion, _source: [],
    )


def test_successful_run_keeps_only_combined_diagnostics(tmp_path: Path) -> None:
    source = tmp_path / "source.osm"
    target = tmp_path / "target.osm"
    source.write_text("<osm version='0.6'/>", encoding="utf-8")

    def convert(_source: Path, output: Path, _intermediate: Path) -> None:
        output.write_text("<osm version='0.6'/>", encoding="utf-8")

    report = run_single(_spec(tmp_path, convert), source, target, tmp_path)

    assert report == tmp_path / "target_diagnostics.json"
    assert report.is_file()
    assert not (tmp_path / "target_stage1_diagnostics.json").exists()


def test_failed_run_promotes_stage1_to_standard_diagnostics_name(tmp_path: Path) -> None:
    source = tmp_path / "source.osm"
    target = tmp_path / "target.osm"
    source.write_text("<osm version='0.6'/>", encoding="utf-8")

    def convert(_source: Path, _output: Path, _intermediate: Path) -> None:
        raise RuntimeError("conversion failed")

    report = run_single(_spec(tmp_path, convert), source, target, tmp_path)
    payload = json.loads(report.read_text(encoding="utf-8"))

    assert report == tmp_path / "target_diagnostics.json"
    assert payload["stage"] == "stage1_acceptance"
    assert payload["result"] == "ERROR"
    assert not (tmp_path / "target_stage1_diagnostics.json").exists()
