"""Convert the bundled OpenDRIVE sample to Lanelet2 with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose
from automap_converter.api.result_layout import (
    allocate_result_run,
    write_result_manifest,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "opendrive" / "commonroad_straight_road.xodr"
RESULTS_ROOT = PROJECT_ROOT / "data" / "results"


def main() -> None:
    run = allocate_result_run(RESULTS_ROOT, "opendrive_to_lanelet2", SOURCE, ".osm")
    result = convert_and_diagnose(
        SOURCE,
        run.target,
        "opendrive",
        "lanelet2",
        diagnostics_dir=run.directory,
    )
    manifest = write_result_manifest(run, result.diagnostics_report)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")
    print(f"Manifest: {manifest}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "opendrive", "--to", "lanelet2",
    #     "--input-dir", "path/to/opendrive_maps",
    #     "--output-dir", "data/results",
    # ])


if __name__ == "__main__":
    main()
