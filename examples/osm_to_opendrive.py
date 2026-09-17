"""Convert the bundled OSM sample to OpenDRIVE with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose
from automap_converter.api.result_layout import (
    allocate_result_run,
    write_result_manifest,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "osm" / "ped_crossing.osm"
RESULTS_ROOT = PROJECT_ROOT / "data" / "results"


def main() -> None:
    run = allocate_result_run(RESULTS_ROOT, "osm_to_opendrive", SOURCE, ".xodr")
    result = convert_and_diagnose(
        SOURCE,
        run.target,
        "osm",
        "opendrive",
        diagnostics_dir=run.directory,
    )
    manifest = write_result_manifest(run, result.diagnostics_report)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")
    print(f"Manifest: {manifest}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "osm", "--to", "opendrive",
    #     "--input-dir", "path/to/osm_maps",
    #     "--output-dir", "data/results",
    # ])


if __name__ == "__main__":
    main()
