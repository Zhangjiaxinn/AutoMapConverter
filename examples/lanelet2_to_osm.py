"""Convert the bundled Lanelet2 sample to OSM with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose
from automap_converter.api.result_layout import (
    allocate_result_run,
    write_result_manifest,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "lanelet2" / "basic_intersection_area.osm"
RESULTS_ROOT = PROJECT_ROOT / "data" / "results"


def main() -> None:
    run = allocate_result_run(RESULTS_ROOT, "lanelet2_to_osm", SOURCE, ".osm")
    result = convert_and_diagnose(
        SOURCE,
        run.target,
        "lanelet2",
        "osm",
        diagnostics_dir=run.directory,
    )
    manifest = write_result_manifest(run, result.diagnostics_report)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")
    print(f"Manifest: {manifest}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "lanelet2", "--to", "osm",
    #     "--input-dir", "path/to/lanelet2_maps",
    #     "--output-dir", "data/results",
    # ])


if __name__ == "__main__":
    main()
