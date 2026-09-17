"""Convert the bundled Lanelet2 sample to semantic raster products."""

from pathlib import Path

from automap_converter import convert_and_diagnose
from automap_converter.api.result_layout import allocate_result_run, write_result_manifest

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "lanelet2" / "basic_intersection_area.osm"
RESULTS_ROOT = PROJECT_ROOT / "data" / "results"


def main() -> None:
    run = allocate_result_run(RESULTS_ROOT, "lanelet2_to_raster", SOURCE, ".tif")
    result = convert_and_diagnose(
        SOURCE,
        run.target,
        "lanelet2",
        "raster",
        diagnostics_dir=run.directory,
    )
    manifest = write_result_manifest(run, result.diagnostics_report)
    print(f"Semantic raster: {result.target}")
    print(f"Occupancy raster: {result.artifacts['occupancy']}")
    print(f"Preview: {result.artifacts['semantic_preview']}")
    print(f"Diagnostics: {result.diagnostics_report}")
    print(f"Manifest: {manifest}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "lanelet2", "--to", "raster",
    #     "--input-dir", "path/to/lanelet2_maps",
    #     "--output-dir", "data/results",
    # ])


if __name__ == "__main__":
    main()
