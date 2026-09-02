"""Convert the bundled Lanelet2 sample to OSM with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "lanelet2" / "basic_intersection_area.osm"
OUTPUT_DIR = PROJECT_ROOT / "data" / "results" / "lanelet2_to_osm"
TARGET = OUTPUT_DIR / "basic_intersection_area.osm"


def main() -> None:
    result = convert_and_diagnose(SOURCE, TARGET, "lanelet2", "osm", diagnostics_dir=OUTPUT_DIR)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "lanelet2", "--to", "osm",
    #     "--input-dir", "path/to/lanelet2_maps",
    #     "--output-dir", "data/results/lanelet2_to_osm_batch",
    # ])


if __name__ == "__main__":
    main()
