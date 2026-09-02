"""Convert the bundled OpenDRIVE sample to OSM with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "opendrive" / "commonroad_straight_road.xodr"
OUTPUT_DIR = PROJECT_ROOT / "data" / "results" / "opendrive_to_osm"
TARGET = OUTPUT_DIR / "commonroad_straight_road.osm"


def main() -> None:
    result = convert_and_diagnose(SOURCE, TARGET, "opendrive", "osm", diagnostics_dir=OUTPUT_DIR)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "opendrive", "--to", "osm",
    #     "--input-dir", "path/to/opendrive_maps",
    #     "--output-dir", "data/results/opendrive_to_osm_batch",
    # ])


if __name__ == "__main__":
    main()
