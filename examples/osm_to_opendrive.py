"""Convert the bundled OSM sample to OpenDRIVE with diagnostics."""

from pathlib import Path

from automap_converter import convert_and_diagnose


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PROJECT_ROOT / "data" / "samples" / "osm" / "ped_crossing.osm"
OUTPUT_DIR = PROJECT_ROOT / "data" / "results" / "osm_to_opendrive"
TARGET = OUTPUT_DIR / "ped_crossing.xodr"


def main() -> None:
    result = convert_and_diagnose(SOURCE, TARGET, "osm", "opendrive", diagnostics_dir=OUTPUT_DIR)
    print(f"Converted map: {result.target}")
    print(f"Diagnostics: {result.diagnostics_report}")

    # Batch conversion example:
    # from automap_converter.cli import main as cli_main
    # cli_main([
    #     "batch", "--from", "osm", "--to", "opendrive",
    #     "--input-dir", "path/to/osm_maps",
    #     "--output-dir", "data/results/osm_to_opendrive_batch",
    # ])


if __name__ == "__main__":
    main()
