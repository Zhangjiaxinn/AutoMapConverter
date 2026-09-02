"""Command-line entry point for the four AutoMapConverter workflows."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from automap_converter.api.converter import (
    _COMPOSED_CONVERSIONS,
    _spec_for,
    conversion_name,
    convert,
    convert_and_diagnose,
    convert_batch_and_diagnose,
)
from automap_converter.api.settings import load_runtime_settings, use_runtime_settings
from automap_converter.validation.diagnostics._workflow_runtime import run_batch


def _add_formats(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--from", dest="source_format", required=True, choices=("lanelet2", "opendrive", "osm"))
    parser.add_argument("--to", dest="target_format", required=True, choices=("lanelet2", "opendrive", "osm"))


def build_parser() -> argparse.ArgumentParser:
    """Build the public command-line parser."""

    parser = argparse.ArgumentParser(prog="automap-converter", description="Convert and diagnose vector road maps.")
    commands = parser.add_subparsers(dest="command", required=True)

    convert_parser = commands.add_parser("convert", help="Convert one map.")
    _add_formats(convert_parser)
    convert_parser.add_argument("--input", required=True, type=Path)
    convert_parser.add_argument("--output", required=True, type=Path)
    convert_parser.add_argument("--diagnose", action="store_true", help="Write Stage 1 and Stage 2 reports.")
    convert_parser.add_argument("--diagnostics-dir", type=Path)
    convert_parser.add_argument("--no-sublayer", action="store_true", help="Disable OSM pedestrian/cyclist sublayers.")
    convert_parser.add_argument("--config", type=Path, help="Runtime YAML configuration file.")

    batch_parser = commands.add_parser("batch", help="Convert every compatible map in one directory.")
    _add_formats(batch_parser)
    batch_parser.add_argument("--input-dir", required=True, type=Path)
    batch_parser.add_argument("--output-dir", required=True, type=Path)
    batch_parser.add_argument("--run-name", default="")
    batch_parser.add_argument("--limit", type=int)
    batch_parser.add_argument("--config", type=Path, help="Runtime YAML configuration file.")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Run the command-line application."""

    args = build_parser().parse_args(argv)
    conversion = conversion_name(args.source_format, args.target_format)
    if args.command == "convert":
        result = (
            convert_and_diagnose(
                args.input,
                args.output,
                args.source_format,
                args.target_format,
                diagnostics_dir=args.diagnostics_dir,
                config_path=args.config,
            )
            if args.diagnose
            else convert(
                args.input,
                args.output,
                args.source_format,
                args.target_format,
                extract_sublayer=not args.no_sublayer,
                config_path=args.config,
            )
        )
        print(f"Converted: {result.source} -> {result.target}")
        if result.diagnostics_report:
            print(f"Diagnostics: {result.diagnostics_report}")
        return 0

    source_dir = args.input_dir.expanduser().resolve()
    output_dir = args.output_dir.expanduser().resolve()
    if conversion in _COMPOSED_CONVERSIONS:
        summary_path = convert_batch_and_diagnose(
            source_dir,
            output_dir,
            args.source_format,
            args.target_format,
            run_name=args.run_name,
            limit=args.limit,
            config_path=args.config,
        )
        print(f"Batch summary written to: {summary_path}")
        return 0
    source_suffix = ".xodr" if args.source_format == "opendrive" else ".osm"
    target_suffix = ".xodr" if args.target_format == "opendrive" else ".osm"
    spec = _spec_for(
        conversion,
        source_dir / f"placeholder{source_suffix}",
        output_dir / f"placeholder{target_suffix}",
    )
    spec.source_dir = source_dir
    spec.output_root = output_dir
    args.batch_source_dir = str(source_dir)
    with use_runtime_settings(load_runtime_settings(args.config)):
        run_batch(spec, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
