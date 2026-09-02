"""Run one conversion with Stage 1 and Stage 2 diagnostics enabled."""

from __future__ import annotations

import argparse

from automap_converter import convert_and_diagnose


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert one map and write diagnostics.")
    parser.add_argument("source")
    parser.add_argument("target")
    parser.add_argument("--from", dest="source_format", required=True)
    parser.add_argument("--to", dest="target_format", required=True)
    args = parser.parse_args()
    result = convert_and_diagnose(args.source, args.target, args.source_format, args.target_format)
    print(result.diagnostics_report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
