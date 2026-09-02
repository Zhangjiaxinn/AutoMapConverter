"""Measure repeated single-map conversions through the public API."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from automap_converter import convert


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark one vector-map conversion direction.")
    parser.add_argument("--from", dest="source_format", required=True)
    parser.add_argument("--to", dest="target_format", required=True)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--repetitions", type=int, default=3)
    args = parser.parse_args()

    if args.repetitions < 1:
        parser.error("--repetitions must be at least one")

    suffix = ".xodr" if args.target_format == "opendrive" else ".osm"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    durations = []
    for index in range(args.repetitions):
        target = args.output_dir / f"{args.input.stem}_{index + 1}{suffix}"
        started = time.perf_counter()
        convert(args.input, target, args.source_format, args.target_format)
        durations.append(round(time.perf_counter() - started, 6))

    report = {
        "source": str(args.input),
        "source_format": args.source_format,
        "target_format": args.target_format,
        "repetitions": args.repetitions,
        "durations_seconds": durations,
        "mean_seconds": round(sum(durations) / len(durations), 6),
    }
    report_path = args.output_dir / "benchmark.json"
    report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(report_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
