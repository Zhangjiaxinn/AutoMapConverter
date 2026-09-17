"""Render a lightweight 16:9 preview for a vector map."""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt

from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    extract_features,
)

plt.switch_backend("Agg")


COLORS = {
    "lanelet2": "#22a6b3",
    "opendrive": "#f5a623",
    "osm": "#45b86b",
}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument(
        "--format",
        required=True,
        choices=("lanelet2", "opendrive", "osm", "raster"),
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--title", default="")
    args = parser.parse_args()

    figure, axes = plt.subplots(figsize=(16, 9), dpi=120)
    figure.patch.set_facecolor("#11161b")
    axes.set_facecolor("#11161b")
    caption = "Semantic raster preview"
    if args.format == "raster":
        axes.imshow(plt.imread(args.input))
        axes.set_aspect("equal", adjustable="box")
    else:
        feature_set = extract_features(args.input, args.format)
        if not feature_set.features:
            raise ValueError(f"No road features found in {args.input}")
        color = COLORS[args.format]
        for feature in feature_set.features:
            if len(feature.points) < 2:
                continue
            x, y = zip(*feature.points)
            axes.plot(x, y, color=color, linewidth=0.65, alpha=0.82)
        axes.set_aspect("equal", adjustable="datalim")
        axes.margins(0.025)
        caption = f"{len(feature_set.features):,} road features"

    axes.axis("off")
    if args.title:
        axes.text(
            0.025,
            0.955,
            args.title,
            color="#f4f7f8",
            fontsize=21,
            fontweight="bold",
            ha="left",
            va="top",
            transform=axes.transAxes,
        )
    axes.text(
        0.025,
        0.045,
        caption,
        color="#a9b4bc",
        fontsize=12,
        ha="left",
        va="bottom",
        transform=axes.transAxes,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.subplots_adjust(left=0.02, right=0.98, bottom=0.02, top=0.98)
    figure.savefig(args.output, facecolor=figure.get_facecolor())
    plt.close(figure)
    print(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
