"""OSM road-level interpretation shared by conversion and evaluation.

Native widths are metres, lanes is the total lane count, and estimated widths
are explicitly distinguished from supplied measurements.
"""

from __future__ import annotations

import math
import re

from automap_converter.core.config.osm_config import osm_config
from automap_converter.formats.osm.info_deduction import assume_missing_info, extract_missing_info


def length_m(value):
    text = str(value or "").strip().lower()
    match = re.fullmatch(r"(\d+(?:\.\d+)?)\s*(m|metres?|meters?|ft|feet)?", text)
    if match:
        result = float(match[1]) * (0.3048 if match[2] in {"ft", "feet"} else 1)
        return result if math.isfinite(result) and result > 0 else None
    feet = re.fullmatch(r"(\d+)'\s*(\d+(?:\.\d+)?)?\"?", text)
    return (float(feet[1]) * 12 + float(feet[2] or 0)) * 0.0254 if feet else None


def road_profile(tags):
    highway = tags.get("highway", "unclassified")
    roadtype = highway if highway in osm_config.LANECOUNTS else "unclassified"

    def count(key):
        try:
            value = int(tags[key])
            return value if value >= 0 else None
        except (KeyError, TypeError, ValueError):
            return None

    oneway = tags.get("oneway")
    one = (
        oneway in {"yes", "1", "true", "-1"}
        if oneway is not None
        else True
        if tags.get("junction") == "roundabout"
        else None
    )
    turns = [
        tags.get(k, "").split("|") if tags.get(k) else None
        for k in ("turn:lanes", "turn:lanes:forward", "turn:lanes:backward")
    ]
    info, _ = extract_missing_info(
        (count("lanes"), count("lanes:forward"), count("lanes:backward"), one, *turns)
    )
    info, assumptions = assume_missing_info(info, roadtype)
    lanes, forward, backward, one = info[:4]
    lanes = max(1, lanes)
    if oneway == "-1":
        forward, backward = 0, lanes
    default = float(osm_config.LANEWIDTHS[roadtype])
    width = length_m(tags.get("width"))
    basis = "width"
    values = tags.get("width:lanes", "").split("|") if tags.get("width:lanes") else []
    if not values:
        values = [
            v
            for key in ("width:lanes:backward", "width:lanes:forward")
            for v in tags.get(key, "").split("|")
            if v
        ]
    per_lane = [length_m(v) or default for v in values]
    if width is None and per_lane:
        width, basis = sum(per_lane), "width:lanes"
    if width is None:
        width, basis = length_m(tags.get("est_width")), "est_width"
    if width is None:
        width, basis = lanes * default, "lanes_and_highway_defaults"
    if len(per_lane) != lanes:
        per_lane = [width / lanes] * lanes
    elif abs(sum(per_lane) - width) > max(0.05, 0.01 * width):
        assumptions = list(assumptions) + ["Road width differs from traffic-lane widths; preserve explicit lane widths, do not silently redistribute shoulders/margins"]
    return {
        "highway": highway,
        "lanes": lanes,
        "forward": forward,
        "backward": backward,
        "oneway": "-1" if oneway == "-1" else "yes" if one else "no",
        "width_m": width,
        "lane_widths_m": per_lane,
        "width_basis": basis,
        "unallocated_width_m": width-sum(per_lane),
        "estimated": basis in {"est_width", "lanes_and_highway_defaults"}
        or (basis == "width:lanes" and any(length_m(v) is None for v in values)),
        "assumptions": list(assumptions),
    }
