"""Proposed quantitative one-way checks; not vector roundtrip acceptance criteria."""

import math

import numpy as np
import rasterio
from rasterio.features import geometry_mask, shapes
from rasterio.windows import Window, from_bounds
from scipy.ndimage import label
from shapely.geometry import mapping, shape
from shapely.ops import unary_union

from automap_converter.conversion.lanelet2_to_raster.converter import _parse_lanelets, _topology


def evaluate_raster_oneway(source, target):
    lanelets, _, source_info, _ = _parse_lanelets(source)
    records = []
    with rasterio.open(target) as ds:
        mask = ds.read(1) > 0
        direction = ds.read(4)
        resolution = max(abs(ds.transform.a), abs(ds.transform.e))
        vector = unary_union([lane.polygon for lane in lanelets])
        pixels = unary_union(
            [
                shape(g)
                for g, value in shapes(mask.astype("uint8"), mask=mask, transform=ds.transform)
                if value
            ]
        )
        union_area = vector.union(pixels).area
        iou = vector.intersection(pixels).area / union_area if union_area else 0
        exemption_m=math.sqrt(2)*resolution
        boundary_band=vector.boundary.buffer(exemption_m)
        difference=vector.symmetric_difference(pixels)
        outside_area=difference.difference(boundary_band).area
        tolerant_iou=1-outside_area/union_area if union_area and pixels.area>0 else 0
        records.append(
            {
                "name": "drivable_area_iou",
                "value": tolerant_iou,
                "raw_iou":iou,
                "boundary_exemption_m":exemption_m,
                "exempt_area_m2":difference.intersection(boundary_band).area,
                "unexempt_difference_area_m2":outside_area,
                "threshold": 0.95,
                "status": "PASS" if tolerant_iou >= 0.95 else "FAIL",
                "method": "Actual pixel footprints; report raw IoU and exempt differences only within one pixel diagonal of the source boundary. Interior loss and remote extra pixels remain errors.",
            }
        )
        correct = total = 0
        direction_correct = 0
        by_id = {lane.source_id: lane for lane in lanelets}
        for lane in lanelets:
            for f in np.linspace(0.01, 0.99, 20):
                p = lane.centerline.interpolate(float(f), normalized=True)
                row, col = ds.index(p.x, p.y)
                total += 1
                if 0 <= row < ds.height and 0 <= col < ds.width:
                    correct += int(mask[row, col])
                    direction_correct += int(bool(int(direction[row, col]) & lane.direction_bit))
        for name, n in (
            ("centerline_drivable_coverage", correct),
            ("direction_bit_match_rate", direction_correct),
        ):
            rate = n / total if total else 0
            records.append(
                {
                    "name": name,
                    "passed": n,
                    "total": total,
                    "value": rate,
                    "threshold": 0.99,
                    "status": "PASS" if rate >= 0.99 else "FAIL",
                }
            )
        # Constrain each path to its two source lane corridors. A distant detour
        # elsewhere in the map cannot hide a broken junction pixel connection.
        edges = []
        for a, links in _topology(lanelets, max(resolution, 0.01)).items():
            for b in links["successors"]:
                first, second = by_id[a], by_id[b]
                corridor = first.polygon.union(second.polygon).buffer(resolution)
                raw = from_bounds(*corridor.bounds, ds.transform)
                col = max(0, math.floor(raw.col_off))
                row = max(0, math.floor(raw.row_off))
                end_col = min(ds.width, math.ceil(raw.col_off + raw.width))
                end_row = min(ds.height, math.ceil(raw.row_off + raw.height))
                window = Window(col, row, end_col - col, end_row - row)
                inside = geometry_mask(
                    [mapping(corridor)],
                    out_shape=(end_row - row, end_col - col),
                    transform=ds.window_transform(window),
                    invert=True,
                )
                components, _ = label(
                    mask[row:end_row, col:end_col] & inside, structure=np.ones((3, 3))
                )
                ids = []
                for line, fraction in ((first.centerline, 0.95), (second.centerline, 0.05)):
                    p = line.interpolate(fraction, normalized=True)
                    r, c = ds.index(p.x, p.y)
                    ids.append(
                        int(components[r - row, c - col])
                        if row <= r < end_row and col <= c < end_col
                        else 0
                    )
                edges.append({"from": a, "to": b, "passed": bool(ids[0]) and ids[0] == ids[1]})
        passed = sum(e["passed"] for e in edges)
        rate = passed / len(edges) if edges else None
        records.append(
            {
                "name": "local_pixel_connection_rate",
                "passed": passed,
                "total": len(edges),
                "value": rate,
                "threshold": 0.99,
                "status": "SKIP" if rate is None else "PASS" if rate >= 0.99 else "FAIL",
                "edges": edges,
            }
        )
    return {
        "schema": "automap-converter/raster-oneway-proposal/v1",
        "direction": "lanelet2_to_raster",
        "acceptance_basis": "Engineering proposal supplementing the test plan's qualitative one-way syntax/semantic checks; not an agreed contract threshold.",
        "result": "FAIL" if any(m["status"] == "FAIL" for m in records) else "PASS",
        "source_lanelets": len(lanelets),
        "resolution_m": resolution,
        "source_parse_details": source_info,
        "metrics": records,
        "functional_review": "QGIS/visual review still required; these checks do not prove simulation functional equivalence.",
    }
