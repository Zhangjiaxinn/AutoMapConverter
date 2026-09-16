"""Conversion and diagnostics profile for Lanelet2 -> raster."""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import rasterio
from lxml import etree
from scipy.ndimage import label

from automap_converter.api.settings import current_runtime_settings
from automap_converter.conversion.lanelet2_to_raster import (
    convert_lanelet2_to_raster,
    raster_artifact_paths,
)
from automap_converter.conversion.lanelet2_to_raster.converter import SEMANTIC_BANDS

from ._workflow_runtime import ConversionSpec, Stage1Check

CONVERSION = "lanelet2_to_raster"


def inspect_lanelet2_raster_source(_: str, source_path: Path) -> list[dict[str, str]]:
    """Validate the XML and relation structure consumed by the rasterizer."""

    try:
        root = etree.parse(str(source_path)).getroot()
        if root.tag != "osm":
            raise ValueError(f"root tag is {root.tag!r}, expected 'osm'")
        relations = [
            relation
            for relation in root.findall("relation")
            if relation.get("action") != "delete"
            and relation.get("visible") != "false"
            and any(
                tag.get("k") == "type" and tag.get("v") == "lanelet"
                for tag in relation.findall("tag")
            )
        ]
        if not relations:
            raise ValueError("no type=lanelet relations were found")
        malformed = sum(
            not any(member.get("role") == "left" for member in relation.findall("member"))
            or not any(member.get("role") == "right" for member in relation.findall("member"))
            for relation in relations
        )
        status = "WARN" if malformed else "PASS"
        return [
            {
                "category": "source-lanelet2-relations",
                "status": status,
                "summary": (
                    f"Lanelet2 XML contains {len(relations)} lanelet relations; "
                    f"malformed={malformed}."
                ),
                "hint": "Malformed relations are reported as skipped lanelets."
                if malformed
                else "",
            }
        ]
    except Exception as exc:  # noqa: BLE001 - report malformed third-party map input.
        return [
            {
                "category": "source-lanelet2-relations",
                "status": "FAIL",
                "summary": "Lanelet2 source cannot be parsed for rasterization.",
                "hint": f"{type(exc).__name__}: {exc}",
            }
        ]


def _convert(source_path: Path, target_path: Path, _: Path) -> list[dict[str, str]]:
    settings = current_runtime_settings()
    artifacts = convert_lanelet2_to_raster(
        source_path,
        target_path,
        resolution_m=settings.raster_resolution_m,
        padding_m=settings.raster_padding_m,
        supersampling=settings.raster_supersampling,
        topology_tolerance_m=settings.raster_topology_tolerance_m,
        max_output_pixels=settings.raster_max_output_pixels,
    )
    return [
        {
            "category": "raster-products",
            "status": "PASS",
            "summary": "Generated semantic, occupancy, preview and metadata products.",
            "hint": str(artifacts.metadata),
        }
    ]


def validate_raster_target(target_path: Path) -> list[Stage1Check]:
    """Check that raster products are readable and conform to the public schema."""

    artifacts = raster_artifact_paths(target_path)
    checks: list[Stage1Check] = []
    try:
        with rasterio.open(artifacts.semantic) as dataset:
            descriptions = tuple(item or "" for item in dataset.descriptions)
            if dataset.driver != "GTiff":
                raise ValueError(f"driver={dataset.driver}, expected GTiff")
            if dataset.count != len(SEMANTIC_BANDS):
                raise ValueError(f"bands={dataset.count}, expected {len(SEMANTIC_BANDS)}")
            if descriptions != SEMANTIC_BANDS:
                raise ValueError(f"band descriptions={descriptions!r}")
            if dataset.width < 1 or dataset.height < 1:
                raise ValueError("raster dimensions are empty")
            if any(dtype != "uint16" for dtype in dataset.dtypes):
                raise ValueError(f"dtypes={dataset.dtypes!r}, expected uint16")
            shape = (dataset.height, dataset.width)
        with rasterio.open(artifacts.occupancy) as occupancy:
            if occupancy.driver != "GTiff" or occupancy.count != 1:
                raise ValueError("occupancy product must be a single-band GeoTIFF")
            if (occupancy.height, occupancy.width) != shape:
                raise ValueError("occupancy and semantic raster dimensions differ")
        if not artifacts.metadata.is_file():
            raise ValueError("raster metadata sidecar is missing")
        if not artifacts.semantic_preview.is_file() or not artifacts.occupancy_preview.is_file():
            raise ValueError("one or more raster preview images are missing")
        checks.append(
            Stage1Check(
                category="raster-products",
                status="PASS",
                summary=f"Raster product set is readable ({shape[1]}x{shape[0]}, bands=6).",
                method="rasterio/GDAL",
                target=str(artifacts.semantic),
            )
        )
    except Exception as exc:  # noqa: BLE001 - preserve GDAL validation details.
        checks.append(
            Stage1Check(
                category="raster-products",
                status="FAIL",
                summary="Raster product validation failed.",
                method="rasterio/GDAL",
                target=str(artifacts.semantic),
                hint=f"{type(exc).__name__}: {exc}",
            )
        )

    return checks


def create_spec(source_path: Path, target_path: Path) -> ConversionSpec:
    """Build the standard single-map and batch execution profile."""

    return ConversionSpec(
        name=CONVERSION,
        source_dir=source_path.parent,
        output_root=target_path.parent / "batch_results",
        source_suffix=".osm",
        target_suffix=".tif",
        convert=_convert,
        validate=validate_raster_target,
        validate_source=inspect_lanelet2_raster_source,
    )


def _check(
    stage: str,
    category: str,
    status: str,
    summary: str,
    *,
    metrics: dict[str, object] | None = None,
    hint: str = "",
) -> dict[str, object]:
    return {
        "stage": stage,
        "category": category,
        "status": status,
        "summary": summary,
        "metrics": metrics or {},
        "hint": hint,
    }


def _result(checks: list[dict[str, object]]) -> str:
    if any(item["status"] == "FAIL" for item in checks):
        return "FAIL"
    if any(item["status"] == "WARN" for item in checks):
        return "PASS_WITH_WARNINGS"
    return "PASS"


def diagnose_lanelet2_to_raster(
    source_path: str | Path,
    target_path: str | Path,
    diagnostics_dir: str | Path,
    stage1_payload: dict[str, object] | None = None,
) -> tuple[Path, dict[str, object]]:
    """Measure semantic coverage, topology retention and product consistency."""

    source = Path(source_path).expanduser().resolve()
    target = Path(target_path).expanduser().resolve()
    output_dir = Path(diagnostics_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = raster_artifact_paths(target)
    started = time.time()
    metadata = json.loads(artifacts.metadata.read_text(encoding="utf-8"))

    checks: list[dict[str, object]] = []
    with rasterio.open(artifacts.semantic) as dataset:
        drivable = dataset.read(1)
        boundaries = dataset.read(2)
        centerlines = dataset.read(3)
        directions = dataset.read(4)
        anchors = dataset.read(5)
        lanelet_indices = dataset.read(6)
        resolution_x = abs(float(dataset.transform.a))
        resolution_y = abs(float(dataset.transform.e))
        semantic_shape = (dataset.height, dataset.width)
        band_descriptions = tuple(item or "" for item in dataset.descriptions)
    with rasterio.open(artifacts.occupancy) as dataset:
        occupancy = dataset.read(1)

    source_lanelets = int(metadata.get("source_lanelets", 0))
    rasterized_lanelets = int(metadata.get("rasterized_lanelets", 0))
    encoded_lanelets = len(set(np.unique(lanelet_indices).tolist()).difference({0}))
    relation_coverage = rasterized_lanelets / source_lanelets if source_lanelets else 0.0
    encoded_coverage = encoded_lanelets / rasterized_lanelets if rasterized_lanelets else 0.0
    if relation_coverage >= 0.99 and encoded_coverage >= 0.75:
        mapping_status = "PASS"
    elif relation_coverage >= 0.95:
        mapping_status = "WARN"
    else:
        mapping_status = "FAIL"
    checks.append(
        _check(
            "element_mapping",
            "lanelet-raster-coverage",
            mapping_status,
            (
                f"Rasterized {rasterized_lanelets}/{source_lanelets} lanelets; "
                f"{encoded_lanelets} have visible index pixels."
            ),
            metrics={
                "source_lanelets": source_lanelets,
                "rasterized_lanelets": rasterized_lanelets,
                "encoded_lanelets": encoded_lanelets,
                "relation_coverage": round(relation_coverage, 6),
                "encoded_coverage": round(encoded_coverage, 6),
            },
            hint=(
                "Inspect skipped_lanelets in the raster metadata; lanelet-index pixels can be "
                "overwritten where multiple lanelets overlap."
                if mapping_status != "PASS"
                else ""
            ),
        )
    )

    expected_occupancy = np.where(drivable > 0, 0, 1).astype(np.uint8)
    occupancy_match = float(np.mean(occupancy == expected_occupancy))
    product_ok = (
        band_descriptions == SEMANTIC_BANDS
        and semantic_shape == occupancy.shape
        and occupancy_match == 1.0
        and resolution_x > 0.0
        and resolution_y > 0.0
    )
    checks.append(
        _check(
            "target_conformance",
            "raster-product-consistency",
            "PASS" if product_ok else "FAIL",
            "Semantic, occupancy and metadata products use one aligned raster grid.",
            metrics={
                "width": semantic_shape[1],
                "height": semantic_shape[0],
                "resolution_x_m": resolution_x,
                "resolution_y_m": resolution_y,
                "band_count": len(band_descriptions),
                "occupancy_match_rate": round(occupancy_match, 6),
                "requested_supersampling": int(metadata.get("requested_supersampling", 1)),
                "effective_supersampling": int(metadata.get("supersampling", 1)),
            },
        )
    )

    nonempty = {
        "drivable_pixels": int(np.count_nonzero(drivable)),
        "boundary_pixels": int(np.count_nonzero(boundaries)),
        "centerline_pixels": int(np.count_nonzero(centerlines)),
        "direction_pixels": int(np.count_nonzero(directions)),
        "anchor_pixels": int(np.count_nonzero(anchors)),
    }
    semantic_ok = all(value > 0 for value in nonempty.values())
    checks.append(
        _check(
            "target_conformance",
            "semantic-layer-population",
            "PASS" if semantic_ok else "FAIL",
            "Required Lanelet2 semantic layers contain rasterized pixels.",
            metrics=nonempty,
        )
    )

    _, connected_components = label(drivable > 0)
    topology_edges = int(metadata.get("topology_edges", 0))
    topology_records = metadata.get("lanelets", [])
    topology_complete = (
        isinstance(topology_records, list) and len(topology_records) == rasterized_lanelets
    )
    checks.append(
        _check(
            "topology",
            "topology-sidecar",
            "PASS" if topology_complete else "FAIL",
            "Lanelet identities and graph relationships are preserved in the metadata sidecar.",
            metrics={
                "lanelet_records": len(topology_records)
                if isinstance(topology_records, list)
                else 0,
                "topology_edges": topology_edges,
                "drivable_components": int(connected_components),
            },
        )
    )

    counts = {
        status.lower(): sum(item["status"] == status for item in checks)
        for status in ("PASS", "WARN", "FAIL", "REVIEW", "SKIP")
    }
    result = _result(checks)
    payload: dict[str, object] = {
        "schema_version": "1.0",
        "stage": "stage1_and_stage2",
        "conversion": CONVERSION,
        "generated": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "source": str(source),
        "target": str(target),
        "prepared_source": (stage1_payload or {}).get("prepared_source", ""),
        "elapsed_seconds": round(time.time() - started, 3),
        "result": result,
        "summary": counts,
        "acceptance": [
            item
            for item in (stage1_payload or {}).get("acceptance", [])
            if item.get("status") != "PASS"
        ],
        "acceptance_summary": (stage1_payload or {}).get("summary", {}),
        "metrics": {
            "relation_coverage": round(relation_coverage, 6),
            "encoded_lanelet_coverage": round(encoded_coverage, 6),
            "occupancy_match_rate": round(occupancy_match, 6),
            "drivable_area_m2": round(nonempty["drivable_pixels"] * resolution_x * resolution_y, 3),
            "topology_edges": topology_edges,
            "drivable_components": int(connected_components),
            "requested_supersampling": int(metadata.get("requested_supersampling", 1)),
            "effective_supersampling": int(metadata.get("supersampling", 1)),
        },
        "artifacts": {key: str(value) for key, value in artifacts.as_dict().items()},
        "diagnostics": {
            "target_conformance": [
                item for item in checks if item["stage"] == "target_conformance"
            ],
            "topology": [item for item in checks if item["stage"] == "topology"],
            "element_mapping": [item for item in checks if item["stage"] == "element_mapping"],
        },
    }
    report_path = output_dir / f"{target.stem}_diagnostics.json"
    report_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return report_path, payload
