"""Diagnostic profile for the Lanelet2 -> OSM workflow."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple

from ._semantic_engine import diagnose_stage2
from ._workflow_runtime import ConversionSpec, convert_lanelet2_to_osm, validate_osm_target

CONVERSION = "lanelet2_to_osm"


def create_spec(source_path: Path, target_path: Path) -> ConversionSpec:
    """Build the batch and single-map execution profile for this direction."""

    return ConversionSpec(
        name=CONVERSION,
        source_dir=source_path.parent,
        output_root=target_path.parent / "batch_results",
        source_suffix=".osm",
        target_suffix=".osm",
        convert=convert_lanelet2_to_osm,
        validate=validate_osm_target,
    )


def diagnose_lanelet2_to_osm(
    source_path: str | Path,
    target_path: str | Path,
    diagnostics_dir: str | Path,
    stage1_payload: Optional[Dict[str, object]] = None,
) -> Tuple[Path, Path, Dict[str, object]]:
    """Write Stage 2 geometry, topology and mapping diagnostics."""

    return diagnose_stage2(
        CONVERSION,
        source_path,
        target_path,
        diagnostics_dir,
        stage1_payload=stage1_payload,
    )
