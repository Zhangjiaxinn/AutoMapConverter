"""Diagnostic profile for the OpenDRIVE -> Lanelet2 workflow."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Optional, Tuple

from ._semantic_engine import diagnose_stage2
from ._workflow_runtime import (
    ConversionSpec,
    convert_opendrive_to_lanelet2,
    validate_lanelet2_target,
)

CONVERSION = "opendrive_to_lanelet2"


def create_spec(source_path: Path, target_path: Path) -> ConversionSpec:
    """Build the batch and single-map execution profile for this direction."""

    return ConversionSpec(
        name=CONVERSION,
        source_dir=source_path.parent,
        output_root=target_path.parent / "batch_results",
        source_suffix=".xodr",
        target_suffix=".osm",
        convert=convert_opendrive_to_lanelet2,
        validate=validate_lanelet2_target,
    )


def diagnose_opendrive_to_lanelet2(
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
