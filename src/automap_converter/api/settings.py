"""Runtime settings loaded from the public YAML configuration files."""

from __future__ import annotations

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Iterator, Mapping

import yaml

from automap_converter.core.config.opendrive_config import open_drive_config


@dataclass(frozen=True)
class RuntimeSettings:
    """Settings that affect public conversion and diagnostic execution."""

    lanelet2_routing_enrichment: bool = True
    osm_extract_sublayer: bool = True
    opendrive_version: str = "1.6.1"
    raster_resolution_m: float = 0.5
    raster_padding_m: float = 5.0
    raster_supersampling: int = 4
    raster_topology_tolerance_m: float = 1.0
    raster_max_output_pixels: int = 40_000_000
    launch_viewer: bool = False
    diagnose_target_conformance: bool = True
    diagnose_topology: bool = True
    diagnose_source_element_mapping: bool = True


_ACTIVE_SETTINGS: ContextVar[RuntimeSettings] = ContextVar(
    "automap_converter_runtime_settings", default=RuntimeSettings()
)


def _packaged_default_path() -> Path:
    return Path(resources.files("automap_converter").joinpath("defaults/default.yaml"))


def default_config_path() -> Path:
    """Return the repository config in a checkout or the packaged fallback."""

    repository_config = Path(__file__).resolve().parents[3] / "configs" / "default.yaml"
    return repository_config if repository_config.is_file() else _packaged_default_path()


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if value is None:
        return {}
    if not isinstance(value, Mapping):
        raise ValueError(f"Configuration field {label!r} must be a mapping.")
    return value


def _bool(mapping: Mapping[str, object], key: str, default: bool) -> bool:
    value = mapping.get(key, default)
    if not isinstance(value, bool):
        raise ValueError(f"Configuration field {key!r} must be true or false.")
    return value


def _version(value: object) -> str:
    version = str(value)
    pieces = version.split(".")
    if len(pieces) not in {2, 3} or not all(piece.isdigit() for piece in pieces):
        raise ValueError("conversion.opendrive_version must look like '1.6' or '1.6.1'.")
    return version


def _float(
    mapping: Mapping[str, object], key: str, default: float, *, minimum: float
) -> float:
    value = mapping.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Configuration field {key!r} must be numeric.")
    result = float(value)
    if result < minimum:
        raise ValueError(f"Configuration field {key!r} must be at least {minimum}.")
    return result


def _int(
    mapping: Mapping[str, object],
    key: str,
    default: int,
    *,
    minimum: int,
    maximum: int | None = None,
) -> int:
    value = mapping.get(key, default)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"Configuration field {key!r} must be an integer.")
    if value < minimum or (maximum is not None and value > maximum):
        range_text = f"{minimum}..{maximum}" if maximum is not None else f">= {minimum}"
        raise ValueError(f"Configuration field {key!r} must be {range_text}.")
    return value


def _load_yaml(path: Path) -> Mapping[str, object]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return _mapping(raw, "root")


def load_runtime_settings(config_path: str | Path | None = None) -> RuntimeSettings:
    """Load public YAML settings and validate fields used by this release."""

    path = Path(config_path).expanduser().resolve() if config_path else default_config_path()
    if not path.is_file():
        raise FileNotFoundError(f"Configuration file does not exist: {path}")
    root = dict(_load_yaml(path))
    if config_path is None or path.name == "default.yaml":
        validation_path = path.with_name("validation.yaml")
        if validation_path.is_file():
            root.update(_load_yaml(validation_path))
    conversion = _mapping(root.get("conversion"), "conversion")
    raster = _mapping(root.get("raster"), "raster")
    stage1 = _mapping(root.get("stage1"), "stage1")
    diagnostics = _mapping(root.get("diagnostics"), "diagnostics")
    return RuntimeSettings(
        lanelet2_routing_enrichment=_bool(
            conversion, "lanelet2_routing_enrichment", True
        ),
        osm_extract_sublayer=_bool(conversion, "osm_extract_sublayer", True),
        opendrive_version=_version(conversion.get("opendrive_version", "1.6.1")),
        raster_resolution_m=_float(raster, "resolution_m", 0.5, minimum=0.01),
        raster_padding_m=_float(raster, "padding_m", 5.0, minimum=0.0),
        raster_supersampling=_int(raster, "supersampling", 4, minimum=1, maximum=8),
        raster_topology_tolerance_m=_float(
            raster, "topology_tolerance_m", 1.0, minimum=0.01
        ),
        raster_max_output_pixels=_int(
            raster, "max_output_pixels", 40_000_000, minimum=1
        ),
        launch_viewer=_bool(stage1, "launch_viewer", False),
        diagnose_target_conformance=_bool(diagnostics, "target_conformance", True),
        diagnose_topology=_bool(diagnostics, "topology", True),
        diagnose_source_element_mapping=_bool(
            diagnostics, "source_element_mapping", True
        ),
    )


def current_runtime_settings() -> RuntimeSettings:
    """Return settings active for the current conversion call."""

    return _ACTIVE_SETTINGS.get()


@contextmanager
def use_runtime_settings(settings: RuntimeSettings) -> Iterator[None]:
    """Temporarily apply settings to code that still uses shared config objects."""

    pieces = settings.opendrive_version.split(".")
    old_major = open_drive_config.target_version_major
    old_minor = open_drive_config.target_version_minor
    token = _ACTIVE_SETTINGS.set(settings)
    open_drive_config.target_version_major = int(pieces[0])
    open_drive_config.target_version_minor = int(pieces[1])
    try:
        yield
    finally:
        open_drive_config.target_version_major = old_major
        open_drive_config.target_version_minor = old_minor
        _ACTIVE_SETTINGS.reset(token)
