"""Composable workflow primitives used by the public conversion API.

The modules in :mod:`automap_converter.conversion` own format-specific logic.
This module only joins their explicit stages when a public route requires an
intermediate CommonRoad scenario.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Union

from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.scenario.scenario import Scenario
from lxml import etree

from automap_converter.conversion.commonroad_to_lanelet2 import convert as commonroad_to_lanelet2
from automap_converter.conversion.lanelet2_to_opendrive.converter import Lanelet2OpendriveConverter
from automap_converter.conversion.lanelet2_to_osm.converter import Lanelet2OSMConverter
from automap_converter.conversion.opendrive_to_commonroad import convert as _opendrive_to_commonroad
from automap_converter.conversion.osm_to_commonroad import convert as _osm_to_commonroad
from automap_converter.core.config.general_config import GeneralConfig, general_config
from automap_converter.core.config.lanelet2_config import Lanelet2Config, lanelet2_config
from automap_converter.core.config.opendrive_config import OpenDriveConfig, open_drive_config
from automap_converter.formats.lanelet2.map import enrich_topology_from_official_routing_graph
from automap_converter.formats.lanelet2.parser import Lanelet2Parser
from automap_converter.formats.lanelet2.to_commonroad import Lanelet2CRConverter


PathLike = Union[str, Path]


def _parse_lanelet2(input_file: PathLike, lanelet_conf: Lanelet2Config):
    return Lanelet2Parser(etree.parse(str(input_file)).getroot(), lanelet_conf).parse()


def _enrich_lanelet2_topology(
    lanelet_map,
    input_file: PathLike,
    lanelet_conf: Lanelet2Config,
    *,
    enabled: bool = True,
) -> None:
    """Prefer official routing topology when available; retain parsed fallback."""

    if not enabled:
        return
    try:
        enrich_topology_from_official_routing_graph(
            osm_lanelet=lanelet_map,
            osm_file_path=str(input_file),
            origin_lat=float(lanelet_conf.routing_origin_lat),
            origin_lon=float(lanelet_conf.routing_origin_lon),
            location=str(lanelet_conf.routing_location),
            participant=str(lanelet_conf.routing_participant),
            overwrite=True,
        )
    except Exception as exc:  # routing plugins remain optional
        logging.warning(
            "Lanelet2 routing enrichment failed for %s; parser topology is retained. Reason: %s",
            input_file,
            exc,
        )


def lanelet2_to_commonroad(
    input_file: PathLike,
    *,
    general_conf: GeneralConfig = general_config,
    lanelet_conf: Lanelet2Config = lanelet2_config,
) -> Scenario:
    """Convert Lanelet2 to the explicit CommonRoad intermediate scenario."""

    return Lanelet2CRConverter(lanelet_conf, general_conf)(
        _parse_lanelet2(input_file, lanelet_conf)
    )


def lanelet2_to_osm(
    input_file: PathLike,
    *,
    lanelet_conf: Lanelet2Config = lanelet2_config,
    enrich_topology: bool = True,
):
    """Convert Lanelet2 to generic OSM centerline ways."""

    lanelet_map = _parse_lanelet2(input_file, lanelet_conf)
    _enrich_lanelet2_topology(
        lanelet_map, input_file, lanelet_conf, enabled=enrich_topology
    )
    return Lanelet2OSMConverter()(lanelet_map)


def lanelet2_to_opendrive(
    input_file: PathLike,
    output_file: PathLike,
    *,
    lanelet_conf: Lanelet2Config = lanelet2_config,
    opendrive_conf: OpenDriveConfig = open_drive_config,
    enrich_topology: bool = True,
) -> None:
    """Convert Lanelet2 directly to OpenDRIVE."""

    lanelet_map = _parse_lanelet2(input_file, lanelet_conf)
    _enrich_lanelet2_topology(
        lanelet_map, input_file, lanelet_conf, enabled=enrich_topology
    )
    target_path = Path(output_file)
    target_path.parent.mkdir(parents=True, exist_ok=True)
    Lanelet2OpendriveConverter(lanelet_map, step_size=2.0, config=opendrive_conf).convert(
        str(target_path)
    )


def opendrive_to_commonroad(
    input_file: PathLike,
    *,
    general_conf: GeneralConfig = general_config,
    opendrive_conf: OpenDriveConfig = open_drive_config,
) -> Scenario:
    """Convert OpenDRIVE to the explicit CommonRoad intermediate scenario."""

    return _opendrive_to_commonroad(
        input_file, general_conf=general_conf, opendrive_conf=opendrive_conf
    )


def osm_to_commonroad(input_file: PathLike) -> Scenario:
    """Convert generic OSM to the explicit CommonRoad intermediate scenario."""

    return _osm_to_commonroad(input_file)


def commonroad_file_to_lanelet2(
    input_file: PathLike,
    output_file: PathLike,
    *,
    lanelet_conf: Lanelet2Config = lanelet2_config,
    general_conf: GeneralConfig = general_config,
) -> Path:
    """Read a CommonRoad file and serialize its scenario as Lanelet2."""

    scenario, _ = CommonRoadFileReader(str(input_file)).open()
    return commonroad_to_lanelet2(
        scenario, output_file, lanelet_conf=lanelet_conf, general_conf=general_conf
    )


def opendrive_to_lanelet2(
    input_file: PathLike,
    output_file: PathLike,
    *,
    opendrive_conf: OpenDriveConfig = open_drive_config,
    general_conf: GeneralConfig = general_config,
    lanelet_conf: Lanelet2Config = lanelet2_config,
) -> Path:
    """Compose OpenDRIVE -> CommonRoad -> Lanelet2."""

    scenario = opendrive_to_commonroad(
        input_file, general_conf=general_conf, opendrive_conf=opendrive_conf
    )
    return commonroad_to_lanelet2(
        scenario, output_file, lanelet_conf=lanelet_conf, general_conf=general_conf
    )


def osm_to_lanelet2(
    input_file: PathLike,
    output_file: PathLike,
    *,
    lanelet_conf: Lanelet2Config = lanelet2_config,
    general_conf: GeneralConfig = general_config,
) -> Path:
    """Compose OSM -> CommonRoad -> Lanelet2."""

    return commonroad_to_lanelet2(
        osm_to_commonroad(input_file),
        output_file,
        lanelet_conf=lanelet_conf,
        general_conf=general_conf,
    )


# Compatibility alias used by existing diagnostic code. It names the actual
# operation precisely and can be removed from a later major API release.
osm_commonroad_file_to_lanelet2 = commonroad_file_to_lanelet2
