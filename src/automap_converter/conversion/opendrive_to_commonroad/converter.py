"""OpenDRIVE to CommonRoad intermediate conversion."""

from __future__ import annotations

from pathlib import Path
from typing import Union

from commonroad.scenario.scenario import Scenario

from automap_converter.core.config.general_config import GeneralConfig, general_config
from automap_converter.core.config.opendrive_config import OpenDriveConfig, open_drive_config
from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion.network import Network
from automap_converter.formats.opendrive.parser.parser import parse_opendrive


PathLike = Union[str, Path]


def convert(
    input_file: PathLike,
    *,
    general_conf: GeneralConfig = general_config,
    opendrive_conf: OpenDriveConfig = open_drive_config,
) -> Scenario:
    """Parse one OpenDRIVE map and construct its CommonRoad scenario."""

    source_path = Path(input_file)
    opendrive = parse_opendrive(source_path, opendrive_conf)
    network = Network()
    network.load_opendrive(opendrive)
    return network.export_commonroad_scenario(general_conf, opendrive_conf)
