"""OSM to CommonRoad intermediate conversion."""

from __future__ import annotations

from pathlib import Path
from typing import Union

from commonroad.scenario.scenario import Scenario

from automap_converter.conversion.osm_to_commonroad.converter_modules.converter import GraphScenario
from automap_converter.conversion.osm_to_commonroad.converter_modules.cr_operations.export import (
    convert_to_scenario,
)


PathLike = Union[str, Path]


def convert(input_file: PathLike) -> Scenario:
    """Build a CommonRoad scenario from a generic OSM road graph."""

    graph = GraphScenario(str(Path(input_file))).graph
    return convert_to_scenario(graph)
