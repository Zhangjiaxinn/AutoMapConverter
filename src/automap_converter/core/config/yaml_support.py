"""YAML extensions needed by CommonRoad configuration values."""

import yaml
from commonroad.common.util import Interval


def register_interval_yaml() -> None:
    """Register a compact YAML representation for CommonRoad intervals."""

    def interval_to_yaml(dumper, data):
        return dumper.represent_mapping("!interval", {"start": data.start, "end": data.end})

    def interval_from_yaml(loader, node):
        value = loader.construct_mapping(node)
        return Interval(value["start"], value["end"])

    yaml.add_representer(Interval, interval_to_yaml)
    yaml.add_constructor("!interval", interval_from_yaml)
