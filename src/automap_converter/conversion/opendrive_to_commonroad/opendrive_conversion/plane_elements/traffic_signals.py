import logging
import enum
from typing import Dict, List, Optional, Set, Tuple, Union

import numpy as np
from commonroad.scenario.lanelet import LineMarking, StopLine
from commonroad.scenario.traffic_light import TrafficLight, TrafficLightDirection
from commonroad.scenario.traffic_sign import (
    TrafficSign,
    TrafficSignElement,
    TrafficSignIDChina,
    TrafficSignIDGermany,
    TrafficSignIDRussia,
    TrafficSignIDSpain,
    TrafficSignIDUsa,
    TrafficSignIDZamunda,
)

from automap_converter.conversion.opendrive_to_commonroad.support.utils import generate_unique_id, get_default_cycle
from automap_converter.conversion.lanelet2_to_opendrive.elements.road import Road
from automap_converter.conversion.opendrive_to_commonroad.opendrive_conversion import utils
from automap_converter.formats.opendrive.parser.elements.roadLanes import (
    LaneSection,
)
from automap_converter.formats.opendrive.parser.elements.roadSignal import (
    Signal,
)


def extract_traffic_element_id(
    signal_type: str, signal_subtype: str, traffic_sign_enum: enum
) -> Union[
    TrafficSignIDZamunda,
    TrafficSignIDGermany,
    TrafficSignIDUsa,
    TrafficSignIDChina,
    TrafficSignIDSpain,
    TrafficSignIDRussia,
]:
    """Extract the traffic element id from the signal type and subtype string.

    :param signal_type: Signal type of the traffic element
    :param signal_subtype: Subtype of the traffic element
    :param traffic_sign_enum: Enumeration of country-specific traffic signs
    :return: The extracted traffic element id.
    """
    if signal_type in set(item.value for item in traffic_sign_enum):
        element_id = traffic_sign_enum(signal_type)
    elif signal_type + "-" + signal_subtype in set(item.value for item in traffic_sign_enum):
        element_id = traffic_sign_enum(signal_type + "-" + str(signal_subtype))
    elif (
        traffic_sign_enum is TrafficSignIDGermany or traffic_sign_enum is TrafficSignIDZamunda
    ) and signal_type == "252":  # traffic sign ID 252 is replaced by 260
        element_id = traffic_sign_enum("260")
    else:
        logging.warning(
            "OpenDRIVE/traffic_signals.py: Unknown {} of ID {} of subtype {}!".format(
                traffic_sign_enum.__name__, signal_type, signal_subtype
            )
        )
        element_id = traffic_sign_enum.UNKNOWN

    return element_id


def _signal_user_data_values(signal: Signal, code: str) -> List[str]:
    return [str(value) for value in getattr(signal, "user_data", {}).get(code, [])]


def _signal_semantics(signal: Signal) -> Set[str]:
    return {
        value.lower()
        for value in _signal_user_data_values(signal, "opendrive:semantic")
        if value
    }


def _signal_value_as_float(signal: Signal) -> Optional[float]:
    value = signal.signal_value
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _signal_has_meaningful_value(signal: Signal) -> bool:
    value = _signal_value_as_float(signal)
    if value is None:
        return False
    return value >= 0.0


def _traffic_sign_additional_values(signal: Signal) -> List[str]:
    if not _signal_has_meaningful_value(signal):
        return []
    value = _signal_value_as_float(signal)
    if value is None:
        return []
    if signal.unit == "km/h":
        return [str(value / 3.6)]
    return [str(value)]


def _is_opendrive_stop_line_signal(signal: Signal) -> bool:
    return str(signal.type) in {"294", "1100001"} or "stop_line" in _signal_semantics(signal)


def _is_semantic_speed_limit_signal(signal: Signal) -> bool:
    return str(signal.type) in {"274"} or "speed_limit" in _signal_semantics(signal)


def assign_traffic_signals_to_road(
    road: Road,
    traffic_light_dirs: Dict[str, Set[str]],
    traffic_light_lanes: Dict[str, Tuple[int, int]],
) -> Tuple[List[TrafficLight], List[TrafficSign], List[StopLine]]:
    """Extracts traffic_lights, traffic_signs, stop_lines from a road.

    :param road: The road object from which to extract signals.
    :param traffic_light_dirs: Dictionary of traffic light IDs to directions.
    :param traffic_light_lanes: Dictionary of traffic light IDs to lane validity.
    """
    traffic_signs = []
    traffic_lights = []
    stop_lines = []
    # TODO: Stop lines are created and appended to the list for DEU and OpenDrive format.
    # This has been replicated for other countries but has not been tested with a test case
    # Stop lines have a signal type of 294 and are handled differently in the CommonRoad format
    for signal in road.signals:
        lanes = (
            (0, 0) if signal.validity_from is None else (signal.validity_from, signal.validity_to)
        )
        position, tangent, _, _ = road.plan_view.calc(signal.s, compute_curvature=False)
        position = np.array(
            [
                position[0] + signal.t * np.cos(tangent + np.pi / 2),
                position[1] + signal.t * np.sin(tangent + np.pi / 2),
            ]
        )
        if signal.dynamic == "no":
            signal_country = utils.get_signal_country(signal.country)
            if str(signal.type) in {"1000003", "1000004"}:
                continue
            # OpenDRIVE permits extension-specific country identifiers.  The
            # CommonRoad lookup table is intentionally narrower, so use the
            # project's country resolver to retain an unknown signal as a
            # generic traffic sign instead of aborting the whole conversion.
            traffic_sign_enum = utils.get_traffic_sign_enum_from_country(signal_country)

            if _is_opendrive_stop_line_signal(signal):
                # Creating stop line object by first calculating the position of the two end points that define the
                # straight stop line
                position_1, position_2 = calculate_stop_line_position(
                    road.lanes.lane_sections, signal, position, tangent
                )
                stop_line = StopLine(position_1, position_2, LineMarking.SOLID)
                road.add_stop_line((stop_line, lanes, signal.s))
                stop_lines.append(stop_line)
                continue

            additional_values = _traffic_sign_additional_values(signal)
            if (
                _is_semantic_speed_limit_signal(signal)
                and str(signal.type) in {"", "-1", "None", "none"}
                and additional_values
            ):
                element_id = traffic_sign_enum.MAX_SPEED
            else:
                element_id = extract_traffic_element_id(
                    signal.type, str(signal.subtype), traffic_sign_enum
                )

            if element_id.name == "MAX_SPEED" and not additional_values:
                logging.warning(
                    "OpenDRIVE/traffic_signals.py: Speed limit signal %s has no usable value.",
                    signal.id,
                )
                continue
            if element_id.value == "":
                continue
            traffic_sign_element = TrafficSignElement(
                traffic_sign_element_id=element_id, additional_values=additional_values
            )
            traffic_sign = TrafficSign(
                traffic_sign_id=generate_unique_id(),
                traffic_sign_elements=list([traffic_sign_element]),
                first_occurrence=None,
                position=position,
                virtual=False,
            )
            road.add_traffic_sign((traffic_sign, lanes, signal.s))
            traffic_signs.append(traffic_sign)

        elif signal.dynamic == "yes":
            # Dynamic OpenDRIVE signals are the closest source semantic carrier for
            # Lanelet2 traffic_light regulatory elements. Keep them as traffic
            # lights instead of dropping signal-head variants such as 1000002.
            tdir = TrafficLightDirection.ALL
            if traffic_light_dirs.get(signal.signal_id) is not None:
                t_light = traffic_light_dirs[signal.signal_id]
                if "Right" in t_light and "Straight" in t_light and "Left" in t_light:
                    tdir = TrafficLightDirection.ALL
                elif "Left" in t_light and "Straight" in t_light:
                    tdir = TrafficLightDirection.LEFT_STRAIGHT
                elif "Right" in t_light and "Straight" in t_light:
                    tdir = TrafficLightDirection.STRAIGHT_RIGHT
                elif "Left" in t_light and "Right" in t_light:
                    tdir = TrafficLightDirection.LEFT_RIGHT
                elif "Left" in t_light:
                    tdir = TrafficLightDirection.LEFT
                elif "Right" in t_light:
                    tdir = TrafficLightDirection.RIGHT
                elif "Straight" in t_light:
                    tdir = TrafficLightDirection.STRAIGHT
                else:
                    tdir = TrafficLightDirection.ALL
            lanes = (
                lanes
                if traffic_light_lanes.get(signal.signal_id) is None
                else traffic_light_lanes[signal.signal_id]
            )
            traffic_light = TrafficLight(
                traffic_light_id=generate_unique_id(),
                position=position,
                traffic_light_cycle=get_default_cycle(),
                direction=tdir,
            )  # TODO remove for new CR-Format
            road.add_traffic_light((traffic_light, lanes, signal.s))
            traffic_lights.append(traffic_light)
    return traffic_lights, traffic_signs, stop_lines


def calculate_stop_line_position(
    lane_sections: List[LaneSection], signal: Signal, position: np.ndarray, tangent: float
) -> Tuple[np.ndarray, np.ndarray]:
    """Function to calculate the 2 points that define the stop line which
    is a straight line from one edge of the road to the other.

    :param lane_sections: OpenDRIVE lane_sections list containing the lane_section parsed lane_section class
    :param signal: Signal object, in this case the stop line.
    :param position: initial position as calculated in the get_traffic_signals function
    :param tangent: tangent value as calculated in the get_traffic_signals function
    :return: Positions of the stop line
    """
    total_width = 0
    for lane_section in lane_sections:
        for lane in lane_section.all_lanes:
            # Stop line width only depends on drivable lanes
            if lane.id != 0 and lane.type in ["driving", "onRamp", "offRamp", "exit", "entry"]:
                for width in lane.widths:
                    # Calculating total width of stop line
                    coefficients = width.polynomial_coefficients
                    lane_width = (
                        coefficients[0]
                        + coefficients[1] * signal.s
                        + coefficients[2] * signal.s**2
                        + coefficients[3] * signal.s**3
                    )

                    total_width += lane_width
    position_1 = position
    # Calculating second point of stop line using trigonometry
    position_2 = np.array(
        [
            position[0] - total_width * np.cos(tangent + np.pi / 2),
            position[1] - total_width * np.sin(tangent + np.pi / 2),
        ]
    )
    return position_1, position_2


def get_traffic_signal_references(
    road: Road,
    traffic_light_dirs: Dict[str, Set[str]],
    traffic_light_lanes: Dict[str, Tuple[int, int]],
):
    """Function to extract relevant information from sign references.

    :param road: The road object from which to extract signals.
    :param traffic_light_dirs: Dictionary, where signal directions should be stored.
    :param traffic_light_lanes: Dictionary, where signal lanes should be stored.
    """
    for signal in road.signal_reference:
        if signal.turn_relation is not None:
            if traffic_light_dirs.get(signal.signal_id) is None:
                traffic_light_dirs[signal.signal_id] = set()
            traffic_light_dirs[signal.signal_id].add(signal.turn_relation)
        if signal.validity_to is not None:
            if traffic_light_lanes.get(signal.signal_id) is not None:
                traffic_light_lanes[signal.signal_id] = (
                    min(traffic_light_lanes[signal.signal_id][0], signal.validity_from),
                    max(traffic_light_lanes[signal.signal_id][0], signal.validity_to),
                )
            else:
                traffic_light_lanes[signal.signal_id] = (signal.validity_to, signal.validity_from)
