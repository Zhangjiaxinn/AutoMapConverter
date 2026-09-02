from commonroad.scenario.traffic_sign import (
    TrafficSignIDGermany,
    TrafficSignIDUsa,
    TrafficSignIDZamunda,
)

from automap_converter.core.config.config_base import Attribute, BaseConfig


class Lanelet2Config(BaseConfig):
    """
    Lanelet2Config contains all the configuration parameters for the conversion from lanelet2 to CommonRoad.
    """
#值/显示名称/描述/单位/选项/验证函数
    # cr2lanelet
    ways_are_equal_tolerance = Attribute(
        0.001, "Ways are equal tolerance", "Value of the tolerance for which we mark ways as equal"
    )#在地图转换过程中，可能会遇到一些道路元素（如车道、道路边界等）在不同的地图格式中表示方式不同，但实际上它们代表的是同一个道路元素。为了处理这种情况，可以设置一个容差值（tolerance），当两个道路元素之间的距离或几何形状的差异小于这个容差值时，就可以认为它们是相同的。这有助于提高地图转换的准确性和一致性，避免因为微小的差异而导致错误的转换结果。

    autoware = Attribute(
        False,
        "Autoware",
        "Boolean indicating whether the conversion " "should be autoware compatible",
    )#Autoware是一个开源的自动驾驶软件平台，提供了各种功能和工具来支持自动驾驶系统的开发和测试。如果将autoware参数设置为True，表示在进行地图转换时会考虑Autoware的兼容性要求，以确保生成的地图能够被Autoware正确识别和使用。这可能涉及到一些特定的格式要求、标签规范或其他与Autoware相关的配置，以确保转换后的地图能够无缝集成到Autoware的自动驾驶系统中。

    use_local_coordinates = Attribute(
        False,
        "Use local coordinates",
        "Boolean indicating whether local coordinates should be added",
    )#在地图转换过程中，坐标系统的选择是一个重要的考虑因素。使用局部坐标（local coordinates）意味着将地图中的位置表示为相对于某个参考点的坐标，而不是使用全局坐标（如经纬度）。如果use_local_coordinates参数设置为True，表示在进行地图转换时会将地图中的位置转换为局部坐标系。这可以有助于提高地图的精度和效率，特别是在处理较小范围的地图时，因为局部坐标系通常具有更高的分辨率和更小的数值范围，从而减少了计算误差和存储需求。

    supported_countries = [TrafficSignIDGermany, TrafficSignIDZamunda, TrafficSignIDUsa]

    supported_countries_prefixes = {
        "TrafficSignIDZamunda": "de",
        "TrafficSignIDGermany": "de",
        "TrafficSignIDUsa": "us",
    }#在地图转换过程中，不同国家的交通标志可能具有不同的表示方式和命名规范。为了确保地图转换的准确性和一致性，可以定义一个支持的国家列表（supported_countries），其中包含了所有被支持的国家的交通标志ID类。同时，还可以定义一个字典（supported_countries_prefixes），将每个国家的交通标志ID类与其对应的前缀进行映射。这有助于在转换过程中正确识别和处理不同国家的交通标志，确保生成的地图能够正确反映各个国家的交通规则和标志系统。

    supported_lanelet2_subtypes = Attribute(
        [
            "urban",
            "country",
            "highway",
            "interstate",
            "busLane",
            "bicycleLane",
            "exitRamp",
            "sidewalk",
            "crosswalk",
            "shoulder",
            "border",
        ],
        "Supported lanelet2 subtypes",
        "Lanelet2 subtypes that are available in commonroad",
    )#在地图转换过程中，不同的地图格式可能会使用不同的子类型（subtypes）来描述道路元素（如车道、道路边界等）。为了确保地图转换的准确性和一致性，可以定义一个支持的子类型列表（supported_lanelet2_subtypes），其中包含了所有被支持的子类型。这有助于在转换过程中正确识别和处理不同子类型的道路元素，确保生成的地图能够正确反映各种道路环境和交通规则。

    supported_lanelet2_vehicles = ["car", "truck", "bus", "emergency", "motorcycle", "taxi"]#在地图转换过程中，不同的地图格式可能会使用不同的车辆类型来描述道路上的交通参与者。为了确保地图转换的准确性和一致性，可以定义一个支持的车辆类型列表（supported_lanelet2_vehicles），其中包含了所有被支持的车辆类型。这有助于在转换过程中正确识别和处理不同车辆类型的交通参与者，确保生成的地图能够正确反映各种交通场景和参与者的行为特征。

    # lanelet2cr
    node_distance_tolerance = Attribute(
        0.01,
        "Node distance tolerance",
        "Value of the tolerance (in meters) for which we mark nodes as equal",
    )#在地图转换过程中，节点（nodes）是构成道路元素的基本单位。为了处理不同地图格式中可能存在的微小差异，可以设置一个节点距离容差值（node_distance_tolerance），当两个节点之间的距离小于这个容差值时，就可以认为它们是相同的节点。这有助于提高地图转换的准确性和一致性，避免因为微小的差异而导致错误的转换结果。

    priority_signs = Attribute(
        ["PRIORITY", "RIGHT_OF_WAY"], "Priority signs", "List of priority signs"
    )#在地图转换过程中，优先权标志（priority signs）是指示道路上交通参与者优先权的标志，例如“优先通行”（PRIORITY）或“让行”（RIGHT_OF_WAY）。为了确保地图转换的准确性和一致性，可以定义一个优先权标志列表（priority_signs），其中包含了所有被支持的优先权标志。这有助于在转换过程中正确识别和处理不同优先权标志的交通参与者，确保生成的地图能够正确反映各种交通场景和参与者的行为特征。

    adjacent_way_distance_tolerance = Attribute(
        0.05, "Adjacent way distance tolerance", "Threshold indicating adjacent way"
    )#在地图转换过程中，邻近道路（adjacent ways）是指在地图上相互靠近的道路元素，例如车道、道路边界等。为了处理不同地图格式中可能存在的微小差异，可以设置一个邻近道路距离容差值（adjacent_way_distance_tolerance），当两个道路元素之间的距离小于这个容差值时，就可以认为它们是相邻的道路元素。这有助于提高地图转换的准确性和一致性，确保生成的地图能够正确反映各种道路环境和交通规则。

    start_node_id_value = Attribute(10, "Start Node ID", "Initial node ID")#在地图转换过程中，节点（nodes）是构成道路元素的基本单位。为了确保生成的地图中的节点具有唯一的标识符，可以设置一个起始节点ID值（start_node_id_value），从这个值开始为每个节点分配一个唯一的ID。这有助于在转换过程中正确识别和处理不同节点，确保生成的地图能够正确反映各种道路环境和交通规则。

    left_driving = Attribute(False, "Left Driving", "Map describes left driving system")

    adjacencies = Attribute(
        True,
        "Adjacencies",
        "Detect left and right adjacencies of lanelets if they do not share a common way",
    )

    translate = Attribute(
        False,
        "Translate",
        "Boolean indicating whether map should be translated by the location coordinate specified "
        "in the CommonRoad map",
    )

    allowed_tags = Attribute(
        [
            "type",
            "subtype",
            "one_way",
            "virtual",
            "location",
            "bicycle",
            "highway",
            "stop_area_type",
            "sign_type",
            "speed_limit",
            "speed",
            "value",
            "unit",
            "region",
            "nusc_token",
        ],
        "Allowed Tags",
        "Lanelet tags which are considered for conversion. "
        "Lanelets with other tags are not converted.",
    )

    eps2_values = Attribute(
        [1, 5, 10, 20, 50],
        "CCS Eps2 Values",
        "Possible values for length of additional segments of curvilinear coordinate system.",
    )

    max_polyline_resampling_step_values = Attribute(
        [2, 0.25, 0.5, 1, 5, 10, 20, 25],
        "Max. Polyline Resampling Step Values",
        "Possible values for resampling step size of reference for curvilinear coordinate system.",
    )

    chaikins_initial_refinements = Attribute(
        5,
        "Initial CCS Refinements",
        "Number of initial refinements of chaikins corner cutting algorithms "
        "for curvilinear coordinate system.",
    )

    chaikins_repeated_refinements = Attribute(
        10,
        "Max. Polyline Resampling Step",
        "Number of repeated refinements of chaikins corner cutting algorithms "
        "for curvilinear coordinate system.",
    )

    resampling_initial_step = Attribute(
        5,
        "Initial Max. Polyline Resampling Step",
        "Initial value for resampling step size of reference for curvilinear coordinate system.",
    )

    resampling_repeated_step = Attribute(
        5,
        "Repeated Max. Polyline Resampling Step",
        "Repeated value for resampling step size of reference for curvilinear coordinate system.",
    )

    perc_vert_wrong_side = Attribute(
        0.6,
        "Percentage Vertices Correct Direction",
        "Min. percentage of correctly assigned vertices to each polyline of lanelet.",
    )

    LAYOUT = [
        [
            "CommonRoad To Lanelet2",
            ways_are_equal_tolerance,
            autoware,
            use_local_coordinates,
            supported_lanelet2_subtypes,
            "General",
            translate,
            left_driving,
        ],
        [
            "Lanelet2 To CommonRoad",
            node_distance_tolerance,
            adjacent_way_distance_tolerance,
            start_node_id_value,
            priority_signs,
            adjacencies,
            allowed_tags,
        ],
    ]


lanelet2_config = Lanelet2Config()
lanelet2_config.enrich_topology = True
lanelet2_config.routing_origin_lat = 1.3160
lanelet2_config.routing_origin_lon = 103.7955
# The installed Lanelet2 traffic-rules plugin only provides Germany. It is
# used as the Vehicle routing-rule profile; map coordinates remain Singapore.
lanelet2_config.routing_location = "Germany"
lanelet2_config.routing_participant = "Vehicle"
