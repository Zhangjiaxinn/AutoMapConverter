from automap_converter.core.config.config_base import Attribute, BaseConfig
from automap_converter.core.config.coordinate_defaults import PSEUDO_MERCATOR


class OpenDriveConfig(BaseConfig):
    """
    This config holds all configs for the Open Drive conversion.
    """

    initial_cr_id = Attribute(1, "Initial CommonRoad element ID", "Initial CommonRoad element ID")

    error_tolerance = Attribute(
        0.15, "Error tolerance", "Max. error between reference geometry and polyline of vertices"
    )

    min_delta_s = Attribute(
        0.5,
        "Min. delta s",
        "Min. step length between two sampling positions on the reference geometry",
    )

    precision = Attribute(
        0.5, "Precision", "Precision with which to convert plane group to lanelet"
    )

    driving_default_lanelet_type = Attribute(
        "urban",
        "Driving default lanelet type",
        "Mapping of OpenDRIVE driveway lane type to a CommonRoad lanelet type",
    )

    general_lanelet_type_activ = Attribute(
        True,
        "General lanelet type active",
        "Activates whether certain lanelet type should be added to all lanelets",
    )

    general_lanelet_type = Attribute(
        "urban",
        "General lanelet type",
        "Lanelet type which is added to every lanelet (if activated)",
    )

    lanelet_types_backwards_compatible = Attribute(
        False,
        "Lanelet types backwards compatible",
        "If active, converts OpenDRIVE lane types only to CommonRoad "
        "lanelet types "
        " with commonroad-io==2022.1 (probably also even older ones)",
    )

    intersection_straight_threshold = Attribute(
        35.0,
        "Intersection straight threshold",
        "Threshold which is used to determine if a successor of an incoming " "lane is ",
    )

    lane_segment_angle = Attribute(
        5.0,
        "Lane segment angle",
        "Least angle for lane segment to be added to the "
        "graph in degrees. If you edit the graph by hand, "
        "a value of 0 is recommended",
    )

    proj_string_odr = Attribute(
        PSEUDO_MERCATOR, "Proj string", "String used for the " "initialization of projection"
    )

    filter_types = Attribute(
        [
            "driving",
            "restricted",
            "onRamp",
            "offRamp",
            "exit",
            "entry",
            "sidewalk",
            "shoulder",
            "crosswalk",
            "bidirectional",
        ],
        "Filter types",
        "OpenDRIVE lane types which are considered for conversion",
    )

    # cr2odr config parameters
    initial_road_counting = Attribute(20, "Initial road ID", "Initial counting for road ID")

    # 0.0174533 == 1deg
    heading_threshold = Attribute(
        0.00174533,
        "Heading threshold",
        "Threshold influencing selection of line (constant heading)",
    )

    # 0.01 == 0.5729578deg
    curvature_threshold = Attribute(
        0.01, "Curvature threshold", "Threshold influencing selection of arc (constant curvature)"
    )

    # 0.01 == 0.5729578deg
    curvature_dif_threshold = Attribute(
        0.01,
        "Curvature difference threshold",
        "Threshold influencing selection of clothoid (constant curvature difference)",
    )
    # Constant for lane parameters evaluation
    lane_evaluation_step = Attribute(
        50, "Curvature threshold clothoid", "Constant for lane parameters evaluation"
    )

    target_version_major = Attribute(
        1,
        "OpenDRIVE target major version",
        "Major OpenDRIVE schema version written by the Lanelet2 converter",
    )

    target_version_minor = Attribute(
        6,
        "OpenDRIVE target minor version",
        "Minor OpenDRIVE schema version; 1.6 selects the compatible 1.6.1 schema",
    )

    # Lanelet2 -> OpenDRIVE semantic mappings. These tables intentionally live
    # in the target-format config so projects can refine mappings without
    # changing conversion code.
    lanelet2_object_type_mapping = Attribute(
        {
            "crosswalk": "crosswalk",
            "ped_crossing": "crosswalk",
            "pedestrian_crossing": "crosswalk",
            "parking": "parkingSpace",
            "parking_space": "parkingSpace",
            "parkingspace": "parkingSpace",
            "tree": "tree",
            "pole": "pole",
            "barrier": "barrier",
            "guard_rail": "barrier",
            "building": "building",
            "traffic_island": "trafficIsland",
            "trafficisland": "trafficIsland",
            "island": "trafficIsland",
        },
        "Lanelet2 object type mapping",
        "Mapping from normalized Lanelet2 area/object subtype to official OpenDRIVE object type",
    )

    lanelet2_simple_object_types = Attribute(
        ["tree", "pole"],
        "Lanelet2 simple object types",
        "OpenDRIVE object types represented by a simple bounding shape instead of an outline",
    )

    lanelet2_simple_object_dimensions = Attribute(
        {
            "tree": {"radius": 0.25, "height": 5.0},
            "pole": {"radius": 0.08, "height": 3.0},
        },
        "Lanelet2 simple object dimensions",
        "Fallback dimensions for simple OpenDRIVE objects when source geometry has no explicit dimensions",
    )

    lanelet2_signal_type_mapping = Attribute(
        {
            "traffic_light": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "1000001",
                "subtype": "-1",
                "dynamic": "yes",
                "orientation": "+",
                "zOffset": 3.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.8,
                "width": 0.4,
            },
            "stop_line": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "1100001",
                "subtype": "-1",
                "dynamic": "no",
                "orientation": "+",
                "zOffset": 0.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.03,
                "width": 3.75,
            },
            "traffic_sign": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "-1",
                "subtype": "-1",
                "dynamic": "no",
                "orientation": "+",
                "zOffset": 2.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.6,
                "width": 0.6,
            },
            "right_of_way": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "-1",
                "subtype": "-1",
                "dynamic": "no",
                "orientation": "+",
                "zOffset": 0.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.0,
                "width": 0.0,
                "semantic": "right_of_way",
                "mapping_status": "unresolved",
            },
            "speed_limit": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "-1",
                "subtype": "-1",
                "dynamic": "no",
                "orientation": "+",
                "zOffset": 0.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.0,
                "width": 0.0,
                "semantic": "speed_limit",
                "mapping_status": "unresolved",
            },
            "unknown": {
                "country": "OpenDRIVE",
                "countryRevision": "2023",
                "type": "-1",
                "subtype": "-1",
                "dynamic": "no",
                "orientation": "+",
                "zOffset": 0.0,
                "hOffset": 0.0,
                "pitch": 0.0,
                "roll": 0.0,
                "height": 0.6,
                "width": 0.6,
            },
        },
        "Lanelet2 signal type mapping",
        "Fallback OpenDRIVE signal fields for common Lanelet2 regulatory and traffic-way types",
    )

    lanelet2_country_signal_value_mapping = Attribute(
        {
            # German StVO Zeichen 274.1: beginning of a Tempo-30 zone.
            "DE:274:1": {"value": "30", "unit": "km/h"},
            # Common explicit Zeichen 274 speed variants.
            "DE:274:5": {"value": "5", "unit": "km/h"},
            "DE:274:10": {"value": "10", "unit": "km/h"},
            "DE:274:20": {"value": "20", "unit": "km/h"},
            "DE:274:30": {"value": "30", "unit": "km/h"},
            "DE:274:40": {"value": "40", "unit": "km/h"},
            "DE:274:50": {"value": "50", "unit": "km/h"},
            "DE:274:60": {"value": "60", "unit": "km/h"},
            "DE:274:70": {"value": "70", "unit": "km/h"},
            "DE:274:80": {"value": "80", "unit": "km/h"},
            "DE:274:90": {"value": "90", "unit": "km/h"},
            "DE:274:100": {"value": "100", "unit": "km/h"},
            "DE:274:120": {"value": "120", "unit": "km/h"},
            "DE:274:130": {"value": "130", "unit": "km/h"},
        },
        "Lanelet2 country signal value mapping",
        "Conservative value/unit lookup for country catalog signs whose meaning fixes a numeric value",
    )

    lanelet2_stop_line_reference_signal_types = Attribute(
        [
            "DE:205",
            "DE:206",
        ],
        "Signals requiring a stop/yield line reference",
        "Country catalog signals that should reference a regulatory ref_line; unresolved right_of_way rules are handled separately",
    )

    lanelet2_stop_line_visualization = Attribute(
        {
            "enabled": True,
            "object_type": "roadMark",
            "line_width": 0.15,
            "height": 0.01,
            "zOffset": 0.005,
        },
        "Lanelet2 stop line visualization",
        "Defaults for the visible roadMark object generated from a Lanelet2 ref_line",
    )

    lanelet2_traffic_light_pole_visualization = Attribute(
        {
            "enabled": True,
            "object_type": "pole",
            "radius": 0.05,
            "height": 3.5,
            "zOffset": 0.0,
            "orientation": "+",
            "hdg": 0.0,
            "pitch": 0.0,
            "roll": 0.0,
        },
        "Lanelet2 traffic light pole visualization",
        "Defaults for the generated pole object paired with each concrete Lanelet2 traffic-light way",
    )

    LAYOUT = [
        [
            "Conversion Parameters odr2cr",
            initial_cr_id,
            error_tolerance,
            min_delta_s,
            precision,
            proj_string_odr,
            "Intersection and Lane Segment Parameters",
            intersection_straight_threshold,
            lane_segment_angle,
        ],
        [
            "Lanelet Type Configuration odr2cr",
            driving_default_lanelet_type,
            general_lanelet_type_activ,
            general_lanelet_type,
            lanelet_types_backwards_compatible,
            filter_types,
        ],
        [
            "Conversion Parameters cr2odr",
            initial_road_counting,
            heading_threshold,
            curvature_threshold,
            curvature_dif_threshold,
            lane_evaluation_step,
        ],
    ]


open_drive_config = OpenDriveConfig()
