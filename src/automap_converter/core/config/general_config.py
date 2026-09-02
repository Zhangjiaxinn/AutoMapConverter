from automap_converter.core.config.config_base import Attribute, BaseConfig
from automap_converter.core.config.coordinate_defaults import PSEUDO_MERCATOR


class GeneralConfig(BaseConfig):#commonroad作为一个通用的道路场景描述格式，包含了许多与道路场景相关的通用配置参数，例如国家ID、地图名称、时间步长、作者信息等。这些参数在不同的地图转换过程中可能会被使用到，因此将它们集中在一个配置类中可以方便地管理和使用这些参数。
    """
    This config holds all general settings.#默认参数集合
    """

    # CommonRoad country ID#国家ID
    country_id = Attribute("ZAM", "CommonRoad country ID")#把一个属性/配置参数的值、默认值、显示名称、描述、单位、选项和验证函数封装在一个类中
    # CommonRoad map name#地图名称
    map_name = Attribute("MUC", "CommonRoad map name")#值/显示名称/描述/单位/选项/验证函数
    # CommonRoad map ID#地图ID
    map_id = Attribute(1, "CommonRoad map ID")
    # CommonRoad scenario time step size#时间步长
    time_step_size = Attribute(0.1, "CommonRoad scenario time step size")
    # CommonRoad map/scenario author#作者
    author = Attribute("Max Mustermann", "CommonRoad map/scenario author")
    # CommonRoad scenario/map author affiliation#作者单位
    affiliation = Attribute(
        "Technical University of Munich", "CommonRoad scenario/map author affiliation"
    )
    # CommonRoad scenario/map source#数据来源
    source = Attribute("AutoMapConverter", "CommonRoad scenario/map source")
    # projection used
    # additional tags for the benchmark#标签
    tags = Attribute("urban", "CommonRoad Tags")
    # projection used for the CommonRoad scenario#投影
    proj_string_cr = Attribute(
        PSEUDO_MERCATOR, "Projection string", "String used for the initialization of projection"
    )

    LAYOUT = [
        [
            "Default CommonRoad Scenario Information",#默认的CommonRoad场景信息
            country_id,
            map_name,
            map_id,
            time_step_size,
            author,
            affiliation,
            source,
            tags,
            proj_string_cr,
        ],
    ]


general_config = GeneralConfig()#创建一个GeneralConfig类的实例，并将其赋值给general_config变量，以便在其他地方使用这个配置实例。
