import logging
from typing import List

import numpy as np
from shapely.geometry.multipoint import MultiPoint

from automap_converter.core.config.opendrive_config import OpenDriveConfig, open_drive_config
from automap_converter.conversion.opendrive_to_commonroad.support.conversion_lanelet import ConversionLanelet
from automap_converter.conversion.opendrive_to_commonroad.support.utils import generate_unique_id
from automap_converter.formats.opendrive.parser.elements.road import (
    Road,
)


def _is_crosswalk_object_type(object_type: str, config: OpenDriveConfig = open_drive_config) -> bool:
    """Return whether an OpenDRIVE object type maps to a Lanelet2 crosswalk-like subtype."""
    normalized_type = str(object_type or "").strip()
    if normalized_type == "crosswalk":
        return True
    for lanelet2_subtype, odr_type in config.lanelet2_object_type_mapping.items():
        if str(odr_type) == normalized_type and str(lanelet2_subtype) in {
            "crosswalk",
            "ped_crossing",
            "pedestrian_crossing",
        }:
            return True
    return False


def get_crosswalks(
    road: Road, config: OpenDriveConfig = open_drive_config
) -> List[ConversionLanelet]:
    """Identify and converts crossing lanelets (represented as OpenDRIVE objects).
    Projection/Transformation will be applied later.

    :param road: The road object from which to extract signals.
    :return: list of ConversionLanelets with lanelet_type='crosswalk'
    """
    crosswalks = []
    for crosswalk in [
        obj
        for obj in road.objects
        if _is_crosswalk_object_type(obj.type, config)
    ]:
        pos, tan, _, _ = road.plan_view.calc(crosswalk.s, compute_curvature=False)
        position = np.array(
            [
                pos[0] + crosswalk.t * np.cos(tan + np.pi / 2),
                pos[1] + crosswalk.t * np.sin(tan + np.pi / 2),
            ]
        )
        corners = np.empty((0, 2))
        # origin of local u/v coordinate system that hdg rotates about
        o = np.atleast_2d(position)
        # rotation matrix
        rotation_angle = tan + crosswalk.hdg
        rotation_matrix = np.array(
            [
                [np.cos(rotation_angle), -np.sin(rotation_angle)],
                [np.sin(rotation_angle), np.cos(rotation_angle)],
            ]
        )

        for outline in crosswalk.outline:
            if outline.coordinate_type == "road":
                road_pos, road_tangent, _, _ = road.plan_view.calc(
                    outline.s, compute_curvature=False
                )
                rot_p = road_pos + outline.t * np.array(
                    [-np.sin(road_tangent), np.cos(road_tangent)]
                )
            else:
                p = np.atleast_2d([position[0] + outline.u, position[1] + outline.v])
                rot_p = np.squeeze((rotation_matrix @ (p.T - o.T) + o.T).T)
            corners = np.vstack((corners, rot_p))

        if len(corners) < 4:
            logging.warning(
                "Skipping crosswalk object %s: fewer than four outline corners", crosswalk.id
            )
            continue

        # object has four elements -> assume they are corners -> left and right
        # boundary each represented by two vertices
        if len(corners) == 5 or len(corners) == 4:
            # select long side as lanelet boundaries assuming index 0 corresponds to lower left corner
            if np.linalg.norm(corners[0] - corners[1]) > np.linalg.norm(corners[0] - corners[3]):
                left_vertices = np.array([corners[0], corners[1]])
                right_vertices = np.array([corners[3], corners[2]])
            else:
                right_vertices = np.array([corners[0], corners[3]])
                left_vertices = np.array([corners[1], corners[2]])
        # object has more than four elements -> fit rectangle over polygon and select closest vertices as corners
        else:
            rect = MultiPoint(corners).minimum_rotated_rectangle
            lower_left = np.argmin(
                np.linalg.norm(np.array(rect.boundary.coords[1]) - corners, axis=1)
            )
            lower_right = np.argmin(
                np.linalg.norm(np.array(rect.boundary.coords[2]) - corners, axis=1)
            )
            upper_left = np.argmin(
                np.linalg.norm(np.array(rect.boundary.coords[0]) - corners, axis=1)
            )

            # select long side as lanelet boundaries assuming index 0 corresponds to lower left corner
            if np.linalg.norm(corners[lower_left] - corners[upper_left]) > np.linalg.norm(
                corners[lower_left] - corners[lower_right]
            ):
                if lower_left == 0:
                    left_vertices = corners[lower_left : upper_left + 1]
                    right_vertices = np.flip(corners[upper_left + 1 : lower_right + 1], axis=0)
                elif lower_left == 3:
                    left_vertices = corners[lower_left : upper_left + 1]
                    right_vertices = np.flip(corners[0 : lower_right + 1], axis=0)
                else:
                    logging.warning("odr2cr crossing computation: case not supported yet.")
                    continue
            else:
                if lower_left == 5:
                    left_vertices = corners[upper_left:lower_right]
                    right_vertices = np.flip(corners[lower_right : lower_left + 1], axis=0)
                elif lower_left == 2:
                    left_vertices = corners[lower_right : lower_left + 1]
                    right_vertices = np.flip(corners[upper_left : lower_right - 1], axis=0)
                else:
                    logging.warning("odr2cr crossing computation: case not supported yet.")
                    continue
        center_vertices = (left_vertices + right_vertices) / 2
        # create ConversionLanelet
        lanelet = ConversionLanelet(
            None,
            left_vertices,
            center_vertices,
            right_vertices,
            generate_unique_id(),
            lanelet_type="crosswalk",
        )
        crosswalks.append(lanelet)
    return crosswalks
