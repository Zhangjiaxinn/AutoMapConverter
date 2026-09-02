"""
This module holds the classes required for the intermediate format
"""

__author__ = "Behtarin Ferdousi"

from automap_converter.conversion.osm_to_commonroad.support import geometry


class Node:
    """
    Class to represent the nodes in the intermediate format
    """

    def __init__(self, node_id: int, point: geometry.Point):
        """
        Initialize a node element

        :param node_id: unique id for node
        :param point: position of the node
        """

        self.id = node_id
        self.point = point
