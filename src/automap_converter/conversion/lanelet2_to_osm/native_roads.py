"""Condense actual Lanelet2 geometry into public OSM road/lanes semantics."""

import json
import math

import networkx as nx
import numpy as np
from pyproj import Transformer
from shapely.geometry import LineString

from automap_converter.formats.lanelet2.osm_model import Node, Relation, RelationMember, Way

from .lane_profiles import _boundaries

NONMOTOR = {"walkway", "pedestrian", "crosswalk", "bicycle_lane", "bike_lane"}
NONROAD = {"border", "shoulder", "median"}


def export_native_roads(converter):
    source, output = converter._source, converter._out
    lanes = source.way_relations
    pairs = {k: _boundaries(converter, lane) for k, lane in lanes.items()}
    graph = nx.Graph()
    graph.add_nodes_from(
        k for k, lane in lanes.items() if lane.tag_dict.get("subtype") not in NONROAD
    )
    users = {}
    for key in graph:
        lane = lanes[key]
        for wid in (lane.left_way, lane.right_way):
            users.setdefault(str(wid), []).append(key)
    for keys in users.values():
        for i, key in enumerate(keys):
            for other in keys[:i]:
                if any(lanes[k].tag_dict.get("subtype") in NONMOTOR for k in (key, other)):
                    continue
                a, b = pairs[key][0], pairs[other][0]
                va = np.subtract(a.coords[-1], a.coords[0])
                vb = np.subtract(b.coords[-1], b.coords[0])
                if abs(np.dot(va, vb)) > 0.85 * np.linalg.norm(va) * np.linalg.norm(vb):
                    graph.add_edge(key, other)
    frame = getattr(source, "coordinate_frame", {})
    crs = frame.get("crs") or frame.get("encoding_crs")
    if frame.get("kind") == "geographic":
        crs = None
    inverse = Transformer.from_crs(crs, 4326, always_xy=True) if crs else None

    def lonlat(point):
        if inverse:
            return inverse.transform(*point)
        return (
            converter._xy_origin_lon + point[0] / converter._xy_m_per_deg_lon,
            converter._xy_origin_lat + point[1] / converter._xy_m_per_deg_lat,
        )

    roads, lane_map = [], {}
    for group in nx.connected_components(graph):
        members = sorted(group)
        seed = pairs[members[0]][0]
        vector = np.subtract(seed.coords[-1], seed.coords[0])
        if np.linalg.norm(vector) <= 1e-9:
            vector = next(
                (
                    np.subtract(p, seed.coords[0])
                    for p in seed.coords[1:]
                    if np.linalg.norm(np.subtract(p, seed.coords[0])) > 1e-9
                ),
                None,
            )
            if vector is None:
                raise ValueError("Cannot condense a zero-length lane boundary")
        normal = np.array([-vector[1], vector[0]]) / np.linalg.norm(vector)
        bounds, directions, widths = [], {}, {}
        for key in members:
            left, right = pairs[key]
            same = np.dot(np.subtract(left.coords[-1], left.coords[0]), vector) > 0
            directions[key] = bool(same)
            bounds.extend(
                [left, right]
                if same
                else [LineString(list(left.coords)[::-1]), LineString(list(right.coords)[::-1])]
            )
            widths[key] = left.interpolate(0.5, normalized=True).distance(
                right.interpolate(0.5, normalized=True)
            )
        bounds.sort(key=lambda b: np.dot(normal, b.interpolate(0.5, normalized=True).coords[0]))
        count = max(20, min(500, math.ceil(max(b.length for b in bounds) / 0.5) + 1))
        coords = [
            tuple(
                (
                    np.array(bounds[0].interpolate(f, normalized=True).coords[0])
                    + np.array(bounds[-1].interpolate(f, normalized=True).coords[0])
                )
                / 2
            )
            for f in np.linspace(0, 1, count)
        ]
        left_to_right = sorted(
            members,
            key=lambda k: -np.dot(normal, pairs[k][0].interpolate(0.5, normalized=True).coords[0]),
        )
        forward = [k for k in left_to_right if directions[k]]
        backward = [k for k in reversed(left_to_right) if not directions[k]]
        motor = lanes[members[0]].tag_dict.get("subtype") not in NONMOTOR
        highway = (
            "unclassified"
            if motor
            else "cycleway"
            if lanes[members[0]].tag_dict.get("subtype") in {"bike_lane", "bicycle_lane"}
            else "footway"
        )
        data = {
            "highway": highway,
            "width": f"{sum(widths.values()):.6f}",
            "oneway": "no"
            if backward or any(lanes[k].tag_dict.get("one_way") == "no" for k in members)
            else "yes",
            "lanelet2:lanelet_ids": ",".join(members),
        }
        if motor:
            data["lanes"] = str(len(members))
            if backward:
                data.update(
                    {"lanes:forward": str(len(forward)), "lanes:backward": str(len(backward))}
                )
            for suffix, keys in (
                (("", forward),)
                if not backward
                else ((":forward", forward), (":backward", backward))
            ):
                if not keys:
                    continue
                data["width:lanes" + suffix] = "|".join(f"{widths[k]:.6f}" for k in keys)
                speeds = []
                for key in keys:
                    values = converter._build_centerline_way_tags(
                        [lanes[key]], (lanes[key].left_way, lanes[key].right_way)
                    )
                    speeds.append(values.get("maxspeed", ""))
                if any(speeds):
                    data["maxspeed:lanes" + suffix] = "|".join(speeds)
                if any(lanes[k].tag_dict.get("subtype") == "bus_lane" for k in keys):
                    data["access:lanes" + suffix] = "|".join(
                        "no" if lanes[k].tag_dict.get("subtype") == "bus_lane" else "yes"
                        for k in keys
                    )
                    data["bus:lanes" + suffix] = "|".join(
                        "designated" if lanes[k].tag_dict.get("subtype") == "bus_lane" else "yes"
                        for k in keys
                    )
            all_speeds = {
                v
                for name, value in data.items()
                if name.startswith("maxspeed:lanes")
                for v in value.split("|")
            }
            if len(all_speeds) == 1 and "" not in all_speeds:
                data["maxspeed"] = all_speeds.pop()
        if lanes[members[0]].tag_dict.get("subtype") == "crosswalk":
            data["footway"] = "crossing"
        data["automap:lineage_selectors"] = json.dumps(
            {
                k: {
                    "forward": directions[k],
                    "lane_index": (forward if directions[k] else backward).index(k) + 1,
                }
                for k in members
            }
        )
        index = len(roads)
        for key in members:
            keys = forward if directions[key] else backward
            lane_map[key] = (index, directions[key], keys.index(key) + 1)
        roads.append(
            {
                "coords": coords,
                "tags": data,
                "way": converter._way_id_gen.next(),
                "members": members,
            }
        )
    # Join only endpoints connected in the actual source topology. No global
    # geometry intersections, old source IDs, or arbitrary nearby-road welding.
    endpoint_graph = nx.Graph()
    endpoint_graph.add_nodes_from((i, end) for i in range(len(roads)) for end in (0, 1))
    connections = []
    for key, (i, same, number) in lane_map.items():
        for successor in lanes[key].successors:
            if successor not in lane_map:
                continue
            j, other_same, other_number = lane_map[successor]
            if i == j:
                continue
            end_a, end_b = int(same), int(not other_same)
            endpoint_graph.add_edge((i, end_a), (j, end_b))
            connections.append((i, end_a, j, number, other_number))
    endpoint_nodes = {}
    for component in nx.connected_components(endpoint_graph):
        points = [roads[i]["coords"][0 if end == 0 else -1] for i, end in component]
        point = tuple(np.mean(points, axis=0))
        nid = converter._node_id_gen.next()
        lon, lat = lonlat(point)
        output.add_node(Node(nid, lat, lon))
        for i, end in component:
            endpoint_nodes[(i, end)] = nid
    for i, road in enumerate(roads):
        refs = [endpoint_nodes[(i, 0)]]
        for point in road["coords"][1:-1]:
            nid = converter._node_id_gen.next()
            lon, lat = lonlat(point)
            output.add_node(Node(nid, lat, lon))
            refs.append(nid)
        refs.append(endpoint_nodes[(i, 1)])
        output.add_way(Way(road["way"], refs, road["tags"]))
    grouped = {}
    for i, end, j, a, b in connections:
        grouped.setdefault((i, end, j), set()).add((a, b))
    reserved_relations = {
        str(key)
        for mapping in (source.way_relations, source.multipolygons, source.regulatory_elements)
        for key in mapping
    }
    relation_id = -1
    for (i, end, j), edges in sorted(grouped.items()):
        while str(relation_id) in reserved_relations:
            relation_id -= 1
        values = {}
        for a, b in sorted(edges):
            values.setdefault(a, []).append(str(b))
        members = [
            RelationMember("way", roads[i]["way"], "from"),
            RelationMember("node", endpoint_nodes[(i, end)], "via"),
            RelationMember("way", roads[j]["way"], "to"),
        ]
        output.add_relation(
            Relation(
                str(relation_id),
                members,
                {
                    "type": "connectivity",
                    "connectivity": "|".join(f"{a}:{','.join(bs)}" for a, bs in values.items()),
                },
            )
        )
        relation_id -= 1
    converter._convert_regulatory_elements_to_road_level_features()
