"""Reassemble imported OSM roads from actual lane boundaries, not source coordinates."""

import json
from collections import defaultdict
from statistics import median

from pyproj import Transformer
from shapely.geometry import LineString, Point

from automap_converter.formats.lanelet2.osm_model import Node, Way

from .lane_profiles import _boundaries


def reassemble_roads(converter):
    lanes = list(converter._source.way_relations.values())
    if not lanes or any(
        not lane.tag_dict.get("automap:osm_road_model")
        and len(lane.tag_dict.get("source:osm:ways", "").split(",")) < 2
        for lane in lanes
    ):
        return False
    groups = defaultdict(lambda: defaultdict(list))
    for lane in lanes:
        if not lane.tag_dict.get("automap:osm_road_model"):
            continue  # Cross-road turn connectors are not additional OSM roads.
        model = json.loads(lane.tag_dict["automap:osm_road_model"])
        if not model.get("connector"):
            groups[model["road_id"]][model["edge_id"]].append((lane, model))
    roads = []
    frame = getattr(converter._source, "coordinate_frame", {})
    crs = frame.get("crs") or frame.get("encoding_crs")
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True) if crs else None
    for segments in groups.values():
        parts, all_lanes, widths = [], [], []
        model = next(iter(segments.values()))[0][1]
        for members in sorted(segments.values(), key=lambda ms: ms[0][1].get("segment_index", 0)):
            boundaries = []
            lane_widths = []
            for lane, info in members:
                pair = []
                for boundary in _boundaries(converter, lane):
                    points = list(boundary.coords)
                    if not info.get("forward_lane", True):
                        points.reverse()
                    if info.get("reversed"):
                        points.reverse()
                    pair.append(LineString(points))
                lane_widths.append(
                    pair[0]
                    .interpolate(0.5, normalized=True)
                    .distance(pair[1].interpolate(0.5, normalized=True))
                )
                boundaries.extend(pair)
                all_lanes.append(lane)
            ref = boundaries[0]
            a, b = ref.interpolate(0.49, normalized=True), ref.interpolate(0.51, normalized=True)
            dx, dy = b.x - a.x, b.y - a.y
            # Only two outer boundaries define the road centre; averaging all
            # lane centres would bias unequal-width lanes.
            boundaries.sort(
                key=lambda line: (
                    -dy * line.interpolate(0.5, normalized=True).x
                    + dx * line.interpolate(0.5, normalized=True).y
                )
            )
            left, right = boundaries[0], boundaries[-1]
            points = []
            for i in range(20):
                p, q = (
                    left.interpolate(i / 19, normalized=True),
                    right.interpolate(i / 19, normalized=True),
                )
                points.append(((p.x + q.x) / 2, (p.y + q.y) / 2))
            if members[0][1].get("reversed"):
                parts.insert(0, points)
            else:
                parts.append(points)
            widths.append(sum(lane_widths))
        coords = [p for part in parts for p in part]
        if len(coords) < 2:
            continue
        data = converter._build_centerline_way_tags(
            all_lanes, (all_lanes[0].left_way, all_lanes[-1].right_way)
        )
        data.update(model.get("tags", {}))
        data.pop("width", None)
        data.pop("width:lanes", None)
        data.update(
            {
                "highway": model["highway"],
                "lanes": str(model["lanes"]),
                "oneway": model["oneway"],
                "lanelet2:lanelet_ids": ",".join(str(l.id_) for l in all_lanes),
                "automap:width_basis": "generated_lane_boundaries",
            }
        )
        if model.get("estimated"):
            data["est_width"] = f"{median(widths):.5f}"
        else:
            data["width"] = f"{median(widths):.5f}"
        data["lanes:forward"], data["lanes:backward"] = (
            str(model["forward"]),
            str(model["backward"]),
        )
        roads.append(
            {"line": LineString(coords), "tags": data, "controls": model.get("controls", [])}
        )
    nodes = {}
    insertions = [[] for _ in roads]
    for i, a in enumerate(roads):
        for j, b in enumerate(roads[:i]):
            if any(
                a["tags"].get(k, "0" if k == "layer" else "no")
                != b["tags"].get(k, "0" if k == "layer" else "no")
                for k in ("layer", "bridge", "tunnel")
            ):
                continue
            hit = a["line"].intersection(b["line"])
            points = (
                [hit]
                if isinstance(hit, Point)
                else [p for p in getattr(hit, "geoms", []) if isinstance(p, Point)]
            )
            insertions[i].extend((p, {}) for p in points)
            insertions[j].extend((p, {}) for p in points)
    for road, extra in zip(roads, insertions):
        line, data = road["line"], road["tags"]
        extra.extend(
            (line.interpolate(c["fraction"], normalized=True), c["tags"]) for c in road["controls"]
        )
        points = [(line.project(Point(p)), Point(p), {}) for p in line.coords]
        points += [(line.project(p), p, t) for p, t in extra]
        refs = []
        for _, point, nt in sorted(points, key=lambda item: item[0]):
            key = (round(point.x, 3), round(point.y, 3), data.get("layer", "0"))
            if key not in nodes:
                nid = converter._node_id_gen.next()
                if inverse:
                    lon, lat = inverse.transform(point.x, point.y)
                else:
                    lat, lon = converter._xy_to_latlon_if_needed((point.x, point.y))
                nodes[key] = Node(
                    nid, lat, lon, tag_dict={"local_x": str(point.x), "local_y": str(point.y)}
                )
                converter._out.add_node(nodes[key])
            nodes[key].tag_dict.update(nt)
            if not refs or refs[-1] != nodes[key].id_:
                refs.append(nodes[key].id_)
        converter._out.add_way(Way(converter._way_id_gen.next(), refs, data))
    return True
