"""Materialize coincident boundary/endpoint objects in native Lanelet2 XML."""

import math

from pyproj import Transformer
from shapely.geometry import LineString
from shapely.strtree import STRtree

from automap_converter.core.traceability import tags


def sort_native_objects(root):
    from automap_converter.formats.lanelet2.osm_model import _osm_id_sort_key

    root[:] = sorted(
        root,
        key=lambda e: (
            {"node": 0, "way": 1, "relation": 2}.get(e.tag, -1) if isinstance(e.tag, str) else -1,
            _osm_id_sort_key(e.get("id", "0")) if isinstance(e.tag, str) else (0, 0),
        ),
    )


def native_relations(lanelet_map):
    """Build physical relations from the current map's native objects."""
    from automap_converter.formats.lanelet2.geometry import orient_boundaries

    first = next(iter(lanelet_map.nodes.values()), None)
    if first is None:
        return
    zone = min(60, max(1, int((float(first.lon) + 180) // 6) + 1))
    projection = Transformer.from_crs(
        4326, (32600 if float(first.lat) >= 0 else 32700) + zone, always_xy=True
    )
    starts, ends, users = {}, {}, {}
    for key, lane in lanelet_map.way_relations.items():
        bounds = []
        for wid in (lane.left_way, lane.right_way):
            way = lanelet_map.ways[str(wid)]
            bounds.append(list(map(str, way.nodes)))
            users.setdefault(str(wid), []).append(str(key))

        def point(nid):
            node = lanelet_map.nodes[nid]
            return (
                (float(node.local_x), float(node.local_y))
                if node.local_x is not None
                else projection.transform(float(node.lon), float(node.lat))
            )

        left, right = bounds
        oriented = orient_boundaries(
            LineString([point(n) for n in left]), LineString([point(n) for n in right])
        )
        for refs, line in zip((left, right), oriented):
            if math.dist(point(refs[0]), line.coords[0]) > 1e-6:
                refs.reverse()
        starts.setdefault((left[0], right[0]), []).append(str(key))
        ends.setdefault((left[-1], right[-1]), []).append(str(key))
        lane._native_endpoints = ((left[0], right[0]), (left[-1], right[-1]))
    for key, lane in lanelet_map.way_relations.items():
        start, end = lane._native_endpoints
        lane.predecessors = [k for k in ends.get(start, []) if k != str(key)]
        lane.successors = [k for k in starts.get(end, []) if k != str(key)]
        lane.adjacent_left = [k for k in users.get(str(lane.left_way), []) if k != str(key)]
        lane.adjacent_right = [k for k in users.get(str(lane.right_way), []) if k != str(key)]


def consolidate_shared_geometry(root, tolerance=0.05):
    nodes = {n.get("id"): n for n in root.findall("node")}
    if not nodes:
        return {"shared_ways": 0, "shared_endpoints": 0}
    first = next(iter(nodes.values()))
    lon, lat = float(first.get("lon")), float(first.get("lat"))
    zone = min(60, max(1, int((lon + 180) // 6) + 1))
    projection = Transformer.from_crs(4326, (32600 if lat >= 0 else 32700) + zone, always_xy=True)
    xy = {
        key: (float(tags(n)["local_x"]), float(tags(n)["local_y"]))
        if "local_x" in tags(n) and "local_y" in tags(n)
        else projection.transform(float(n.get("lon")), float(n.get("lat")))
        for key, n in nodes.items()
    }
    elevation = {
        key: float(tags(n).get("ele", tags(n).get("local_z", 0))) for key, n in nodes.items()
    }
    lanes = root.xpath("relation[tag[@k='type' and @v='lanelet']]")
    used = {
        m.get("ref")
        for lane in lanes
        for m in lane.findall("member")
        if m.get("role") in {"left", "right"}
    }
    ways = {w.get("id"): w for w in root.findall("way") if w.get("id") in used}
    keys = sorted(ways)
    lines = [LineString([xy[n.get("ref")] for n in ways[key].findall("nd")]) for key in keys]
    heights = [
        sum(elevation[n.get("ref")] for n in ways[key].findall("nd")) / len(ways[key].findall("nd"))
        for key in keys
    ]
    tree = STRtree(lines)
    replace = {}
    for i, line in enumerate(lines):
        if keys[i] in replace:
            continue
        for j in tree.query(line.buffer(tolerance)):
            if j <= i or keys[int(j)] in replace:
                continue
            other = lines[int(j)]
            if (
                abs(heights[i] - heights[int(j)]) <= tolerance
                and line.hausdorff_distance(other) <= tolerance
                and abs(line.length - other.length) <= 2 * tolerance
            ):
                replace[keys[int(j)]] = keys[i]
    for lane in lanes:
        for member in lane.findall("member"):
            if member.get("role") in {"left", "right"} and member.get("ref") in replace:
                member.set("ref", replace[member.get("ref")])
    # Only actual boundary endpoints are eligible; no source topology declarations.
    endpoint_ids = {
        n.get("ref")
        for key, w in ways.items()
        if key not in replace
        for n in (w.findall("nd")[0], w.findall("nd")[-1])
    }
    bins = {}
    node_replace = {}
    for key in sorted(endpoint_ids):
        p = xy[key]
        cell = (math.floor(p[0] / tolerance), math.floor(p[1] / tolerance))
        candidates = [
            k
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for k in bins.get((cell[0] + dx, cell[1] + dy), [])
            if abs(elevation[key] - elevation[k]) <= tolerance and math.dist(p, xy[k]) <= tolerance
        ]
        if candidates:
            node_replace[key] = min(candidates)
        else:
            bins.setdefault(cell, []).append(key)
    for way in ways.values():
        for nd in way.findall("nd"):
            if nd.get("ref") in node_replace:
                nd.set("ref", node_replace[nd.get("ref")])
    return {
        "shared_ways": len(replace),
        "shared_endpoints": len(node_replace),
        "tolerance_m": tolerance,
    }
