"""Road ways plus optional OSM extension relations for lane-level reconstruction.

The extension stores normal offsets/widths along the generated road, not a
cached source polyline. Ordinary OSM consumers can ignore these relations.
"""

import math
from itertools import pairwise

import networkx as nx
import numpy as np
from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point

from automap_converter.core.traceability import tags
from automap_converter.formats.lanelet2.geometry import orient_boundaries
from automap_converter.formats.lanelet2.osm_model import Node, Relation, RelationMember, Way

PROFILE = "automap:lane_profile"
GEOMETRY_PROFILE = "automap:road_relative_geometry"
FRACTIONS = np.linspace(0, 1, 9)


def road_axes(road, fraction):
    p = road.interpolate(float(fraction), normalized=True)
    a = road.interpolate(max(0, fraction-1e-6), normalized=True)
    b = road.interpolate(min(1, fraction+1e-6), normalized=True)
    tangent = np.array([b.x-a.x,b.y-a.y])
    length = np.linalg.norm(tangent)
    if length <= 1e-12:
        raise ValueError("Degenerate road-relative geometry reference")
    tangent /= length
    return np.array(p.coords[0]), tangent, np.array([-tangent[1],tangent[0]])


def encode_geometry(road, points):
    """Road-local Frenet coordinates, including end-cap longitudinal residuals."""
    values = []
    for a,b in pairwise(points):
        for t in np.linspace(0,1,max(2,math.ceil(math.dist(a,b)/.5)+1))[:-1]:
            values.append(np.array(a)+(np.array(b)-np.array(a))*t)
    values.append(np.array(points[-1]))
    stations, lateral, longitudinal = [], [], []
    for point in values:
        f = road.project(Point(point), normalized=True)
        origin,tangent,normal = road_axes(road,f)
        stations.append(f)
        lateral.append(float(np.dot(point-origin,normal)))
        longitudinal.append(float(np.dot(point-origin,tangent)))
    return {"stations":','.join(f'{v:.12g}' for v in stations),
            "lateral_offsets":','.join(f'{v:.12g}' for v in lateral),
            "longitudinal_offsets":','.join(f'{v:.12g}' for v in longitudinal)}


def decode_geometry(road, data):
    arrays = [[float(v) for v in data[k].split(',')] for k in ('stations','lateral_offsets','longitudinal_offsets')]
    if not 2 <= len(arrays[0]) <= 100000 or len({len(a) for a in arrays})!=1 or not all(math.isfinite(v) for a in arrays for v in a) or any(not 0<=s<=1 for s in arrays[0]):
        raise ValueError('Invalid road-relative geometry profile')
    points = []
    for s,t,u in zip(*arrays):
        origin,tangent,normal = road_axes(road,s)
        points.append(tuple(origin+t*normal+u*tangent))
    return points


def _boundaries(converter, lane):
    lines = []
    for wid in (lane.left_way, lane.right_way):
        way = converter._source.ways[str(wid)]
        lines.append(
            LineString([converter._node_xy(converter._source.nodes[str(n)]) for n in way.nodes])
        )
    return orient_boundaries(*lines)


def export_lane_profiles(converter):
    source, out = converter._source, converter._out
    lanes = source.way_relations
    frame = getattr(source, "coordinate_frame", {})
    from automap_converter.core.config.general_config import general_config

    crs = (
        (frame.get("crs") or frame.get("encoding_crs") or general_config.proj_string_cr)
        if frame.get("kind") != "geographic"
        else None
    )
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True) if crs else None
    pairs = {k: _boundaries(converter, lane) for k, lane in lanes.items()}
    graph = nx.Graph()
    graph.add_nodes_from(lanes)
    boundary_users = {}
    starts, ends, endpoints = {}, {}, {}
    for key, lane in lanes.items():
        for wid in (lane.left_way, lane.right_way):
            boundary_users.setdefault(str(wid), []).append(key)
        refs = []
        for wid, line in zip((lane.left_way, lane.right_way), pairs[key]):
            nodes = list(source.ways[str(wid)].nodes)
            first = converter._node_xy(source.nodes[str(nodes[0])])
            if np.linalg.norm(np.array(first) - np.array(line.coords[0])) > 1e-6:
                nodes.reverse()
            refs.append(nodes)
        start, end = (refs[0][0], refs[1][0]), (refs[0][-1], refs[1][-1])
        endpoints[key] = (start, end)
        starts.setdefault(start, []).append(key)
        ends.setdefault(end, []).append(key)
    for users in boundary_users.values():
        for i, a in enumerate(users):
            for b in users[:i]:
                pa, pb = pairs[a][0], pairs[b][0]
                va = np.array(pa.coords[-1]) - np.array(pa.coords[0])
                vb = np.array(pb.coords[-1]) - np.array(pb.coords[0])
                if (
                    np.dot(va, vb) > 0.9 * np.linalg.norm(va) * np.linalg.norm(vb)
                    and min(pa.length, pb.length) / max(pa.length, pb.length) > 0.9
                ):
                    graph.add_edge(a, b)
    rel_next = (
        min(
            [0]
            + [
                int(k)
                for k in list(source.way_relations)
                + list(source.regulatory_elements)
                + list(source.multipolygons)
            ]
        )
        - 1
    )
    profiles = {}
    for key in lanes:
        profiles[key] = str(rel_next)
        rel_next -= 1
    geometries = {}

    def add_geometry(way_key, points, road, road_id, endpoints=None, boundary_tags=None):
        nonlocal rel_next
        if way_key not in geometries:
            gid = str(rel_next)
            rel_next -= 1
            data = {"type":GEOMETRY_PROFILE, **encode_geometry(road,points)}
            if endpoints:
                data.update(endpoint_start=str(endpoints[0]),endpoint_end=str(endpoints[1]))
            data.update({f'boundary:{k}':v for k,v in (boundary_tags or {}).items()
                         if k in {'type','subtype','lane_change','width','height'}})
            out.add_relation(Relation(gid,[RelationMember('way',road_id,'road')],data))
            geometries[way_key] = gid
        return geometries[way_key]
    node_cache = {}

    def node(point):
        key = tuple(round(v, 6) for v in point)
        if key not in node_cache:
            nid = converter._node_id_gen.next()
            lon, lat = (
                inverse.transform(*point)
                if inverse
                else converter._xy_to_latlon_if_needed(point)[::-1]
            )
            out.add_node(
                Node(nid, lat, lon, tag_dict={"local_x": str(point[0]), "local_y": str(point[1])})
            )
            node_cache[key] = nid
        return node_cache[key]

    excluded = {
        "walkway",
        "pedestrian",
        "crosswalk",
        "bicycle_lane",
        "bike_lane",
        "shoulder",
        "border",
    }
    for group in nx.connected_components(graph):
        vehicle = [k for k in group if lanes[k].tag_dict.get("subtype") not in excluded]
        road_members = vehicle or list(group)
        ref = pairs[road_members[0]][0]
        p, q = ref.interpolate(0.49, normalized=True), ref.interpolate(0.51, normalized=True)
        normal = np.array([-(q.y - p.y), q.x - p.x])
        outer = sorted(
            [line for k in road_members for line in pairs[k]],
            key=lambda line: np.dot(normal, line.interpolate(0.5, normalized=True).coords[0]),
        )
        road_points = [
            tuple(
                (
                    np.array(outer[0].interpolate(float(f), normalized=True).coords[0])
                    + np.array(outer[-1].interpolate(float(f), normalized=True).coords[0])
                )
                / 2
            )
            for f in np.linspace(0, 1, 40)
        ]
        road = LineString(road_points)
        group_lanes = [lanes[k] for k in group]
        data = converter._build_centerline_way_tags(
            group_lanes, (group_lanes[0].left_way, group_lanes[-1].right_way)
        )
        road_widths = [
            pairs[k][0]
            .interpolate(0.5, normalized=True)
            .distance(pairs[k][1].interpolate(0.5, normalized=True))
            for k in road_members
        ]
        data.update(
            {
                "highway": "unclassified"
                if vehicle
                else "cycleway"
                if lanes[road_members[0]].tag_dict.get("subtype") in {"bicycle_lane", "bike_lane"}
                else "footway",
                "lanes": str(len(vehicle) or 1),
                "width": f"{sum(road_widths):.6f}",
                "width:lanes": "|".join(f"{w:.6f}" for w in road_widths),
                "oneway": "yes" if vehicle else "no",
                "lanelet2:lanelet_ids": ",".join(sorted(group)),
            }
        )
        wid = converter._way_id_gen.next()
        out.add_way(Way(wid, [node(point) for point in road_points], data))
        for key in group:
            left, right = pairs[key]
            offsets, widths = [], []
            for f in FRACTIONS:
                p = road.interpolate(float(f), normalized=True)
                a, b = (
                    road.interpolate(max(0, f - 1e-4), normalized=True),
                    road.interpolate(min(1, f + 1e-4), normalized=True),
                )
                tangent = np.array([b.x - a.x, b.y - a.y])
                normal = np.array([-tangent[1], tangent[0]]) / np.linalg.norm(tangent)
                l, r = (
                    left.interpolate(float(f), normalized=True),
                    right.interpolate(float(f), normalized=True),
                )
                center = (np.array(l.coords[0]) + np.array(r.coords[0])) / 2
                offsets.append(round(float(np.dot(center - np.array(p.coords[0]), normal)), 8))
                widths.append(round(l.distance(r), 8))
            profile_tags = {
                "type": PROFILE,
                "lanelet2:lanelet_ids": key,
                "fractions": ",".join(f"{v:g}" for v in FRACTIONS),
                "offsets": ",".join(map(str, offsets)),
                "widths": ",".join(map(str, widths)),
            }
            for k in (
                "subtype",
                "location",
                "one_way",
                "one_way:vehicle",
                "automap:od_lane_type",
                "automap:od_speeds",
            ):
                if k in lanes[key].tag_dict:
                    profile_tags[k] = lanes[key].tag_dict[k]
            members = [RelationMember("way", wid, "road")]
            for role,boundary_id in (('left',lanes[key].left_way),('right',lanes[key].right_way)):
                boundary = source.ways[str(boundary_id)]
                coords = [converter._node_xy(source.nodes[str(n)]) for n in boundary.nodes]
                gid = add_geometry(('boundary',str(boundary_id)),coords,road,wid,(boundary.nodes[0],boundary.nodes[-1]),boundary.tag_dict)
                members.append(RelationMember('relation',gid,role))
            count=max(8,min(40,max(len(left.coords),len(right.coords))))
            center=[tuple((np.array(left.interpolate(f,normalized=True).coords[0])+np.array(right.interpolate(f,normalized=True).coords[0]))/2) for f in np.linspace(0,1,count)]
            center_id=add_geometry(('centerline',key),center,road,wid)
            members.append(RelationMember('relation',center_id,'centerline'))
            profile_tags['geometry_version']='2'
            for role, field in (
                ("predecessor", "predecessors"),
                ("successor", "successors"),
                ("left", "adjacent_left"),
                ("right", "adjacent_right"),
            ):
                if any(k.startswith("automap:topology:") for k in lanes[key].tag_dict):
                    neighbors = getattr(lanes[key], field, [])
                elif role in {"left", "right"}:
                    neighbors = boundary_users.get(
                        str(lanes[key].left_way if role == "left" else lanes[key].right_way), []
                    )
                else:
                    start, end = endpoints[key]
                    neighbors = (
                        ends.get(start, []) if role == "predecessor" else starts.get(end, [])
                    )
                members.extend(
                    RelationMember("relation", profiles[n], role)
                    for n in neighbors
                    if n in profiles and n != key
                )
            members.extend(
                RelationMember("relation", str(r), "regulatory_element")
                for r in lanes[key].regulatory_elements
            )
            out.add_relation(Relation(profiles[key], members, profile_tags))
    # Regulatory geometry is part of the converted map, not a hidden metadata
    # answer. Reuse original object IDs, disjoint from newly allocated road IDs.
    for reg in source.regulatory_elements.values():
        xml = reg.serialize_to_xml()
        members = []
        for m in xml.findall("member"):
            kind, ref, role = m.get("type"), m.get("ref"), m.get("role")
            if kind == "relation" and ref in profiles:
                ref = profiles[ref]
            if kind == "way" and ref in source.ways:
                way = source.ways[ref]
                for nid in way.nodes:
                    n = source.nodes[str(nid)]
                    out.add_node(
                        Node(
                            n.id_,
                            n.lat,
                            n.lon,
                            tag_dict={
                                "local_x": str(converter._node_xy(n)[0]),
                                "local_y": str(converter._node_xy(n)[1]),
                            },
                        )
                    )
                out.add_way(Way(way.id_, way.nodes, way.tag_dict))
            members.append(RelationMember(kind, ref, role))
        out.add_relation(
            Relation(str(reg.id_), members, {**reg.tag_dict, "type": "automap:regulatory_element"})
        )
    converter._convert_regulatory_elements_to_road_level_features()


def import_lane_profiles(source_path, target_path, crs):
    src = etree.parse(str(source_path)).getroot()
    profiles = [r for r in src.findall("relation") if tags(r).get("type") == PROFILE]
    if not profiles:
        return False
    forward = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    source_nodes = {n.get("id"): n for n in src.findall("node")}
    dst = etree.Element("osm", version="0.6", generator="AutoMapConverter lane-profile expansion")
    next_id = min([0] + [int(e.get("id")) for e in src if e.get("id")]) - 1

    def allocate():
        nonlocal next_id
        value = str(next_id)
        next_id -= 1
        return value

    ids = {r.get("id"): allocate() for r in profiles}
    nodes_cache, boundaries_cache = {}, {}
    endpoint_positions = {}

    def add_boundary(points, endpoint_ids=None):
        refs = []
        for index,(x, y) in enumerate(points):
            key = (round(x, 5), round(y, 5))
            if endpoint_ids and index in (0,len(points)-1):
                identity=endpoint_ids[0 if index==0 else 1]
                endpoint_key=('endpoint',identity)
                old=endpoint_positions.get(endpoint_key)
                if old is None or math.dist(old,(x,y))<=.05:
                    key=endpoint_key
                    endpoint_positions.setdefault(endpoint_key,(x,y))
            if key not in nodes_cache:
                nid = allocate()
                lon, lat = inverse.transform(x, y)
                node = etree.SubElement(
                    dst, "node", id=nid, lat=str(lat), lon=str(lon), version="1"
                )
                for k, v in (("local_x", x), ("local_y", y)):
                    etree.SubElement(node, "tag", k=k, v=str(v))
                nodes_cache[key] = nid
            refs.append(nodes_cache[key])
        signature = min(tuple(refs), tuple(reversed(refs)))
        if signature not in boundaries_cache:
            wid = allocate()
            way = etree.SubElement(dst, "way", id=wid, version="1")
            for nid in refs:
                etree.SubElement(way, "nd", ref=nid)
            etree.SubElement(way, "tag", k="type", v="virtual")
            boundaries_cache[signature] = wid
        return boundaries_cache[signature]

    native_geometry = {}
    for geometry in src.findall('relation'):
        data=tags(geometry)
        if data.get('type')!=GEOMETRY_PROFILE:
            continue
        road_id=next(m.get('ref') for m in geometry.findall('member') if m.get('role')=='road')
        way=src.find(f"way[@id='{road_id}']")
        if way is None or not tags(way).get('highway'):
            raise ValueError('Geometry profile references a missing road')
        road=LineString([forward.transform(float(source_nodes[n.get('ref')].get('lon')),float(source_nodes[n.get('ref')].get('lat'))) for n in way.findall('nd')])
        points=decode_geometry(road,data)
        endpoint_ids=(data['endpoint_start'],data['endpoint_end']) if 'endpoint_start' in data else None
        native_geometry[geometry.get('id')]=add_boundary(points,endpoint_ids)
        native_way=dst.find(f"way[@id='{native_geometry[geometry.get('id')]}']")
        for key,value in data.items():
            if key.startswith('boundary:'):
                field=key.split(':',1)[1]
                tag=native_way.find(f"tag[@k='{field}']")
                if tag is None:
                    etree.SubElement(native_way,'tag',k=field,v=value)
                else:
                    tag.set('v',value)

    for profile in profiles:
        data = tags(profile)
        road_id = next(m.get("ref") for m in profile.findall("member") if m.get("role") == "road")
        way = src.find(f"way[@id='{road_id}']")
        if way is None or not tags(way).get("highway"):
            raise ValueError("Lane profile references a missing/non-road way")
        points = []
        for n in way.findall("nd"):
            node = source_nodes[n.get("ref")]
            points.append(forward.transform(float(node.get("lon")), float(node.get("lat"))))
        road = LineString(points)
        fractions = [float(v) for v in data["fractions"].split(",")]
        offsets = [float(v) for v in data["offsets"].split(",")]
        widths = [float(v) for v in data["widths"].split(",")]
        if (
            len(fractions) != len(offsets)
            or len(fractions) != len(widths)
            or len(fractions) < 2
            or any(w <= 0 for w in widths)
            or not all(math.isfinite(v) for v in fractions + offsets + widths)
            or fractions[0] != 0
            or fractions[-1] != 1
            or any(b <= a for a, b in pairwise(fractions))
        ):
            raise ValueError("Invalid lane normal-offset/width profile")
        boundaries = [[], []]
        for f in np.linspace(0, 1, 40):
            p = road.interpolate(float(f), normalized=True)
            a, b = (
                road.interpolate(max(0, f - 1e-4), normalized=True),
                road.interpolate(min(1, f + 1e-4), normalized=True),
            )
            dx, dy = b.x - a.x, b.y - a.y
            length = math.hypot(dx, dy)
            if length <= 1e-9:
                raise ValueError("Degenerate generated OSM road")
            offset, width = np.interp(f, fractions, offsets), np.interp(f, fractions, widths)
            for sign, boundary in zip((1, -1), boundaries):
                distance = offset + sign * width / 2
                boundary.append((p.x - dy / length * distance, p.y + dx / length * distance))
        lane = etree.SubElement(dst, "relation", id=ids[profile.get("id")], version="1")
        for role, boundary in zip(("left", "right"), boundaries):
            geometry_id=next((m.get('ref') for m in profile.findall('member') if m.get('role')==role),None)
            ref=native_geometry.get(geometry_id) or add_boundary(boundary)
            etree.SubElement(lane, "member", type="way", ref=ref, role=role)
        center_id=next((m.get('ref') for m in profile.findall('member') if m.get('role')=='centerline'),None)
        if center_id in native_geometry:
            etree.SubElement(lane,'member',type='way',ref=native_geometry[center_id],role='centerline')
        for m in profile.findall("member"):
            if m.get("role") == "regulatory_element":
                etree.SubElement(lane, "member", **dict(m.attrib))
        data.update(
            {
                "type": "lanelet",
                "source:osm:ways": road_id,
                "source:osm:lane_profile": profile.get("id"),
            }
        )
        for k in ("lanelet2:lanelet_ids", "fractions", "offsets", "widths"):
            data.pop(k, None)
        for role in ("predecessor", "successor", "left", "right"):
            data[f"automap:topology:{role}"] = ",".join(
                ids[m.get("ref")]
                for m in profile.findall("member")
                if m.get("role") == role and m.get("ref") in ids
            )
        for k, v in data.items():
            etree.SubElement(lane, "tag", k=k, v=v)
    auxiliary_ways = set()
    for reg in src.findall("relation"):
        if tags(reg).get("type") != "automap:regulatory_element":
            continue
        copy = etree.fromstring(etree.tostring(reg))
        for t in copy.findall("tag"):
            if t.get("k") == "type":
                t.set("v", "regulatory_element")
        for m in copy.findall("member"):
            if m.get("type") == "relation" and m.get("ref") in ids:
                m.set("ref", ids[m.get("ref")])
            elif m.get("type") == "way":
                auxiliary_ways.add(m.get("ref"))
        dst.append(copy)
    for wid in auxiliary_ways:
        way = src.find(f"way[@id='{wid}']")
        if way is None:
            raise ValueError("Missing regulatory geometry in augmented OSM")
        dst.append(etree.fromstring(etree.tostring(way)))
        for nd in way.findall("nd"):
            node = source_nodes[nd.get("ref")]
            if dst.find(f"node[@id='{node.get('id')}']") is None:
                copy = etree.fromstring(etree.tostring(node))
                xy = forward.transform(float(node.get("lon")), float(node.get("lat")))
                for t in list(copy.findall("tag")):
                    if t.get("k") in {"local_x", "local_y"}:
                        copy.remove(t)
                for k, v in zip(("local_x", "local_y"), xy):
                    etree.SubElement(copy, "tag", k=k, v=str(v))
                dst.append(copy)
    etree.ElementTree(dst).write(
        str(target_path), encoding="UTF-8", xml_declaration=True, pretty_print=True
    )
    return True
