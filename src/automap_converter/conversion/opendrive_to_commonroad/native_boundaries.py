"""Materialize shared Lanelet2 boundaries from the current native OpenDRIVE.

The existing importer owns rules and connectivity. Native road/lane-section
events identify its output lanes; no ancestral geometry or identity is read.
"""

import math
from copy import deepcopy

import numpy as np
from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString

from automap_converter.core.traceability import tags
from automap_converter.formats.lanelet2.geometry import orient_boundaries
from automap_converter.formats.lanelet2.shared_geometry import (
    consolidate_shared_geometry,
    sort_native_objects,
)
from automap_converter.formats.opendrive.parser.parser import parse_opendrive


def polynomial(records, station):
    active = [r for r in records if r.start_pos <= station + 1e-9]
    if not active:
        return 0.0
    record = max(active, key=lambda r: r.start_pos)
    ds = station - record.start_pos
    return sum(c * ds**i for i, c in enumerate(record.polynomial_coefficients))


def materialize_boundaries(source, target, crs):
    native = parse_opendrive(source)
    tree = etree.parse(str(target))
    root = tree.getroot()
    inverse = Transformer.from_crs(crs, 4326, always_xy=True)
    next_id = (
        min([0] + [int(e.get("id")) for e in root if isinstance(e.tag, str) and e.get("id")]) - 1
    )

    def allocate():
        nonlocal next_id
        value = str(next_id)
        next_id -= 1
        return value

    way_cache, node_cache, lane_geometry, lane_types = {}, {}, {}, {}

    def way(key, points, endpoint_ids=None):
        if key in way_cache:
            return way_cache[key]
        refs = []
        for index, (x, y) in enumerate(points):
            if endpoint_ids and index in (0, len(points) - 1):
                nid = endpoint_ids[0 if index == 0 else 1]
                if nid:
                    refs.append(nid)
                    continue
            signature = (key, round(x, 7), round(y, 7))
            if signature not in node_cache:
                nid = allocate()
                lon, lat = inverse.transform(x, y)
                node = etree.SubElement(
                    root, "node", id=nid, lon=str(lon), lat=str(lat), version="1"
                )
                for k, v in (("local_x", x), ("local_y", y)):
                    etree.SubElement(node, "tag", k=k, v=str(v))
                node_cache[signature] = nid
            refs.append(node_cache[signature])
        wid = allocate()
        element = etree.SubElement(root, "way", id=wid, version="1")
        for ref in refs:
            etree.SubElement(element, "nd", ref=ref)
        etree.SubElement(element, "tag", k="type", v="virtual")
        way_cache[key] = wid
        return wid

    native_classes = {}
    for road in native.roads:
        for index, section in enumerate(road.lanes.lane_sections):
            active = [record for record in road.types if record.start_pos <= section.sPos]
            road_type = (
                max(active, key=lambda record: record.start_pos).use_type if active else "unknown"
            )
            for lane in section.all_lanes:
                if lane.type == "driving" and road_type == "motorway":
                    native_classes[f"{road.id}|{index}|{lane.id}"] = "highway"
    skipped = {}
    for road in native.roads:
        # Preserve the established 3-D importer for banked/elevated roads until
        # the shared mesh has an explicit 3-D implementation.
        if any(
            any(abs(c) > 1e-12 for c in record.polynomial_coefficients)
            for record in road.elevation_profile.elevations
            + road.lateral_profile.superelevations
            + road.lateral_profile.crossfalls
        ):
            skipped[str(road.id)] = "nonplanar elevation or banking"
            continue
        if not road.driving_direction:
            skipped[str(road.id)] = "LHT geometry retained from established importer"
            continue
        headings = [
            road.plan_view.calc(float(s), compute_curvature=False)[1]
            for s in np.linspace(0, road.plan_view.length, 20)
        ]
        if any(
            abs(math.atan2(math.sin(h - headings[0]), math.cos(h - headings[0]))) > 0.01
            for h in headings
        ):
            skipped[str(road.id)] = (
                "curved geometry requires joint station fitting; retained without forced welding"
            )
            continue
        if any(l.has_border_record for s in road.lanes.lane_sections for l in s.all_lanes):
            skipped[str(road.id)] = (
                "native border-polynomial geometry retained from established importer"
            )
            continue
        sections = road.lanes.lane_sections
        for index, section in enumerate(sections):
            start = section.sPos
            end = sections[index + 1].sPos if index + 1 < len(sections) else road.plan_view.length
            if end <= start:
                continue
            events = {start, end}
            events.update(
                start + w.start_pos
                for lane in section.all_lanes
                for w in lane.widths + lane.borders
                if start <= start + w.start_pos <= end
            )
            events.update(
                r.start_pos for r in road.lanes.lane_offsets if start <= r.start_pos <= end
            )
            # Include native plan-view corners as well as width/section events.
            events.update(float(s) for s in road.plan_view._geo_lengths if start <= s <= end)
            stations = set(events)
            for a, b in zip(sorted(events), sorted(events)[1:]):
                stations.update(np.linspace(a, b, max(2, math.ceil((b - a) / 0.25) + 1)))
            stations = sorted(stations)
            boundaries = {0: []}
            ordered = [
                sorted(section.left_lanes, key=lambda l: abs(l.id)),
                sorted(section.right_lanes, key=lambda l: abs(l.id)),
            ]
            for group in ordered:
                for lane in group:
                    boundaries[lane.id] = []
            for station in stations:
                point, tangent, _, _ = road.plan_view.calc(station, compute_curvature=False)
                normal = np.array([-math.sin(tangent), math.cos(tangent)])
                offset = polynomial(road.lanes.lane_offsets, station)
                boundaries[0].append(tuple(point + offset * normal))
                for group in ordered:
                    lateral = offset
                    for lane in group:
                        sign = 1 if lane.id > 0 else -1
                        value = polynomial(lane.widths or lane.borders, station - start)
                        lateral = (
                            lateral + sign * max(0, value)
                            if lane.widths
                            else offset + sign * abs(value)
                        )
                        boundaries[lane.id].append(tuple(point + lateral * normal))
            for group in ordered:
                previous = 0
                for lane in group:
                    inner, outer = boundaries[previous], boundaries[lane.id]
                    # Both groups have their inner edge on their left in the
                    # normal driving direction (RHT); left lanes run against s.
                    reverse = lane.id > 0
                    left, right = (
                        (list(reversed(inner)), list(reversed(outer)))
                        if reverse
                        else (inner, outer)
                    )
                    lane_geometry[f"{road.id}|{index}|{lane.id}"] = (
                        left,
                        right,
                        (road.id, index, previous),
                        (road.id, index, lane.id),
                        reverse,
                    )
                    lane_types[f"{road.id}|{index}|{lane.id}"] = lane.type
                    previous = lane.id
    repaired = 0
    original_nodes = {n.get("id"): n for n in root.findall("node")}
    original_ways = {w.get("id"): w for w in root.findall("way")}
    original_bounds = {
        m.get("ref")
        for r in root.findall("relation")
        if tags(r).get("type") == "lanelet"
        for m in r.findall("member")
        if m.get("role") in {"left", "right", "centerline"}
    }
    for relation in list(root.findall("relation")):
        if tags(relation).get("type") != "lanelet":
            continue
        data = tags(relation)
        detailed = data.get("source:opendrive:lane_locations_detailed", "")
        keys = [item.replace(",", "|") for item in detailed.split(";") if item]
        if len(keys) == 1 and keys[0] in native_classes:
            tag = relation.find("tag[@k='subtype']")
            if tag is not None:
                tag.set("v", native_classes[keys[0]])
        if len(keys) != 1 or keys[0] not in lane_geometry:
            continue
        left, right, inner_key, outer_key, reverse = lane_geometry[keys[0]]
        subtype = {
            "driving": "road",
            "bus": "bus_lane",
            "biking": "bicycle_lane",
            "shoulder": "shoulder",
            "border": "border",
            "sidewalk": "walkway",
        }.get(lane_types[keys[0]])
        if keys[0] in native_classes:
            subtype = native_classes[keys[0]]
        if subtype:
            tag = relation.find("tag[@k='subtype']")
            if tag is None:
                etree.SubElement(relation, "tag", k="subtype", v=subtype)
            else:
                tag.set("v", subtype)
        old = []
        marking_tags = []
        for role in ("left", "right"):
            wid = relation.find(f"member[@role='{role}']").get("ref")
            refs = [n.get("ref") for n in original_ways[wid].findall("nd")]
            marking_tags.append(
                {
                    k: v
                    for k, v in tags(original_ways[wid]).items()
                    if k in {"type", "subtype", "lane_change", "width", "height"}
                }
            )
            points = [
                (
                    float(tags(original_nodes[n])["local_x"]),
                    float(tags(original_nodes[n])["local_y"]),
                )
                for n in refs
            ]
            old.append((refs, LineString(points)))
        oriented = orient_boundaries(old[0][1], old[1][1])
        for i, line in enumerate(oriented):
            if line.coords[0] != old[i][1].coords[0]:
                old[i] = (list(reversed(old[i][0])), line)
        # Only apply a jointly sampled interior when it is numerically
        # equivalent to this native importer's geometry. Curved/ambiguous
        # cases remain diagnosed instead of sacrificing geometry for topology.
        if any(
            LineString(points)
            .interpolate(f, normalized=True)
            .distance(old[i][1].interpolate(f, normalized=True))
            > 0.05
            for i, points in enumerate((left, right))
            for f in (0.25, 0.35, 0.5, 0.65, 0.75)
        ):
            continue
        # Keep the established native importer's connected end caps. Jointly
        # sampled interiors become shared objects; finite skew tails stay as
        # separate connected lanelets rather than being welded across metres.
        fractions = (0.1, 0.9)
        middle = []
        for points in (left, right):
            line = LineString(points)
            middle.append(
                [
                    tuple(line.interpolate(f, normalized=True).coords[0])
                    for f in np.linspace(*fractions, max(3, len(points) - 2))
                ]
            )
        middle_refs = []
        for key, points in zip((inner_key, outer_key), middle):
            canonical = list(reversed(points)) if reverse else points
            wid = way(("middle", key), canonical)
            refs = [n.get("ref") for n in root.find(f"way[@id='{wid}']").findall("nd")]
            middle_refs.append((wid, list(reversed(refs)) if reverse else refs))
        original = deepcopy(relation)
        for member in list(relation.findall("member")):
            if member.get("role") in {"left", "right", "centerline"}:
                relation.remove(member)
        for part in range(3):
            item = relation if part == 0 else deepcopy(original)
            if part:
                item.set("id", allocate())
                for member in list(item.findall("member")):
                    if member.get("role") in {"left", "right", "centerline"}:
                        item.remove(member)
                root.append(item)
            parts = []
            for i, role in enumerate(("left", "right")):
                if part == 1:
                    wid = middle_refs[i][0]
                    points = middle[i]
                else:
                    line = old[i][1]
                    fs = np.linspace(0, 0.1, 4) if part == 0 else np.linspace(0.9, 1, 4)
                    points = [tuple(line.interpolate(f, normalized=True).coords[0]) for f in fs]
                    if part == 0:
                        points[-1] = middle[i][0]
                        endpoints = (old[i][0][0], middle_refs[i][1][0])
                    else:
                        points[0] = middle[i][-1]
                        endpoints = (middle_refs[i][1][-1], old[i][0][-1])
                    wid = way(("cap", keys[0], part, role), points, endpoints)
                parts.append(points)
                etree.SubElement(item, "member", type="way", ref=wid, role=role)
                boundary = root.find(f"way[@id='{wid}']")
                for key, value in marking_tags[i].items():
                    tag = boundary.find(f"tag[@k='{key}']")
                    if tag is None:
                        etree.SubElement(boundary, "tag", k=key, v=value)
                    else:
                        tag.set("v", value)
            lines = [LineString(points) for points in parts]
            center = [
                tuple(
                    (
                        np.array(lines[0].interpolate(f, normalized=True).coords[0])
                        + np.array(lines[1].interpolate(f, normalized=True).coords[0])
                    )
                    / 2
                )
                for f in np.linspace(0, 1, max(len(p) for p in parts))
            ]
            wid = way(("center", keys[0], part), center)
            etree.SubElement(item, "member", type="way", ref=wid, role="centerline")
        repaired += 1
    stats = consolidate_shared_geometry(root)
    from automap_converter.formats.lanelet2.partial_geometry import share_partial_boundaries

    stats.update(share_partial_boundaries(root, crs))
    stats["after_interval_consolidation"] = consolidate_shared_geometry(root)
    referenced = {
        m.get("ref")
        for r in root.findall("relation")
        for m in r.findall("member")
        if m.get("type") == "way"
    }
    for element in list(root.findall("way")):
        if element.get("id") in original_bounds and element.get("id") not in referenced:
            root.remove(element)
    used_nodes = {n.get("ref") for w in root.findall("way") for n in w.findall("nd")}
    used_nodes.update(
        m.get("ref")
        for r in root.findall("relation")
        for m in r.findall("member")
        if m.get("type") == "node"
    )
    for element in list(root.findall("node")):
        if element.get("id") not in used_nodes and set(tags(element)) <= {
            "local_x",
            "local_y",
            "ele",
        }:
            root.remove(element)
    sort_native_objects(root)
    tree.write(str(target), encoding="utf-8", xml_declaration=True, pretty_print=True)
    return {"native_lane_geometries": repaired, "retained_roads": skipped, **stats}
