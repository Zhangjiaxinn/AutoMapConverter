"""Declared OSM/Lanelet2 detail layer; never a serialized source-map snapshot.

Geometry is stored as current WGS84 nodes/ways. Lanelet2 owns lane/rule roles;
two small relation types declare the profile and bind its road/detail views.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString

from automap_converter.core.traceability import (
    document_id,
    read_frame,
    write_frame,
)
from automap_converter.formats.lanelet2.shared_geometry import sort_native_objects

PROFILE = "lanelet2-v1"
DECLARATION = "automap:map_profile"
EVENT = "automap:input_key"
NATIVE_TAGS = frozenset(
    {
        "type",
        "subtype",
        "location",
        "region",
        "name",
        "ref",
        "color",
        "width",
        "height",
        "ele",
        "layer",
        "bridge",
        "tunnel",
        "area",
        "one_way",
        "participant",
        "speed_limit",
        "speed_limit_unit",
        "sign_type",
        "traffic_sign",
        "dynamic",
        "direction",
        "highway",
        "crossing",
        "traffic_signals",
        "turn_direction",
        "lane_change",
        "fallback",
        "cancel",
    }
)


def _native_tags(element):
    # Do not decode provenance chunks or carry ancestral reconstruction fields.
    return {
        t.get("k"): t.get("v", "")
        for t in element.findall("tag")
        if t.get("k") in NATIVE_TAGS
        or t.get("k", "").startswith(("participant:", "one_way:", "traffic_sign:"))
    }


def _tag(element, key, value):
    value = str(value)
    if len(key) > 255 or len(value) > 255:
        raise ValueError("Detail profile tags must fit native OSM key/value limits")
    etree.SubElement(element, "tag", k=key, v=value)


def _index(root, *, allow_events=False, strict_tags=True):
    result = {}
    for element in root:
        if element.tag not in {"node", "way", "relation"}:
            continue
        key = (element.tag, element.get("id"))
        if not -(2**63) <= int(key[1]) < 2**63:
            raise ValueError("OSM ID exceeds signed 64-bit range")
        if key in result:
            raise ValueError("Duplicate OSM object: " + str(key))
        fields = [(tag.get("k", ""), tag.get("v", "")) for tag in element.findall("tag")]
        if len({k for k, _ in fields}) != len(fields):
            raise ValueError("Duplicate OSM tag key: " + str(key))
        if strict_tags and any(
            (len(k) > 255 or len(v) > 255)
            and not (allow_events and k in {EVENT, "automap:lineage_selectors"})
            for k, v in fields
        ):
            raise ValueError("OSM profile tag exceeds 255 characters")
        if element.tag == "node":
            lon, lat = float(element.get("lon")), float(element.get("lat"))
            if not math.isfinite(lon + lat) or not -180 <= lon <= 180 or not -90 <= lat <= 90:
                raise ValueError("Invalid WGS84 profile node")
        result[key] = element
    return result


def _closure(index, seeds):
    selected, pending = set(), list(seeds)
    while pending:
        key = pending.pop()
        if key in selected:
            continue
        if key not in index:
            raise ValueError("Dangling detail-profile reference: " + str(key))
        selected.add(key)
        element = index[key]
        pending.extend(("node", n.get("ref")) for n in element.findall("nd"))
        pending.extend((m.get("type"), m.get("ref")) for m in element.findall("member"))
    return selected


def declared_lanes(root, *, allow_events=False):
    declarations = [
        r for r in root.findall("relation") if _native_tags(r).get("type") == DECLARATION
    ]
    if not declarations:
        return None
    if len(declarations) != 1:
        raise ValueError("Exactly one detail-profile declaration is required")
    declaration = declarations[0]
    data = {t.get("k"): t.get("v") for t in declaration.findall("tag")}
    if data.get("automap:profile") != PROFILE:
        raise ValueError("Unsupported OSM detail profile: " + str(data.get("automap:profile")))
    members = declaration.findall("member")
    if not members or any(
        m.get("type") != "relation" or m.get("role") not in {"lanelet", "road_detail"}
        for m in members
    ):
        raise ValueError("Profile declaration must reference native Lanelet2 relations")
    seeds = {("relation", m.get("ref")) for m in members if m.get("role") == "lanelet"}
    if not seeds:
        raise ValueError("Detail profile contains no lanelets")
    index = _index(root, allow_events=allow_events)
    _closure(index, {("relation", declaration.get("id"))})
    for key in seeds:
        if key not in index or _native_tags(index[key]).get("type") != "lanelet":
            raise ValueError("Profile member is not a Lanelet2 lanelet: " + str(key))
        roles = [m.get("role") for m in index[key].findall("member")]
        if roles.count("left") != 1 or roles.count("right") != 1 or roles.count("centerline") > 1:
            raise ValueError("Lanelet2 requires exactly one left/right boundary")
        if any(r not in {"left", "right", "centerline", "regulatory_element"} for r in roles):
            raise ValueError("Unexpected Lanelet2 member role")
        if any(
            m.get("type") != ("relation" if m.get("role") == "regulatory_element" else "way")
            for m in index[key].findall("member")
        ):
            raise ValueError("Incorrect Lanelet2 native member type")
    selected = _closure(index, seeds)
    _validate_road_links(root, index, declaration)
    return selected


def _validate_road_links(root, index, declaration):
    first = root.find("node")
    lon, lat = float(first.get("lon")), float(first.get("lat"))
    zone = min(60, max(1, int((lon + 180) / 6) + 1))
    forward = Transformer.from_crs(4326, (32600 if lat >= 0 else 32700) + zone, always_xy=True)

    def line(way):
        return LineString(
            [
                forward.transform(
                    float(index[("node", n.get("ref"))].get("lon")),
                    float(index[("node", n.get("ref"))].get("lat")),
                )
                for n in way.findall("nd")
            ]
        )

    for member in declaration.findall("member"):
        if member.get("role") != "road_detail":
            continue
        link = index.get(("relation", member.get("ref")))
        if link is None or _native_tags(link).get("type") != "automap:road_detail":
            raise ValueError("Invalid road/detail binding")
        roads = [m for m in link.findall("member") if m.get("role") == "road"]
        lanes = [
            m
            for m in link.findall("member")
            if m.get("role", "").startswith(("forward:", "backward:"))
        ]
        if len(roads) != 1 or not lanes:
            raise ValueError("Road/detail binding needs one road and actual lanelets")
        road = index[("way", roads[0].get("ref"))]
        data = {t.get("k"): t.get("v") for t in road.findall("tag")}
        centers, widths = [], []
        for item in lanes:
            lane = index[("relation", item.get("ref"))]
            if _native_tags(lane).get("type") != "lanelet":
                raise ValueError("Road detail member is not a lanelet")
            direction, number = item.get("role").split(":", 1)
            if not number.isdigit() or int(number) < 1:
                raise ValueError("Invalid directional lane slot in road/detail binding")
            lane_index = int(number) - 1
            public_value = data.get("maxspeed")
            for name in ("maxspeed:lanes:" + direction, "maxspeed:lanes"):
                if data.get(name):
                    values = data[name].split("|")
                    if lane_index >= len(values):
                        raise ValueError(
                            "Public lane-speed list is shorter than the declared lane slots"
                        )
                    public_value = values[lane_index] or public_value
                    break
            native_values = []
            for member in lane.findall("member"):
                if member.get("role") != "regulatory_element":
                    continue
                rule = _native_tags(index[("relation", member.get("ref"))])
                if rule.get("subtype") == "speed_limit":
                    native_values.append(rule.get("sign_type", ""))
            public_speed = _speed_mps(public_value)
            native_speeds = [v for raw in native_values if (v := _speed_mps(raw)) is not None]
            if (
                public_speed is not None
                and native_speeds
                and any(abs(v - public_speed) > 1e-4 for v in native_speeds)
            ):
                raise ValueError("Public OSM speed conflicts with declared Lanelet2 rules")
            boundaries = [
                line(index[("way", lane.find(f"member[@role='{role}']").get("ref"))])
                for role in ("left", "right")
            ]
            widths.append(
                boundaries[0]
                .interpolate(0.5, normalized=True)
                .distance(boundaries[1].interpolate(0.5, normalized=True))
            )
            centers.append(
                LineString(
                    [
                        (
                            (
                                boundaries[0].interpolate(f, normalized=True).x
                                + boundaries[1].interpolate(f, normalized=True).x
                            )
                            / 2,
                            (
                                boundaries[0].interpolate(f, normalized=True).y
                                + boundaries[1].interpolate(f, normalized=True).y
                            )
                            / 2,
                        )
                        for f in (0, 0.25, 0.5, 0.75, 1)
                    ]
                )
            )
        width = sum(widths)
        if data.get("width") and abs(float(data["width"]) - width) > max(0.05, 0.01 * width):
            raise ValueError("Public OSM width conflicts with declared Lanelet2 detail geometry")
        axis = line(road)
        # A generalized road axis may vary within its carriageway, but cannot
        # silently move away from the declared detail layer.
        if any(
            min(c.distance(axis.interpolate(f, normalized=True)) for c in centers) > width / 2 + 0.5
            for f in (0.25, 0.5, 0.75)
        ):
            raise ValueError("Public OSM geometry conflicts with declared Lanelet2 detail geometry")


def _speed_mps(value):
    match = re.fullmatch(r"\s*([\d.eE+-]+)\s*(km/h|kmh|kph|m/s|mps|mph)?\s*", str(value or ""))
    if match is None:
        return None
    try:
        speed = (
            float(match[1])
            * {"km/h": 1 / 3.6, "kmh": 1 / 3.6, "kph": 1 / 3.6, "m/s": 1, "mps": 1, "mph": 0.44704}[
                match[2] or "km/h"
            ]
        )
    except ValueError:
        return None
    return speed if math.isfinite(speed) and speed >= 0 else None


def append_detail_layer(source: Path, target: Path, encoding_crs: str):
    src = etree.parse(str(source)).getroot()
    tree = etree.parse(str(target))
    dst = tree.getroot()
    if declared_lanes(dst) is not None:
        raise ValueError("Target already has a detail layer")
    index = _index(src, strict_tags=False)
    seeds = {
        key
        for key, e in index.items()
        if e.tag == "relation" and _native_tags(e).get("type") == "lanelet"
    }
    if not seeds:
        raise ValueError("Cannot export an empty Lanelet2 detail layer")
    selected = _closure(index, seeds)
    next_id = min([0] + [int(e.get("id")) for e in dst if e.get("id")]) - 1
    identities = {}
    for key in sorted(selected):
        identities[key] = str(next_id)
        next_id -= 1
    frame = read_frame(src, "lanelet2", fallback_id=document_id(source))
    inverse = Transformer.from_crs(
        frame.get("crs") or frame.get("encoding_crs") or encoding_crs, 4326, always_xy=True
    )
    for key in sorted(selected):
        original = index[key]
        attrs = {"id": identities[key], "version": "1"}
        if original.tag == "node":
            data = {t.get("k"): t.get("v") for t in original.findall("tag")}
            if frame.get("kind") != "geographic" and "local_x" in data and "local_y" in data:
                lon, lat = inverse.transform(float(data["local_x"]), float(data["local_y"]))
            else:
                lon, lat = float(original.get("lon")), float(original.get("lat"))
            if not math.isfinite(lon + lat) or not -180 <= lon <= 180 or not -90 <= lat <= 90:
                raise ValueError("Invalid WGS84 detail node")
            attrs.update(lon=str(lon), lat=str(lat))
        element = etree.SubElement(dst, original.tag, **attrs)
        for n in original.findall("nd"):
            etree.SubElement(element, "nd", ref=identities[("node", n.get("ref"))])
        for m in original.findall("member"):
            etree.SubElement(
                element,
                "member",
                type=m.get("type"),
                ref=identities[(m.get("type"), m.get("ref"))],
                role=m.get("role", ""),
            )
        for k, v in _native_tags(original).items():
            _tag(element, k, v)
        _tag(element, EVENT, key[1] if key in seeds else key[0] + ":" + key[1])
    declaration = etree.SubElement(dst, "relation", id=str(next_id), version="1")
    for key in sorted(seeds):
        etree.SubElement(
            declaration, "member", type="relation", ref=identities[key], role="lanelet"
        )
    for road in list(dst.findall("way")):
        data = {t.get("k"): t.get("v") for t in road.findall("tag")}
        lane_ids = data.get("lanelet2:lanelet_ids", "").split(",")
        lane_keys = [("relation", rid) for rid in lane_ids if ("relation", rid) in seeds]
        if not data.get("highway") or not lane_keys:
            continue
        next_id -= 1
        link = etree.SubElement(dst, "relation", id=str(next_id), version="1")
        etree.SubElement(link, "member", type="way", ref=road.get("id"), role="road")
        slots = json.loads(data["automap:lineage_selectors"])
        for key in lane_keys:
            slot = slots[key[1]]
            role = ("forward:" if slot["forward"] else "backward:") + str(slot["lane_index"])
            etree.SubElement(link, "member", type="relation", ref=identities[key], role=role)
        _tag(link, "type", "automap:road_detail")
        etree.SubElement(
            declaration, "member", type="relation", ref=str(next_id), role="road_detail"
        )
    _tag(declaration, "type", DECLARATION)
    _tag(declaration, "automap:profile", PROFILE)
    declared_lanes(dst, allow_events=True)
    sort_native_objects(dst)
    tree.write(str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True)


def read_detail_layer(source: Path, target: Path, crs: str):
    src = etree.parse(str(source)).getroot()
    selected = declared_lanes(src)
    if selected is None:
        return False
    index = _index(src)
    root = etree.Element("osm", version="0.6", generator="AutoMapConverter Lanelet2 profile reader")
    forward = Transformer.from_crs(4326, crs, always_xy=True)
    for key in sorted(selected):
        original = index[key]
        attrs = {"id": key[1], "version": "1"}
        if original.tag == "node":
            attrs.update(lon=original.get("lon"), lat=original.get("lat"))
        element = etree.SubElement(root, original.tag, **attrs)
        for n in original.findall("nd"):
            etree.SubElement(element, "nd", ref=n.get("ref"))
        for m in original.findall("member"):
            etree.SubElement(
                element, "member", type=m.get("type"), ref=m.get("ref"), role=m.get("role", "")
            )
        for k, v in _native_tags(original).items():
            _tag(element, k, v)
        if original.tag == "node":
            x, y = forward.transform(float(attrs["lon"]), float(attrs["lat"]))
            _tag(element, "local_x", x)
            _tag(element, "local_y", y)
        _tag(element, EVENT, key[0] + ":" + key[1])
    write_frame(root, "lanelet2", {"kind": "projected", "crs": crs, "units": "m", "axis": "x,y"})
    sort_native_objects(root)
    target.parent.mkdir(parents=True, exist_ok=True)
    etree.ElementTree(root).write(
        str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True
    )
    return True
