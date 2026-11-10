"""Complete nonmotor roads omitted by the vehicle-oriented CommonRoad engine."""

import json

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point

from automap_converter.core.traceability import tags
from automap_converter.formats.osm.road_semantics import road_profile


def complete_nonmotor_roads(source, target, crs):
    src = etree.parse(str(source)).getroot()
    tree = etree.parse(str(target))
    dst = tree.getroot()
    seen = {
        v for r in dst.findall("relation") for v in tags(r).get("source:osm:ways", "").split(",")
    }
    nodes = {n.get("id"): n for n in src.findall("node")}
    forward = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    inverse = Transformer.from_crs(crs, "EPSG:4326", always_xy=True)
    next_id = min([0] + [int(e.get("id")) for e in dst if e.get("id")]) - 1

    def allocate():
        nonlocal next_id
        value = str(next_id)
        next_id -= 1
        return value

    for way in src.findall("way"):
        data = tags(way)
        if way.get("id") in seen or data.get("highway") not in {
            "footway",
            "path",
            "cycleway",
            "pedestrian",
            "steps",
        }:
            continue
        coords = [
            forward.transform(
                float(nodes[n.get("ref")].get("lon")), float(nodes[n.get("ref")].get("lat"))
            )
            for n in way.findall("nd")
        ]
        if len(coords) < 2:
            continue
        profile = road_profile(data)
        line = LineString(coords)
        boundaries = [
            line.offset_curve(d, join_style=2)
            for d in (profile["width_m"] / 2, -profile["width_m"] / 2)
        ]
        if any(b.geom_type != "LineString" or b.is_empty for b in boundaries):
            continue
        ids = []
        for boundary in boundaries:
            refs = []
            for x, y in boundary.coords:
                nid = allocate()
                lon, lat = inverse.transform(x, y)
                node = etree.SubElement(
                    dst, "node", id=nid, lat=str(lat), lon=str(lon), version="1"
                )
                for k, v in (("local_x", x), ("local_y", y)):
                    etree.SubElement(node, "tag", k=k, v=str(v))
                refs.append(nid)
            wid = allocate()
            boundary_way = etree.SubElement(dst, "way", id=wid, version="1")
            for nid in refs:
                etree.SubElement(boundary_way, "nd", ref=nid)
            etree.SubElement(boundary_way, "tag", k="type", v="virtual")
            ids.append(wid)
        rel = etree.SubElement(dst, "relation", id=allocate(), version="1")
        for role, wid in zip(("left", "right"), ids):
            etree.SubElement(rel, "member", type="way", ref=wid, role=role)
        model = {
            **profile,
            "road_id": way.get("id"),
            "edge_id": way.get("id"),
            "segment_index": 0,
            "tags": {
                k: v
                for k, v in data.items()
                if k in {"highway", "name", "ref", "layer", "bridge", "tunnel"}
            },
            "forward_lane": True,
            "reversed": False,
            "connector": False,
        }
        for k, v in {
            "type": "lanelet",
            "subtype": "walkway" if data["highway"] != "cycleway" else "bicycle_lane",
            "one_way": profile['oneway'],
            "source:osm:ways": way.get("id"),
            "source:osm:lane_index": "1",
            "source:osm:forward": "yes",
            "automap:osm_road_model": json.dumps(model, sort_keys=True),
        }.items():
            etree.SubElement(rel, "tag", k=k, v=v)
    source_ways = {w.get("id"): w for w in src.findall("way")}
    for rel in dst.findall("relation"):
        raw = next((t for t in rel.findall("tag") if t.get("k") == "automap:osm_road_model"), None)
        if raw is None:
            continue
        model = json.loads(raw.get("v"))
        way = source_ways.get(model["road_id"])
        if way is None:
            continue
        refs = [n.get("ref") for n in way.findall("nd")]
        coords = [
            forward.transform(float(nodes[n].get("lon")), float(nodes[n].get("lat"))) for n in refs
        ]
        line = LineString(coords)
        model["controls"] = [
            {
                "fraction": line.project(Point(p), normalized=True),
                "tags": {
                    k: v
                    for k, v in tags(nodes[n]).items()
                    if k in {"highway", "crossing", "traffic_signals", "direction"}
                },
            }
            for n, p in zip(refs, coords)
            if tags(nodes[n]).get("highway") in {"crossing", "traffic_signals", "stop", "give_way"}
        ]
        raw.set("v", json.dumps(model, sort_keys=True))
    tree.write(str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True)
