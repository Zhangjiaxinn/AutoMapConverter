"""Identity-only lineage and explicit coordinate frames for vector conversions.

Lineage never stores a source geometry/attribute snapshot to restore an answer.
Consumers must measure the generated map, not the accompanying audit record.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from lxml import etree
from pyproj import CRS, Geod, Transformer

LINEAGE = "automap:lineage"
FRAME = "automap:coordinate_frame"
SCHEMA = "automap-converter/lineage/v1"


def parse_xml(path):
    tree = etree.parse(str(path))
    for element in tree.iter():
        if isinstance(element.tag, str):
            element.tag = etree.QName(element).localname
    etree.cleanup_namespaces(tree)
    return tree


def document_id(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tags(element) -> dict[str, str]:
    result = {t.get("k", ""): t.get("v", "") for t in element.findall("tag")}
    for key, count in list(result.items()):
        if key.startswith("automap:chunks:"):
            name = key[len("automap:chunks:") :]
            legacy_profile = result.get("type", "").startswith("automap:")
            if not legacy_profile and not name.startswith(("automap:", "lanelet2:", "source:")):
                continue
            try:
                size = int(count)
                if not 0 < size <= 10000:
                    continue
                result[name] = "".join(result[f"automap:chunk:{name}:{i:04}"] for i in range(size))
            except (ValueError, KeyError):
                continue
    return {
        k: v for k, v in result.items() if not k.startswith(("automap:chunks:", "automap:chunk:"))
    }


def append_tag(element, key, value):
    """Chunk metadata only; native tags must remain readable without this tool."""
    value = str(value)
    legacy_profile = any(
        tag.get("k") == "type" and tag.get("v", "").startswith("automap:")
        for tag in element.findall("tag")
    )
    metadata = key.startswith(("automap:", "lanelet2:", "source:")) or legacy_profile
    if not metadata:
        if len(value) > 255:
            raise ValueError("Native OSM tag exceeds 255 characters: " + key)
        etree.SubElement(element, "tag", k=key, v=value)
        return
    if len(value.encode("utf-8")) <= 255:
        etree.SubElement(element, "tag", k=key, v=value)
        return
    parts, current = [], ""
    for char in value:
        if len((current + char).encode("utf-8")) > 240:
            parts.append(current)
            current = ""
        current += char
    parts.append(current)
    etree.SubElement(element, "tag", k="automap:chunks:" + key, v=str(len(parts)))
    for i, part in enumerate(parts):
        etree.SubElement(element, "tag", k=f"automap:chunk:{key}:{i:04}", v=part)


def extension(element, name: str):
    values = tags(element)
    raw = values.get(name)
    if raw is None:
        raw = next(
            (u.get("value") for u in element.findall("userData") if u.get("code") == name), None
        )
    return json.loads(raw) if raw else None


def set_extension(element, name: str, value, *, opendrive: bool = False) -> None:
    for child in list(element):
        if (
            child.tag == "tag"
            and (
                child.get("k") == name
                or child.get("k") == "automap:chunks:" + name
                or child.get("k", "").startswith("automap:chunk:" + name + ":")
            )
        ) or (child.tag == "userData" and child.get("code") == name):
            element.remove(child)
    raw = json.dumps(value, separators=(",", ":"), sort_keys=True)
    if opendrive:
        etree.SubElement(element, "userData", code=name, value=raw)
    else:
        append_tag(element, name, raw)


def native_elements(root, map_format: str) -> dict[str, object]:
    if map_format == "opendrive":
        return {
            f"{road.get('id')}|{section_index}|{lane.get('id')}": lane
            for road in root.findall("road")
            for section_index, section in enumerate(road.findall("lanes/laneSection"))
            for lane in section.findall("*/lane")
            if lane.get("id") != "0"
        }
    if map_format == "lanelet2":
        return {
            r.get("id"): r for r in root.findall("relation") if tags(r).get("type") == "lanelet"
        }
    return {w.get("id"): w for w in root.findall("way") if tags(w).get("highway")}


def trace_elements(root, map_format: str) -> dict[str, object]:
    result = native_elements(root, map_format)
    if map_format == "opendrive":
        for road in root.findall("road"):
            result["road:" + road.get("id")] = road
            for kind, path in (("signal", "signals/signal"), ("object", "objects/object")):
                for element in road.findall(path):
                    result[f"{road.get('id')}|{kind}|{element.get('id')}"] = element
        for element in root.findall("controller"):
            result["controller:" + element.get("id")] = element
    else:
        present = {id(element) for element in result.values()}
        for kind in ("node", "way", "relation"):
            for element in root.findall(kind):
                if id(element) not in present:
                    result[f"{kind}:{element.get('id')}"] = element
    return result


def read_frame(root, map_format: str, *, fallback_id: str = "") -> dict:
    if map_format == "opendrive":
        header = root.find("header")
        if header is not None:
            stored = extension(header, FRAME)
            if stored:
                return stored
            geo = header.findtext("geoReference", "").strip()
            if geo and geo.upper() != "LOCAL_CARTESIAN":
                CRS.from_user_input(geo)
                return {"kind": "projected", "crs": geo, "units": "m", "axis": "x,y"}
    else:
        for comment in root.xpath("./comment()"):
            if (comment.text or "").startswith(FRAME + " "):
                return json.loads(comment.text[len(FRAME) + 1 :])
        nodes = root.findall("node")
        if map_format == "lanelet2" and any(
            "local_x" in tags(n) and "local_y" in tags(n) for n in nodes
        ):
            return {
                "kind": "engineering",
                "id": fallback_id,
                "units": "m",
                "axis": "x,y",
                "note": "Local XY present; no external CRS is declared.",
            }
        return {"kind": "geographic", "crs": "EPSG:4326", "units": "degree", "axis": "lon,lat"}
    return {
        "kind": "engineering",
        "id": fallback_id,
        "units": "m",
        "axis": "x,y",
        "note": "OpenDRIVE local engineering coordinates; no external CRS is declared.",
    }


def write_frame(root, map_format: str, frame: dict) -> None:
    if map_format == "opendrive":
        header = root.find("header")
        set_extension(header, FRAME, frame, opendrive=True)
        geo = header.find("geoReference")
        if frame.get("crs"):
            if geo is None:
                geo = etree.Element("geoReference")
                header.insert(0, geo)
            from pyproj import CRS

            geo.text = etree.CDATA(CRS.from_user_input(frame["crs"]).to_proj4())
        elif geo is not None:
            header.remove(geo)
    else:
        for comment in root.xpath("./comment()"):
            if (comment.text or "").startswith(FRAME + " "):
                root.remove(comment)
        root.insert(0, etree.Comment(FRAME + " " + json.dumps(frame, sort_keys=True)))


def transform_points(points, source: dict, target: dict):
    if source == target or (
        source.get("kind") == target.get("kind") == "engineering"
        and source.get("id")
        and source.get("id") == target.get("id")
    ):
        return list(points)
    source_origin = source.get("engineering_frame", {})
    target_origin = target.get("engineering_frame", {})
    if target.get("kind") == "engineering" and source_origin.get("id") == target.get("id"):
        return transform_points(points, {"crs": source["crs"]}, {"crs": source["encoding_crs"]})
    if source.get("kind") == "engineering" and target_origin.get("id") == source.get("id"):
        return transform_points(points, {"crs": target["encoding_crs"]}, {"crs": target["crs"]})
    if not source.get("crs") or not target.get("crs"):
        raise ValueError(
            "Coordinate frames cannot be reconciled without a declared CRS/engineering-frame identity."
        )
    transformer = Transformer.from_crs(source["crs"], target["crs"], always_xy=True)
    result = [transformer.transform(*point[:2]) for point in points]
    if any(not math.isfinite(v) for point in result for v in point):
        raise ValueError("Coordinate transformation produced non-finite coordinates.")
    return result


def metric_frame(frame: dict, geographic_origin=None) -> dict:
    if frame.get("kind") != "geographic":
        return frame
    lon, lat = geographic_origin
    return {
        "kind": "projected",
        "units": "m",
        "axis": "x,y",
        "crs": f"+proj=aeqd +lat_0={lat} +lon_0={lon} +datum=WGS84 +units=m",
    }


def _parents(element, source_format: str, target_format: str) -> list[str]:
    data = tags(element)
    if data.get("automap:input_key"):
        return [data["automap:input_key"]]
    if source_format == "lanelet2" and target_format == "opendrive" and element.tag != "lane":
        ud = {u.get("code"): u.get("value", "") for u in element.findall("userData")}
        kind = ud.get("lanelet2:source_kind")
        source_id = ud.get("lanelet2:source_id")
        result = []
        if source_id:
            result.append(
                ("relation:" if kind in {"regulatory_element", "multipolygon"} else "way:")
                + source_id
            )
        for key, prefix in (
            ("lanelet2:source_way_id", "way:"),
            ("lanelet2:regulatory_element", "relation:"),
        ):
            if ud.get(key):
                result.append(prefix + ud[key])
        return result
    if source_format == "lanelet2" and target_format == "opendrive":
        return list(
            dict.fromkeys(
                u.get("value", "").split("|", 1)[0]
                for u in element.findall("userData")
                if u.get("code") == "lanelet2:lanelet_mapping"
            )
        )
    if source_format == "lanelet2" and target_format == "osm":
        result = [s for s in data.get("lanelet2:lanelet_ids", "").split(",") if s]
        for key, prefix in (
            ("lanelet2:source_regulatory_element", "relation:"),
            ("lanelet2:source_refers_way", "way:"),
        ):
            if data.get(key):
                result.append(prefix + data[key])
        return result
    if source_format == "opendrive" and target_format == "lanelet2":
        if data.get("source:opendrive:signal"):
            return data["source:opendrive:signal"].split(";")
        detailed = data.get("source:opendrive:lane_locations_detailed", "")
        if detailed:
            return [s.replace(",", "|") for s in detailed.split(";") if s]
        if "source:opendrive:road" in data:
            return [
                "|".join(
                    data.get(k, "0")
                    for k in (
                        "source:opendrive:road",
                        "source:opendrive:lane_section",
                        "source:opendrive:lane",
                    )
                )
            ]
    if source_format == "osm" and target_format == "lanelet2":
        if data.get("source:osm:lane_profile"):
            return ["relation:" + data["source:osm:lane_profile"]]
        return [s for s in data.get("source:osm:ways", "").split(",") if s]
    return []


def _target_frame(source_root, target_root, source_format, target_format, source_frame):
    from automap_converter.core.config.general_config import general_config
    from automap_converter.core.config.opendrive_config import open_drive_config

    if target_format == "osm":
        result = {"kind": "geographic", "crs": "EPSG:4326", "units": "degree", "axis": "lon,lat"}
        if source_frame.get("kind") == "engineering":
            result.update(
                engineering_frame=source_frame,
                encoding_crs=source_frame.get("encoding_crs") or str(general_config.proj_string_cr),
            )
        elif source_frame.get("engineering_frame"):
            result.update(
                engineering_frame=source_frame["engineering_frame"],
                encoding_crs=source_frame["encoding_crs"],
            )
        return result
    if target_format == "opendrive":
        if source_frame["kind"] == "geographic":
            # This is the exact spherical equirectangular projection used by the writer.
            first = source_root.find("node")
            lat, lon = float(first.get("lat")), float(first.get("lon"))
            return {
                "kind": "projected",
                "units": "m",
                "axis": "x,y",
                "crs": f"+proj=eqc +lat_ts={lat} +lat_0={lat} +lon_0={lon} +R=6371000 +units=m",
            }
        return source_frame
    if source_format == "opendrive" and source_frame["kind"] == "engineering":
        return {
            **source_frame,
            "encoding_crs": str(open_drive_config.proj_string_odr or general_config.proj_string_cr),
            "geographic_encoding": "Synthetic inverse projection used for OSM serialization; not a surveyed world position.",
        }

    crs = (
        open_drive_config.proj_string_odr
        if source_format == "opendrive"
        else general_config.proj_string_cr
    )
    result = {"kind": "projected", "crs": str(crs), "units": "m", "axis": "x,y"}
    if source_frame.get("engineering_frame"):
        result.update(
            engineering_frame=source_frame["engineering_frame"],
            encoding_crs=source_frame["encoding_crs"],
        )
    return result


def annotate_conversion(source: Path, target: Path, source_format: str, target_format: str) -> dict:
    """Propagate exact native object IDs through all ancestors, without spatial matching."""
    source, target = Path(source), Path(target)
    src = parse_xml(source).getroot()
    tree = parse_xml(target)
    dst = tree.getroot()
    digest = document_id(source)
    src_elements = trace_elements(src, source_format)
    primary_source_keys = set(native_elements(src, source_format))
    dst_elements = trace_elements(dst, target_format)
    parent_map = {
        key: _parents(element, source_format, target_format)
        for key, element in dst_elements.items()
    }
    keys_by_id = {id(e): key for key, e in dst_elements.items()}
    dst_by_id = {(element.tag, element.get("id")): element for element in dst_elements.values()}
    # Derived vertices/boundaries record the actual generating parent lane/way;
    # they are not mislabeled as copies of a particular source vertex.
    for _ in range(3):
        for key, element in dst_elements.items():
            inherited = parent_map[key]
            if not inherited:
                continue
            children = []
            if target_format != "opendrive":
                for member in element.findall("member"):
                    allowed_roles = (
                        {"left", "right", "centerline"}
                        if tags(element).get("type") == "lanelet"
                        else {"refers", "ref_line", "outer", "inner"}
                    )
                    if member.get("role") not in allowed_roles or member.get("type") == "relation":
                        continue
                    child = dst_by_id.get((member.get("type"), member.get("ref")))
                    if child is not None:
                        children.append(child)
                for node in element.findall("nd"):
                    child = dst_by_id.get(("node", node.get("ref")))
                    if child is not None:
                        children.append(child)
            else:
                if element.tag == "lane":
                    road = element.getparent().getparent().getparent().getparent()
                    children.append(road)
            for child in children:
                child_key = keys_by_id.get(id(child))
                if child_key is not None:
                    parent_map[child_key] = list(dict.fromkeys(parent_map[child_key] + inherited))
    source_frame = read_frame(src, source_format, fallback_id=digest)
    target_frame = _target_frame(src, dst, source_format, target_format, source_frame)
    mapped = set()
    records = []
    for key, element in dst_elements.items():
        parent_keys = parent_map[key]
        refs = []
        data = tags(element)
        selectors = (
            json.loads(data.get("automap:lineage_selectors", "{}"))
            if target_format == "osm"
            else {}
        )
        selection = (
            {
                "forward": data.get("source:osm:forward") == "yes",
                "lane_index": int(data["source:osm:lane_index"]),
            }
            if source_format == "osm" and data.get("source:osm:lane_index")
            else None
        )
        for parent in parent_keys:
            if parent not in src_elements:
                continue
            mapped.add(parent)
            selector = selectors.get(parent)
            ref = {"format": source_format, "document": digest, "key": parent}
            generated_connector = (
                source_format == "osm" and data.get("source:osm:generated_connector") == "yes"
            )
            if generated_connector:
                ref["role"] = "generated_connector"
            if selector:
                ref["selector"] = selector
            refs.append(ref)
            try:
                previous = extension(src_elements[parent], LINEAGE) or {}
            except (ValueError, TypeError, KeyError):
                previous = {}
            if not isinstance(previous, dict):
                previous = {}
            ancestors = previous.get("ancestors", [])
            if not isinstance(ancestors, list):
                ancestors = []
            for ancestor in ancestors:
                if not isinstance(ancestor, dict) or not all(
                    k in ancestor for k in ("format", "document", "key")
                ):
                    continue
                if (
                    source_format == "osm"
                    and ancestor.get("selector")
                    and ancestor["selector"] != selection
                ):
                    continue
                inherited = {k: ancestor[k] for k in ("format", "document", "key")}
                if generated_connector or ancestor.get("role") == "generated_connector":
                    inherited["role"] = "generated_connector"
                if selector:
                    inherited["selector"] = selector
                refs.append(inherited)
        unique = {json.dumps(r, sort_keys=True): r for r in refs}
        payload = {
            "schema": SCHEMA,
            "parents": parent_keys,
            "ancestors": list(unique.values()),
            "status": "recorded" if refs else "generated_or_unresolved",
        }
        set_extension(element, LINEAGE, payload, opendrive=target_format == "opendrive")
        for tag in list(element.findall("tag")):
            if tag.get("k") in {"automap:lineage_selectors", "automap:input_key"}:
                element.remove(tag)
        if (
            target_format == "lanelet2"
            and element.tag == "relation"
            and tags(element).get("type") == "lanelet"
        ):
            osm_ids = sorted(
                {r["key"] for r in refs if r["format"] == "osm" and ":" not in r["key"]}
            )
            if osm_ids and not tags(element).get("source:osm:ways"):
                etree.SubElement(element, "tag", k="source:osm:ways", v=",".join(osm_ids))
        records.append({"target_key": key, "source_keys": parent_keys, "status": payload["status"]})
    write_frame(dst, target_format, target_frame)
    if target_format == "osm":
        for element in dst:
            for tag in list(element.findall("tag")):
                if len(tag.get("v", "").encode("utf-8")) > 255:
                    key, value = tag.get("k"), tag.get("v")
                    if not key.startswith(("automap:", "lanelet2:", "source:")):
                        if len(value) > 255:
                            raise ValueError(
                                "Native OSM tag exceeds 255 characters; refusing private chunk encoding: "
                                + key
                            )
                        continue
                    element.remove(tag)
                    append_tag(element, key, value)
    coordinate_checks = coordinate_controls(dst, target_format, target_frame)
    tree.write(str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True)
    audit = {
        "schema": "automap-converter/conversion-audit/v1",
        "source": str(source),
        "source_document": digest,
        "target": str(target),
        "source_format": source_format,
        "target_format": target_format,
        "source_frame": source_frame,
        "target_frame": target_frame,
        "coordinate_checks": coordinate_checks,
        "objects": records,
        "source_objects": len(src_elements),
        "mapped_source_objects": len(mapped),
        "source_lanes_or_roads": len(primary_source_keys),
        "mapped_source_lanes_or_roads": len(primary_source_keys & mapped),
        "unmapped_source_keys": sorted(set(src_elements) - mapped),
        "note": "Lineage is identity only. Geometry/topology/attributes are read from actual target data.",
    }
    target.with_name(target.stem + "_conversion_audit.json").write_text(
        json.dumps(audit, indent=2, ensure_ascii=False), encoding="utf-8"
    )
    return audit


def coordinate_controls(root, map_format: str, frame: dict) -> dict:
    """Check independent native XY and geographic encodings, never fit a target to its source."""
    if map_format == "opendrive":
        points = [
            (float(g.get("x")), float(g.get("y"))) for g in root.findall("road/planView/geometry")
        ]
        if not points:
            return {"status": "FAIL", "reason": "No coordinate control points."}
        points = [points[i] for i in sorted({0, len(points) // 2, len(points) - 1})]
        return {
            "status": "PASS" if all(math.isfinite(v) for p in points for v in p) else "FAIL",
            "method": "Finite native XY controls; frame declared in header; no post-hoc alignment.",
            "controls": points,
        }
    nodes = root.findall("node")
    nodes = [nodes[i] for i in sorted({0, len(nodes) // 2, len(nodes) - 1})] if nodes else []
    controls = []
    for node in nodes:
        data = tags(node)
        lonlat = (float(node.get("lon")), float(node.get("lat")))
        record = {"node": node.get("id"), "lonlat": lonlat}
        if "local_x" in data and "local_y" in data:
            xy = (float(data["local_x"]), float(data["local_y"]))
            record["local_xy"] = xy
            if frame.get("crs") or frame.get("encoding_crs"):
                encoded_frame = {"crs": frame.get("crs", frame.get("encoding_crs"))}
                predicted = transform_points([xy], encoded_frame, {"crs": "EPSG:4326"})[0]
                record["encoding_error_m"] = abs(Geod(ellps="WGS84").inv(*lonlat, *predicted)[2])
        controls.append(record)
    errors = [c["encoding_error_m"] for c in controls if "encoding_error_m" in c]
    return {
        "status": (
            "FAIL"
            if any(e > 0.05 for e in errors)
            else "REVIEW"
            if frame.get("kind") == "engineering"
            else "PASS"
        ),
        "method": "Native XY/geographic consistency at first/middle/last node; no fitted alignment.",
        "controls": controls,
    }
