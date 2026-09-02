from __future__ import annotations

import math
import os
import re
import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Dict, List, Tuple

from lxml import etree


PROJECT_ROOT = Path(__file__).resolve().parents[4]
INT_RE = re.compile(r"^-?\d+$")
ODR_EPS = 1e-9


def prepare_source_for_conversion(
    conversion: str,
    source_path: Path,
    work_dir: Path,
) -> Tuple[Path, List[Dict[str, str]]]:
    """Return a source path that is safer for the current converter to parse."""
    work_dir.mkdir(parents=True, exist_ok=True)
    if conversion in {
        "osm_to_lanelet2",
        "osm_to_opendrive",
        "lanelet2_to_osm",
        "lanelet2_to_opendrive",
    }:
        return _prepare_osm_source(conversion, source_path, work_dir)
    if conversion in {"opendrive_to_lanelet2", "opendrive_to_osm"}:
        return _prepare_opendrive_source(source_path, work_dir)
    return source_path, [_record("source-precheck", "SKIP", "No source precheck profile is registered.")]


def repair_target_after_conversion(
    conversion: str,
    target_path: Path,
    work_dir: Path,
) -> List[Dict[str, str]]:
    """Apply conservative target-side repairs required by strict target loaders."""
    work_dir.mkdir(parents=True, exist_ok=True)
    if conversion in {"opendrive_to_lanelet2", "osm_to_lanelet2"}:
        return _repair_lanelet2_target(target_path)
    if conversion in {"lanelet2_to_osm", "opendrive_to_osm"}:
        return _repair_osm_versions(target_path, label="target-osm")
    return [_record("target-repair", "SKIP", "No target repair profile is registered.")]


def _prepare_osm_source(
    conversion: str,
    source_path: Path,
    work_dir: Path,
) -> Tuple[Path, List[Dict[str, str]]]:
    tree = etree.parse(str(source_path))
    root = tree.getroot()
    records: List[Dict[str, str]] = []
    changed = False

    changed_versions = _ensure_osm_versions(root)
    if changed_versions:
        changed = True
        records.append(
            _record(
                "source-osm-version",
                "WARN",
                f"Added missing OSM version attributes ({changed_versions} primitives/root items).",
                "OSM/Lanelet2 loaders expect version attributes on primitives.",
            )
        )

    if conversion in {"osm_to_lanelet2", "osm_to_opendrive"}:
        removed_signs = _drop_unparseable_osm_traffic_signs(root)
        if removed_signs:
            changed = True
            records.append(
                _record(
                    "source-osm-traffic-signs",
                    "WARN",
                    f"Removed {removed_signs} traffic_sign tags that crash the current OSM parser.",
                    "Example: DE:274.1 declares a speed-limit zone sign without a concrete speed value.",
                )
            )
        normalized_controls = _normalize_osm_stop_give_way_tags(root)
        if normalized_controls:
            changed = True
            records.append(
                _record(
                    "source-osm-stop-give-way",
                    "WARN",
                    f"Added parser-readable traffic_sign tags for {normalized_controls} stop/give_way primitives.",
                    "Plain OSM highway=stop/give_way is preserved and also exposed as DE:206/DE:205 for the current parser.",
                )
            )

    if not changed:
        records.append(_record("source-precheck", "PASS", "Source OSM passed lightweight precheck."))
        _append_source_load_checks(conversion, source_path, records)
        return source_path, records

    prepared_path = work_dir / source_path.name
    _write_xml(tree, prepared_path)
    _append_source_load_checks(conversion, prepared_path, records)
    return prepared_path, records


def _prepare_opendrive_source(source_path: Path, work_dir: Path) -> Tuple[Path, List[Dict[str, str]]]:
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(str(source_path), parser)
    root = tree.getroot()
    records: List[Dict[str, str]] = []
    changed = False

    id_changes = _normalize_opendrive_ids(root)
    if id_changes:
        changed = True
        records.append(
            _record(
                "source-opendrive-id-normalization",
                "WARN",
                f"Normalized {id_changes} non-integer OpenDRIVE ids/references for the current parser.",
                "The installed OpenDRIVE parser stores road/junction ids as int.",
            )
        )

    direct_changes = _normalize_direct_junctions(root)
    if direct_changes:
        changed = True
        records.append(
            _record(
                "source-opendrive-direct-junction",
                "WARN",
                f"Converted {direct_changes} direct-junction linkedRoad references to parser-compatible connectingRoad references.",
                "OpenDRIVE direct junctions do not always contain a physical connectingRoad.",
            )
        )

    arc_changes = _downgrade_zero_curvature_arcs(root)
    if arc_changes:
        changed = True
        records.append(
            _record(
                "source-opendrive-zero-curvature-arc",
                "WARN",
                f"Converted {arc_changes} zero-curvature arc geometries to line geometries.",
                "A zero-curvature arc is geometrically a line and avoids division by zero.",
            )
        )

    spiral_changes = _downgrade_constant_spirals(root)
    if spiral_changes:
        changed = True
        records.append(
            _record(
                "source-opendrive-constant-spiral",
                "WARN",
                f"Converted {spiral_changes} constant-curvature spiral geometries to arc/line geometries.",
                "The installed parser rejects spirals whose start and end curvature are equal.",
            )
        )

    object_outline_changes = _localize_crosswalk_corner_road_outlines(root)
    if object_outline_changes:
        changed = True
        records.append(
            _record(
                "source-opendrive-crosswalk-outlines",
                "WARN",
                f"Converted {object_outline_changes} crosswalk cornerRoad vertices to parser-compatible cornerLocal vertices.",
                "The installed crosswalk converter expects object outline vertices to expose local u/v coordinates.",
            )
        )

    invalid_crosswalks = _drop_invalid_crosswalk_objects(root)
    if invalid_crosswalks:
        changed = True
        records.append(
            _record(
                "source-opendrive-invalid-crosswalk",
                "WARN",
                f"Removed {invalid_crosswalks} crosswalk objects with fewer than four valid outline corners.",
                "A crosswalk object must describe an area; two-point outlines cannot be converted to Lanelet2 polygons.",
            )
        )

    if not changed:
        records.append(_record("source-precheck", "PASS", "Source OpenDRIVE passed lightweight precheck."))
        _append_source_load_checks("opendrive_to_lanelet2", source_path, records)
        return source_path, records

    prepared_path = work_dir / source_path.name
    _write_xml(tree, prepared_path)
    _append_source_load_checks("opendrive_to_lanelet2", prepared_path, records)
    return prepared_path, records


def _append_source_load_checks(conversion: str, source_path: Path, records: List[Dict[str, str]]) -> None:
    if conversion in {"lanelet2_to_opendrive", "lanelet2_to_osm"}:
        records.append(_load_lanelet2_map(source_path, "source-lanelet2-loader"))
    elif conversion in {"osm_to_lanelet2", "osm_to_opendrive"}:
        records.append(_load_osm_xml(source_path, "source-osm-xml"))
    elif conversion in {"opendrive_to_lanelet2", "opendrive_to_osm"}:
        records.append(_load_opendrive_with_esmini(source_path, "source-opendrive-loader"))


def _load_lanelet2_map(path: Path, category: str) -> Dict[str, str]:
    try:
        import lanelet2
        from automap_converter.core.config.lanelet2_config import lanelet2_config

        origin = lanelet2.io.Origin(
            float(lanelet2_config.routing_origin_lat),
            float(lanelet2_config.routing_origin_lon),
        )
        lanelet2.io.load(str(path), origin)
        return _record(category, "PASS", "Lanelet2 Python loader loaded the source map.")
    except Exception as exc:
        return _record(
            category,
            "WARN",
            (
                "The official Lanelet2 Python loader could not load the source map; "
                "the conversion-specific XML parser will be used as the fallback."
            ),
            (
                f"{type(exc).__name__}: {exc}. This commonly means the installed "
                "Lanelet2 plugin does not implement a regulatory-element subtype; "
                "a later converter-parser failure remains blocking."
            ),
        )


def _load_osm_xml(path: Path, category: str) -> Dict[str, str]:
    try:
        root = etree.parse(str(path)).getroot()
        if root.tag != "osm":
            raise ValueError(f"Root tag is {root.tag!r}, expected 'osm'.")
        return _record(category, "PASS", "Source OSM XML is well formed and has an osm root.")
    except Exception as exc:
        return _record(
            category,
            "FAIL",
            "Source OSM XML could not be parsed after precheck.",
            f"{type(exc).__name__}: {exc}",
        )


def _load_opendrive_with_esmini(path: Path, category: str) -> Dict[str, str]:
    odrplot = _find_project_tool("odrplot") or shutil.which("odrplot")
    if not odrplot:
        return _record(
            category,
            "SKIP",
            "No esmini odrplot executable was found for source OpenDRIVE load validation.",
            "Install esmini or put odrplot on PATH to enable this check.",
        )
    try:
        with tempfile.TemporaryDirectory(prefix="automap-odrplot-") as temporary_directory:
            output_path = Path(temporary_directory) / f"{path.stem}_source_odrplot.csv"
            completed = subprocess.run(
                [odrplot, str(path), str(output_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=20,
                check=False,
            )
    except Exception as exc:
        return _record(
            category,
            "FAIL",
            "esmini odrplot could not run on the source OpenDRIVE map.",
            f"{type(exc).__name__}: {exc}",
        )
    output = completed.stdout.strip()
    error_lines = [
        line.strip()
        for line in output.splitlines()
        if "[error]" in line.lower() or "parse error" in line.lower()
    ]
    if completed.returncode == 0 and not error_lines:
        return _record(category, "PASS", "esmini odrplot loaded the source OpenDRIVE map.")
    if completed.returncode == 0:
        return _record(
            category,
            "WARN",
            "esmini odrplot loaded the source OpenDRIVE map but reported non-fatal messages.",
            "\n".join(error_lines)[:1200] if error_lines else output[:1200],
        )
    return _record(
        category,
        "FAIL",
        f"esmini odrplot failed to load the source OpenDRIVE map (exit={completed.returncode}).",
        "\n".join(error_lines)[:1200] if error_lines else output[:1200],
    )


def _find_project_tool(name: str) -> str:
    candidate = PROJECT_ROOT / "tools" / "esmini" / "bin" / name
    if candidate.is_file() and os.access(candidate, os.X_OK):
        return str(candidate)
    return ""


def _repair_lanelet2_target(target_path: Path) -> List[Dict[str, str]]:
    if not target_path.exists():
        return [_record("target-lanelet2-repair", "SKIP", "Target does not exist yet.")]

    tree = etree.parse(str(target_path))
    root = tree.getroot()
    records: List[Dict[str, str]] = []
    changed = False

    version_changes = _ensure_osm_versions(root)
    if version_changes:
        changed = True
        records.append(
            _record(
                "target-lanelet2-version",
                "WARN",
                f"Added missing OSM/Lanelet2 version attributes ({version_changes} primitives/root items).",
            )
        )

    ref_changes = _drop_dangling_members(root)
    if ref_changes:
        changed = True
        records.append(
            _record(
                "target-lanelet2-dangling-refs",
                "WARN",
                f"Removed {ref_changes} relation members that referenced missing primitives.",
                "Lanelet2 loader rejects relations with unresolved members.",
            )
        )

    invalid_relations = _drop_invalid_lanelet2_relations(root)
    if invalid_relations:
        changed = True
        records.append(
            _record(
                "target-lanelet2-invalid-relations",
                "WARN",
                f"Removed {invalid_relations} invalid Lanelet2 relations and their references.",
                "This is a loader-safety fallback; semantic losses are still visible in Stage2.",
            )
        )

    if changed:
        _write_xml(tree, target_path)
    else:
        records.append(_record("target-repair", "PASS", "Target did not require loader-safety repair."))
    return records


def _repair_osm_versions(target_path: Path, label: str) -> List[Dict[str, str]]:
    if not target_path.exists():
        return [_record(label, "SKIP", "Target does not exist yet.")]
    tree = etree.parse(str(target_path))
    root = tree.getroot()
    version_changes = _ensure_osm_versions(root)
    order_changed = _normalize_osm_primitive_order(root)
    if version_changes or order_changed:
        _write_xml(tree, target_path)
        notes = []
        if version_changes:
            notes.append(
                f"added missing OSM version attributes ({version_changes} primitives/root items)"
            )
        if order_changed:
            notes.append("ordered nodes, ways and relations by numeric id for osmium")
        return [_record(label, "WARN", "Target OSM normalized: " + "; ".join(notes) + ".")]
    return [_record(label, "PASS", "OSM version attributes are present and primitives are ordered.")]


def _ensure_osm_versions(root: etree._Element) -> int:
    changes = 0
    if root.tag == "osm" and not root.get("version"):
        root.set("version", "0.6")
        changes += 1
    for elem in root.xpath("./node|./way|./relation"):
        if not elem.get("version"):
            elem.set("version", "1")
            changes += 1
    return changes


def _normalize_osm_primitive_order(root: etree._Element) -> bool:
    """Put OSM primitives in the canonical osmium order without changing references."""
    if root.tag != "osm":
        return False
    children = list(root)
    grouped = {
        name: [child for child in children if child.tag == name]
        for name in ("node", "way", "relation")
    }
    ordered_primitives = []
    for name in ("node", "way", "relation"):
        ordered_primitives.extend(
            sorted(grouped[name], key=lambda child: _osm_id_sort_key(child.get("id", "")))
        )
    other_children = [
        child for child in children if child.tag not in {"node", "way", "relation"}
    ]
    normalized = other_children + ordered_primitives
    if normalized == children:
        return False
    root[:] = normalized
    return True


def _osm_id_sort_key(value: str) -> Tuple[int, int, str]:
    """Match osmium's ordering when generated OSM uses temporary negative IDs.

    ``osmium check-refs`` compares IDs through its internal unsigned ordering.
    Therefore temporary negative IDs come first in descending signed order
    (``-1, -2, ...``), followed by positive IDs in ascending order, rather
    than ordinary numeric ascending order.
    """
    try:
        numeric_id = int(value)
    except ValueError:
        return 2, 0, value
    if numeric_id >= 0:
        return 1, numeric_id, value
    return 0, -numeric_id, value


def _drop_unparseable_osm_traffic_signs(root: etree._Element) -> int:
    removed = 0
    for tag in list(root.xpath(".//tag[@k='traffic_sign']")):
        value = tag.get("v", "")
        if _traffic_sign_value_is_unparseable(value):
            parent = tag.getparent()
            if parent is not None:
                parent.remove(tag)
                removed += 1
    return removed


def _traffic_sign_value_is_unparseable(value: str) -> bool:
    if value.strip().lower() in {"", "none", "unknown"}:
        return True
    for block in value.split(";"):
        sign_data = block[3:] if ":" in block[:3] else block
        for sign in sign_data.split(","):
            sign = sign.strip()
            if sign in {"274.1", "274.2"}:
                return True
    return False


def _normalize_osm_stop_give_way_tags(root: etree._Element) -> int:
    changes = 0
    for elem in root.xpath("./node|./way"):
        tags = {tag.get("k"): tag for tag in elem.findall("tag") if tag.get("k") is not None}
        highway = (tags.get("highway").get("v") if tags.get("highway") is not None else "").strip()
        if highway == "stop":
            sign_value = "DE:206"
        elif highway in {"give_way", "yield"}:
            sign_value = "DE:205"
        else:
            continue

        traffic_sign = tags.get("traffic_sign")
        if traffic_sign is None:
            etree.SubElement(elem, "tag", k="traffic_sign", v=sign_value)
            changes += 1
            continue

        values = [value.strip() for value in traffic_sign.get("v", "").split(";") if value.strip()]
        if sign_value not in values:
            values.append(sign_value)
            traffic_sign.set("v", ";".join(values))
            changes += 1
    return changes


def _normalize_opendrive_ids(root: etree._Element) -> int:
    road_ids = [road.get("id") for road in root.findall("road") if road.get("id") is not None]
    junction_ids = [
        junction.get("id") for junction in root.findall("junction") if junction.get("id") is not None
    ]
    used = {int(value) for value in road_ids + junction_ids if _is_int(value)}
    road_map = _make_int_id_map(road_ids, used)
    junction_map = _make_int_id_map(junction_ids, used)
    changes = 0

    for road in root.findall("road"):
        old_id = road.get("id")
        if old_id in road_map:
            road.set("id", road_map[old_id])
            _add_userdata(road, "source:opendrive:id", old_id or "")
            changes += 1
        junction = road.get("junction")
        if junction in junction_map:
            road.set("junction", junction_map[junction])
            changes += 1
        for link in road.xpath("./link/*[@elementId]"):
            element_type = link.get("elementType")
            element_id = link.get("elementId")
            if element_type == "road" and element_id in road_map:
                link.set("elementId", road_map[element_id])
                changes += 1
            elif element_type == "junction" and element_id in junction_map:
                link.set("elementId", junction_map[element_id])
                changes += 1

    for junction in root.findall("junction"):
        old_id = junction.get("id")
        if old_id in junction_map:
            junction.set("id", junction_map[old_id])
            _add_userdata(junction, "source:opendrive:id", old_id or "")
            changes += 1
        for connection in junction.findall("connection"):
            for attr in ("incomingRoad", "connectingRoad", "linkedRoad"):
                value = connection.get(attr)
                if value in road_map:
                    connection.set(attr, road_map[value])
                    changes += 1

    return changes


def _make_int_id_map(values: List[str], used: set[int]) -> Dict[str, str]:
    mapping: Dict[str, str] = {}
    next_id = 1
    for value in values:
        if value is None or _is_int(value) or value in mapping:
            continue
        while next_id in used:
            next_id += 1
        mapping[value] = str(next_id)
        used.add(next_id)
    return mapping


def _normalize_direct_junctions(root: etree._Element) -> int:
    changes = 0
    for junction in root.findall("junction"):
        if junction.get("type") != "direct":
            continue
        for connection in junction.findall("connection"):
            if connection.get("connectingRoad") is None and connection.get("linkedRoad") is not None:
                connection.set("connectingRoad", connection.get("linkedRoad") or "")
                _add_userdata(connection, "source:opendrive:linkedRoad", connection.get("linkedRoad") or "")
                changes += 1
    return changes


def _downgrade_zero_curvature_arcs(root: etree._Element) -> int:
    changes = 0
    for arc in list(root.xpath(".//geometry/arc")):
        curvature = _float_or_none(arc.get("curvature"))
        if curvature is None or abs(curvature) > ODR_EPS:
            continue
        parent = arc.getparent()
        if parent is None:
            continue
        parent.remove(arc)
        etree.SubElement(parent, "line")
        changes += 1
    return changes


def _downgrade_constant_spirals(root: etree._Element) -> int:
    changes = 0
    for spiral in list(root.xpath(".//geometry/spiral")):
        curv_start = _float_or_none(spiral.get("curvStart"))
        curv_end = _float_or_none(spiral.get("curvEnd"))
        if curv_start is None or curv_end is None or not math.isclose(curv_start, curv_end, abs_tol=ODR_EPS):
            continue
        parent = spiral.getparent()
        if parent is None:
            continue
        parent.remove(spiral)
        if abs(curv_start) <= ODR_EPS:
            etree.SubElement(parent, "line")
        else:
            etree.SubElement(parent, "arc", curvature=str(curv_start))
        changes += 1
    return changes


def _drop_dangling_members(root: etree._Element) -> int:
    node_ids = {elem.get("id") for elem in root.findall("node")}
    way_ids = {elem.get("id") for elem in root.findall("way")}
    relation_ids = {elem.get("id") for elem in root.findall("relation")}
    removed = 0

    removed_way_ids: set[str] = set()
    for way in list(root.findall("way")):
        for nd in list(way.findall("nd")):
            if nd.get("ref") not in node_ids:
                way.remove(nd)
                removed += 1
        if len(way.findall("nd")) == 0:
            way_id = way.get("id")
            if way_id:
                removed_way_ids.add(way_id)
            parent = way.getparent()
            if parent is not None:
                parent.remove(way)
                removed += 1

    if removed_way_ids:
        way_ids = {elem.get("id") for elem in root.findall("way")}

    ids_by_type = {
        "node": node_ids,
        "way": way_ids,
        "relation": relation_ids,
    }
    for relation in root.findall("relation"):
        for member in list(relation.findall("member")):
            member_type = member.get("type")
            member_ref = member.get("ref")
            if member_type in ids_by_type and member_ref not in ids_by_type[member_type]:
                relation.remove(member)
                removed += 1
    return removed


def _localize_crosswalk_corner_road_outlines(root: etree._Element) -> int:
    changes = 0
    for obj in root.xpath(".//object[@type='crosswalk']"):
        object_s = _float_or_none(obj.get("s")) or 0.0
        object_t = _float_or_none(obj.get("t")) or 0.0
        for corner_road in list(obj.xpath(".//outline/cornerRoad")):
            parent = corner_road.getparent()
            if parent is None:
                continue
            corner_s = _float_or_none(corner_road.get("s"))
            corner_t = _float_or_none(corner_road.get("t"))
            if corner_s is None or corner_t is None:
                continue
            corner_local = etree.Element("cornerLocal")
            corner_local.set("u", str(corner_s - object_s))
            corner_local.set("v", str(corner_t - object_t))
            corner_local.set("z", corner_road.get("dz", "0.0"))
            if corner_road.get("height") is not None:
                corner_local.set("height", corner_road.get("height") or "0.0")
            if corner_road.get("id") is not None:
                corner_local.set("id", corner_road.get("id") or "0")
            parent.replace(corner_road, corner_local)
            changes += 1
    return changes


def _drop_invalid_crosswalk_objects(root: etree._Element) -> int:
    removed = 0
    for obj in list(root.xpath(".//object[@type='crosswalk']")):
        corner_count = 0
        for corner in obj.xpath(".//outline/cornerLocal"):
            if _float_or_none(corner.get("u")) is not None and _float_or_none(corner.get("v")) is not None:
                corner_count += 1
        if corner_count >= 4:
            continue
        parent = obj.getparent()
        if parent is not None:
            parent.remove(obj)
            removed += 1
    return removed


def _drop_invalid_lanelet2_relations(root: etree._Element) -> int:
    removed_ids: set[str] = set()
    changed = True
    while changed:
        changed = False
        for relation in list(root.findall("relation")):
            relation_id = relation.get("id")
            tags = {tag.get("k"): tag.get("v") for tag in relation.findall("tag")}
            members = relation.findall("member")
            remove = False
            if tags.get("type") == "lanelet":
                roles = {member.get("role") for member in members}
                remove = "left" not in roles or "right" not in roles
            elif tags.get("type") == "regulatory_element" and tags.get("subtype") == "right_of_way":
                remove = not any(member.get("role") == "right_of_way" for member in members)
            if not remove:
                continue
            parent = relation.getparent()
            if parent is not None:
                parent.remove(relation)
                if relation_id:
                    removed_ids.add(relation_id)
                changed = True

        if removed_ids:
            for relation in root.findall("relation"):
                for member in list(relation.findall("member")):
                    if member.get("type") == "relation" and member.get("ref") in removed_ids:
                        relation.remove(member)

    return len(removed_ids)


def _add_userdata(parent: etree._Element, code: str, value: str) -> None:
    etree.SubElement(parent, "userData", code=code, value=value)


def _is_int(value: str | None) -> bool:
    return value is not None and bool(INT_RE.match(value))


def _float_or_none(value: str | None) -> float | None:
    try:
        return float(value) if value is not None else None
    except ValueError:
        return None


def _write_xml(tree: etree._ElementTree, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(str(path), encoding="UTF-8", xml_declaration=True, pretty_print=True)


def _record(category: str, status: str, summary: str, hint: str = "") -> Dict[str, str]:
    return {
        "category": category,
        "status": status,
        "summary": summary,
        "hint": hint,
    }
