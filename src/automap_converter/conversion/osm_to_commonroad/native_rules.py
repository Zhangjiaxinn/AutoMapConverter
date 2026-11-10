"""Apply public OSM per-lane rules using this leg's explicit creation events."""

from lxml import etree

from automap_converter.core.traceability import tags
from automap_converter.formats.lanelet2.shared_geometry import sort_native_objects


def apply_lane_rules(source, target):
    src = etree.parse(str(source)).getroot()
    tree = etree.parse(str(target))
    root = tree.getroot()
    roads = {w.get("id"): tags(w) for w in src.findall("way")}
    next_id = (
        min([0] + [int(e.get("id")) for e in root if isinstance(e.tag, str) and e.get("id")]) - 1
    )

    def set_tag(element, key, value):
        tag = element.find(f"tag[@k='{key}']")
        if tag is None:
            etree.SubElement(element, "tag", k=key, v=str(value))
        else:
            tag.set("v", str(value))

    for lane in root.findall("relation"):
        data = tags(lane)
        source_ids = data.get("source:osm:ways", "").split(",")
        if data.get("type") != "lanelet" or len(source_ids) != 1 or source_ids[0] not in roads:
            continue
        road = roads[source_ids[0]]
        index = int(data.get("source:osm:lane_index", "1")) - 1
        suffix = ":forward" if data.get("source:osm:forward", "yes") == "yes" else ":backward"

        def lane_value(key, road=road, index=index, suffix=suffix):
            for field in (key + ":lanes" + suffix, key + ":lanes"):
                values = road.get(field, "").split("|")
                if index < len(values) and values[index]:
                    return values[index]
            return road.get(key)

        if lane_value("bus") == "designated":
            set_tag(lane, "subtype", "bus_lane")
        if road.get("highway") == "footway" and road.get("footway") == "crossing":
            set_tag(lane, "subtype", "crosswalk")
        value = lane_value("maxspeed")
        if not value:
            continue
        # Replace only generated speed associations for this lane. Keep signs,
        # lights and other rules; no ancestral speed arrays are consulted.
        existing = {r.get("id"): tags(r) for r in root.findall("relation")}
        for member in list(lane.findall("member")):
            if (
                member.get("role") == "regulatory_element"
                and existing.get(member.get("ref"), {}).get("subtype") == "speed_limit"
            ):
                lane.remove(member)
        rule_id = str(next_id)
        next_id -= 1
        rule = etree.SubElement(root, "relation", id=rule_id, version="1")
        for key, val in (
            ("type", "regulatory_element"),
            ("subtype", "speed_limit"),
            ("sign_type", value if any(c.isalpha() for c in value) else value + " km/h"),
        ):
            set_tag(rule, key, val)
        etree.SubElement(lane, "member", type="relation", ref=rule_id, role="regulatory_element")
    # The legacy generator's transient current-road model is not serialized as
    # a reconstruction aid for the next conversion.
    for tag in list(root.xpath(".//tag[starts-with(@k,'automap:osm_road_model')]")):
        tag.getparent().remove(tag)
    sort_native_objects(root)
    tree.write(str(target), encoding="UTF-8", xml_declaration=True, pretty_print=True)
