"""Apply documented OSM from/via/to lane connectivity to a generated graph."""

from lxml import etree


def apply_connectivity(graph, source):
    records = []
    for relation in etree.parse(str(source)).getroot().findall("relation"):
        data = {t.get("k"): t.get("v") for t in relation.findall("tag")}
        if data.get("type") != "connectivity":
            continue
        members = {m.get("role"): m for m in relation.findall("member")}
        if not all(k in members for k in ("from", "via", "to")):
            raise ValueError("OSM connectivity requires from/via/to members")
        if members["via"].get("type") != "node":
            records.append(
                {
                    "relation": relation.get("id"),
                    "status": "WARN",
                    "detail": "via-way connectivity is not implemented",
                }
            )
            continue
        via = int(members["via"].get("ref"))
        sources = [
            e
            for e in graph.edges
            if str(members["from"].get("ref"))
            in set(map(str, getattr(e, "osm_source_way_ids", [])))
            and via in (e.node1.id, e.node2.id)
        ]
        targets = [
            e
            for e in graph.edges
            if str(members["to"].get("ref")) in set(map(str, getattr(e, "osm_source_way_ids", [])))
            and via in (e.node1.id, e.node2.id)
        ]
        if len(sources) != 1 or len(targets) != 1:
            records.append(
                {
                    "relation": relation.get("id"),
                    "status": "WARN",
                    "detail": "Cannot uniquely locate native via-node edges after graph simplification",
                }
            )
            continue
        a, b = sources[0], targets[0]
        incoming = [l for l in a.lanes if l.forward == (a.node2.id == via)]
        outgoing = [l for l in b.lanes if l.forward == (b.node1.id == via)]
        if incoming and not incoming[0].forward:
            incoming.reverse()
        if outgoing and not outgoing[0].forward:
            outgoing.reverse()
        edges = []
        for entry in data.get("connectivity", "").split("|"):
            head, separator, tail = entry.partition(":")
            if not separator or not head.isdigit() or any(not v.isdigit() for v in tail.split(",")):
                raise ValueError(
                    "Only explicit numeric OSM lane connectivity is supported: " + entry
                )
            for value in tail.split(","):
                x, y = int(head) - 1, int(value) - 1
                if not 0 <= x < len(incoming) or not 0 <= y < len(outgoing):
                    raise ValueError("OSM connectivity lane index exceeds actual lane count")
                edges.append((incoming[x], outgoing[y]))
        for lane in incoming:
            for successor in list(lane.successors):
                if successor in outgoing:
                    lane.successors.discard(successor)
                    successor.predecessors.discard(lane)
        for lane, successor in edges:
            lane.successors.add(successor)
            successor.predecessors.add(lane)
        records.append({"relation": relation.get("id"), "status": "PASS", "edges": len(edges)})
    return records
