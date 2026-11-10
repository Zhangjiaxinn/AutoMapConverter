"""Split actual coincident boundary intervals into shared Lanelet2 objects."""

from copy import deepcopy
from itertools import pairwise

from lxml import etree
from pyproj import Transformer
from shapely.geometry import LineString, Point
from shapely.ops import substring
from shapely.strtree import STRtree

from automap_converter.core.traceability import tags
from automap_converter.formats.lanelet2.geometry import orient_boundaries
from automap_converter.formats.lanelet2.shared_geometry import sort_native_objects


def share_partial_boundaries(root, crs, tolerance=0.05, max_repairs=32):
    """Keep independent tails, split both users and share the verified interval.

    No source/ancestor identity or claimed adjacency participates in selection.
    The 5-cm tolerance is the existing numerical geometry tolerance, not a
    licence to weld metres of skew road or different height layers.
    """
    inverse = Transformer.from_crs(crs, 4326, always_xy=True)
    forward = Transformer.from_crs(4326, crs, always_xy=True)
    next_id = min([0] + [int(e.get("id")) for e in root if e.get("id")]) - 1

    def allocate():
        nonlocal next_id
        value = str(next_id)
        next_id -= 1
        return value

    repaired, evidence, blocked, retries = 0, [], set(), {}
    for _ in range(max_repairs * 6):
        if repaired >= max_repairs:
            break
        nodes = {n.get("id"): n for n in root.findall("node")}
        ways = {w.get("id"): w for w in root.findall("way")}
        lanes = [r for r in root.findall("relation") if tags(r).get("type") == "lanelet"]

        def xy(nid, nodes=nodes):
            node = nodes[nid]
            data = tags(node)
            if "local_x" in data and "local_y" in data:
                return float(data["local_x"]), float(data["local_y"])
            return forward.transform(float(node.get("lon")), float(node.get("lat")))

        users = {}
        for lane in lanes:
            for member in lane.findall("member"):
                if member.get("role") in {"left", "right"}:
                    users.setdefault(member.get("ref"), []).append(lane)
        keys = sorted(users)
        lines = {
            key: LineString([xy(n.get("ref")) for n in ways[key].findall("nd")]) for key in keys
        }
        tree = STRtree([lines[key] for key in keys])
        chosen = None
        for i, ka in enumerate(keys):
            a = lines[ka]
            for index in sorted(tree.query(a.buffer(tolerance / 2))):
                if index <= i:
                    continue
                kb = keys[int(index)]
                if (ka, kb) in blocked:
                    continue
                b = lines[kb]
                if {r.get("id") for r in users[ka]} & {r.get("id") for r in users[kb]}:
                    continue
                if any(
                    tags(ways[ka]).get(k) != tags(ways[kb]).get(k)
                    for k in ("type", "subtype", "color", "lane_change")
                ):
                    continue
                heights = [
                    sum(
                        float(tags(nodes[n.get("ref")]).get("ele", "0"))
                        for n in ways[k].findall("nd")
                    )
                    / len(ways[k].findall("nd"))
                    for k in (ka, kb)
                ]
                if any(
                    max(values) - min(values) > tolerance
                    for values in (
                        [
                            float(tags(nodes[n.get("ref")]).get("ele", "0"))
                            for n in ways[k].findall("nd")
                        ]
                        for k in (ka, kb)
                    )
                ):
                    continue
                if abs(heights[0] - heights[1]) > tolerance:
                    continue
                if (
                    a.hausdorff_distance(b) <= tolerance
                    and abs(a.length - b.length) <= 2 * tolerance
                ):
                    continue  # The existing whole-object consolidator owns this case.
                hit = a.intersection(b.buffer(tolerance / 4, cap_style=2))
                parts = (
                    [hit]
                    if hit.geom_type == "LineString"
                    else [g for g in getattr(hit, "geoms", []) if g.geom_type == "LineString"]
                )
                parts = sorted(parts, key=lambda part: -part.length)
                if not parts or parts[0].length < max(1.0, 0.25 * min(a.length, b.length)):
                    continue
                common = parts[0]
                inset = (0.0, 0.25, 0.5, 1.0, 2.0)[retries.get((ka, kb), 0)]
                if inset:
                    if common.length - 2 * inset < max(1.0, 0.25 * min(a.length, b.length)):
                        blocked.add((ka, kb))
                        continue
                    common = substring(common, inset, common.length - inset)
                stations = [
                    b.project(common.interpolate(f, normalized=True))
                    for f in (0, 0.25, 0.5, 0.75, 1)
                ]
                if not (
                    all(y >= x - 1e-8 for x, y in pairwise(stations))
                    or all(y <= x + 1e-8 for x, y in pairwise(stations))
                ):
                    continue
                if abs(stations[-1] - stations[0]) < 0.9 * common.length:
                    continue  # Excludes transverse crossings and hairpin projection ambiguity.
                chosen = ka, kb, common
                break
            if chosen:
                break
        if chosen is None:
            break
        ka, kb, common = chosen
        snapshot = [deepcopy(element) for element in root]
        safe = True
        shared_refs = []
        for point in common.coords:
            nid = allocate()
            lon, lat = inverse.transform(*point)
            node = etree.SubElement(root, "node", id=nid, lon=str(lon), lat=str(lat), version="1")
            for k, v in (("local_x", point[0]), ("local_y", point[1])):
                etree.SubElement(node, "tag", k=k, v=str(v))
            # Preserve this interval's physical height, not only its 2-D trace.
            z = sum(heights) / 2
            if z:
                etree.SubElement(node, "tag", k="ele", v=str(z))
            shared_refs.append(nid)
        shared_id = allocate()
        shared = etree.SubElement(root, "way", id=shared_id, version="1")
        for nid in shared_refs:
            etree.SubElement(shared, "nd", ref=nid)
        for k, v in tags(ways[ka]).items():
            if not k.startswith(("source:", "automap:", "lanelet2:")):
                etree.SubElement(shared, "tag", k=k, v=v)

        def make_way(original, points, endpoint_refs, nodes=nodes):
            elevations = [
                float(tags(nodes[n.get("ref")]).get("ele", "0")) for n in original.findall("nd")
            ]
            elevation = sum(elevations) / len(elevations) if elevations else 0
            refs = []
            for j, point in enumerate(points):
                nid = (
                    endpoint_refs[0]
                    if j == 0
                    else endpoint_refs[1]
                    if j == len(points) - 1
                    else None
                )
                if nid is None:
                    nid = allocate()
                    lon, lat = inverse.transform(*point)
                    node = etree.SubElement(
                        root, "node", id=nid, lon=str(lon), lat=str(lat), version="1"
                    )
                    for k, v in (("local_x", point[0]), ("local_y", point[1])):
                        etree.SubElement(node, "tag", k=k, v=str(v))
                    if elevation:
                        etree.SubElement(node, "tag", k="ele", v=str(elevation))
                refs.append(nid)
            wid = allocate()
            way = etree.SubElement(root, "way", id=wid, version="1")
            for nid in refs:
                etree.SubElement(way, "nd", ref=nid)
            for k, v in tags(original).items():
                etree.SubElement(way, "tag", k=k, v=v)
            return wid, refs

        for key in (ka, kb):
            for lane in list(users[key]):
                original = deepcopy(lane)
                members = {
                    m.get("role"): m.get("ref")
                    for m in original.findall("member")
                    if m.get("role") in {"left", "right", "centerline"}
                }
                raw = [lines[members[role]] for role in ("left", "right")]
                oriented = list(orient_boundaries(*raw))
                side = 0 if members["left"] == key else 1
                selected = oriented[side]
                cuts = sorted(
                    selected.project(Point(p), normalized=True)
                    for p in (common.coords[0], common.coords[-1])
                )
                bounds = [0] + [f for f in cuts if 1e-6 < f < 1 - 1e-6] + [1]
                bounds = sorted(set(bounds))
                center = (
                    LineString(
                        [xy(n.get("ref")) for n in ways[members["centerline"]].findall("nd")]
                    )
                    if members.get("centerline") in ways
                    else None
                )
                if center is None:
                    count = max(8, min(40, max(len(line.coords) for line in oriented)))
                    center = LineString(
                        [
                            tuple(
                                (a + b) / 2
                                for a, b in zip(
                                    oriented[0].interpolate(f, normalized=True).coords[0],
                                    oriented[1].interpolate(f, normalized=True).coords[0],
                                )
                            )
                            for f in [j / (count - 1) for j in range(count)]
                        ]
                    )
                previous_ends = [None, None]
                original_refs = []
                for j, role in enumerate(("left", "right")):
                    refs = [n.get("ref") for n in ways[members[role]].findall("nd")]
                    if Point(xy(refs[0])).distance(Point(oriented[j].coords[0])) > 1e-8:
                        refs.reverse()
                    original_refs.append(refs)
                fractions = [{0: 0, 1: 1}, {0: 0, 1: 1}, {0: 0, 1: 1}]
                for f in bounds[1:-1]:
                    p = selected.interpolate(f, normalized=True)
                    a = selected.interpolate(max(0, f - 1e-5), normalized=True)
                    b = selected.interpolate(min(1, f + 1e-5), normalized=True)
                    length = a.distance(b)
                    if length <= 1e-10:
                        safe = False
                        break
                    nx, ny = -(b.y - a.y) / length, (b.x - a.x) / length
                    reach = max(100, *(line.length for line in oriented))
                    normal = LineString(
                        [(p.x - nx * reach, p.y - ny * reach), (p.x + nx * reach, p.y + ny * reach)]
                    )
                    fractions[side][f] = f
                    for j, line in ((1 - side, oriented[1 - side]), (2, center)):
                        hit = normal.intersection(line)
                        points = (
                            [hit]
                            if isinstance(hit, Point)
                            else [g for g in getattr(hit, "geoms", []) if isinstance(g, Point)]
                        )
                        if not points:
                            safe = False
                            break
                        point = min(points, key=p.distance)
                        fractions[j][f] = line.project(point, normalized=True)
                if not safe or any(
                    any(mapping[y] <= mapping[x] + 1e-8 for x, y in pairwise(bounds))
                    for mapping in fractions
                ):
                    safe = False
                    break
                for part, (f0, f1) in enumerate(pairwise(bounds)):
                    item = lane if part == 0 else deepcopy(original)
                    if part:
                        item.set("id", allocate())
                        root.append(item)
                    for member in list(item.findall("member")):
                        if member.get("role") in {"left", "right", "centerline"}:
                            item.remove(member)
                    piece_boundaries = []
                    for j, role in enumerate(("left", "right")):
                        in_common = f0 >= cuts[0] - 1e-6 and f1 <= cuts[1] + 1e-6
                        if j == side and in_common:
                            wid = shared_id
                            points = list(common.coords)
                            if Point(points[0]).distance(
                                selected.interpolate(f0, normalized=True)
                            ) > Point(points[-1]).distance(
                                selected.interpolate(f0, normalized=True)
                            ):
                                refs = list(reversed(shared_refs))
                                points.reverse()
                            else:
                                refs = shared_refs
                        else:
                            segment = substring(
                                oriented[j], fractions[j][f0], fractions[j][f1], normalized=True
                            )
                            points = list(segment.coords)
                            start = original_refs[j][0] if part == 0 else previous_ends[j]
                            end = original_refs[j][-1] if part == len(bounds) - 2 else None
                            if j == side:
                                for edge, fraction in ((0, f0), (-1, f1)):
                                    if any(abs(fraction - cut) < 1e-6 for cut in cuts):
                                        closest = min(
                                            range(2),
                                            key=lambda q: Point(points[edge]).distance(
                                                Point(common.coords[0 if q == 0 else -1])
                                            ),
                                        )
                                        points[edge] = common.coords[0 if closest == 0 else -1]
                                        if edge == 0:
                                            start = shared_refs[0 if closest == 0 else -1]
                                        else:
                                            end = shared_refs[0 if closest == 0 else -1]
                            wid, refs = make_way(ways[members[role]], points, (start, end))
                        previous_ends[j] = refs[-1]
                        piece_boundaries.append(LineString(points))
                        etree.SubElement(item, "member", type="way", ref=wid, role=role)
                    oriented_piece = orient_boundaries(*piece_boundaries)
                    if any(
                        Point(a.coords[0]).distance(Point(b.coords[0])) > 1e-6
                        for a, b in zip(piece_boundaries, oriented_piece)
                    ):
                        safe = False
                    center_part = substring(
                        center, fractions[2][f0], fractions[2][f1], normalized=True
                    )
                    center_stub = etree.Element("way")
                    etree.SubElement(center_stub, "tag", k="type", v="virtual")
                    wid, _ = make_way(center_stub, list(center_part.coords), (None, None))
                    etree.SubElement(item, "member", type="way", ref=wid, role="centerline")
        if not safe:
            root[:] = snapshot
            attempts = retries.get((ka, kb), 0) + 1
            if attempts >= 5:
                blocked.add((ka, kb))
            else:
                retries[(ka, kb)] = attempts
            continue
        evidence.append(
            {
                "ways": [ka, kb],
                "shared_way": shared_id,
                "shared_length_m": common.length,
                "input_lengths_m": [lines[ka].length, lines[kb].length],
            }
        )
        repaired += 1
    sort_native_objects(root)
    return {
        "partial_boundary_repairs": repaired,
        "repair_limit_reached": repaired == max_repairs,
        "intervals": evidence,
        "retained_ambiguous_intervals": len(blocked),
    }
