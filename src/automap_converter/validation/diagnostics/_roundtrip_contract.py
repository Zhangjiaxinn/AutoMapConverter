"""Four test-plan measures with identity-based correspondence."""

from __future__ import annotations

import math
from dataclasses import replace
from itertools import pairwise

import networkx as nx
from shapely.geometry import LineString, Point

from automap_converter.core.traceability import metric_frame, transform_points

POINT_FRACTIONS = tuple(i / 19 for i in range(20))
WIDTH_FRACTIONS = (0.25, 0.35, 0.5, 0.65, 0.75)
METRIC_NAMES = (
    "centerline_mean_error_m",
    "lane_width_mean_relative_error",
    "topology_edge_match_rate",
    "type_direction_attribute_match_rate",
)
SUPPORTED_RULES = frozenset(
    {
        "speed_limit",
        "traffic_light",
        "stop_line",
        "stop",
        "yield",
        "right_of_way",
        "crosswalk",
        "turn_restriction",
    }
)


def _sample(points, fraction):
    return LineString(points).interpolate(fraction, normalized=True)


def section_width(feature, fraction, reference=None):
    """Intersect both actual boundaries with a normal to this lane's centerline."""
    if not feature.left or not feature.right:
        # OSM width has already been interpreted by the shared import policy;
        # its measurement/estimation basis is recorded on the feature.
        return feature.width if feature.width is not None and feature.width > 0 else None
    reference = reference or feature
    line = LineString(reference.points)
    if line.length <= 1e-9:
        return None
    # End sections use a one-sided tangent; a tiny inset avoids floating-point
    # endpoint misses while retaining the declared start/middle/end stations.
    fraction = min(1 - 1e-8, max(1e-8, fraction))
    p = _sample(reference.points, fraction)
    a = _sample(reference.points, max(0, fraction - 1e-5))
    b = _sample(reference.points, min(1, fraction + 1e-5))
    dx, dy = b.x - a.x, b.y - a.y
    length = math.hypot(dx, dy)
    if length <= 1e-12:
        return None
    nx_, ny_ = -dy / length, dx / length
    reach = max(100.0, LineString(feature.left).length, LineString(feature.right).length)
    normal = LineString(
        [(p.x - nx_ * reach, p.y - ny_ * reach), (p.x + nx_ * reach, p.y + ny_ * reach)]
    )

    def intersection(boundary):
        hit = normal.intersection(LineString(boundary))
        points = (
            [hit]
            if isinstance(hit, Point)
            else [g for g in getattr(hit, "geoms", []) if isinstance(g, Point)]
        )
        return min(points, key=lambda q: p.distance(q)) if points else None

    left, right = intersection(feature.left), intersection(feature.right)
    if left is None or right is None:
        return None
    return left.distance(right)


def _metric(name, value, threshold, operator, passed, total, status, method):
    rate = passed / total if total else None
    return {
        "name": name,
        "value": value,
        "threshold": threshold,
        "operator": operator,
        "status": status,
        "method": method,
        "passed": passed,
        "total": total,
        "pass_rate": rate,
        "pass_rate_percent": round(100 * rate, 4) if rate is not None else None,
    }


def _metric_features(feature_set, frame):
    return [
        replace(
            f,
            points=transform_points(f.points, feature_set.frame, frame),
            left=transform_points(f.left, feature_set.frame, frame) if f.left else [],
            right=transform_points(f.right, feature_set.frame, frame) if f.right else [],
            rules=[
                {**r, "points": transform_points(r["points"], feature_set.frame, frame)}
                for r in f.rules
            ],
        )
        for f in feature_set.features
    ]


def _correspondence(source, target):
    index = {}
    for i, feature in enumerate(target.features):
        for ancestor in feature.ancestors:
            if (
                ancestor.get("format") == source.format
                and ancestor.get("document") == source.document
                and ancestor.get("role") != "generated_connector"
            ):
                index.setdefault(ancestor["key"], set()).add(i)
    return {
        i: sorted(set().union(*(index.get(k, set()) for k in (f.native_keys or [f.key]))))
        for i, f in enumerate(source.features)
    }


def _relations(feature_set):
    graph = nx.MultiDiGraph()
    for f in feature_set.features:
        graph.add_node(f.key)
        for role, neighbors in f.relations.items():
            for neighbor in neighbors:
                graph.add_edge(f.key, neighbor, key=role, role=role)
    # FeatureSet.topology_edges remains available to other diagnostic callers.
    for a, b in feature_set.topology_edges:
        graph.add_edge(a, b, key="successor", role="successor")
        if feature_set.format != "osm":
            graph.add_edge(b, a, key="predecessor", role="predecessor")
    return graph


def _relation_path(starts, ends, role, graph, target_features, owned, physical_adjacency=False):
    for x in starts:
        for y in ends:
            if graph.has_edge(x, y, key=role):
                return [x, y], "native labeled edge"
            if role in {"successor", "predecessor"}:
                if x == y:
                    return [x], "recorded longitudinal contraction"
                allowed = (set(graph) - owned) | set(starts) | set(ends)
                chain = nx.DiGraph(
                    (a, b)
                    for a, b, r in graph.edges(keys=True)
                    if r == role and a in allowed and b in allowed
                )
                if x in chain and y in chain and nx.has_path(chain, x, y):
                    return nx.shortest_path(
                        chain, x, y
                    ), "actual directed path through generated subdivisions/connectors"
            elif role in {"left", "right"} and physical_adjacency:
                # OpenDRIVE may place two physically adjacent directions in
                # separate roads. Measure physical adjacency, not lane-change
                # permission or an arbitrary graph path across other lanes.
                a, b = target_features[x], target_features[y]
                boundary = a.left if role == "left" else a.right
                if not boundary or not b.left or not b.right:
                    continue
                edge = LineString(boundary)
                other_edges = (LineString(b.left), LineString(b.right))
                tangent = (
                    _sample(a.points, 0.51).x - _sample(a.points, 0.49).x,
                    _sample(a.points, 0.51).y - _sample(a.points, 0.49).y,
                )
                delta = (
                    _sample(b.points, 0.5).x - _sample(a.points, 0.5).x,
                    _sample(b.points, 0.5).y - _sample(a.points, 0.5).y,
                )
                cross = tangent[0] * delta[1] - tangent[1] * delta[0]
                correct_side = cross > 0 if role == "left" else cross < 0
                if correct_side and any(
                    all(
                        edge.interpolate(f, normalized=True).distance(e) <= 0.05
                        for f in WIDTH_FRACTIONS
                    )
                    for e in other_edges
                ):
                    return [
                        x,
                        y,
                    ], "physical shared boundary in generated geometry (5 cm numerical tolerance)"
    return [], "no equivalent generated relation"


def _lateral_interval_evidence(source, starts, ends, role, graph, features):
    edge = getattr(source, role)
    if not edge:
        return None
    original = LineString(edge)
    if original.length <= 1e-9:
        return None
    intervals, pieces = [], []
    for x in starts:
        for y in ends:
            if not graph.has_edge(x, y, key=role):
                continue
            a, b = features[x], features[y]
            key = a.measurements.get("native_boundary_ids", {}).get(role)
            if not key or key not in b.measurements.get("native_boundary_ids", {}).values():
                continue
            actual = LineString(getattr(a, role))
            stations = [original.project(Point(p), normalized=True) for p in actual.coords]
            lo, hi = min(stations), max(stations)
            if (hi - lo) * original.length <= 1e-6:
                continue
            intervals.append((lo, hi))
            pieces.append(
                {
                    "target_from": x,
                    "target_to": y,
                    "shared_way": key,
                    "source_fraction_range": [lo, hi],
                    "shared_length_m": actual.length,
                    "max_distance_to_source_m": max(
                        original.distance(Point(p)) for p in actual.coords
                    ),
                }
            )
    merged = []
    for lo, hi in sorted(intervals):
        if merged and lo <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(merged[-1][1], hi)
        else:
            merged.append([lo, hi])
    length = original.length * sum(hi - lo for lo, hi in merged)
    return {
        "source_interval_length_m": original.length,
        "covered_length_m": length,
        "covered_fraction": length / original.length,
        "source_fraction_intervals": merged,
        "full_interval_preserved": original.length - length <= 0.05,
        "pieces": pieces,
        "sample_fractions": list(WIDTH_FRACTIONS),
        "sample_covered": [
            any(lo - 1e-9 <= f <= hi + 1e-9 for lo, hi in merged) for f in WIDTH_FRACTIONS
        ],
        "interpretation": "Lateral graph matches require actual shared-boundary intervals at all five fixed source stations. Full interval retention is reported separately; no fifth metric is added.",
    }


def _ordered_target(source_feature, indices, target_features, graph):
    if not indices:
        return None, "No recorded source-to-target correspondence."
    parts = [target_features[i] for i in indices]
    if len(parts) == 1:
        return parts[0], "recorded identity"
    keys = {f.key for f in parts}
    sub = nx.DiGraph()
    sub.add_nodes_from(keys)
    for a, b, role in graph.edges(keys=True):
        if role == "successor" and a in keys and b in keys:
            sub.add_edge(a, b)
    if (
        not source_feature.left
        and not source_feature.right
        and all(f.direction in {"no", "both", "bidirectional"} for f in parts)
    ):
        # A bidirectional OSM road yields reciprocal edges. Its subdivision is
        # a physical chain, not a directed cycle/parallel-lane ambiguity.
        physical = sub.to_undirected()
        ends = [k for k in physical if physical.degree(k) == 1]
        if (
            nx.is_connected(physical)
            and len(ends) == 2
            and all(physical.degree(k) <= 2 for k in physical)
        ):
            path = nx.shortest_path(physical, ends[0], ends[1])
            by_key = {f.key: f for f in parts}
            ordered = []
            for index, key in enumerate(path):
                feature = by_key[key]
                if index:
                    reverse = Point(ordered[-1].points[-1]).distance(
                        Point(feature.points[-1])
                    ) < Point(ordered[-1].points[-1]).distance(Point(feature.points[0]))
                else:
                    following = by_key[path[1]]
                    reverse = min(
                        Point(feature.points[0]).distance(Point(p))
                        for p in (following.points[0], following.points[-1])
                    ) < min(
                        Point(feature.points[-1]).distance(Point(p))
                        for p in (following.points[0], following.points[-1])
                    )
                ordered.append(
                    replace(feature, points=list(reversed(feature.points))) if reverse else feature
                )
            vector = (
                ordered[-1].points[-1][0] - ordered[0].points[0][0],
                ordered[-1].points[-1][1] - ordered[0].points[0][1],
            )
            direction = (
                source_feature.points[-1][0] - source_feature.points[0][0],
                source_feature.points[-1][1] - source_feature.points[0][1],
            )
            if vector[0] * direction[0] + vector[1] * direction[1] < 0:
                ordered = [replace(f, points=list(reversed(f.points))) for f in reversed(ordered)]
            if all(
                Point(a.points[-1]).distance(Point(b.points[0])) <= 0.05
                for a, b in pairwise(ordered)
            ):
                return replace(
                    ordered[0], points=[p for f in ordered for p in f.points]
                ), "recorded bidirectional OSM subdivisions joined through actual shared endpoints"
    if (
        not nx.is_directed_acyclic_graph(sub)
        or any(sub.in_degree(k) > 1 or sub.out_degree(k) > 1 for k in keys)
        or not nx.is_weakly_connected(sub)
    ):
        return (
            None,
            "Recorded correspondence is branching/parallel/disconnected; not a unique lane chain.",
        )
    by_key = {f.key: f for f in parts}
    ordered = [by_key[k] for k in nx.topological_sort(sub)]
    for a, b in pairwise(ordered):
        if Point(a.points[-1]).distance(Point(b.points[0])) > 0.05:
            return None, "Generated pieces have a geometric gap; no artificial bridge was inserted."
    return replace(
        ordered[0],
        points=[p for f in ordered for p in f.points],
        left=[p for f in ordered for p in f.left] if all(f.left for f in ordered) else [],
        right=[p for f in ordered for p in f.right] if all(f.right for f in ordered) else [],
    ), "recorded longitudinal pieces joined by actual successor links"


def compare(source, target):
    mapping = _correspondence(source, target)
    coordinate_error = None
    try:
        if any(fs.coordinate_checks.get("status") == "FAIL" for fs in (source, target)):
            raise ValueError(
                "Declared XY/geographic encodings are inconsistent; inspect coordinate control points."
            )
        first = source.features[0].points[0] if source.features else (0, 0)
        frame = metric_frame(source.frame, first)
        src = _metric_features(source, frame)
        dst = _metric_features(target, frame)
    except (ValueError, TypeError) as exc:
        coordinate_error = str(exc)
        frame, src, dst = source.frame, source.features, target.features
    source_graph, target_graph = _relations(source), _relations(target)
    lane_records, geometry_means, width_errors = [], [], []
    geometry_passed = width_passed = width_total = attribute_passed = 0
    geometry_ok = True
    for i, feature in enumerate(src):
        actual, reason = _ordered_target(feature, mapping[i], dst, target_graph)
        if coordinate_error:
            actual, reason = None, coordinate_error
        errors = (
            [
                _sample(feature.points, f).distance(_sample(actual.points, f))
                for f in POINT_FRACTIONS
            ]
            if actual is not None
            else []
        )
        mean = sum(errors) / len(errors) if errors else None
        lane_geometry_ok = mean is not None and mean <= 1.0
        geometry_ok = geometry_ok and lane_geometry_ok
        geometry_passed += int(lane_geometry_ok)
        if mean is not None:
            geometry_means.append(mean)
        widths = []
        for fraction in (0.5,) if source.format == "osm" else WIDTH_FRACTIONS:
            source_width = section_width(feature, fraction)
            # A source with declared boundaries but an invalid section is an
            # invalid measurement, not an excuse to remove it from the denominator.
            applicable = bool(feature.left and feature.right) or feature.width is not None
            if not applicable:
                continue
            target_width = (
                section_width(actual, fraction, reference=feature) if actual is not None else None
            )
            target_widths = None
            if source.format == "osm" and actual is not None:
                target_widths = [dst[j].width for j in mapping[i]]
            error = (
                abs(target_width - source_width) / source_width
                if source_width is not None and source_width > 1e-9 and target_width is not None
                else None
            )
            if target_widths:
                error = (
                    max(abs(w - source_width) / source_width for w in target_widths)
                    if source_width is not None
                    and source_width > 1e-9
                    and all(w is not None and w > 0 for w in target_widths)
                    else None
                )
            passed = error is not None and error <= 0.10
            width_total += 1
            width_passed += int(passed)
            if error is not None:
                width_errors.append(error)
            widths.append(
                {
                    "fraction": fraction,
                    "source_width_m": source_width,
                    "target_width_m": target_width,
                    "target_width_properties_m": target_widths,
                    "relative_error": error,
                    "passed": passed,
                    "section_basis": "same source station and source normal for both maps",
                }
            )
        fields = {"type": feature.kind, "direction": feature.direction, **feature.attributes}
        field_checks = []
        for name, value in fields.items():
            values = [
                {"type": dst[j].kind, "direction": dst[j].direction, **dst[j].attributes}.get(name)
                for j in mapping[i]
            ]
            passed = bool(values) and all(v == value for v in values)
            if name == "direction" and value not in {"no", "both", "bidirectional"} and actual:
                a = feature.points[-1][0] - feature.points[0][0]
                b = feature.points[-1][1] - feature.points[0][1]
                c = actual.points[-1][0] - actual.points[0][0]
                d = actual.points[-1][1] - actual.points[0][1]
                passed = passed and a * c + b * d >= 0
            field_checks.append(
                {"field": name, "source": value, "target": values, "passed": passed}
            )
        generated_rules = [r for j in mapping[i] for r in dst[j].rules]
        for rule in feature.rules:

            def center(r):
                return Point(
                    sum(p[0] for p in r["points"]) / len(r["points"]),
                    sum(p[1] for p in r["points"]) / len(r["points"]),
                )

            distances = [
                center(rule).distance(center(r))
                for r in generated_rules
                if r["kind"] == rule["kind"] and r.get("value") == rule.get("value")
            ]
            field_checks.append(
                {
                    "field": "rule_location:" + rule["kind"],
                    "source": rule,
                    "nearest_actual_rule_distance_m": min(distances) if distances else None,
                    "passed": bool(distances)
                    and min(distances) <= 1.0
                    and coordinate_error is None,
                }
            )
        attributes_ok = bool(field_checks) and all(c["passed"] for c in field_checks)
        attribute_passed += int(attributes_ok)
        lane_records.append(
            {
                "source_key": feature.key,
                "source_measurement_basis": feature.measurements,
                "target_keys": [dst[j].key for j in mapping[i]],
                "mapping_detail": reason,
                "point_errors_m": errors,
                "centerline_mean_error_m": mean,
                "centerline_mean_passed": lane_geometry_ok,
                "point_passed": sum(e <= 1.0 for e in errors),
                "point_total": 20,
                "width_sections": widths,
                "attributes": field_checks,
                "attributes_passed": attributes_ok,
            }
        )

    src_indices = {f.key: i for i, f in enumerate(src)}
    relation_records = []
    role_counts = {}
    owned = {dst[j].key for indices in mapping.values() for j in indices}
    dst_by_key = {f.key: f for f in dst}
    for a, b, role in source_graph.edges(keys=True):
        if a not in src_indices or b not in src_indices:
            continue
        starts = [dst[j].key for j in mapping[src_indices[a]]]
        ends = [dst[j].key for j in mapping[src_indices[b]]]
        # Longitudinal contraction is allowed only for identities genuinely
        # represented by the same generated object. Lateral edges cannot collapse.
        path, method = _relation_path(
            starts,
            ends,
            role,
            target_graph,
            dst_by_key,
            owned,
            physical_adjacency=target.format == "opendrive",
        )
        passed = bool(path)
        relation_records.append(
            {
                "from": a,
                "to": b,
                "role": role,
                "passed": passed,
                "target_path": path,
                "method": method,
            }
        )
        if role in {"left", "right"} and target.format == "lanelet2":
            interval = _lateral_interval_evidence(
                src[src_indices[a]], starts, ends, role, target_graph, dst_by_key
            )
            relation_records[-1]["interval_evidence"] = interval
            relation_records[-1]["native_graph_edge_matched"] = passed
            if interval is not None:
                passed = passed and all(interval["sample_covered"])
                relation_records[-1]["passed"] = passed
                if not passed and relation_records[-1]["native_graph_edge_matched"]:
                    relation_records[-1]["method"] = (
                        "native adjacency exists, but does not retain every fixed source-interval station"
                    )
        counts = role_counts.setdefault(role, {"passed": 0, "total": 0})
        counts["total"] += 1
        counts["passed"] += int(passed)
    topo_total = len(relation_records)
    topo_passed = sum(r["passed"] for r in relation_records)
    topo_rate = topo_passed / topo_total if topo_total else None
    attr_rate = attribute_passed / len(src) if src else None
    geometry_value = sum(geometry_means) / len(geometry_means) if geometry_means else None
    width_value = sum(width_errors) / len(width_errors) if width_errors else None
    metrics = [
        _metric(
            METRIC_NAMES[0],
            geometry_value,
            1.0,
            "<=",
            geometry_passed,
            len(src),
            "PASS" if geometry_ok and src else "FAIL",
            "Source-line mean pass rate: lines with mean paired-point error <=1 m / all source lines. Each line uses 20 normalized-station point pairs. OSM: each highway way; other formats: each lane. Every line mean must pass; missing identities fail.",
        ),
        _metric(
            METRIC_NAMES[1],
            width_value,
            0.10,
            "<=",
            width_passed,
            width_total,
            ("PASS" if width_passed == width_total else "FAIL") if width_total else "SKIP",
            "OSM: native/estimated road-width property. Other formats: same source normal at fixed interior stations 25/35/50/65/75%; every applicable sample <=10%.",
        ),
        _metric(
            METRIC_NAMES[2],
            topo_rate,
            0.85,
            ">=",
            topo_passed,
            topo_total,
            ("PASS" if topo_rate >= 0.85 else "FAIL") if topo_rate is not None else "SKIP",
            "OSM: road-way successor graph at shared nodes with restrictions. Others: labeled lane graph and directed subdivision paths. Lanelet2 lateral edges require native shared-boundary intervals at all five fixed source stations; no arbitrary reachability across another mapped source lane.",
        ),
        _metric(
            METRIC_NAMES[3],
            attr_rate,
            0.85,
            ">=",
            attribute_passed,
            len(src),
            "PASS" if attr_rate is not None and attr_rate >= 0.85 else "FAIL",
            "A lane matches only when all its applicable supported type/direction/rule fields match the actual generated lane fields.",
        ),
    ]
    metrics[0]["max_lane_mean_error_m"] = max(geometry_means) if geometry_means else None
    metrics[0]["pass_rate_basis"] = "source_line_mean"
    metrics[0]["unmeasured_lanes"] = sum(r["centerline_mean_error_m"] is None for r in lane_records)
    metrics[1]["max_section_relative_error"] = max(width_errors) if width_errors else None
    metrics[1]["unmeasured_sections"] = width_total - len(width_errors)
    failed = [m for m in metrics if m["status"] == "FAIL"]
    return {
        "result": "FAIL" if failed else "PASS",
        "metrics": metrics,
        "issues": [
            {"category": m["name"], "status": "FAIL", "summary": m["method"]} for m in failed
        ],
        "sampling": {
            "centerline_fractions": POINT_FRACTIONS,
            "width_fractions": WIDTH_FRACTIONS,
            "osm_width": "one native/estimated road-width property comparison per highway way",
        },
        "evidence": {
            "matching_method": "recorded_conversion_lineage",
            "source_frame": source.frame,
            "target_frame": target.frame,
            "comparison_frame": frame,
            "coordinate_error": coordinate_error,
            "source_coordinate_checks": source.coordinate_checks,
            "target_coordinate_checks": target.coordinate_checks,
            "mapped_source_features": sum(bool(v) for v in mapping.values()),
            "source_features": len(src),
            "unmapped_source_keys": [src[i].key for i, v in mapping.items() if not v],
            "lanes": lane_records,
            "topology_relations": relation_records,
            "topology_by_role": role_counts,
            "supported_rules": sorted(SUPPORTED_RULES),
        },
        "functional_equivalence": {
            "status": "REVIEW",
            "method": "Four numeric measures do not replace viewer/routing/simulation functional review.",
        },
    }
