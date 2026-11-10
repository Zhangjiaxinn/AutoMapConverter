"""Finalize native OpenDRIVE geometry without changing road/lane identities."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass
from itertools import pairwise

import numpy as np
from lxml import etree
from scipy.integrate import trapezoid
from scipy.interpolate import CubicHermiteSpline, PchipInterpolator
from scipy.optimize import minimize_scalar
from shapely.geometry import LineString, Point

from automap_converter.conversion.lanelet2_to_opendrive.boundary_sampling import local_boundary_hit


@dataclass
class BoundaryPart:
    start: float
    end: float
    left: LineString
    right: LineString


def _unit(vector):
    length = float(np.linalg.norm(vector))
    if length < 1e-10:
        raise ValueError("Reference line has a zero tangent or an exact reversal")
    return np.asarray(vector, dtype=float) / length


def _rounded_supports(points, directions, lengths, tangents, max_error):
    knots = np.r_[0.0, np.cumsum(lengths)]
    rows = [(0.0, points[0], tangents[0])]
    for index in range(1, len(points) - 1):
        incoming, outgoing = directions[index - 1 : index + 1]
        sine_half = np.linalg.norm(outgoing - incoming) / 2
        if sine_half < 1e-8:
            rows.append((knots[index], points[index], incoming))
            continue
        cut = min(0.45 * min(lengths[index - 1 : index + 1]), 2 * max_error / sine_half)
        rows.extend(
            [
                (knots[index] - cut, points[index] - cut * incoming, incoming),
                (knots[index] + cut, points[index] + cut * outgoing, outgoing),
            ]
        )
    rows.append((knots[-1], points[-1], tangents[-1]))
    # Keep endpoint constraints local to the first/last straight approach.
    extra = []
    for end, first, second in [(0, rows[0], rows[1]), (1, rows[-2], rows[-1])]:
        direction = _unit(second[1] - first[1])
        tangent = first[2] if end == 0 else second[2]
        sine = abs(direction[0] * tangent[1] - direction[1] * tangent[0])
        span = second[0] - first[0]
        if sine > 1e-10 and span * sine / 3 > max_error:
            distance = min(span / 3, 2.7 * max_error / sine)
            if end == 0:
                extra.append((first[0] + distance, first[1] + distance * direction, direction))
            else:
                extra.append((second[0] - distance, second[1] - distance * direction, direction))
    rows = sorted(rows + extra, key=lambda row: row[0])
    return [r[1] for r in rows], [r[2] for r in rows], [r[0] for r in rows]


class SmoothReference:
    """C1 Hermite interpolation, with control points inside a bounded corridor."""

    def __init__(
        self, points, max_error=0.05, start_tangent=None, end_tangent=None, round_corners=False
    ):
        if not math.isfinite(max_error) or max_error <= 0:
            raise ValueError("max_error must be finite and positive")
        points = np.asarray(points, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2 or not np.isfinite(points).all():
            raise ValueError("Reference vertices must be finite XY pairs")
        keep = np.r_[True, np.linalg.norm(np.diff(points, axis=0), axis=1) > 1e-8]
        points = points[keep]
        if len(points) < 2:
            raise ValueError("Reference line needs two distinct points")
        delta = np.diff(points, axis=0)
        lengths = np.linalg.norm(delta, axis=1)
        directions = delta / lengths[:, None]
        tangents = np.vstack([directions[0], directions[:-1] + directions[1:], directions[-1]])
        tangents = np.array([_unit(v) for v in tangents])
        if start_tangent is not None:
            tangents[0] = _unit(start_tangent)
        if end_tangent is not None:
            tangents[-1] = _unit(end_tangent)

        # Extra collinear supports localize each turn instead of shrinking its tangent.
        # Each Bezier control lies within max_error of its original line segment.
        vertices, derivatives, stations = [points[0]], [tangents[0]], [0.0]
        station = 0.0
        for index, (length, direction) in enumerate(zip(lengths, directions)):
            inserts = []
            for end, tangent in enumerate(tangents[index : index + 2]):
                if np.dot(tangent, direction) <= 0:
                    raise ValueError("Reference endpoint tangent reverses its line segment")
                sine = abs(float(direction[0] * tangent[1] - direction[1] * tangent[0]))
                if sine > 1e-10 and length * sine / 3 > max_error:
                    distance = min(length / 3, 2.7 * max_error / sine)
                    inserts.append(distance if end == 0 else length - distance)
            for distance in sorted(set(inserts)):
                vertices.append(points[index] + distance * direction)
                derivatives.append(direction)
                stations.append(station + distance)
            station += length
            vertices.append(points[index + 1])
            derivatives.append(tangents[index + 1])
            stations.append(station)
        if round_corners:
            vertices, derivatives, stations = _rounded_supports(
                points, directions, lengths, tangents, max_error
            )
        self.points = np.asarray(vertices)
        self.old_stations = np.asarray(stations)
        self.old_length = float(station)
        spline = CubicHermiteSpline(self.old_stations, self.points, derivatives)
        sizes = np.diff(self.old_stations)
        self.coefficients = np.moveaxis(spline.c, 1, 0).copy()
        for power in range(4):
            self.coefficients[:, power] *= sizes[:, None] ** (3 - power)
        self.forward, self.inverse, lengths = [], [], []
        nodes, weights = np.polynomial.legendre.leggauss(12)
        parameters = np.linspace(0, 1, 65)
        mids = (parameters[:-1] + parameters[1:]) / 2
        half = np.diff(parameters) / 2
        samples = mids[:, None] + half[:, None] * nodes
        for coefficient in self.coefficients:
            speed = np.linalg.norm(
                (3 * coefficient[0] * samples[..., None] + 2 * coefficient[1]) * samples[..., None]
                + coefficient[2],
                axis=-1,
            )
            arc = np.r_[0.0, np.cumsum(half * np.sum(speed * weights, axis=1))]
            if np.any(np.diff(arc) <= 0):
                raise ValueError("Reference curve has non-increasing arc length")
            self.forward.append(PchipInterpolator(parameters, arc))
            self.inverse.append(PchipInterpolator(arc, parameters))
            lengths.append(arc[-1])
        self.stations = np.r_[0.0, np.cumsum(lengths)]
        self.length = float(self.stations[-1])

    def old_to_new(self, station):
        if station < 0:
            return float(station)
        if station > self.old_length:
            return self.length + station - self.old_length
        index = min(
            len(self.coefficients) - 1,
            np.searchsorted(self.old_stations, station, side="right") - 1,
        )
        p = (station - self.old_stations[index]) / (
            self.old_stations[index + 1] - self.old_stations[index]
        )
        return float(self.stations[index] + self.forward[index](p))

    def parameter(self, station):
        station = float(np.clip(station, 0, self.length))
        index = min(
            len(self.coefficients) - 1, np.searchsorted(self.stations, station, side="right") - 1
        )
        return index, float(self.inverse[index](station - self.stations[index]))

    def at_parameter(self, index, p):
        a, b, c, d = self.coefficients[index]
        return ((a * p + b) * p + c) * p + d, _unit((3 * a * p + 2 * b) * p + c)

    def at(self, station):
        index, p = self.parameter(station)
        point, tangent = self.at_parameter(index, p)
        if station < 0 or station > self.length:
            point = point + (station - np.clip(station, 0, self.length)) * tangent
        return point, tangent

    def new_to_old(self, station):
        index, p = self.parameter(station)
        return float(
            self.old_stations[index] + p * (self.old_stations[index + 1] - self.old_stations[index])
        )

    def curvature(self, station):
        index, p = self.parameter(station)
        a, b, c, _ = self.coefficients[index]
        first = (3 * a * p + 2 * b) * p + c
        second = 6 * a * p + 2 * b
        return float((first[0] * second[1] - first[1] * second[0]) / np.linalg.norm(first) ** 3)

    def project(self, point, old_hint):
        approximate = self.at(self.old_to_new(old_hint))[0]
        reach = max(1.0, 2 * float(np.linalg.norm(point - approximate)))
        first = max(0, np.searchsorted(self.old_stations, old_hint - reach, side="right") - 1)
        last = min(
            len(self.coefficients),
            np.searchsorted(self.old_stations, old_hint + reach, side="right") + 1,
        )
        best = None
        for index in range(first, last):

            def objective(p, index=index):
                delta = self.at_parameter(index, p)[0] - point
                return float(np.dot(delta, delta))

            result = minimize_scalar(
                objective, bounds=(0.0, 1.0), method="bounded", options={"xatol": 1e-12}
            )
            for p in (0.0, float(result.x), 1.0):
                candidate = (objective(p), index, p)
                if best is None or candidate < best:
                    best = candidate
        _, index, p = best
        base, tangent = self.at_parameter(index, p)
        normal = np.array([-tangent[1], tangent[0]])
        return (
            float(self.stations[index] + self.forward[index](p)),
            float(np.dot(point - base, normal)),
            math.atan2(tangent[1], tangent[0]),
        )

    def write(self, plan):
        for child in list(plan):
            plan.remove(child)
        for index, coefficient in enumerate(self.coefficients):
            point, tangent = self.at_parameter(index, 0.0)
            heading = math.atan2(tangent[1], tangent[0])
            rotation = np.array(
                [[math.cos(heading), math.sin(heading)], [-math.sin(heading), math.cos(heading)]]
            )
            a, b, c = [rotation @ vector for vector in coefficient[:3]]
            geometry = etree.SubElement(
                plan,
                "geometry",
                s=f"{self.stations[index]:.12f}",
                x=f"{point[0]:.12f}",
                y=f"{point[1]:.12f}",
                hdg=f"{heading:.14f}",
                length=f"{self.stations[index + 1] - self.stations[index]:.12f}",
            )
            if max(abs(a).max(), abs(b).max(), abs(c[1])) < 1e-10:
                etree.SubElement(geometry, "line")
            else:
                etree.SubElement(
                    geometry,
                    "paramPoly3",
                    aU="0",
                    aV="0",
                    bU=f"{c[0]:.14g}",
                    bV="0",
                    cU=f"{b[0]:.14g}",
                    cV=f"{b[1]:.14g}",
                    dU=f"{a[0]:.14g}",
                    dV=f"{a[1]:.14g}",
                    pRange="normalized",
                )


def _value(records, station, key):
    if not records:
        return 0.0
    index = max(0, np.searchsorted([float(r[key]) for r in records], station, side="right") - 1)
    record = records[index]
    ds = station - float(record[key])
    return sum(
        float(record.get(name, 0)) * ds**power for power, name in enumerate(("a", "b", "c", "d"))
    )


def _linear_records(parent, tag, stations, values, key, origin=0.0):
    for child in list(parent.findall(tag)):
        parent.remove(child)
    for index, (start, end) in enumerate(pairwise(stations)):
        if end - start <= 1e-9:
            continue
        etree.SubElement(
            parent,
            tag,
            **{
                key: f"{start - origin:.12f}",
                "a": f"{values[index]:.12g}",
                "b": f"{(values[index + 1] - values[index]) / (end - start):.12g}",
                "c": "0",
                "d": "0",
            },
        )


def _hit(boundary, normal, point, tangent):
    hit = local_boundary_hit(boundary, normal, Point(point))
    if hit is None:
        return None
    return float(np.dot(np.array([hit.x, hit.y]) - point, [-tangent[1], tangent[0]]))


def _native_section(parts, old_station, point, tangent):
    normal_vector = np.array([-tangent[1], tangent[0]])
    normal = LineString([point - 200 * normal_vector, point + 200 * normal_vector])
    candidates = []
    for part in parts:
        if old_station < part.start - 0.05 or old_station > part.end + 0.05:
            continue
        left = _hit(part.left, normal, point, tangent)
        right = _hit(part.right, normal, point, tangent)
        if left is not None and right is not None and left - right > 1e-6:
            candidates.append((abs((left + right) / 2), left, right))
    return min(candidates)[1:] if candidates else None


def _rebuild_lanes(road, curve, boundary_parts):
    lanes = road.find("lanes")
    offsets = [dict(e.attrib) for e in lanes.findall("laneOffset")]
    sections = lanes.findall("laneSection")
    starts = [float(s.get("s")) for s in sections]
    fallback = 0
    inverted_area = 0.0
    offset_samples = {}
    for index, section in enumerate(sections):
        old_start = starts[index]
        old_end = starts[index + 1] if index + 1 < len(starts) else curve.old_length
        start, end = curve.old_to_new(old_start), curve.old_to_new(old_end)
        section.set("s", f"{start:.12f}")
        native_lanes = section.findall("right/lane")
        native_lanes.sort(key=lambda lane: -int(lane.get("id")))
        original = [[dict(w.attrib) for w in lane.findall("width")] for lane in native_lanes]
        grid = set(np.linspace(start, end, max(2, math.ceil((end - start) / 0.5) + 1)))
        grid.update(float(s) for s in curve.stations if start < s < end)
        for records in original:
            grid.update(
                curve.old_to_new(old_start + float(w["sOffset"]))
                for w in records
                if old_start + float(w["sOffset"]) < old_end
            )
        cache = {}

        def sample(
            station, cache=cache, native_lanes=native_lanes, original=original, old_start=old_start
        ):
            nonlocal fallback
            if station in cache:
                return cache[station]
            point, tangent = curve.at(station)
            old_station = curve.new_to_old(station)
            values = [_value(offsets, old_station, "s")]
            for lane, records in zip(native_lanes, original):
                measured = _native_section(
                    boundary_parts.get(int(lane.get("id")), []), old_station, point, tangent
                )
                if measured is None:
                    fallback += 1
                    width = _value(records, old_station - old_start, "sOffset")
                else:
                    left, right = measured
                    width = left - right
                    if int(lane.get("id")) == -1:
                        values[0] = left
                values.append(width)
            cache[station] = np.asarray(values)
            return cache[station]

        for _ in range(3):
            stations = sorted(grid)
            extra = []
            for first, second in pairwise(stations):
                mid = (first + second) / 2
                if (
                    second - first > 0.02
                    and np.max(abs(sample(mid) - (sample(first) + sample(second)) / 2)) > 0.02
                ):
                    extra.append(mid)
            grid.update(extra)
            if not extra:
                break
        stations = sorted(grid)
        values = np.asarray([sample(s) for s in stations])
        negative_sections = []
        for station, row in zip(stations, values):
            curvature = curve.curvature(station)
            inner = row[0]
            area = 0.0
            for width in row[1:]:
                outer = inner - width
                if curvature > 1e-10 and inner > 1 / curvature:
                    low = max(outer, 1 / curvature)
                    area += curvature * (inner * inner - low * low) / 2 - (inner - low)
                elif curvature < -1e-10 and outer < 1 / curvature:
                    high = min(inner, 1 / curvature)
                    area += curvature * (high * high - outer * outer) / 2 - (high - outer)
                inner = outer
            negative_sections.append(max(0.0, area))
        inverted_area += float(trapezoid(negative_sections, stations))
        offset_samples.update(zip(stations, values[:, 0]))
        for lane_index, lane in enumerate(native_lanes):
            _linear_records(lane, "width", stations, values[:, lane_index + 1], "sOffset", start)
        for lane in section.findall("*/lane"):
            for record in lane:
                if record.tag != "width" and "sOffset" in record.attrib:
                    record.set(
                        "sOffset",
                        f"{curve.old_to_new(old_start + float(record.get('sOffset'))) - start:.12f}",
                    )
    stations = sorted(offset_samples)
    _linear_records(lanes, "laneOffset", stations, [offset_samples[s] for s in stations], "s")
    return fallback, inverted_area


def _remap_anchors(road, curve, original_points):
    vertices = np.asarray(original_points)
    old_stations = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(vertices, axis=0), axis=1))]

    def old_at(station):
        index = max(
            0, min(len(vertices) - 2, np.searchsorted(old_stations, station, side="right") - 1)
        )
        tangent = _unit(vertices[index + 1] - vertices[index])
        point = vertices[index] + (station - old_stations[index]) * tangent
        return point, tangent

    def anchor(element):
        old_s = float(element.get("s"))
        base, tangent = old_at(old_s)
        point = base + float(element.get("t", "0")) * np.array([-tangent[1], tangent[0]])
        s, t, heading = curve.project(point, old_s)
        element.set("s", f"{s:.12f}")
        element.set("t", f"{t:.12f}")
        angle = "hdg" if element.tag == "object" else "hOffset"
        if element.tag in {"object", "signal"}:
            absolute = math.atan2(tangent[1], tangent[0]) + float(element.get(angle, "0"))
            element.set(angle, f"{math.remainder(absolute - heading, 2 * math.pi):.14f}")

    for path in (
        "objects/object",
        "signals/signal",
        "signals/signalReference",
        "objects/object/outlines/outline/cornerRoad",
    ):
        for element in road.findall(path):
            if "s" in element.attrib:
                anchor(element)
    for repeat in road.findall("objects/object/repeat"):
        old_start = float(repeat.get("s"))
        old_end = old_start + float(repeat.get("length"))
        repeat.set("s", f"{curve.old_to_new(old_start):.12f}")
        repeat.set("length", f"{curve.old_to_new(old_end) - curve.old_to_new(old_start):.12f}")
    for kind in road.findall("type"):
        kind.set("s", f"{curve.old_to_new(float(kind.get('s'))):.12f}")


def _endpoint_constraints(roads, vertices):
    parent = {}
    vectors = {}
    protected = set()
    for key, points in vertices.items():
        for end, vector in ((0, points[1] - points[0]), (1, points[-1] - points[-2])):
            parent[key, end] = (key, end)
            vectors[key, end] = _unit(vector)
        length = float(roads[key].get("length"))
        for path in ("signals/signal", "signals/signalReference", "objects/object"):
            for anchor in roads[key].findall(path):
                if abs(float(anchor.get("t", "0"))) <= 1e-8:
                    continue
                station = float(anchor.get("s"))
                if station <= 1e-5:
                    protected.add((key, 0))
                if station >= length - 1e-5:
                    protected.add((key, 1))

    def find(key):
        while parent[key] != key:
            parent[key] = parent[parent[key]]
            key = parent[key]
        return key

    for key, road in roads.items():
        for role, own_end, contact in (("successor", 1, "start"), ("predecessor", 0, "end")):
            link = road.find(f"link/{role}")
            if (
                link is None
                or link.get("elementType") != "road"
                or link.get("contactPoint") != contact
            ):
                continue
            other = link.get("elementId")
            if other not in vertices:
                continue
            other_end = 1 - own_end
            a, b = (key, own_end), (other, other_end)
            point_a = vertices[key][-1 if own_end else 0]
            point_b = vertices[other][-1 if other_end else 0]
            if np.linalg.norm(point_a - point_b) < 1e-5 and np.dot(
                vectors[a], vectors[b]
            ) > math.cos(math.radians(30)):
                parent[find(a)] = find(b)
    groups = {}
    for key in parent:
        groups.setdefault(find(key), []).append(key)
    constraints = {}
    for group in groups.values():
        if any(key in protected for key in group):
            continue
        directions = np.asarray([vectors[key] for key in group])
        if len(group) > 1 and np.min(directions @ directions.T) > math.cos(math.radians(30)):
            common = _unit(directions.sum(axis=0))
            constraints.update((key, common) for key in group)
    return constraints


def smooth_generated_roads(root, groups, boundaries, max_error=0.05):
    """Refit generated polylines, then rebuild profiles and native s/t anchors."""
    roads, vertices = {}, {}
    for road in root.findall("road"):
        geometries = road.findall("planView/geometry")
        if not geometries or any(g.find("line") is None for g in geometries):
            continue
        points = [[float(g.get("x")), float(g.get("y"))] for g in geometries]
        last = geometries[-1]
        heading, length = float(last.get("hdg")), float(last.get("length"))
        points.append(
            [points[-1][0] + length * math.cos(heading), points[-1][1] + length * math.sin(heading)]
        )
        roads[road.get("id")] = road
        vertices[road.get("id")] = np.asarray(points)
    constraints = _endpoint_constraints(roads, vertices)
    report = {
        "base_reference_deviation_m": max_error,
        "max_reference_deviation_m": 4 * max_error,
        "adaptive_roads": 0,
        "rounded_roads": 0,
        "roads_smoothed": 0,
        "boundary_fallback_samples": 0,
        "endpoint_constraints": len(constraints),
    }
    for key, road in roads.items():
        points = vertices[key]
        directions = np.diff(points, axis=0)
        directions /= np.linalg.norm(directions, axis=1)[:, None]
        start = constraints.get((key, 0), directions[0])
        end = constraints.get((key, 1), directions[-1])
        changes = [np.linalg.norm(start - directions[0]), np.linalg.norm(end - directions[-1])]
        changes.extend(np.linalg.norm(np.diff(directions, axis=0), axis=1))
        if max(changes) < 1e-7:
            continue
        curve = SmoothReference(points, max_error, start, end)
        group = groups.get(int(key))
        parts = {}
        if group is not None:
            old_line = LineString(points)
            lane_groups = group.longitudinal_road_lanes or [group.road_lanes]
            references = group.longitudinal_ref_points or [group.ref_points]
            for lanelets, reference in zip(lane_groups, references):
                first, last = sorted(
                    [old_line.project(Point(reference[0])), old_line.project(Point(reference[-1]))]
                )
                for index, lanelet in enumerate(reversed(lanelets), 1):
                    left, right = boundaries(lanelet)
                    if len(left) > 1 and len(right) > 1:
                        parts.setdefault(-index, []).append(
                            BoundaryPart(first, last, LineString(left), LineString(right))
                        )
        candidate = copy.deepcopy(road)
        fallback, folding = _rebuild_lanes(candidate, curve, parts)
        if folding > 0.01:
            wider_curve = SmoothReference(points, 4 * max_error, start, end)
            wider_candidate = copy.deepcopy(road)
            wider_fallback, wider_folding = _rebuild_lanes(wider_candidate, wider_curve, parts)
            if wider_folding + 0.001 < folding:
                curve, candidate, fallback = wider_curve, wider_candidate, wider_fallback
                report["adaptive_roads"] += 1
                folding = wider_folding
            rounded_curve = SmoothReference(points, 4 * max_error, start, end, round_corners=True)
            rounded_candidate = copy.deepcopy(road)
            rounded_fallback, rounded_folding = _rebuild_lanes(
                rounded_candidate, rounded_curve, parts
            )
            if rounded_folding + 0.001 < folding:
                if curve is not wider_curve:
                    report["adaptive_roads"] += 1
                curve, candidate, fallback = rounded_curve, rounded_candidate, rounded_fallback
                report["rounded_roads"] += 1
        road.replace(road.find("lanes"), candidate.find("lanes"))
        _remap_anchors(road, curve, points)
        curve.write(road.find("planView"))
        road.set("length", f"{curve.length:.12f}")
        if group is not None:
            group.length = curve.length
        report["roads_smoothed"] += 1
        report["boundary_fallback_samples"] += fallback
    return report
