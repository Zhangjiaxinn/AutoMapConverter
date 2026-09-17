"""Topology-aware semantic rasterization of Lanelet2 maps."""

from __future__ import annotations

import hashlib
import json
import math
import warnings
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rasterio
from lxml import etree
from pyproj import CRS, Transformer
from rasterio.errors import NotGeoreferencedWarning
from rasterio.features import rasterize
from rasterio.transform import from_origin
from shapely.geometry import LineString, Point, Polygon

SEMANTIC_BANDS = (
    "DRIVABLE_AREA",
    "LANE_BOUNDARY",
    "CENTERLINE",
    "DIRECTION_BITS",
    "TOPOLOGY_ANCHOR",
    "LANELET_INDEX",
)


@dataclass(frozen=True)
class RasterArtifacts:
    """Files generated for one Lanelet2 raster conversion."""

    semantic: Path
    occupancy: Path
    semantic_preview: Path
    occupancy_preview: Path
    metadata: Path

    def as_dict(self) -> dict[str, Path]:
        return {
            "semantic": self.semantic,
            "occupancy": self.occupancy,
            "semantic_preview": self.semantic_preview,
            "occupancy_preview": self.occupancy_preview,
            "metadata": self.metadata,
        }


@dataclass(frozen=True)
class _LaneletGeometry:
    source_id: str
    index: int
    left_way: str
    right_way: str
    left: LineString
    right: LineString
    centerline: LineString
    polygon: Polygon
    regulatory_elements: tuple[str, ...]
    tags: dict[str, str]
    direction_bit: int


def raster_artifact_paths(target_path: str | Path) -> RasterArtifacts:
    """Return the deterministic companion paths for a semantic GeoTIFF."""

    semantic = Path(target_path).expanduser().resolve()
    stem = semantic.stem
    return RasterArtifacts(
        semantic=semantic,
        occupancy=semantic.with_name(f"{stem}_occupancy.tif"),
        semantic_preview=semantic.with_name(f"{stem}_preview.png"),
        occupancy_preview=semantic.with_name(f"{stem}_occupancy_preview.png"),
        metadata=semantic.with_name(f"{stem}_raster_metadata.json"),
    )


def _tags(element: etree._Element) -> dict[str, str]:
    return {
        str(tag.get("k")): str(tag.get("v"))
        for tag in element.findall("tag")
        if tag.get("k") and tag.get("v") is not None
    }


def _optional_float(value: str | None) -> float | None:
    try:
        result = float(value) if value is not None else None
    except ValueError:
        return None
    return result if result is not None and math.isfinite(result) else None


def _direction_bit(line: LineString) -> int:
    start = line.coords[0]
    end = line.coords[-1]
    angle = (math.degrees(math.atan2(end[1] - start[1], end[0] - start[0])) + 360.0) % 360.0
    sector = int((angle + 22.5) // 45.0) % 8
    return 1 << sector


def _sample_line(line: LineString, count: int) -> list[tuple[float, float]]:
    if count <= 2:
        return [tuple(line.coords[0]), tuple(line.coords[-1])]
    return [
        tuple(line.interpolate(index / (count - 1), normalized=True).coords[0])
        for index in range(count)
    ]


def _aligned_boundaries(
    left_coords: Sequence[tuple[float, float]],
    right_coords: Sequence[tuple[float, float]],
) -> tuple[LineString, LineString]:
    left = LineString(left_coords)
    right = LineString(right_coords)
    same = Point(left.coords[0]).distance(Point(right.coords[0])) + Point(left.coords[-1]).distance(
        Point(right.coords[-1])
    )
    opposite = Point(left.coords[0]).distance(Point(right.coords[-1])) + Point(
        left.coords[-1]
    ).distance(Point(right.coords[0]))
    if opposite < same:
        right = LineString(list(right.coords)[::-1])
    return left, right


def _derived_centerline(left: LineString, right: LineString) -> tuple[LineString, float]:
    count = max(2, math.ceil(max(left.length, right.length)) + 1)
    left_samples = _sample_line(left, count)
    right_samples = _sample_line(right, count)
    center = [
        ((left_point[0] + right_point[0]) / 2.0, (left_point[1] + right_point[1]) / 2.0)
        for left_point, right_point in zip(left_samples, right_samples)
    ]
    widths = [Point(a).distance(Point(b)) for a, b in zip(left_samples, right_samples)]
    return LineString(center), float(np.median(widths))


def _valid_polygon(
    left: LineString, right: LineString, centerline: LineString, width: float
) -> Polygon:
    polygon = Polygon([*left.coords, *list(right.coords)[::-1]])
    if not polygon.is_valid:
        repaired = polygon.buffer(0)
        if repaired.geom_type == "Polygon":
            polygon = repaired
        elif repaired.geom_type == "MultiPolygon" and repaired.geoms:
            polygon = max(repaired.geoms, key=lambda item: item.area)
    if polygon.is_empty or polygon.area <= 1e-6:
        polygon = centerline.buffer(max(width / 2.0, 0.5), cap_style="flat", join_style="mitre")
    if polygon.geom_type != "Polygon" or polygon.is_empty:
        raise ValueError("Lanelet boundaries do not form a usable polygon.")
    return polygon


def _projection(
    root: etree._Element,
    referenced_nodes: set[str],
) -> tuple[dict[str, tuple[float, float]], str, CRS | None]:
    raw_nodes: dict[str, tuple[float, float, float | None, float | None]] = {}
    for node in root.findall("node"):
        node_id = node.get("id")
        if not node_id:
            continue
        tags = _tags(node)
        lon = _optional_float(node.get("lon"))
        lat = _optional_float(node.get("lat"))
        local_x = _optional_float(tags.get("local_x"))
        local_y = _optional_float(tags.get("local_y"))
        if lon is None and lat is None and local_x is None and local_y is None:
            continue
        raw_nodes[node_id] = (
            lon if lon is not None else math.nan,
            lat if lat is not None else math.nan,
            local_x,
            local_y,
        )

    required = [raw_nodes[node_id] for node_id in referenced_nodes if node_id in raw_nodes]
    if not required:
        raise ValueError("Lanelet ways do not reference any readable nodes.")
    if len(required) != len(referenced_nodes):
        missing = sorted(referenced_nodes.difference(raw_nodes))
        raise ValueError(f"Lanelet ways reference {len(missing)} missing or invalid nodes.")

    if all(item[2] is not None and item[3] is not None for item in required):
        return (
            {
                node_id: (float(values[2]), float(values[3]))
                for node_id, values in raw_nodes.items()
                if values[2] is not None and values[3] is not None
            },
            "local_xy",
            None,
        )

    if not all(math.isfinite(item[0]) and math.isfinite(item[1]) for item in required):
        raise ValueError(
            "Lanelet nodes need either local_x/local_y tags or valid longitude/latitude values."
        )
    center_lon = sum(item[0] for item in required) / len(required)
    center_lat = sum(item[1] for item in required) / len(required)
    zone = max(1, min(60, int((center_lon + 180.0) // 6.0) + 1))
    epsg = (32600 if center_lat >= 0.0 else 32700) + zone
    crs = CRS.from_epsg(epsg)
    transformer = Transformer.from_crs("EPSG:4326", crs, always_xy=True)
    coordinates = {
        node_id: transformer.transform(values[0], values[1])
        for node_id, values in raw_nodes.items()
        if math.isfinite(values[0]) and math.isfinite(values[1])
    }
    return coordinates, f"utm_epsg_{epsg}", crs


def _parse_lanelets(
    source_path: Path,
) -> tuple[
    list[_LaneletGeometry], dict[str, list[tuple[float, float]]], dict[str, object], CRS | None
]:
    root = etree.parse(str(source_path)).getroot()
    if root.tag != "osm":
        raise ValueError(f"Expected an OSM/Lanelet2 root element, found {root.tag!r}.")

    ways: dict[str, tuple[list[str], dict[str, str]]] = {}
    for way in root.findall("way"):
        way_id = way.get("id")
        if not way_id:
            continue
        ways[way_id] = (
            [str(node.get("ref")) for node in way.findall("nd") if node.get("ref")],
            _tags(way),
        )

    relation_records: list[dict[str, object]] = []
    referenced_nodes: set[str] = set()
    for relation in root.findall("relation"):
        if relation.get("action") == "delete" or relation.get("visible") == "false":
            continue
        tags = _tags(relation)
        if tags.get("type") != "lanelet":
            continue
        members: dict[str, list[str]] = {}
        for member in relation.findall("member"):
            role = member.get("role", "")
            reference = member.get("ref")
            if reference:
                members.setdefault(role, []).append(reference)
        left_way = next(iter(members.get("left", [])), "")
        right_way = next(iter(members.get("right", [])), "")
        if left_way in ways:
            referenced_nodes.update(ways[left_way][0])
        if right_way in ways:
            referenced_nodes.update(ways[right_way][0])
        center_way = next(iter(members.get("centerline", [])), "")
        if center_way in ways:
            referenced_nodes.update(ways[center_way][0])
        relation_records.append(
            {
                "id": str(relation.get("id", "")),
                "left": left_way,
                "right": right_way,
                "centerline": center_way,
                "regulatory": tuple(members.get("regulatory_element", [])),
                "tags": tags,
            }
        )
    if not relation_records:
        raise ValueError("The source contains no type=lanelet relations.")

    coordinates, coordinate_mode, crs = _projection(root, referenced_nodes)
    projected_ways = {
        way_id: [coordinates[node_id] for node_id in node_ids if node_id in coordinates]
        for way_id, (node_ids, _) in ways.items()
    }
    lanelets: list[_LaneletGeometry] = []
    skipped: list[dict[str, str]] = []
    for record in relation_records:
        source_id = str(record["id"])
        left_way = str(record["left"])
        right_way = str(record["right"])
        try:
            left_coords = projected_ways[left_way]
            right_coords = projected_ways[right_way]
            if len(left_coords) < 2 or len(right_coords) < 2:
                raise ValueError("left or right boundary has fewer than two coordinates")
            left, right = _aligned_boundaries(left_coords, right_coords)
            derived_centerline, width = _derived_centerline(left, right)
            explicit = projected_ways.get(str(record["centerline"]), [])
            centerline = LineString(explicit) if len(explicit) >= 2 else derived_centerline
            polygon = _valid_polygon(left, right, centerline, width)
        except (KeyError, ValueError) as exc:
            skipped.append({"lanelet_id": source_id, "reason": str(exc)})
            continue
        lanelets.append(
            _LaneletGeometry(
                source_id=source_id,
                index=len(lanelets) + 1,
                left_way=left_way,
                right_way=right_way,
                left=left,
                right=right,
                centerline=centerline,
                polygon=polygon,
                regulatory_elements=tuple(record["regulatory"]),
                tags=dict(record["tags"]),
                direction_bit=_direction_bit(centerline),
            )
        )
    if not lanelets:
        raise ValueError("No Lanelet2 relation could be reconstructed as a drivable polygon.")
    if len(lanelets) > np.iinfo(np.uint16).max:
        raise ValueError("The raster schema supports at most 65,535 lanelets per output tile.")

    metadata = {
        "coordinate_mode": coordinate_mode,
        "source_lanelets": len(relation_records),
        "rasterized_lanelets": len(lanelets),
        "skipped_lanelets": skipped,
    }
    return lanelets, projected_ways, metadata, crs


def _topology(
    lanelets: Sequence[_LaneletGeometry], tolerance: float
) -> dict[str, dict[str, list[str]]]:
    starts: dict[tuple[int, int], list[_LaneletGeometry]] = {}
    lanelets_by_left_way: dict[str, list[_LaneletGeometry]] = {}
    lanelets_by_right_way: dict[str, list[_LaneletGeometry]] = {}
    for lanelet in lanelets:
        start = lanelet.centerline.coords[0]
        key = (math.floor(start[0] / tolerance), math.floor(start[1] / tolerance))
        starts.setdefault(key, []).append(lanelet)
        lanelets_by_left_way.setdefault(lanelet.left_way, []).append(lanelet)
        lanelets_by_right_way.setdefault(lanelet.right_way, []).append(lanelet)

    result: dict[str, dict[str, list[str]]] = {}
    for lanelet in lanelets:
        end = lanelet.centerline.coords[-1]
        end_key = (math.floor(end[0] / tolerance), math.floor(end[1] / tolerance))
        successors: set[str] = set()
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for candidate in starts.get((end_key[0] + dx, end_key[1] + dy), []):
                    if (
                        candidate.source_id != lanelet.source_id
                        and Point(end).distance(Point(candidate.centerline.coords[0])) <= tolerance
                    ):
                        successors.add(candidate.source_id)
        left_adjacent = {
            candidate.source_id
            for candidate in lanelets_by_right_way.get(lanelet.left_way, [])
            if candidate.source_id != lanelet.source_id
        }
        right_adjacent = {
            candidate.source_id
            for candidate in lanelets_by_left_way.get(lanelet.right_way, [])
            if candidate.source_id != lanelet.source_id
        }
        result[lanelet.source_id] = {
            "successors": sorted(successors),
            "left_adjacent": sorted(left_adjacent),
            "right_adjacent": sorted(right_adjacent),
        }
    for source_id, links in result.items():
        for successor in links["successors"]:
            result[successor].setdefault("predecessors", []).append(source_id)
    for links in result.values():
        links.setdefault("predecessors", [])
        links["predecessors"] = sorted(set(links["predecessors"]))
    return result


def _reduce_max(array: np.ndarray, height: int, width: int, factor: int) -> np.ndarray:
    if factor == 1:
        return array
    return array.reshape(height, factor, width, factor).max(axis=(1, 3))


def _reduce_or(array: np.ndarray, height: int, width: int, factor: int) -> np.ndarray:
    if factor == 1:
        return array
    reshaped = array.reshape(height, factor, width, factor)
    return np.bitwise_or.reduce(np.bitwise_or.reduce(reshaped, axis=3), axis=1)


def _rasterize(
    lanelets: Sequence[_LaneletGeometry],
    resolution: float,
    padding: float,
    supersampling: int,
    max_output_pixels: int,
) -> tuple[list[np.ndarray], np.ndarray, object, tuple[float, float, float, float], int]:
    min_x = min(lanelet.polygon.bounds[0] for lanelet in lanelets) - padding
    min_y = min(lanelet.polygon.bounds[1] for lanelet in lanelets) - padding
    max_x = max(lanelet.polygon.bounds[2] for lanelet in lanelets) + padding
    max_y = max(lanelet.polygon.bounds[3] for lanelet in lanelets) + padding
    width = max(1, math.ceil((max_x - min_x) / resolution))
    height = max(1, math.ceil((max_y - min_y) / resolution))
    if width * height > max_output_pixels:
        raise ValueError(
            f"Raster would contain {width * height:,} pixels, above the configured "
            f"limit of {max_output_pixels:,}; increase raster_resolution_m or tile the source."
        )
    max_supersampled_pixels = max_output_pixels * 4
    memory_limited_factor = max(
        1, math.floor(math.sqrt(max_supersampled_pixels / (width * height)))
    )
    effective_supersampling = min(supersampling, memory_limited_factor)
    high_width = width * effective_supersampling
    high_height = height * effective_supersampling
    high_transform = from_origin(
        min_x,
        max_y,
        resolution / effective_supersampling,
        resolution / effective_supersampling,
    )
    shape = (high_height, high_width)
    polygons = [(lanelet.polygon, 1) for lanelet in lanelets]
    boundaries = [
        (boundary, 1) for lanelet in lanelets for boundary in (lanelet.left, lanelet.right)
    ]
    centerlines = [(lanelet.centerline, 1) for lanelet in lanelets]
    lanelet_indices = [(lanelet.polygon, lanelet.index) for lanelet in lanelets]
    anchor_radius = max(resolution / 2.0, 0.25)
    anchors = [
        (Point(point).buffer(anchor_radius), 1)
        for lanelet in lanelets
        for point in (lanelet.centerline.coords[0], lanelet.centerline.coords[-1])
    ]

    drivable = _reduce_max(
        rasterize(
            polygons, out_shape=shape, transform=high_transform, fill=0, dtype="uint8"
        ),
        height,
        width,
        effective_supersampling,
    )
    boundary = _reduce_max(
        rasterize(
            boundaries,
            out_shape=shape,
            transform=high_transform,
            fill=0,
            all_touched=True,
            dtype="uint8",
        ),
        height,
        width,
        effective_supersampling,
    )
    centerline = _reduce_max(
        rasterize(
            centerlines,
            out_shape=shape,
            transform=high_transform,
            fill=0,
            all_touched=True,
            dtype="uint8",
        ),
        height,
        width,
        effective_supersampling,
    )
    direction_high = np.zeros(shape, dtype=np.uint16)
    for bit in (1, 2, 4, 8, 16, 32, 64, 128):
        sector_shapes = [
            (lanelet.polygon, bit) for lanelet in lanelets if lanelet.direction_bit == bit
        ]
        if sector_shapes:
            direction_high |= rasterize(
                sector_shapes,
                out_shape=shape,
                transform=high_transform,
                fill=0,
                dtype="uint16",
            )
    direction = _reduce_or(
        direction_high, height, width, effective_supersampling
    )
    del direction_high
    anchor = _reduce_max(
        rasterize(
            anchors,
            out_shape=shape,
            transform=high_transform,
            fill=0,
            all_touched=True,
            dtype="uint8",
        ),
        height,
        width,
        effective_supersampling,
    )
    lanelet_index = _reduce_max(
        rasterize(
            lanelet_indices,
            out_shape=shape,
            transform=high_transform,
            fill=0,
            dtype="uint16",
        ),
        height,
        width,
        effective_supersampling,
    )

    bands = [drivable, boundary, centerline, direction, anchor, lanelet_index]
    occupancy = np.where(bands[0] > 0, 0, 1).astype(np.uint8)
    transform = from_origin(min_x, max_y, resolution, resolution)
    return (
        bands,
        occupancy,
        transform,
        (min_x, min_y, max_x, max_y),
        effective_supersampling,
    )


def _write_semantic(
    path: Path,
    bands: Sequence[np.ndarray],
    transform: object,
    crs: CRS | None,
) -> None:
    height, width = bands[0].shape
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=height,
        width=width,
        count=len(bands),
        dtype="uint16",
        crs=crs,
        transform=transform,
        compress="deflate",
        predictor=2,
    ) as target:
        for index, (band, name) in enumerate(zip(bands, SEMANTIC_BANDS), start=1):
            target.write(band.astype(np.uint16, copy=False), index)
            target.set_band_description(index, name)
            target.update_tags(index, semantic_role=name)
        target.update_tags(
            schema="automap-converter/lanelet2-raster/v1",
            direction_encoding="8-sector bitset: E,NE,N,NW,W,SW,S,SE",
        )


def _write_occupancy(path: Path, occupancy: np.ndarray, transform: object, crs: CRS | None) -> None:
    height, width = occupancy.shape
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=height,
        width=width,
        count=1,
        dtype="uint8",
        crs=crs,
        transform=transform,
        compress="deflate",
    ) as target:
        target.write(occupancy, 1)
        target.set_band_description(1, "OCCUPANCY")
        target.update_tags(1, occupied="1", free="0")


def _write_png(path: Path, red: np.ndarray, green: np.ndarray, blue: np.ndarray) -> None:
    height, width = red.shape
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", NotGeoreferencedWarning)
        with rasterio.open(
            path,
            "w",
            driver="PNG",
            height=height,
            width=width,
            count=3,
            dtype="uint8",
        ) as target:
            target.write(red, 1)
            target.write(green, 2)
            target.write(blue, 3)


def _write_previews(
    artifacts: RasterArtifacts, bands: Sequence[np.ndarray], occupancy: np.ndarray
) -> None:
    height, width = bands[0].shape
    preview = np.full((3, height, width), 24, dtype=np.uint8)
    drivable = bands[0] > 0
    preview[:, drivable] = np.array([[226], [229], [220]], dtype=np.uint8)
    preview[:, bands[1] > 0] = np.array([[245], [166], [35]], dtype=np.uint8)
    preview[:, bands[2] > 0] = np.array([[38], [169], [184]], dtype=np.uint8)
    preview[:, bands[4] > 0] = np.array([[69], [184], [107]], dtype=np.uint8)
    _write_png(artifacts.semantic_preview, preview[0], preview[1], preview[2])
    occupied = np.where(occupancy > 0, 255, 0).astype(np.uint8)
    _write_png(artifacts.occupancy_preview, occupied, occupied, occupied)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert_lanelet2_to_raster(
    source_path: str | Path,
    target_path: str | Path,
    *,
    resolution_m: float = 0.5,
    padding_m: float = 5.0,
    supersampling: int = 4,
    topology_tolerance_m: float = 1.0,
    max_output_pixels: int = 40_000_000,
) -> RasterArtifacts:
    """Convert one Lanelet2 map into semantic and occupancy raster products."""

    source = Path(source_path).expanduser().resolve()
    artifacts = raster_artifact_paths(target_path)
    if not source.is_file():
        raise FileNotFoundError(f"Lanelet2 source does not exist: {source}")
    if artifacts.semantic.suffix.lower() not in {".tif", ".tiff"}:
        raise ValueError("Lanelet2 raster output must use a .tif or .tiff suffix.")
    if resolution_m <= 0.0 or padding_m < 0.0 or topology_tolerance_m <= 0.0:
        raise ValueError("Raster resolution/tolerance must be positive and padding non-negative.")
    if supersampling < 1 or supersampling > 8:
        raise ValueError("Raster supersampling must be between 1 and 8.")

    artifacts.semantic.parent.mkdir(parents=True, exist_ok=True)
    lanelets, _projected_ways, source_metadata, crs = _parse_lanelets(source)
    topology = _topology(lanelets, topology_tolerance_m)
    bands, occupancy, transform, bounds, effective_supersampling = _rasterize(
        lanelets,
        resolution_m,
        padding_m,
        supersampling,
        max_output_pixels,
    )
    _write_semantic(artifacts.semantic, bands, transform, crs)
    _write_occupancy(artifacts.occupancy, occupancy, transform, crs)
    _write_previews(artifacts, bands, occupancy)

    lanelet_records = []
    for lanelet in lanelets:
        lanelet_records.append(
            {
                "source_id": lanelet.source_id,
                "raster_index": lanelet.index,
                "left_way": lanelet.left_way,
                "right_way": lanelet.right_way,
                "length_m": round(lanelet.centerline.length, 3),
                "area_m2": round(lanelet.polygon.area, 3),
                "direction_bit": lanelet.direction_bit,
                "regulatory_elements": list(lanelet.regulatory_elements),
                "topology": topology[lanelet.source_id],
                "tags": lanelet.tags,
            }
        )
    payload = {
        "schema": "automap-converter/lanelet2-raster-metadata/v1",
        "source": str(source),
        "source_sha256": _sha256(source),
        "semantic_raster": str(artifacts.semantic),
        "occupancy_raster": str(artifacts.occupancy),
        "semantic_preview": str(artifacts.semantic_preview),
        "occupancy_preview": str(artifacts.occupancy_preview),
        "bands": {str(index): name for index, name in enumerate(SEMANTIC_BANDS, start=1)},
        "occupancy_encoding": {"free": 0, "occupied": 1},
        "direction_encoding": {
            "1": "east",
            "2": "north_east",
            "4": "north",
            "8": "north_west",
            "16": "west",
            "32": "south_west",
            "64": "south",
            "128": "south_east",
        },
        "coordinate_mode": source_metadata["coordinate_mode"],
        "crs": crs.to_string() if crs is not None else "local engineering coordinates",
        "bounds": {
            "min_x": bounds[0],
            "min_y": bounds[1],
            "max_x": bounds[2],
            "max_y": bounds[3],
        },
        "resolution_m": resolution_m,
        "padding_m": padding_m,
        "requested_supersampling": supersampling,
        "supersampling": effective_supersampling,
        "width": int(bands[0].shape[1]),
        "height": int(bands[0].shape[0]),
        "source_lanelets": source_metadata["source_lanelets"],
        "rasterized_lanelets": source_metadata["rasterized_lanelets"],
        "skipped_lanelets": source_metadata["skipped_lanelets"],
        "drivable_pixels": int(np.count_nonzero(bands[0])),
        "boundary_pixels": int(np.count_nonzero(bands[1])),
        "centerline_pixels": int(np.count_nonzero(bands[2])),
        "anchor_pixels": int(np.count_nonzero(bands[4])),
        "topology_edges": sum(len(item["successors"]) for item in topology.values()),
        "lanelets": lanelet_records,
    }
    artifacts.metadata.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return artifacts
