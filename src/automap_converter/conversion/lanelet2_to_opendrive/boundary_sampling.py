"""Reject normal intersections on a remote branch of a source boundary."""

from shapely.geometry import Point


def local_boundary_hit(boundary, normal, origin):
    intersection = boundary.intersection(normal)
    points = (
        [intersection] if isinstance(intersection, Point) and not intersection.is_empty
        else [part for part in getattr(intersection, "geoms", []) if isinstance(part, Point)]
    )
    if not points:
        return None
    hit = min(points, key=origin.distance)
    # A missed oblique port must not become a hit on the far side of a loop.
    # The bound scales with real boundary distance, so wide lanes remain valid.
    if origin.distance(hit) > max(2.0, 4.0 * origin.distance(boundary) + 0.5):
        return None
    return hit
