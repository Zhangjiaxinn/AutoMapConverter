"""Shared boundary orientation in a metric travel frame."""

from statistics import median

from shapely.geometry import LineString


def orient_boundaries(left, right):
    if left.interpolate(0).distance(right.interpolate(1, normalized=True)) + left.interpolate(
        1, normalized=True
    ).distance(right.interpolate(0)) < left.interpolate(0).distance(
        right.interpolate(0)
    ) + left.interpolate(1, normalized=True).distance(right.interpolate(1, normalized=True)):
        right = LineString(list(right.coords)[::-1])
    signs = []
    for f in (0.1, 0.3, 0.5, 0.7, 0.9):
        a, b = (
            left.interpolate(f - 1e-4, normalized=True),
            left.interpolate(f + 1e-4, normalized=True),
        )
        p, q = left.interpolate(f, normalized=True), right.interpolate(f, normalized=True)
        length = a.distance(b)
        if length > 1e-12:
            signs.append(((b.x - a.x) * (q.y - p.y) - (b.y - a.y) * (q.x - p.x)) / length)
    # A crossing or locally pinched boundary can make one midpoint sign unstable.
    # Use several stations, not source-to-target proximity, to determine travel.
    if signs and median(signs) > 1e-6:
        left, right = LineString(list(left.coords)[::-1]), LineString(list(right.coords)[::-1])
    return left, right
