"""
Predefined-area patrol: lawnmower coverage waypoints.

Pure maths, no ROS import, so it can be tested on any machine.
Coordinates are (north_m, east_m, altitude_m) with altitude positive up.
"""

import math


def lawnmower_waypoints(north_min, north_max, east_min, east_max, spacing_m, altitude_m):
    """Return back-and-forth waypoints that cover a rectangle.

    Rows run along North. Each row is one lane, and lanes are spaced
    `spacing_m` apart along East. The last lane always lands on east_max so
    the whole rectangle is covered.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be positive")
    if north_max < north_min or east_max < east_min:
        raise ValueError("max bounds must be >= min bounds")

    lanes = []
    east = east_min
    while east < east_max:
        lanes.append(east)
        east += spacing_m
    lanes.append(east_max)

    waypoints = []
    for i, east in enumerate(lanes):
        # Alternate direction so the drone never flies back across the area.
        start, end = (north_min, north_max) if i % 2 == 0 else (north_max, north_min)
        waypoints.append((start, east, altitude_m))
        waypoints.append((end, east, altitude_m))
    return waypoints


def route_length(points):
    """Return the total 2-D length of a route through (north, east, ...) points."""
    return sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:])
    )


def _demo():
    """Quick self-check: 10 m x 20 m area, 10 m lanes -> two lanes, four waypoints."""
    pts = lawnmower_waypoints(0.0, 20.0, 0.0, 10.0, 10.0, 6.0)
    assert pts == [(0.0, 0.0, 6.0), (20.0, 0.0, 6.0), (20.0, 10.0, 6.0), (0.0, 10.0, 6.0)]
    assert route_length(pts) == 20.0 + 10.0 + 20.0


if __name__ == "__main__":
    _demo()
    print("area_patrol self-check passed")
