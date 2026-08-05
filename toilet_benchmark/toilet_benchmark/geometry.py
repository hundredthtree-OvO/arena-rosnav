"""Backend-neutral geometry helpers shared by diagnostics and replay."""

from __future__ import annotations

import math
from typing import Any, Sequence


def person_yaw(person: Any) -> float | None:
    names = list(getattr(person, "tagnames", []) or [])
    values = list(getattr(person, "tags", []) or [])
    tags = {str(name): str(value) for name, value in zip(names, values)}
    if tags.get("yaw_valid", "false").lower() != "true":
        return None
    try:
        yaw = float(tags["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None
    return yaw if math.isfinite(yaw) else None


def point_to_polyline_distance(
    x: float,
    y: float,
    points: Sequence[Sequence[float]],
) -> float | None:
    if not points:
        return None
    if len(points) == 1:
        return math.hypot(
            float(x) - float(points[0][0]),
            float(y) - float(points[0][1]),
        )

    best = math.inf
    px = float(x)
    py = float(y)
    for start, end in zip(points, points[1:]):
        ax, ay = float(start[0]), float(start[1])
        bx, by = float(end[0]), float(end[1])
        dx, dy = bx - ax, by - ay
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            distance = math.hypot(px - ax, py - ay)
        else:
            projection = max(
                0.0,
                min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq),
            )
            distance = math.hypot(
                px - (ax + projection * dx),
                py - (ay + projection * dy),
            )
        best = min(best, distance)
    return best
