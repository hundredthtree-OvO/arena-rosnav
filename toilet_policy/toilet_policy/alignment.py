"""Pure temporal and geometric alignment helpers."""

from __future__ import annotations

import bisect
import math
from typing import Sequence


def nearest_index(timestamps: Sequence[float], target: float) -> int:
    if not timestamps:
        raise ValueError("cannot select from an empty timestamp sequence")
    index = bisect.bisect_left(timestamps, float(target))
    if index <= 0:
        return 0
    if index >= len(timestamps):
        return len(timestamps) - 1
    before = index - 1
    return before if target - timestamps[before] <= timestamps[index] - target else index


def previous_index(timestamps: Sequence[float], target: float) -> int:
    if not timestamps:
        raise ValueError("cannot select from an empty timestamp sequence")
    return max(0, min(len(timestamps) - 1, bisect.bisect_right(timestamps, target) - 1))


def relative_goal(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    goal_x: float,
    goal_y: float,
    goal_yaw: float,
) -> tuple[float, float, float, float]:
    dx = float(goal_x) - float(robot_x)
    dy = float(goal_y) - float(robot_y)
    cosine = math.cos(float(robot_yaw))
    sine = math.sin(float(robot_yaw))
    body_x = cosine * dx + sine * dy
    body_y = -sine * dx + cosine * dy
    heading = float(goal_yaw) - float(robot_yaw)
    return body_x, body_y, math.sin(heading), math.cos(heading)
