from __future__ import annotations

import math
from typing import Sequence


RobotState = tuple[float, float, float, float, float]
DynamicObstacle = tuple[float, float, float]


def dynamic_robot_obstacles(
    *,
    mode: str,
    robot_state: RobotState | None,
    now: float,
    timeout_sec: float,
    radius_m: float,
    prediction_horizon_sec: float,
) -> list[DynamicObstacle]:
    """Build a robot layer only for the opt-in dynamic replanning mode."""
    normalized_mode = str(mode or "static_route").strip().lower()
    if normalized_mode == "static_route":
        return []
    if normalized_mode != "dynamic_replan":
        raise ValueError(f"Unsupported robot obstacle mode: {mode!r}")
    if robot_state is None or radius_m <= 0.0:
        return []
    x, y, velocity_x, velocity_y, observed_at = robot_state
    if float(now) - float(observed_at) > float(timeout_sec):
        return []
    obstacles = [(float(x), float(y), float(radius_m))]
    if prediction_horizon_sec > 0.0 and math.hypot(velocity_x, velocity_y) >= 0.05:
        obstacles.append(
            (
                float(x) + float(velocity_x) * float(prediction_horizon_sec),
                float(y) + float(velocity_y) * float(prediction_horizon_sec),
                float(radius_m),
            )
        )
    return obstacles


def robot_blocks_pedestrian(
    *,
    pedestrian_pose: Sequence[float],
    robot_state: RobotState | None,
    now: float,
    timeout_sec: float,
    robot_radius_m: float,
    pedestrian_radius_m: float,
    clearance_m: float,
    prediction_horizon_sec: float,
) -> bool:
    """Return whether a fresh current or predicted robot pose requires yielding."""
    if robot_state is None or robot_radius_m <= 0.0:
        return False
    x, y, velocity_x, velocity_y, observed_at = robot_state
    if float(now) - float(observed_at) > float(timeout_sec):
        return False
    threshold = max(
        0.0,
        float(robot_radius_m) + float(pedestrian_radius_m) + float(clearance_m),
    )
    pedestrian_x = float(pedestrian_pose[0])
    pedestrian_y = float(pedestrian_pose[1])
    candidates = [(float(x), float(y))]
    if prediction_horizon_sec > 0.0 and math.hypot(velocity_x, velocity_y) >= 0.05:
        candidates.append(
            (
                float(x) + float(velocity_x) * float(prediction_horizon_sec),
                float(y) + float(velocity_y) * float(prediction_horizon_sec),
            )
        )
    return any(
        math.hypot(candidate_x - pedestrian_x, candidate_y - pedestrian_y) <= threshold
        for candidate_x, candidate_y in candidates
    )
