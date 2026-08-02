from __future__ import annotations

import math
from typing import Sequence


RobotState = tuple[float, ...]
DynamicObstacle = tuple[float, ...]


def guard_block_requires_wait(reason: str | None) -> bool:
    """External actors may wait indefinitely without consuming recovery attempts."""
    return str(reason or "").strip().lower() in {"robot", "pedestrian"}


def dynamic_robot_obstacles(
    *,
    mode: str,
    robot_state: RobotState | None,
    now: float,
    timeout_sec: float,
    radius_m: float,
    prediction_horizon_sec: float,
    footprint_half_length_m: float | None = None,
    footprint_half_width_m: float | None = None,
    stationary_speed_threshold_mps: float = 0.05,
    stationary_soft_clearance_m: float = 0.05,
    moving_soft_clearance_m: float = 0.12,
) -> list[DynamicObstacle]:
    """Build a speed-aware robot footprint for opt-in dynamic replanning."""
    normalized_mode = str(mode or "static_route").strip().lower()
    if normalized_mode == "static_route":
        return []
    if normalized_mode != "dynamic_replan":
        raise ValueError(f"Unsupported robot obstacle mode: {mode!r}")
    if robot_state is None or radius_m <= 0.0:
        return []
    x, y, yaw, velocity_x, velocity_y, angular_velocity, observed_at = (
        _unpack_robot_state(robot_state)
    )
    if float(now) - float(observed_at) > float(timeout_sec):
        return []
    speed = math.hypot(velocity_x, velocity_y)
    moving = speed >= max(0.0, float(stationary_speed_threshold_mps))
    if footprint_half_length_m is None and footprint_half_width_m is None:
        obstacles: list[DynamicObstacle] = [
            (float(x), float(y), float(radius_m))
        ]
        if prediction_horizon_sec > 0.0 and moving:
            horizon = float(prediction_horizon_sec)
            obstacles.append(
                (
                    float(x) + float(velocity_x) * horizon,
                    float(y) + float(velocity_y) * horizon,
                    float(radius_m),
                )
            )
        return obstacles
    half_length = (
        max(0.01, float(footprint_half_length_m))
        if footprint_half_length_m is not None
        else float(radius_m)
    )
    half_width = (
        max(0.01, float(footprint_half_width_m))
        if footprint_half_width_m is not None
        else float(radius_m)
    )
    soft_clearance = (
        float(moving_soft_clearance_m)
        if moving
        else float(stationary_soft_clearance_m)
    )
    obstacles = oriented_footprint_circles(
        x=float(x),
        y=float(y),
        yaw=float(yaw),
        half_length_m=half_length,
        half_width_m=half_width,
        soft_clearance_m=soft_clearance,
    )
    if prediction_horizon_sec > 0.0 and moving:
        horizon = float(prediction_horizon_sec)
        obstacles.extend(
            oriented_footprint_circles(
                x=float(x) + float(velocity_x) * horizon,
                y=float(y) + float(velocity_y) * horizon,
                yaw=float(yaw) + float(angular_velocity) * horizon,
                half_length_m=half_length,
                half_width_m=half_width,
                soft_clearance_m=float(moving_soft_clearance_m),
            )
        )
    return obstacles


def oriented_footprint_circles(
    *,
    x: float,
    y: float,
    yaw: float,
    half_length_m: float,
    half_width_m: float,
    soft_clearance_m: float,
) -> list[DynamicObstacle]:
    """Approximate an oriented rectangular footprint with a tight capsule."""
    radius = max(0.01, float(half_width_m))
    center_half_span = max(0.0, float(half_length_m) - radius)
    offsets = (
        (0.0,)
        if center_half_span <= 1e-9
        else (-center_half_span, 0.0, center_half_span)
    )
    forward_x = math.cos(float(yaw))
    forward_y = math.sin(float(yaw))
    return [
        (
            float(x) + offset * forward_x,
            float(y) + offset * forward_y,
            radius,
            max(0.0, float(soft_clearance_m)),
        )
        for offset in offsets
    ]


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
    x, y, _yaw, velocity_x, velocity_y, _angular_velocity, observed_at = (
        _unpack_robot_state(robot_state)
    )
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


def _unpack_robot_state(
    robot_state: RobotState,
) -> tuple[float, float, float, float, float, float, float]:
    if len(robot_state) >= 7:
        x, y, yaw, velocity_x, velocity_y, angular_velocity, observed_at = (
            robot_state[:7]
        )
        return (
            float(x),
            float(y),
            float(yaw),
            float(velocity_x),
            float(velocity_y),
            float(angular_velocity),
            float(observed_at),
        )
    if len(robot_state) == 5:
        x, y, velocity_x, velocity_y, observed_at = robot_state
        return (
            float(x),
            float(y),
            0.0,
            float(velocity_x),
            float(velocity_y),
            0.0,
            float(observed_at),
        )
    raise ValueError(f"Unsupported robot state shape: {robot_state!r}")
