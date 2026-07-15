from __future__ import annotations

import math
from dataclasses import dataclass

from .pose_utils import SemanticPose


@dataclass(frozen=True)
class PlacementRule:
    offset_local: tuple[float, float, float]
    yaw_mode: str = "prim"
    yaw_local: float = 0.0


@dataclass(frozen=True)
class QueueRule:
    slots: int
    first_gap: float
    spacing: float
    axis_local: tuple[float, float, float]
    yaw_mode: str = "same_as_resource"
    yaw_local: float = 0.0


def normalize_xy_axis(axis_local) -> tuple[float, float, float]:
    vals = [float(v) for v in axis_local]
    if len(vals) < 2:
        raise ValueError(f"Axis must contain at least x and y: {axis_local!r}")
    x = float(vals[0])
    y = float(vals[1])
    z = float(vals[2] if len(vals) > 2 else 0.0)
    norm = math.hypot(x, y)
    if norm <= 1e-6:
        raise ValueError(f"Axis must not be zero-length in XY plane: {axis_local!r}")
    return (x / norm, y / norm, z)


def rotate_local_offset(offset_local, anchor_yaw: float) -> tuple[float, float, float]:
    vals = [float(v) for v in offset_local]
    if len(vals) < 2:
        raise ValueError(f"Offset must contain at least x and y: {offset_local!r}")
    dx = float(vals[0])
    dy = float(vals[1])
    dz = float(vals[2] if len(vals) > 2 else 0.0)
    c = math.cos(float(anchor_yaw))
    s = math.sin(float(anchor_yaw))
    return (
        dx * c - dy * s,
        dx * s + dy * c,
        dz,
    )


def yaw_from_quaternion_xyzw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (float(w) * float(z) + float(x) * float(y))
    cosy_cosp = 1.0 - 2.0 * (float(y) * float(y) + float(z) * float(z))
    return math.atan2(siny_cosp, cosy_cosp)


def resolve_yaw(
    *,
    anchor_yaw: float,
    anchor_position: tuple[float, float, float],
    resolved_position: tuple[float, float, float],
    yaw_mode: str,
    yaw_local: float = 0.0,
    fallback_yaw: float | None = None,
) -> float:
    mode = str(yaw_mode or "prim").strip().lower()
    if mode == "prim":
        return float(anchor_yaw) + float(yaw_local)
    if mode == "prim_plus_pi":
        return float(anchor_yaw) + math.pi + float(yaw_local)
    if mode == "same_as_resource":
        if fallback_yaw is None:
            raise ValueError("same_as_resource yaw mode requires fallback_yaw")
        return float(fallback_yaw) + float(yaw_local)
    dx = float(anchor_position[0]) - float(resolved_position[0])
    dy = float(anchor_position[1]) - float(resolved_position[1])
    if mode == "face_anchor":
        return math.atan2(dy, dx) + float(yaw_local)
    if mode == "face_away_from_anchor":
        return math.atan2(-dy, -dx) + float(yaw_local)
    raise ValueError(f"Unsupported yaw_mode: {yaw_mode!r}")


def resolve_local_placement(
    *,
    anchor_position: tuple[float, float, float],
    anchor_yaw: float,
    placement_rule: PlacementRule,
) -> SemanticPose:
    world_offset = rotate_local_offset(placement_rule.offset_local, anchor_yaw)
    position = (
        float(anchor_position[0]) + float(world_offset[0]),
        float(anchor_position[1]) + float(world_offset[1]),
        float(anchor_position[2]) + float(world_offset[2]),
    )
    yaw = resolve_yaw(
        anchor_yaw=anchor_yaw,
        anchor_position=anchor_position,
        resolved_position=position,
        yaw_mode=placement_rule.yaw_mode,
        yaw_local=placement_rule.yaw_local,
    )
    return SemanticPose(position=position, yaw=yaw)


def build_queue_poses(
    *,
    anchor_position: tuple[float, float, float],
    anchor_yaw: float,
    resource_pose: SemanticPose,
    queue_rule: QueueRule,
) -> list[SemanticPose]:
    axis_local = normalize_xy_axis(queue_rule.axis_local)
    queue_poses: list[SemanticPose] = []
    for idx in range(max(int(queue_rule.slots), 0)):
        distance = float(queue_rule.first_gap) + float(idx) * float(queue_rule.spacing)
        offset_local = (
            axis_local[0] * distance,
            axis_local[1] * distance,
            axis_local[2] * distance,
        )
        world_offset = rotate_local_offset(offset_local, anchor_yaw)
        position = (
            float(resource_pose.position[0]) + float(world_offset[0]),
            float(resource_pose.position[1]) + float(world_offset[1]),
            float(resource_pose.position[2]) + float(world_offset[2]),
        )
        yaw = resolve_yaw(
            anchor_yaw=anchor_yaw,
            anchor_position=anchor_position,
            resolved_position=position,
            yaw_mode=queue_rule.yaw_mode,
            yaw_local=queue_rule.yaw_local,
            fallback_yaw=resource_pose.yaw,
        )
        queue_poses.append(SemanticPose(position=position, yaw=yaw))
    return queue_poses


def remaining_polyline_waypoints(
    points: list[list[float]],
    current_pose: list[float],
    *,
    rejoin_tolerance_m: float = 0.05,
) -> list[list[float]]:
    """Return a safe forward-only suffix, rejoining the nearest segment first."""
    parsed: list[list[float]] = []
    for point in points:
        value = [float(point[0]), float(point[1]), float(point[2])]
        if not parsed or math.dist(parsed[-1][:2], value[:2]) > 1e-6:
            parsed.append(value)
    if len(parsed) <= 1:
        return parsed

    current_x = float(current_pose[0])
    current_y = float(current_pose[1])
    best: tuple[float, int, list[float]] | None = None
    for index, (start, end) in enumerate(zip(parsed, parsed[1:])):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            continue
        progress = max(
            0.0,
            min(1.0, ((current_x - start[0]) * dx + (current_y - start[1]) * dy) / length_sq),
        )
        projection = [
            float(start[0]) + progress * dx,
            float(start[1]) + progress * dy,
            float(start[2]) + progress * (float(end[2]) - float(start[2])),
        ]
        distance = math.hypot(current_x - projection[0], current_y - projection[1])
        candidate = (distance, index, projection)
        if best is None or candidate[:2] < best[:2]:
            best = candidate

    if best is None:
        return [parsed[-1]]
    distance, segment_index, projection = best
    remaining = parsed[segment_index + 1 :]
    if distance > max(0.0, float(rejoin_tolerance_m)):
        remaining.insert(0, projection)
    return remaining or [parsed[-1]]
