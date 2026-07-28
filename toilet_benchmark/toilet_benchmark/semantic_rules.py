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


@dataclass(frozen=True)
class MotionObservation:
    moved: bool
    progressed: bool
    distance_to_target: float


@dataclass(frozen=True)
class PortalCorridor:
    """Oriented portal geometry shared by event and motion backends."""

    outside: tuple[float, float, float]
    inside: tuple[float, float, float]
    half_width_m: float
    clearance_m: float
    capacity: int = 1

    def __post_init__(self) -> None:
        if self.length_m <= 1e-6:
            raise ValueError("Portal outside and inside poses must be distinct")
        if float(self.half_width_m) <= 0.0:
            raise ValueError("Portal half_width_m must be positive")
        if float(self.clearance_m) < 0.0:
            raise ValueError("Portal clearance_m must not be negative")
        if int(self.capacity) < 1:
            raise ValueError("Portal capacity must be at least one")

    @property
    def length_m(self) -> float:
        return math.hypot(
            float(self.inside[0]) - float(self.outside[0]),
            float(self.inside[1]) - float(self.outside[1]),
        )

    @property
    def axis(self) -> tuple[float, float]:
        length = self.length_m
        return (
            (float(self.inside[0]) - float(self.outside[0])) / length,
            (float(self.inside[1]) - float(self.outside[1])) / length,
        )

    @property
    def yaw(self) -> float:
        axis_x, axis_y = self.axis
        return math.atan2(axis_y, axis_x)

    def coordinates(self, pose) -> tuple[float, float]:
        """Return longitudinal progress from outside and signed lateral offset."""
        axis_x, axis_y = self.axis
        relative_x = float(pose[0]) - float(self.outside[0])
        relative_y = float(pose[1]) - float(self.outside[1])
        return (
            relative_x * axis_x + relative_y * axis_y,
            relative_x * -axis_y + relative_y * axis_x,
        )

    def clear_pose(self, direction: str) -> list[float]:
        """Return the first pose that fully clears the portal in a direction."""
        axis_x, axis_y = self.axis
        if direction == "entering":
            anchor = self.inside
            sign = 1.0
        elif direction == "exiting":
            anchor = self.outside
            sign = -1.0
        else:
            raise ValueError(f"Unsupported portal direction: {direction!r}")
        return [
            float(anchor[0]) + sign * float(self.clearance_m) * axis_x,
            float(anchor[1]) + sign * float(self.clearance_m) * axis_y,
            float(anchor[2]),
        ]

    def contains_laterally(self, pose, *, margin_m: float = 0.0) -> bool:
        _, lateral = self.coordinates(pose)
        return abs(lateral) <= float(self.half_width_m) + max(0.0, float(margin_m))

    def cleared(self, pose, direction: str, *, lateral_margin_m: float = 0.0) -> bool:
        """Check directional clear-plane crossing without requiring point arrival."""
        if not self.contains_laterally(pose, margin_m=lateral_margin_m):
            return False
        longitudinal, _ = self.coordinates(pose)
        if direction == "entering":
            return longitudinal >= self.length_m + float(self.clearance_m)
        if direction == "exiting":
            return longitudinal <= -float(self.clearance_m)
        raise ValueError(f"Unsupported portal direction: {direction!r}")

    def traversal_goal(
        self,
        direction: str,
        *,
        overshoot_m: float = 0.0,
    ) -> list[float]:
        """Place locomotion goals beyond the event clear plane when needed."""
        goal = self.clear_pose(direction)
        axis_x, axis_y = self.axis
        sign = 1.0 if direction == "entering" else -1.0
        overshoot = max(0.0, float(overshoot_m))
        goal[0] += sign * overshoot * axis_x
        goal[1] += sign * overshoot * axis_y
        return goal


def portal_inside_plane_reached(
    *,
    current_pose: list[float],
    inside_pose: list[float],
    staging_pose: list[float],
    lateral_tolerance_m: float,
) -> bool:
    """Legacy entry-plane check retained for the current Isaac backend."""
    dx = float(staging_pose[0]) - float(inside_pose[0])
    dy = float(staging_pose[1]) - float(inside_pose[1])
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        return math.dist(current_pose[:2], inside_pose[:2]) <= max(
            0.01,
            float(lateral_tolerance_m),
        )
    axis_x = dx / length
    axis_y = dy / length
    relative_x = float(current_pose[0]) - float(inside_pose[0])
    relative_y = float(current_pose[1]) - float(inside_pose[1])
    progress = relative_x * axis_x + relative_y * axis_y
    lateral = abs(relative_x * axis_y - relative_y * axis_x)
    return progress >= -0.02 and lateral <= max(0.01, float(lateral_tolerance_m))


def classify_motion_observation(
    *,
    previous_pose: list[float] | None,
    current_pose: list[float],
    target_pose: list[float],
    best_distance: float,
    displacement_epsilon_m: float,
    progress_epsilon_m: float,
) -> MotionObservation:
    """Separate actual root motion from progress toward the phase target."""
    moved = previous_pose is None or (
        math.dist(previous_pose[:2], current_pose[:2]) >= max(0.001, float(displacement_epsilon_m))
    )
    distance = math.dist(current_pose[:2], target_pose[:2])
    progressed = not math.isfinite(float(best_distance)) or (
        distance <= float(best_distance) - max(0.001, float(progress_epsilon_m))
    )
    return MotionObservation(moved=moved, progressed=progressed, distance_to_target=distance)


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
