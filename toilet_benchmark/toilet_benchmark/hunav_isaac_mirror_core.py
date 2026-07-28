"""Pure parsing and metrics helpers for the HuNav-Isaac shadow mirror."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Any, Iterable, Mapping, Sequence, Tuple


Point3D = Tuple[float, float, float]


def _point3(value: Sequence[Any]) -> Point3D:
    if len(value) < 2:
        raise ValueError("pose must contain at least x and y")
    z = float(value[2]) if len(value) >= 3 else 0.0
    point = (float(value[0]), float(value[1]), z)
    if not all(math.isfinite(item) for item in point):
        raise ValueError("pose contains a non-finite value")
    return point


@dataclass(frozen=True)
class PortalMotionIntent:
    portal_id: str
    direction: str
    outside: Point3D
    inside: Point3D
    half_width_m: float
    clearance_m: float
    capacity: int

    def __post_init__(self) -> None:
        if self.direction not in {"entering", "exiting"}:
            raise ValueError(f"Unsupported portal direction: {self.direction!r}")
        if self.half_width_m <= 0.0:
            raise ValueError("Portal half_width_m must be positive")
        if self.clearance_m < 0.0:
            raise ValueError("Portal clearance_m must not be negative")
        if self.capacity < 1:
            raise ValueError("Portal capacity must be at least one")


@dataclass(frozen=True)
class MotionIntent:
    agent_id: str
    generation: int
    phase: str
    resource_id: str | None
    goal_pose: Point3D
    path_points: Tuple[Point3D, ...]
    velocity: float
    final_yaw: float
    stop: bool
    use_direct_pose: bool
    constrain_to_path: bool
    portal: PortalMotionIntent | None = None

    @property
    def shadow_enabled(self) -> bool:
        return (
            not self.stop
            and not self.use_direct_pose
            and self.velocity > 0.0
        )

    def goals(self) -> Tuple[Point3D, ...]:
        points = list(self.path_points)
        if not points or math.hypot(
            points[-1][0] - self.goal_pose[0],
            points[-1][1] - self.goal_pose[1],
        ) > 1e-4:
            points.append(self.goal_pose)
        return tuple(points)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def build_motion_intent_payload(
    *,
    agent_id: str,
    generation: int,
    phase: str,
    resource_id: str | None,
    goal_pose: Sequence[float],
    path_points: Iterable[Sequence[float]],
    velocity: float,
    final_yaw: float,
    stop: bool,
    use_direct_pose: bool,
    constrain_to_path: bool,
    portal: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    payload = {
        "event": "motion_intent",
        "agent_id": str(agent_id),
        "generation": int(generation),
        "phase": str(phase),
        "resource_id": resource_id,
        "goal_pose": list(_point3(goal_pose)),
        "path_points": [list(_point3(point)) for point in path_points],
        "velocity": float(velocity),
        "final_yaw": float(final_yaw),
        "stop": bool(stop),
        "use_direct_pose": bool(use_direct_pose),
        "constrain_to_path": bool(constrain_to_path),
        "route_kind": "portal_corridor" if portal is not None else "polyline",
    }
    if portal is not None:
        payload["portal"] = {
            "portal_id": str(portal["portal_id"]),
            "direction": str(portal["direction"]),
            "outside": list(_point3(portal["outside"])),
            "inside": list(_point3(portal["inside"])),
            "half_width_m": float(portal["half_width_m"]),
            "clearance_m": float(portal["clearance_m"]),
            "capacity": int(portal.get("capacity", 1)),
        }
    return payload


def parse_motion_intent(payload: Mapping[str, Any]) -> MotionIntent | None:
    if payload.get("event") != "motion_intent":
        return None
    portal_payload = payload.get("portal")
    portal = None
    if portal_payload is not None:
        portal = PortalMotionIntent(
            portal_id=str(portal_payload["portal_id"]),
            direction=str(portal_payload["direction"]),
            outside=_point3(portal_payload["outside"]),
            inside=_point3(portal_payload["inside"]),
            half_width_m=float(portal_payload["half_width_m"]),
            clearance_m=float(portal_payload["clearance_m"]),
            capacity=int(portal_payload.get("capacity", 1)),
        )
    intent = MotionIntent(
        agent_id=str(payload["agent_id"]).strip(),
        generation=int(payload["generation"]),
        phase=str(payload.get("phase", "")),
        resource_id=(
            None
            if payload.get("resource_id") is None
            else str(payload.get("resource_id"))
        ),
        goal_pose=_point3(payload["goal_pose"]),
        path_points=tuple(
            _point3(point) for point in (payload.get("path_points") or [])
        ),
        velocity=float(payload.get("velocity", 0.0)),
        final_yaw=float(payload.get("final_yaw", 0.0)),
        stop=bool(payload.get("stop", False)),
        use_direct_pose=bool(payload.get("use_direct_pose", False)),
        constrain_to_path=bool(payload.get("constrain_to_path", False)),
        portal=portal,
    )
    if not intent.agent_id:
        raise ValueError("motion intent agent_id is empty")
    if intent.generation < 1:
        raise ValueError("motion intent generation must be positive")
    if not all(
        math.isfinite(value) for value in (intent.velocity, intent.final_yaw)
    ):
        raise ValueError("motion intent contains a non-finite scalar")
    return intent


def tag_mapping(person: Any) -> dict[str, str]:
    names = list(getattr(person, "tagnames", []) or [])
    values = list(getattr(person, "tags", []) or [])
    return {str(name): str(value) for name, value in zip(names, values)}


def person_yaw(person: Any) -> float | None:
    tags = tag_mapping(person)
    if tags.get("yaw_valid", "false").lower() != "true":
        return None
    try:
        yaw = float(tags["yaw_rad"])
    except (KeyError, TypeError, ValueError):
        return None
    return yaw if math.isfinite(yaw) else None


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def velocity_heading(
    velocity_x: float,
    velocity_y: float,
    *,
    min_speed: float,
) -> float | None:
    if math.hypot(float(velocity_x), float(velocity_y)) < float(min_speed):
        return None
    return math.atan2(float(velocity_y), float(velocity_x))


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
        ax = float(start[0])
        ay = float(start[1])
        bx = float(end[0])
        by = float(end[1])
        dx = bx - ax
        dy = by - ay
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


def percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = max(
        0,
        min(
            len(ordered) - 1,
            int(math.ceil(float(fraction) * len(ordered))) - 1,
        ),
    )
    return ordered[index]


def _path_length(samples: Sequence[Mapping[str, Any]], key: str) -> float:
    total = 0.0
    previous = None
    previous_segment = None
    for sample in samples:
        state = sample.get(key) or {}
        point = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        segment = sample.get("segment_index")
        if previous is not None and segment == previous_segment:
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = point
        previous_segment = segment
    return total


def _near_target_path_length(
    samples: Sequence[Mapping[str, Any]],
    key: str,
) -> float:
    total = 0.0
    previous = None
    previous_segment = None
    for sample in samples:
        state = sample.get(key) or {}
        point = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        segment = sample.get("segment_index")
        near_target = bool(sample.get(f"{key}_near_target", False))
        if (
            previous is not None
            and segment == previous_segment
            and near_target
            and previous[2]
        ):
            total += math.hypot(point[0] - previous[0], point[1] - previous[1])
        previous = (point[0], point[1], near_target)
        previous_segment = segment
    return total


def _guard_blocked_duration(samples: Sequence[Mapping[str, Any]]) -> float:
    total = 0.0
    previous = None
    for sample in samples:
        if (
            previous is not None
            and sample.get("segment_index") == previous.get("segment_index")
            and bool(previous.get("actual", {}).get("guard_blocked", False))
        ):
            total += max(
                0.0,
                float(sample["source_stamp_sec"])
                - float(previous["source_stamp_sec"]),
            )
        previous = sample
    return total


def _settled_jitter_path_length(
    samples: Sequence[Mapping[str, Any]],
    key: str,
    *,
    near_target_radius_m: float,
    settled_speed_mps: float,
    settled_hold_sec: float,
) -> float:
    total = 0.0
    candidate_start = None
    previous = None
    previous_segment = None
    previous_settled = False
    for sample in samples:
        segment = sample.get("segment_index")
        if segment != previous_segment:
            candidate_start = None
            previous = None
            previous_settled = False
        state = sample.get(key) or {}
        stamp = float(sample.get("source_stamp_sec", 0.0))
        distance = sample.get(f"{key}_target_distance_m")
        speed = math.hypot(
            float(state.get("vx", 0.0)),
            float(state.get("vy", 0.0)),
        )
        qualifies = (
            distance is not None
            and float(distance) <= float(near_target_radius_m)
            and speed <= float(settled_speed_mps)
        )
        if qualifies:
            if candidate_start is None:
                candidate_start = stamp
            settled = stamp - candidate_start >= float(settled_hold_sec)
        else:
            candidate_start = None
            settled = False
        point = (float(state.get("x", 0.0)), float(state.get("y", 0.0)))
        if previous is not None and settled and previous_settled:
            total += math.hypot(
                point[0] - previous[0],
                point[1] - previous[1],
            )
        previous = point
        previous_segment = segment
        previous_settled = settled
    return total


def summarize_samples(
    samples: Sequence[Mapping[str, Any]],
    *,
    intent_count: int,
    compute_failures: int,
    near_target_radius_m: float = 0.35,
    settled_speed_mps: float = 0.05,
    settled_hold_sec: float = 1.0,
) -> dict[str, Any]:
    position_errors = [
        float(sample["position_error_m"])
        for sample in samples
        if sample.get("position_error_m") is not None
    ]
    yaw_errors = [
        abs(float(sample["yaw_error_rad"]))
        for sample in samples
        if sample.get("yaw_error_rad") is not None
    ]
    motion_heading_errors = [
        abs(float(sample["motion_heading_error_rad"]))
        for sample in samples
        if sample.get("motion_heading_error_rad") is not None
    ]
    actual_root_heading_offsets = [
        abs(float(sample["actual_root_to_motion_offset_rad"]))
        for sample in samples
        if sample.get("actual_root_to_motion_offset_rad") is not None
    ]
    actual_cross_track_errors = [
        float(sample["actual_cross_track_error_m"])
        for sample in samples
        if sample.get("actual_cross_track_error_m") is not None
    ]
    hunav_cross_track_errors = [
        float(sample["hunav_cross_track_error_m"])
        for sample in samples
        if sample.get("hunav_cross_track_error_m") is not None
    ]
    hunav_raw_cross_track_errors = [
        float(sample["hunav_raw_cross_track_error_m"])
        for sample in samples
        if sample.get("hunav_raw_cross_track_error_m") is not None
    ]
    actual_robot_clearances = [
        float(sample["interaction"]["actual_robot_clearance_m"])
        for sample in samples
        if (sample.get("interaction") or {}).get("actual_robot_clearance_m")
        is not None
    ]
    hunav_robot_clearances = [
        float(sample["interaction"]["hunav_robot_clearance_m"])
        for sample in samples
        if (sample.get("interaction") or {}).get("hunav_robot_clearance_m")
        is not None
    ]
    hunav_raw_robot_clearances = [
        float(sample["interaction"]["hunav_raw_robot_clearance_m"])
        for sample in samples
        if (sample.get("interaction") or {}).get(
            "hunav_raw_robot_clearance_m"
        )
        is not None
    ]
    robot_ages = [
        float(sample["robot"]["age_sec"])
        for sample in samples
        if (sample.get("robot") or {}).get("available")
        and (sample.get("robot") or {}).get("age_sec") is not None
    ]
    stamps = [
        float(sample["source_stamp_sec"])
        for sample in samples
        if sample.get("source_stamp_sec") is not None
    ]
    duration = max(stamps) - min(stamps) if len(stamps) >= 2 else 0.0
    return {
        "sample_count": len(samples),
        "intent_count": int(intent_count),
        "segment_count": len(
            {int(sample["segment_index"]) for sample in samples}
        ),
        "compute_failures": int(compute_failures),
        "duration_sec": duration,
        "effective_sample_hz": (
            (len(stamps) - 1) / duration if duration > 1e-9 else 0.0
        ),
        "mean_position_error_m": (
            sum(position_errors) / len(position_errors)
            if position_errors
            else None
        ),
        "p95_position_error_m": percentile(position_errors, 0.95),
        "max_position_error_m": (
            max(position_errors) if position_errors else None
        ),
        "mean_abs_yaw_error_rad": (
            sum(yaw_errors) / len(yaw_errors) if yaw_errors else None
        ),
        "p95_abs_yaw_error_rad": percentile(yaw_errors, 0.95),
        "max_abs_yaw_error_rad": max(yaw_errors) if yaw_errors else None,
        "mean_abs_motion_heading_error_rad": (
            sum(motion_heading_errors) / len(motion_heading_errors)
            if motion_heading_errors
            else None
        ),
        "p95_abs_motion_heading_error_rad": percentile(
            motion_heading_errors, 0.95
        ),
        "mean_abs_actual_root_to_motion_offset_rad": (
            sum(actual_root_heading_offsets) / len(actual_root_heading_offsets)
            if actual_root_heading_offsets
            else None
        ),
        "p95_actual_cross_track_error_m": percentile(
            actual_cross_track_errors, 0.95
        ),
        "max_actual_cross_track_error_m": (
            max(actual_cross_track_errors)
            if actual_cross_track_errors
            else None
        ),
        "p95_hunav_cross_track_error_m": percentile(
            hunav_cross_track_errors, 0.95
        ),
        "max_hunav_cross_track_error_m": (
            max(hunav_cross_track_errors)
            if hunav_cross_track_errors
            else None
        ),
        "p95_hunav_raw_cross_track_error_m": percentile(
            hunav_raw_cross_track_errors, 0.95
        ),
        "max_hunav_raw_cross_track_error_m": (
            max(hunav_raw_cross_track_errors)
            if hunav_raw_cross_track_errors
            else None
        ),
        "robot_observation_sample_count": len(robot_ages),
        "max_robot_observation_age_sec": (
            max(robot_ages) if robot_ages else None
        ),
        "min_actual_robot_clearance_m": (
            min(actual_robot_clearances) if actual_robot_clearances else None
        ),
        "min_hunav_robot_clearance_m": (
            min(hunav_robot_clearances) if hunav_robot_clearances else None
        ),
        "min_hunav_raw_robot_clearance_m": (
            min(hunav_raw_robot_clearances)
            if hunav_raw_robot_clearances
            else None
        ),
        "safety_intervention_sample_count": sum(
            bool((sample.get("safety") or {}).get("intervened", False))
            for sample in samples
        ),
        "safety_correction_count": sum(
            int((sample.get("safety") or {}).get("correction_count", 0))
            for sample in samples
        ),
        "safety_residual_violation_count": sum(
            int(
                (sample.get("safety") or {}).get(
                    "residual_violation_count", 0
                )
            )
            for sample in samples
        ),
        "guard_blocked_sample_count": sum(
            bool((sample.get("actual") or {}).get("guard_blocked", False))
            for sample in samples
        ),
        "guard_blocked_duration_sec": _guard_blocked_duration(samples),
        "actual_near_target_jitter_m": _near_target_path_length(
            samples, "actual"
        ),
        "hunav_near_target_jitter_m": _near_target_path_length(
            samples, "hunav"
        ),
        "actual_settled_jitter_m": _settled_jitter_path_length(
            samples,
            "actual",
            near_target_radius_m=near_target_radius_m,
            settled_speed_mps=settled_speed_mps,
            settled_hold_sec=settled_hold_sec,
        ),
        "hunav_settled_jitter_m": _settled_jitter_path_length(
            samples,
            "hunav",
            near_target_radius_m=near_target_radius_m,
            settled_speed_mps=settled_speed_mps,
            settled_hold_sec=settled_hold_sec,
        ),
        "hunav_raw_settled_jitter_m": _settled_jitter_path_length(
            samples,
            "hunav_raw",
            near_target_radius_m=near_target_radius_m,
            settled_speed_mps=settled_speed_mps,
            settled_hold_sec=settled_hold_sec,
        ),
        "actual_path_length_m": _path_length(samples, "actual"),
        "hunav_path_length_m": _path_length(samples, "hunav"),
        "hunav_raw_path_length_m": _path_length(samples, "hunav_raw"),
    }
