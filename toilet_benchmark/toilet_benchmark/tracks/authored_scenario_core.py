"""Simulator-neutral state for authored pedestrian motion."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Callable, Sequence

from ..episodes.schema import (
    EpisodeSpec,
    PedestrianEpisodeSpec,
    PedestrianHoldSpec,
    PedestrianTerminalBehavior,
)


# Isaac People character roots face local -Y while authored routes use the
# conventional +X planar heading. Keep this conversion at the route boundary.
ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD = math.pi / 2.0
PLANNER_ENDPOINT_KINK_MAX_LENGTH_M = 0.10
PLANNER_ENDPOINT_KINK_MIN_TURN_RAD = math.radians(45.0)


@dataclass(frozen=True)
class ActivationGateStatus:
    eligible: bool
    reasons: tuple[str, ...]
    generation: int
    yaw: float
    position_error_m: float
    yaw_error_rad: float


def _observe_activation(
    runtime,
    pose: Sequence[float],
    metadata: dict[str, str],
    *,
    position_tolerance_m: float,
    position_stability_m: float,
    yaw_tolerance_rad: float,
    yaw_stability_rad: float,
    required_samples: int,
) -> bool:
    status = activation_gate_status(
        runtime.spec,
        pose,
        metadata,
        position_tolerance_m=position_tolerance_m,
        yaw_tolerance_rad=yaw_tolerance_rad,
    )
    if not status.eligible:
        runtime.reset_activation_observation()
        return False
    current_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
    if runtime.activation_pose is not None:
        position_step = math.dist(current_pose[:2], runtime.activation_pose[:2])
        yaw_step = abs(_angle_difference(status.yaw, float(runtime.activation_yaw)))
        if position_step > position_stability_m or yaw_step > yaw_stability_rad:
            runtime.reset_activation_observation()
    runtime.generation = status.generation
    runtime.activation_pose = current_pose
    runtime.activation_yaw = status.yaw
    runtime.activation_stable_samples += 1
    return runtime.activation_stable_samples >= max(1, int(required_samples))


def activation_gate_status(
    spec: PedestrianEpisodeSpec,
    pose: Sequence[float],
    metadata: dict[str, str],
    *,
    position_tolerance_m: float,
    yaw_tolerance_rad: float,
) -> ActivationGateStatus:
    """Describe the same instantaneous activation gate used by the runtime."""

    reasons: list[str] = []
    try:
        generation = int(metadata.get("embodiment_generation", "0"))
    except (TypeError, ValueError):
        generation = 0
    try:
        yaw = float(metadata.get("yaw_rad", "nan"))
    except (TypeError, ValueError):
        yaw = math.nan
    if spec.start_pose is None:
        raise ValueError(f"{spec.agent_id}: start_pose is required for activation")
    start = spec.start_pose
    position_error = math.hypot(float(pose[0]) - start[0], float(pose[1]) - start[1])
    expected_yaw = float(spec.start_yaw or 0.0)
    yaw_error = abs(_angle_difference(yaw, expected_yaw)) if math.isfinite(yaw) else math.inf
    if generation <= 0:
        reasons.append("generation_missing")
    if metadata.get("reactivation_ready", "").lower() != "true":
        reasons.append("reactivation_not_ready")
    if metadata.get("motion_state", "").lower() != "idle":
        reasons.append("motion_state_not_idle")
    if not math.isfinite(yaw):
        reasons.append("yaw_invalid")
    elif yaw_error > yaw_tolerance_rad:
        reasons.append("yaw_out_of_tolerance")
    if position_error > position_tolerance_m:
        reasons.append("position_out_of_tolerance")
    return ActivationGateStatus(
        eligible=not reasons,
        reasons=tuple(reasons),
        generation=generation,
        yaw=yaw,
        position_error_m=position_error,
        yaw_error_rad=yaw_error,
    )


def with_agent_id_suffix(episode: EpisodeSpec, suffix: str) -> EpisodeSpec:
    """Create fresh runtime identities without changing authored route semantics."""

    suffix = str(suffix).strip()
    if not suffix:
        return episode
    if not suffix.isascii() or not suffix.replace("_", "").isalnum():
        raise ValueError("agent id suffix may only contain letters, digits, and underscores")
    return replace(
        episode,
        pedestrians=tuple(
            replace(spec, agent_id=f"{spec.agent_id}{suffix}")
            for spec in episode.pedestrians
        ),
        metadata={**episode.metadata, "runtime_agent_id_suffix": suffix},
    )


def normalize_yaw(yaw: float) -> float:
    return math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))


def route_heading_to_character_root_yaw(route_heading: float) -> float:
    return normalize_yaw(float(route_heading) + ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD)


def initial_character_root_yaw(
    start_pose: Sequence[float],
    route_waypoints: Sequence[Sequence[float]],
) -> float:
    """Derive the root yaw from the first non-degenerate route segment."""

    cursor = (float(start_pose[0]), float(start_pose[1]))
    for waypoint in route_waypoints:
        target = (float(waypoint[0]), float(waypoint[1]))
        if math.dist(cursor, target) <= 1e-4:
            continue
        heading = math.atan2(target[1] - cursor[1], target[0] - cursor[0])
        return route_heading_to_character_root_yaw(heading)
    raise ValueError("cannot derive character yaw from a degenerate route")


def align_spec_start_yaw(spec: PedestrianEpisodeSpec) -> PedestrianEpisodeSpec:
    if spec.start_pose is None:
        raise ValueError(f"{spec.agent_id}: start_pose is required for yaw alignment")
    if not spec.auto_start_yaw:
        if spec.start_yaw is None:
            raise ValueError(f"{spec.agent_id}: manual start yaw is required")
        return spec
    return replace(
        spec,
        start_yaw=initial_character_root_yaw(spec.start_pose, spec.route_waypoints),
    )


def _turn_magnitude(
    start: Sequence[float],
    corner: Sequence[float],
    end: Sequence[float],
) -> float:
    incoming = math.atan2(corner[1] - start[1], corner[0] - start[0])
    outgoing = math.atan2(end[1] - corner[1], end[0] - corner[0])
    return abs(normalize_yaw(outgoing - incoming))


def _normalize_planner_endpoint_kinks(
    points: Sequence[Sequence[float]],
) -> list[tuple[float, float, float]]:
    """Drop centimetre-scale sharp bends caused by planner endpoint snapping."""

    normalized: list[tuple[float, float, float]] = []
    for point in points:
        parsed = tuple(float(value) for value in point[:3])
        if normalized and math.dist(normalized[-1][:2], parsed[:2]) <= 1e-6:
            continue
        normalized.append(parsed)
    changed = True
    while changed and len(normalized) >= 3:
        changed = False
        if (
            math.dist(normalized[0][:2], normalized[1][:2])
            < PLANNER_ENDPOINT_KINK_MAX_LENGTH_M
            and _turn_magnitude(normalized[0], normalized[1], normalized[2])
            >= PLANNER_ENDPOINT_KINK_MIN_TURN_RAD
        ):
            normalized.pop(1)
            changed = True
        if len(normalized) >= 3 and (
            math.dist(normalized[-2][:2], normalized[-1][:2])
            < PLANNER_ENDPOINT_KINK_MAX_LENGTH_M
            and _turn_magnitude(normalized[-3], normalized[-2], normalized[-1])
            >= PLANNER_ENDPOINT_KINK_MIN_TURN_RAD
        ):
            normalized.pop(-2)
            changed = True
    return normalized


def expand_authored_route(
    spec: PedestrianEpisodeSpec,
    planner: Callable[[Sequence[float], Sequence[float]], Sequence[Sequence[float]]],
    *,
    max_segment_length_m: float = 0.50,
) -> PedestrianEpisodeSpec:
    """Treat authored waypoints as targets and plan each connecting segment."""

    if spec.start_pose is None:
        raise ValueError(f"{spec.agent_id}: start_pose is required for route planning")
    if not spec.route_waypoints:
        raise ValueError(f"{spec.agent_id}: at least one authored target is required")
    expanded = [tuple(float(value) for value in spec.start_pose)]
    target_to_expanded: dict[int, int] = {}
    cursor = expanded[0]
    for target_index, target in enumerate(spec.route_waypoints):
        target = tuple(float(value) for value in target)
        if math.dist(cursor[:2], target[:2]) <= 1e-6:
            target_to_expanded[target_index] = len(expanded) - 1
            cursor = target
            continue
        segment = _normalize_planner_endpoint_kinks((cursor, *planner(cursor, target)))
        if not segment:
            raise ValueError(
                f"{spec.agent_id}: planner returned no route to authored target {target_index}"
            )
        for point in segment:
            distance = math.dist(expanded[-1][:2], point[:2])
            if distance <= 1e-6:
                continue
            steps = max(1, math.ceil(distance / max(0.05, float(max_segment_length_m))))
            start = expanded[-1]
            for step in range(1, steps + 1):
                ratio = step / steps
                expanded.append(
                    (
                        start[0] + ratio * (point[0] - start[0]),
                        start[1] + ratio * (point[1] - start[1]),
                        start[2] + ratio * (point[2] - start[2]),
                    )
                )
        if math.dist(expanded[-1][:2], target[:2]) > 1e-4:
            raise ValueError(
                f"{spec.agent_id}: planned route did not reach authored target {target_index}"
            )
        target_to_expanded[target_index] = len(expanded) - 1
        cursor = target
    remapped_holds = tuple(
        PedestrianHoldSpec(
            waypoint_index=target_to_expanded[hold.waypoint_index],
            duration_sec=hold.duration_sec,
            yaw=hold.yaw,
        )
        for hold in spec.holds
    )
    return align_spec_start_yaw(replace(
        spec,
        route_waypoints=tuple(expanded),
        holds=remapped_holds,
    ))


@dataclass
class RouteRuntime:
    spec: PedestrianEpisodeSpec
    boundary_indices: tuple[int, ...]
    boundary_cursor: int = 0
    state: str = "WAITING_FOR_POSE"
    hold_until: float | None = None
    generation: int = 1
    command_generation: int = 0
    activation_stable_samples: int = 0
    activation_pose: tuple[float, float, float] | None = None
    activation_yaw: float | None = None

    @classmethod
    def create(cls, spec: PedestrianEpisodeSpec) -> "RouteRuntime":
        if not spec.route_waypoints:
            raise ValueError(f"{spec.agent_id}: runtime route needs a spawn waypoint")
        last = len(spec.route_waypoints) - 1
        boundaries = sorted({hold.waypoint_index for hold in spec.holds} | {last})
        if len(spec.route_waypoints) > 1 and boundaries[0] <= 0:
            raise ValueError(f"{spec.agent_id}: a hold cannot target the spawn waypoint")
        if boundaries[-1] > last:
            raise ValueError(f"{spec.agent_id}: hold waypoint is outside the route")
        return cls(
            spec=spec,
            boundary_indices=tuple(boundaries),
        )

    @property
    def current_boundary(self) -> int:
        return self.boundary_indices[self.boundary_cursor]

    @property
    def previous_boundary(self) -> int:
        return 0 if self.boundary_cursor == 0 else self.boundary_indices[self.boundary_cursor - 1]

    @property
    def complete(self) -> bool:
        return self.state == "COMPLETE"

    def observe_activation(
        self,
        pose: Sequence[float],
        metadata: dict[str, str],
        *,
        position_tolerance_m: float,
        position_stability_m: float,
        yaw_tolerance_rad: float,
        yaw_stability_rad: float,
        required_samples: int,
    ) -> bool:
        """Accept reactivation only after several stable simulator samples."""

        return _observe_activation(
            self,
            pose,
            metadata,
            position_tolerance_m=position_tolerance_m,
            position_stability_m=position_stability_m,
            yaw_tolerance_rad=yaw_tolerance_rad,
            yaw_stability_rad=yaw_stability_rad,
            required_samples=required_samples,
        )

    def reset_activation_observation(self) -> None:
        self.activation_stable_samples = 0
        self.activation_pose = None
        self.activation_yaw = None

    def segment(self) -> tuple[tuple[float, float, float], ...]:
        return self.spec.route_waypoints[self.previous_boundary : self.current_boundary + 1]

    def execution_segment(self) -> tuple[tuple[float, float, float], ...]:
        """Return only forward targets; the actor already occupies the boundary start."""
        return self.spec.route_waypoints[
            self.previous_boundary + 1 : self.current_boundary + 1
        ]

    @property
    def has_forward_route(self) -> bool:
        return len(self.spec.route_waypoints) > 1

    def activate(self, now: float) -> str:
        duration = self.spec.start_hold_duration_sec
        if duration is None:
            self.state = "HOLDING_UNTIL_EPISODE_END"
            return "terminal_hold"
        if float(duration) > 0.0:
            self.state = "START_HOLDING"
            self.hold_until = float(now) + float(duration)
            return "start_hold"
        if not self.has_forward_route:
            if self.spec.terminal_behavior == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END:
                self.state = "HOLDING_UNTIL_EPISODE_END"
                return "terminal_hold"
            self.state = "COMPLETE"
            return "complete"
        return "dispatch"

    def release_start_hold(self, now: float) -> str | None:
        if self.state != "START_HOLDING" or self.hold_until is None or now < self.hold_until:
            return None
        self.hold_until = None
        if not self.has_forward_route:
            if self.spec.terminal_behavior == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END:
                self.state = "HOLDING_UNTIL_EPISODE_END"
                return "terminal_hold"
            self.state = "COMPLETE"
            return "complete"
        return "dispatch"

    def hold_duration(self) -> float | None:
        for hold in self.spec.holds:
            if hold.waypoint_index == self.current_boundary:
                return hold.duration_sec
        return None

    def hold_yaw(self) -> float | None:
        for hold in self.spec.holds:
            if hold.waypoint_index == self.current_boundary:
                return hold.yaw
        return None

    def stop_yaw(self) -> float | None:
        hold_yaw = self.hold_yaw()
        if hold_yaw is not None:
            return hold_yaw
        if (
            self.current_boundary == len(self.spec.route_waypoints) - 1
            and self.spec.terminal_behavior
            == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
        ):
            return self.spec.terminal_yaw
        return None

    def arrive(self, now: float) -> str:
        duration = self.hold_duration()
        if self._current_hold() is not None and self.state != "HOLDING":
            if duration is None:
                self.state = "HOLDING_UNTIL_EPISODE_END"
                return "terminal_hold"
            self.state = "HOLDING"
            self.hold_until = float(now) + float(duration)
            return "hold"
        if self.current_boundary == len(self.spec.route_waypoints) - 1:
            if self.spec.terminal_behavior == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END:
                self.state = "HOLDING_UNTIL_EPISODE_END"
                return "terminal_hold"
            self.state = "COMPLETE"
            return "complete"
        self.boundary_cursor += 1
        self.state = "MOVING"
        self.hold_until = None
        return "dispatch"

    def _current_hold(self) -> PedestrianHoldSpec | None:
        for hold in self.spec.holds:
            if hold.waypoint_index == self.current_boundary:
                return hold
        return None

    def release_hold(self, now: float) -> str | None:
        if self.state != "HOLDING" or self.hold_until is None or now < self.hold_until:
            return None
        self.hold_until = None
        if self.current_boundary == len(self.spec.route_waypoints) - 1:
            if self.spec.terminal_behavior == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END:
                self.state = "HOLDING_UNTIL_EPISODE_END"
                return "terminal_hold"
            self.state = "COMPLETE"
            return "complete"
        self.boundary_cursor += 1
        self.state = "MOVING"
        return "dispatch"


def _angle_difference(lhs: float, rhs: float) -> float:
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))
