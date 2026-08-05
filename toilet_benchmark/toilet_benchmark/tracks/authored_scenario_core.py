"""Simulator-neutral state for authored pedestrian routes."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Callable, Sequence

from ..episodes.schema import EpisodeSpec, PedestrianEpisodeSpec, PedestrianHoldSpec


# Isaac People character roots face local -Y while authored routes use the
# conventional +X planar heading. Keep this conversion at the route boundary.
ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD = math.pi / 2.0


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
    return replace(
        spec,
        start_yaw=initial_character_root_yaw(spec.start_pose, spec.route_waypoints),
    )


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
        segment = [tuple(float(value) for value in point[:3]) for point in planner(cursor, target)]
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
        if len(spec.route_waypoints) < 2:
            raise ValueError(f"{spec.agent_id}: authored route needs at least two waypoints")
        last = len(spec.route_waypoints) - 1
        boundaries = sorted({hold.waypoint_index for hold in spec.holds} | {last})
        if boundaries[0] <= 0:
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

        try:
            generation = int(metadata.get("embodiment_generation", "0"))
            yaw = float(metadata.get("yaw_rad", "nan"))
        except (TypeError, ValueError):
            self.reset_activation_observation()
            return False
        expected_yaw = float(self.spec.start_yaw or 0.0)
        start = self.spec.route_waypoints[0]
        valid = (
            generation > 0
            and metadata.get("reactivation_ready", "").lower() == "true"
            and metadata.get("motion_state", "").lower() == "idle"
            and math.isfinite(yaw)
            and math.hypot(float(pose[0]) - start[0], float(pose[1]) - start[1])
            <= position_tolerance_m
            and abs(_angle_difference(yaw, expected_yaw)) <= yaw_tolerance_rad
        )
        if not valid:
            self.reset_activation_observation()
            return False

        current_pose = (float(pose[0]), float(pose[1]), float(pose[2]))
        if self.activation_pose is not None:
            position_step = math.dist(current_pose[:2], self.activation_pose[:2])
            yaw_step = abs(_angle_difference(yaw, float(self.activation_yaw)))
            if position_step > position_stability_m or yaw_step > yaw_stability_rad:
                self.reset_activation_observation()

        self.generation = generation
        self.activation_pose = current_pose
        self.activation_yaw = yaw
        self.activation_stable_samples += 1
        return self.activation_stable_samples >= max(1, int(required_samples))

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

    def hold_duration(self) -> float | None:
        for hold in self.spec.holds:
            if hold.waypoint_index == self.current_boundary:
                return hold.duration_sec
        return None

    def arrive(self, now: float) -> str:
        duration = self.hold_duration()
        if duration is not None and self.state != "HOLDING":
            self.state = "HOLDING"
            self.hold_until = float(now) + float(duration)
            return "hold"
        if self.current_boundary == len(self.spec.route_waypoints) - 1:
            self.state = "COMPLETE"
            return "complete"
        self.boundary_cursor += 1
        self.state = "MOVING"
        self.hold_until = None
        return "dispatch"

    def release_hold(self, now: float) -> str | None:
        if self.state != "HOLDING" or self.hold_until is None or now < self.hold_until:
            return None
        self.hold_until = None
        if self.current_boundary == len(self.spec.route_waypoints) - 1:
            self.state = "COMPLETE"
            return "complete"
        self.boundary_cursor += 1
        self.state = "MOVING"
        return "dispatch"


def _angle_difference(lhs: float, rhs: float) -> float:
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))
