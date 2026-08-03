"""Simulator-neutral state for authored pedestrian routes."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Callable, Sequence

from ..episodes.schema import PedestrianEpisodeSpec, PedestrianHoldSpec


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
    return replace(
        spec,
        route_waypoints=tuple(expanded),
        holds=remapped_holds,
    )


@dataclass
class RouteRuntime:
    spec: PedestrianEpisodeSpec
    boundary_indices: tuple[int, ...]
    boundary_cursor: int = 0
    state: str = "WAITING_FOR_POSE"
    hold_until: float | None = None

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
        return cls(spec=spec, boundary_indices=tuple(boundaries))

    @property
    def current_boundary(self) -> int:
        return self.boundary_indices[self.boundary_cursor]

    @property
    def previous_boundary(self) -> int:
        return 0 if self.boundary_cursor == 0 else self.boundary_indices[self.boundary_cursor - 1]

    @property
    def complete(self) -> bool:
        return self.state == "COMPLETE"

    def segment(self) -> tuple[tuple[float, float, float], ...]:
        return self.spec.route_waypoints[self.previous_boundary : self.current_boundary + 1]

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
