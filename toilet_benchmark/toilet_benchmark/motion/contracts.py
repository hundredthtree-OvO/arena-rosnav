"""Simulator-neutral contracts for the E2 pedestrian motion pipeline."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from typing import Mapping, Sequence

from ..domain.agent import AgentSnapshot


Point2D = tuple[float, float]
Point3D = tuple[float, float, float]


def _finite(values: Sequence[float], *, label: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{label} values must be finite")


class BehaviorMode(str, Enum):
    WALKING = "walking"
    YIELDING = "yielding"
    PASSING_LEFT = "passing_left"
    PASSING_RIGHT = "passing_right"
    FOLLOWING = "following"
    WAITING = "waiting"
    GROUPING = "grouping"


@dataclass(frozen=True)
class RoutePlan:
    """Immutable global route with enough provenance for replay and diagnosis."""

    agent_id: str
    points: tuple[Point3D, ...]
    planner_id: str
    map_version: str = ""
    dynamic_obstacle_count: int = 0
    diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.agent_id:
            raise ValueError("agent_id must not be empty")
        if not self.planner_id:
            raise ValueError("planner_id must not be empty")
        if not self.points:
            raise ValueError("route points must not be empty")
        for point in self.points:
            if len(point) != 3:
                raise ValueError("route points must contain x, y and z")
            _finite(point, label="route point")
        if self.dynamic_obstacle_count < 0:
            raise ValueError("dynamic_obstacle_count must be non-negative")

    @classmethod
    def from_points(
        cls,
        *,
        agent_id: str,
        points: Sequence[Sequence[float]],
        planner_id: str,
        map_version: str = "",
        dynamic_obstacle_count: int = 0,
        diagnostics: Mapping[str, object] | None = None,
    ) -> "RoutePlan":
        return cls(
            agent_id=str(agent_id),
            points=tuple(
                (float(point[0]), float(point[1]), float(point[2]))
                for point in points
            ),
            planner_id=str(planner_id),
            map_version=str(map_version),
            dynamic_obstacle_count=int(dynamic_obstacle_count),
            diagnostics=dict(diagnostics or {}),
        )


@dataclass(frozen=True)
class BehaviorPolicyRequest:
    timestamp_sec: float
    agent: AgentSnapshot
    route: RoutePlan
    preferred_speed_mps: float
    robot: AgentSnapshot | None = None
    peers: tuple[AgentSnapshot, ...] = ()
    task_phase: str = ""
    requested_mode: BehaviorMode | None = None
    conflict_ttc_sec: float | None = None
    left_clearance_m: float | None = None
    right_clearance_m: float | None = None
    leader_id: str | None = None
    group_id: str | None = None

    def __post_init__(self) -> None:
        _finite((self.timestamp_sec, self.preferred_speed_mps), label="behavior request")
        if self.preferred_speed_mps < 0.0:
            raise ValueError("preferred_speed_mps must be non-negative")
        if self.route.agent_id != self.agent.agent_id:
            raise ValueError("route and behavior agent_id must match")
        optional_values = (
            self.conflict_ttc_sec,
            self.left_clearance_m,
            self.right_clearance_m,
        )
        if any(
            value is not None and not math.isfinite(float(value))
            for value in optional_values
        ):
            raise ValueError("optional behavior geometry values must be finite")


@dataclass(frozen=True)
class BehaviorDecision:
    mode: BehaviorMode
    speed_scale: float = 1.0
    hold_position: bool = False
    preferred_side: int = 0
    reason: str = ""
    diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _finite((self.speed_scale,), label="behavior decision")
        if self.speed_scale < 0.0:
            raise ValueError("speed_scale must be non-negative")
        if self.preferred_side not in {-1, 0, 1}:
            raise ValueError("preferred_side must be -1, 0 or 1")


@dataclass(frozen=True)
class LocalMotionRequest:
    timestamp_sec: float
    dt_sec: float
    agent: AgentSnapshot
    peers: tuple[AgentSnapshot, ...]
    robot: AgentSnapshot | None
    route: RoutePlan
    behavior: BehaviorDecision
    preferred_speed_mps: float
    route_target_xy: Point2D | None = None

    def __post_init__(self) -> None:
        _finite(
            (self.timestamp_sec, self.dt_sec, self.preferred_speed_mps),
            label="local motion request",
        )
        if self.dt_sec <= 0.0:
            raise ValueError("dt_sec must be positive")
        if self.preferred_speed_mps < 0.0:
            raise ValueError("preferred_speed_mps must be non-negative")
        if self.route.agent_id != self.agent.agent_id:
            raise ValueError("route and local motion agent_id must match")
        if self.route_target_xy is not None:
            _finite(self.route_target_xy, label="route target")


@dataclass(frozen=True)
class LocalMotionResult:
    velocity_xy: Point2D
    heading_rad: float
    feasible: bool
    reason: str = ""
    diagnostics: Mapping[str, object] = field(default_factory=dict)

    def __post_init__(self) -> None:
        _finite((*self.velocity_xy, self.heading_rad), label="local motion result")


@dataclass(frozen=True)
class GeometrySafetyRequest:
    start_xy: Point2D
    proposed_xy: Point2D
    start_yaw: float
    proposed_yaw: float

    def __post_init__(self) -> None:
        _finite(
            (*self.start_xy, *self.proposed_xy, self.start_yaw, self.proposed_yaw),
            label="geometry safety request",
        )


@dataclass(frozen=True)
class GeometrySafetyResult:
    position_xy: Point2D
    applied_fraction: float
    clipped: bool
    minimum_clearance_m: float
    recovering_overlap: bool = False

    def __post_init__(self) -> None:
        _finite(
            (*self.position_xy, self.applied_fraction),
            label="geometry safety result",
        )
        if math.isnan(float(self.minimum_clearance_m)):
            raise ValueError("minimum_clearance_m must not be NaN")
        if not 0.0 <= self.applied_fraction <= 1.0:
            raise ValueError("applied_fraction must be between 0 and 1")
