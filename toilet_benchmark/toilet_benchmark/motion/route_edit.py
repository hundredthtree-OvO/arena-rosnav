"""Versioned route-edit and subgoal commands shared by UI, runtime, and replay."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
import math
from typing import Any, Sequence

from .contracts import Point2D, Point3D


def _finite(values: Sequence[float], *, label: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError(f"{label} values must be finite")


def _point3(value: Sequence[float], *, label: str) -> Point3D:
    if len(value) != 3:
        raise ValueError(f"{label} must contain x, y and z")
    point = tuple(float(item) for item in value)
    _finite(point, label=label)
    return point  # type: ignore[return-value]


def _point2(value: Sequence[float], *, label: str) -> Point2D:
    if len(value) != 2:
        raise ValueError(f"{label} must contain x and y")
    point = tuple(float(item) for item in value)
    _finite(point, label=label)
    return point  # type: ignore[return-value]


class ResumePolicy(str, Enum):
    """What happens after a temporary subgoal completes or expires."""

    REJOIN_ROUTE = "rejoin_route"
    HOLD = "hold"
    CANCEL = "cancel"


@dataclass(frozen=True)
class RouteEdit:
    """A proposed replacement for one agent's semantic global route."""

    agent_id: str
    base_generation: int
    route_version: int
    phase: str
    waypoints: tuple[Point3D, ...]
    commit: bool = False
    source: str = "ui"

    def __post_init__(self) -> None:
        if not str(self.agent_id).strip():
            raise ValueError("agent_id must not be empty")
        if int(self.base_generation) < 0:
            raise ValueError("base_generation must be non-negative")
        if int(self.route_version) < 1:
            raise ValueError("route_version must be positive")
        if not str(self.phase).strip():
            raise ValueError("phase must not be empty")
        if not self.waypoints:
            raise ValueError("route edit must contain at least one waypoint")
        for point in self.waypoints:
            _point3(point, label="route waypoint")
        if not str(self.source).strip():
            raise ValueError("source must not be empty")

    @classmethod
    def from_points(
        cls,
        *,
        agent_id: str,
        base_generation: int,
        route_version: int,
        phase: str,
        waypoints: Sequence[Sequence[float]],
        commit: bool = False,
        source: str = "ui",
    ) -> "RouteEdit":
        return cls(
            agent_id=str(agent_id).strip(),
            base_generation=int(base_generation),
            route_version=int(route_version),
            phase=str(phase),
            waypoints=tuple(
                _point3(point, label="route waypoint") for point in waypoints
            ),
            commit=bool(commit),
            source=str(source).strip(),
        )

    def can_apply(
        self,
        *,
        current_generation: int,
        current_route_version: int,
    ) -> bool:
        """Reject stale edits and edits that do not advance the route version."""
        return (
            int(self.base_generation) == int(current_generation)
            and int(self.route_version) > int(current_route_version)
        )

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["waypoints"] = [list(point) for point in self.waypoints]
        return payload


@dataclass(frozen=True)
class SubgoalCommand:
    """A temporary local target that must rejoin a versioned global route."""

    agent_id: str
    base_generation: int
    route_version: int
    target_xy: Point2D
    issued_at_sec: float
    ttl_sec: float
    reason: str
    resume_policy: ResumePolicy = ResumePolicy.REJOIN_ROUTE
    speed_limit_mps: float | None = None
    priority: int = 0
    source: str = "ui"

    def __post_init__(self) -> None:
        if not str(self.agent_id).strip():
            raise ValueError("agent_id must not be empty")
        if int(self.base_generation) < 0:
            raise ValueError("base_generation must be non-negative")
        if int(self.route_version) < 1:
            raise ValueError("route_version must be positive")
        _point2(self.target_xy, label="subgoal target")
        _finite((self.issued_at_sec, self.ttl_sec), label="subgoal timing")
        if float(self.ttl_sec) <= 0.0:
            raise ValueError("ttl_sec must be positive")
        if not str(self.reason).strip():
            raise ValueError("reason must not be empty")
        if int(self.priority) < 0:
            raise ValueError("priority must be non-negative")
        if self.speed_limit_mps is not None:
            _finite((self.speed_limit_mps,), label="speed_limit_mps")
            if float(self.speed_limit_mps) < 0.0:
                raise ValueError("speed_limit_mps must be non-negative")
        if not isinstance(self.resume_policy, ResumePolicy):
            raise ValueError("resume_policy must be a ResumePolicy")
        if not str(self.source).strip():
            raise ValueError("source must not be empty")

    @classmethod
    def create(
        cls,
        *,
        agent_id: str,
        base_generation: int,
        route_version: int,
        target_xy: Sequence[float],
        issued_at_sec: float,
        ttl_sec: float,
        reason: str,
        resume_policy: ResumePolicy = ResumePolicy.REJOIN_ROUTE,
        speed_limit_mps: float | None = None,
        priority: int = 0,
        source: str = "ui",
    ) -> "SubgoalCommand":
        return cls(
            agent_id=str(agent_id).strip(),
            base_generation=int(base_generation),
            route_version=int(route_version),
            target_xy=_point2(target_xy, label="subgoal target"),
            issued_at_sec=float(issued_at_sec),
            ttl_sec=float(ttl_sec),
            reason=str(reason).strip(),
            resume_policy=resume_policy,
            speed_limit_mps=(
                None if speed_limit_mps is None else float(speed_limit_mps)
            ),
            priority=int(priority),
            source=str(source).strip(),
        )

    def can_apply(
        self,
        *,
        current_generation: int,
        current_route_version: int,
    ) -> bool:
        """A subgoal belongs to exactly one current route generation/version."""
        return (
            int(self.base_generation) == int(current_generation)
            and int(self.route_version) == int(current_route_version)
        )

    def expires_at_sec(self) -> float:
        return float(self.issued_at_sec) + float(self.ttl_sec)

    def is_expired(self, now_sec: float) -> bool:
        _finite((now_sec,), label="now_sec")
        return float(now_sec) >= self.expires_at_sec()

    def to_dict(self) -> dict[str, Any]:
        payload = asdict(self)
        payload["target_xy"] = list(self.target_xy)
        payload["resume_policy"] = self.resume_policy.value
        return payload

