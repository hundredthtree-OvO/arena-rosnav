"""Deterministic event-level coordination policies for Phase 0 scenarios."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Mapping, Tuple


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class BottleneckSpec:
    zone_x_min: float
    zone_x_max: float
    release_clearance_m: float = 0.15
    priority_order: Tuple[int, ...] = ()


class DeterministicBottleneckCoordinator:
    """Serialize agents through a capacity-one x-axis conflict zone."""

    def __init__(
        self,
        *,
        routes: Mapping[int, Tuple[Point2D, Point2D]],
        spec: BottleneckSpec,
    ):
        self._routes = dict(routes)
        self._spec = spec
        priority = spec.priority_order or tuple(sorted(routes))
        if set(priority) != set(routes):
            raise ValueError("Bottleneck priority_order must contain every route id")
        self._queue = list(priority)
        self._active_index = 0
        self._entered: Dict[int, bool] = {agent_id: False for agent_id in routes}

    @property
    def active_agent_id(self) -> int | None:
        if self._active_index >= len(self._queue):
            return None
        return self._queue[self._active_index]

    def update(self, positions: Mapping[int, Point2D]) -> None:
        owner = self.active_agent_id
        if owner is None:
            return
        start, goal = self._routes[owner]
        direction = 1.0 if goal[0] >= start[0] else -1.0
        x = float(positions[owner][0])
        if direction > 0.0:
            self._entered[owner] = self._entered[owner] or x >= self._spec.zone_x_min
            cleared = x >= self._spec.zone_x_max + self._spec.release_clearance_m
        else:
            self._entered[owner] = self._entered[owner] or x <= self._spec.zone_x_max
            cleared = x <= self._spec.zone_x_min - self._spec.release_clearance_m
        if self._entered[owner] and cleared:
            self._active_index += 1

    def held_agent_ids(self) -> Tuple[int, ...]:
        if self.active_agent_id is None:
            return ()
        return tuple(self._queue[self._active_index + 1 :])

    def status(self) -> Dict[str, object]:
        return {
            "enabled": True,
            "active_agent_id": self.active_agent_id,
            "held_agent_ids": list(self.held_agent_ids()),
            "completed_agent_ids": self._queue[: self._active_index],
        }
