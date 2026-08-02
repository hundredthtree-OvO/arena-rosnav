"""Deterministic context policy above interchangeable local motion solvers."""

from __future__ import annotations

from dataclasses import dataclass

from .contracts import (
    BehaviorDecision,
    BehaviorMode,
    BehaviorPolicyRequest,
)


@dataclass(frozen=True)
class ContextualBehaviorConfig:
    conflict_horizon_sec: float = 2.0
    minimum_passing_clearance_m: float = 0.45
    following_speed_scale: float = 0.8

    def __post_init__(self) -> None:
        values = (
            self.conflict_horizon_sec,
            self.minimum_passing_clearance_m,
            self.following_speed_scale,
        )
        if not all(float(value) >= 0.0 for value in values):
            raise ValueError("behavior policy configuration must be non-negative")


class ContextualBehaviorPolicy:
    """Select a stable social intent; local motion remains a separate concern."""

    def __init__(self, config: ContextualBehaviorConfig = ContextualBehaviorConfig()):
        self.config = config

    def decide(self, request: BehaviorPolicyRequest) -> BehaviorDecision:
        requested = request.requested_mode
        if requested in {BehaviorMode.YIELDING, BehaviorMode.WAITING}:
            return BehaviorDecision(
                mode=requested,
                speed_scale=0.0,
                hold_position=True,
                reason="explicit task behavior",
            )
        if requested == BehaviorMode.GROUPING:
            return BehaviorDecision(
                mode=requested,
                reason=f"group={request.group_id or 'unassigned'}",
            )
        if requested in {BehaviorMode.PASSING_LEFT, BehaviorMode.PASSING_RIGHT}:
            return self._passing_decision(requested, reason="explicit passing side")
        if request.leader_id:
            return BehaviorDecision(
                mode=BehaviorMode.FOLLOWING,
                speed_scale=max(0.0, float(self.config.following_speed_scale)),
                reason=f"following {request.leader_id}",
            )
        conflict_ttc_sec = self._request_value(
            request.conflict_ttc_sec,
            request.route.diagnostics.get("predicted_conflict_horizon_sec"),
        )
        if (
            conflict_ttc_sec is None
            or conflict_ttc_sec > self.config.conflict_horizon_sec
        ):
            return BehaviorDecision(mode=BehaviorMode.WALKING)

        minimum = max(0.0, float(self.config.minimum_passing_clearance_m))
        corridor_clearance = request.route.diagnostics.get(
            "corridor_clearance_by_side_m", {}
        )
        left = self._request_value(
            request.left_clearance_m,
            corridor_clearance.get(1) if hasattr(corridor_clearance, "get") else None,
        )
        right = self._request_value(
            request.right_clearance_m,
            corridor_clearance.get(-1) if hasattr(corridor_clearance, "get") else None,
        )
        candidates = []
        if left is not None and left >= minimum:
            candidates.append((float(left), BehaviorMode.PASSING_LEFT))
        if right is not None and right >= minimum:
            candidates.append((float(right), BehaviorMode.PASSING_RIGHT))
        if not candidates:
            return BehaviorDecision(
                mode=BehaviorMode.YIELDING,
                speed_scale=0.0,
                hold_position=True,
                reason="conflict has no feasible passing corridor",
            )
        # Prefer larger clearance; left is the stable tie-breaker.
        _clearance, mode = max(
            candidates,
            key=lambda item: (item[0], item[1] == BehaviorMode.PASSING_LEFT),
        )
        return self._passing_decision(mode, reason="predicted conflict")

    @staticmethod
    def _passing_decision(mode: BehaviorMode, *, reason: str) -> BehaviorDecision:
        return BehaviorDecision(
            mode=mode,
            preferred_side=1 if mode == BehaviorMode.PASSING_LEFT else -1,
            reason=reason,
        )

    @staticmethod
    def _request_value(explicit: float | None, fallback: object) -> float | None:
        if explicit is not None:
            return float(explicit)
        if fallback is None:
            return None
        return float(fallback)
