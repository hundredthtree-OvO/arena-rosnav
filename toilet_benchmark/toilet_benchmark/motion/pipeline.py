"""Simulator-neutral composition of behavior policy and local motion."""

from __future__ import annotations

from dataclasses import dataclass

from .contracts import (
    BehaviorDecision,
    BehaviorPolicyRequest,
    LocalMotionRequest,
    LocalMotionResult,
    Point2D,
)
from .ports import BehaviorPolicyPort, LocalMotionPort


@dataclass(frozen=True)
class LocalMotionTickResult:
    behavior: BehaviorDecision
    motion: LocalMotionResult


class LocalMotionPipeline:
    """Own one local-motion tick without depending on HuNav or Isaac."""

    def __init__(
        self,
        behavior_policy: BehaviorPolicyPort,
        local_motion: LocalMotionPort,
    ):
        self.behavior_policy = behavior_policy
        self.local_motion = local_motion

    def step(
        self,
        request: BehaviorPolicyRequest,
        *,
        dt_sec: float,
        route_target_xy: Point2D | None = None,
    ) -> LocalMotionTickResult:
        behavior = self.behavior_policy.decide(request)
        motion = self.local_motion.step(
            LocalMotionRequest(
                timestamp_sec=request.timestamp_sec,
                dt_sec=float(dt_sec),
                agent=request.agent,
                peers=request.peers,
                robot=request.robot,
                route=request.route,
                behavior=behavior,
                preferred_speed_mps=request.preferred_speed_mps,
                route_target_xy=route_target_xy,
            )
        )
        return LocalMotionTickResult(behavior=behavior, motion=motion)
