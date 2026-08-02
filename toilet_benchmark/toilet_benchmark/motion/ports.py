"""Authority boundaries for interchangeable E2 motion implementations."""

from __future__ import annotations

from typing import Protocol

from .contracts import (
    BehaviorDecision,
    BehaviorPolicyRequest,
    GeometrySafetyRequest,
    GeometrySafetyResult,
    LocalMotionRequest,
    LocalMotionResult,
    RoutePlan,
)


class GlobalRouterPort(Protocol):
    def plan(self, request) -> RoutePlan | None:
        ...


class BehaviorPolicyPort(Protocol):
    def decide(self, request: BehaviorPolicyRequest) -> BehaviorDecision:
        ...


class LocalMotionPort(Protocol):
    def step(self, request: LocalMotionRequest) -> LocalMotionResult:
        ...


class GeometrySafetyPort(Protocol):
    def project(self, request: GeometrySafetyRequest) -> GeometrySafetyResult:
        ...
