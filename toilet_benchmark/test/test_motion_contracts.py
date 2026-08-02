import math

import pytest

from toilet_benchmark.domain import AgentSnapshot
from toilet_benchmark.motion import (
    BehaviorDecision,
    BehaviorMode,
    BehaviorPolicyRequest,
    GeometrySafetyRequest,
    GeometrySafetyResult,
    LocalMotionRequest,
    LocalMotionResult,
    RoutePlan,
    SweptEnvelopeGeometrySafety,
)
from toilet_benchmark.motion.swept_envelope import SweptEnvelope, SweptEnvelopeConfig


def _agent(agent_id="agent_01"):
    return AgentSnapshot(
        agent_id=agent_id,
        x=0.0,
        y=0.0,
        z=0.0,
        yaw=0.0,
        timestamp_sec=1.0,
        source="test",
    )


def test_route_plan_freezes_points_and_preserves_planner_metadata():
    plan = RoutePlan.from_points(
        agent_id="agent_01",
        points=[[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]],
        planner_id="walkable_theta_star",
        map_version="scene-123",
        dynamic_obstacle_count=2,
    )

    assert plan.points == ((0.0, 0.0, 0.0), (1.0, 2.0, 0.0))
    assert plan.planner_id == "walkable_theta_star"
    assert plan.map_version == "scene-123"
    assert plan.dynamic_obstacle_count == 2


def test_behavior_hold_keeps_semantic_route_available_for_resume():
    route = RoutePlan.from_points(
        agent_id="agent_01",
        points=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        planner_id="test",
    )
    request = BehaviorPolicyRequest(
        timestamp_sec=1.0,
        agent=_agent(),
        route=route,
        preferred_speed_mps=0.8,
    )
    decision = BehaviorDecision(
        mode=BehaviorMode.YIELDING,
        speed_scale=0.0,
        hold_position=True,
        reason="doorway occupied",
    )

    assert request.route is route
    assert decision.hold_position is True
    assert decision.speed_scale == 0.0


def test_local_motion_result_rejects_non_finite_velocity():
    with pytest.raises(ValueError, match="finite"):
        LocalMotionResult(
            velocity_xy=(math.nan, 0.0),
            heading_rad=0.0,
            feasible=True,
        )


def test_local_motion_request_uses_one_timestamped_world_snapshot():
    route = RoutePlan.from_points(
        agent_id="agent_01",
        points=[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        planner_id="test",
    )
    behavior = BehaviorDecision(mode=BehaviorMode.WALKING)

    request = LocalMotionRequest(
        timestamp_sec=1.0,
        dt_sec=0.1,
        agent=_agent(),
        peers=(_agent("agent_02"),),
        robot=None,
        route=route,
        behavior=behavior,
        preferred_speed_mps=0.8,
    )

    assert request.agent.timestamp_sec == request.timestamp_sec
    assert request.peers[0].timestamp_sec == request.timestamp_sec


def test_geometry_safety_result_bounds_applied_fraction():
    request = GeometrySafetyRequest(
        start_xy=(0.0, 0.0),
        proposed_xy=(1.0, 0.0),
        start_yaw=0.0,
        proposed_yaw=0.0,
    )
    result = GeometrySafetyResult(
        position_xy=(0.4, 0.0),
        applied_fraction=0.4,
        clipped=True,
        minimum_clearance_m=0.0,
    )

    assert request.proposed_xy == (1.0, 0.0)
    assert result.applied_fraction == 0.4

    blocked = GeometrySafetyResult(
        position_xy=(0.0, 0.0),
        applied_fraction=0.0,
        clipped=True,
        minimum_clearance_m=-math.inf,
    )
    assert blocked.minimum_clearance_m == -math.inf

    with pytest.raises(ValueError, match="between 0 and 1"):
        GeometrySafetyResult(
            position_xy=(1.1, 0.0),
            applied_fraction=1.1,
            clipped=False,
            minimum_clearance_m=0.1,
        )


def test_swept_envelope_adapter_projects_without_choosing_a_detour():
    class Planner:
        @staticmethod
        def clearance_at(point):
            return 1.0 if point[0] <= 0.5 else 0.0

    safety = SweptEnvelopeGeometrySafety(
        SweptEnvelope(
            SweptEnvelopeConfig(
                disc_radius_m=0.1,
                half_length_m=0.0,
                clearance_m=0.0,
                sweep_sample_spacing_m=0.02,
            )
        )
    )
    safety.bind(Planner())

    result = safety.project(
        GeometrySafetyRequest(
            start_xy=(0.0, 0.0),
            proposed_xy=(1.0, 0.0),
            start_yaw=0.0,
            proposed_yaw=0.0,
        )
    )

    assert result.clipped is True
    assert 0.0 < result.applied_fraction < 1.0
    assert result.position_xy[1] == 0.0
