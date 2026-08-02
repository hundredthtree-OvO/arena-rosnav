import importlib
import math

import pytest

from toilet_benchmark.domain import AgentSnapshot
from toilet_benchmark.motion import (
    BehaviorDecision,
    BehaviorMode,
    GeometrySafetyRequest,
    LocalMotionRequest,
    RoutePlan,
)
from toilet_benchmark.motion.swept_envelope import SweptEnvelope, SweptEnvelopeConfig


def _motion_api():
    module = importlib.import_module("toilet_benchmark.motion")
    return (
        getattr(module, "SampledRvoLocalMotion"),
        getattr(module, "SampledRvoConfig"),
        getattr(module, "SweptEnvelopeGeometrySafety"),
    )


def _agent(
    agent_id: str,
    *,
    x: float,
    y: float,
    yaw: float = 0.0,
    vx: float = 0.0,
    vy: float = 0.0,
    radius_m: float = 0.30,
    timestamp_sec: float = 10.0,
) -> AgentSnapshot:
    return AgentSnapshot(
        agent_id=agent_id,
        x=x,
        y=y,
        z=0.0,
        yaw=yaw,
        vx=vx,
        vy=vy,
        radius_m=radius_m,
        timestamp_sec=timestamp_sec,
        source="test",
    )


def _route(
    agent_id: str,
    *,
    start: tuple[float, float],
    points: tuple[tuple[float, float], ...],
    planner_id: str = "test",
) -> RoutePlan:
    return RoutePlan.from_points(
        agent_id=agent_id,
        points=((start[0], start[1], 0.0),) + tuple((x, y, 0.0) for x, y in points),
        planner_id=planner_id,
    )


def _request(
    *,
    agent: AgentSnapshot,
    peers: tuple[AgentSnapshot, ...] = (),
    behavior: BehaviorDecision,
    route_points: tuple[tuple[float, float], ...] = ((2.0, 0.0),),
    dt_sec: float = 0.2,
) -> LocalMotionRequest:
    return LocalMotionRequest(
        timestamp_sec=agent.timestamp_sec,
        dt_sec=dt_sec,
        agent=agent,
        peers=peers,
        robot=None,
        route=_route(
            agent.agent_id,
            start=(agent.x, agent.y),
            points=route_points,
        ),
        behavior=behavior,
        preferred_speed_mps=0.8,
    )


class _TopWallPlanner:
    def __init__(self, top_y: float):
        self.top_y = float(top_y)

    def clearance_at(self, point):
        return self.top_y - float(point[1])


class _DoorwayCorridorPlanner:
    def clearance_at(self, point):
        x = float(point[0])
        y = float(point[1])
        top = 0.18 if x < 0.5 else 0.35
        bottom = -0.35
        return min(top - y, y - bottom)


def _geometry_safety(planner):
    _, _, SweptEnvelopeGeometrySafety = _motion_api()
    safety = SweptEnvelopeGeometrySafety(
        SweptEnvelope(
            SweptEnvelopeConfig(
                disc_radius_m=0.10,
                half_length_m=0.0,
                clearance_m=0.01,
                sweep_sample_spacing_m=0.01,
            )
        )
    )
    safety.bind(planner)
    return safety


def test_adaptive_boundary_candidates_keep_head_on_agent_moving():
    SampledRvoLocalMotion, SampledRvoConfig, _ = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=1.2, y=0.0, vx=-0.8, vy=0.0)

    result = planner.step(
        _request(
            agent=agent,
            peers=(peer,),
            behavior=BehaviorDecision(
                mode=BehaviorMode.PASSING_LEFT,
                preferred_side=1,
            ),
        )
    )

    assert result.diagnostics["boundary_candidate_count"] > 0
    assert result.feasible is True
    assert result.velocity_xy[0] > 0.0
    assert result.velocity_xy[1] > 0.0


def test_geometry_prefilter_rejects_blocked_side_without_backtracking():
    SampledRvoLocalMotion, SampledRvoConfig, _ = _motion_api()
    planner = SampledRvoLocalMotion(
        SampledRvoConfig(max_acceleration_mps2=20.0),
        geometry_safety=_geometry_safety(_TopWallPlanner(0.18)),
    )
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=1.2, y=0.0, vx=-0.8, vy=0.0)

    result = planner.step(
        _request(
            agent=agent,
            peers=(peer,),
            behavior=BehaviorDecision(
                mode=BehaviorMode.PASSING_LEFT,
                preferred_side=1,
            ),
        )
    )

    assert result.feasible is True
    assert result.diagnostics["static_rejected_count"] > 0
    assert result.diagnostics["minimum_static_clearance_m"] > 0.0
    assert result.velocity_xy[1] < 0.0
    assert result.velocity_xy[0] > 0.0


def test_velocity_continuity_keeps_lateral_commitment_under_small_static_jitter():
    SampledRvoLocalMotion, SampledRvoConfig, _ = _motion_api()
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=1.2, y=0.0, vx=-0.8, vy=0.0)
    request = _request(
        agent=agent,
        peers=(peer,),
        behavior=BehaviorDecision(
            mode=BehaviorMode.PASSING_LEFT,
            preferred_side=1,
        ),
    )
    blocked = SampledRvoLocalMotion(
        SampledRvoConfig(max_acceleration_mps2=20.0),
        geometry_safety=_geometry_safety(_TopWallPlanner(0.18)),
    ).step(request)
    continued_agent = _agent(
        "agent_01",
        x=blocked.velocity_xy[0] * 0.2,
        y=blocked.velocity_xy[1] * 0.2,
        yaw=blocked.heading_rad,
        vx=blocked.velocity_xy[0],
        vy=blocked.velocity_xy[1],
        timestamp_sec=10.2,
    )
    continued_peer = _agent(
        "agent_02",
        x=1.2 - 0.8 * 0.2,
        y=0.0,
        vx=-0.8,
        vy=0.0,
        timestamp_sec=10.2,
    )
    widened = SampledRvoLocalMotion(
        SampledRvoConfig(max_acceleration_mps2=20.0),
        geometry_safety=_geometry_safety(_TopWallPlanner(0.25)),
    ).step(
        _request(
            agent=continued_agent,
            peers=(continued_peer,),
            behavior=BehaviorDecision(
                mode=BehaviorMode.PASSING_RIGHT,
                preferred_side=-1,
            ),
        )
    )

    assert blocked.feasible is True
    assert widened.feasible is True
    assert math.copysign(1.0, blocked.velocity_xy[1]) == math.copysign(
        1.0, widened.velocity_xy[1]
    )
    assert math.dist(blocked.velocity_xy, widened.velocity_xy) < 0.6


def test_stationary_heading_lock_preserves_current_yaw_while_yielding():
    SampledRvoLocalMotion, SampledRvoConfig, _ = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    agent = _agent("agent_01", x=0.0, y=0.0, yaw=1.25, vx=0.2, vy=0.0)

    result = planner.step(
        _request(
            agent=agent,
            behavior=BehaviorDecision(
                mode=BehaviorMode.YIELDING,
                speed_scale=0.0,
                hold_position=True,
            ),
            route_points=((2.0, -1.0),),
        )
    )

    assert result.velocity_xy == (0.0, 0.0)
    assert result.heading_rad == pytest.approx(agent.yaw)


def test_doorway_route_keeps_forward_progress_once_corridor_is_available():
    SampledRvoLocalMotion, SampledRvoConfig, _ = _motion_api()
    planner = SampledRvoLocalMotion(
        SampledRvoConfig(max_acceleration_mps2=20.0),
        geometry_safety=_geometry_safety(_DoorwayCorridorPlanner()),
    )
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=1.2, y=0.0, vx=-0.8, vy=0.0)

    result = planner.step(
        _request(
            agent=agent,
            peers=(peer,),
            behavior=BehaviorDecision(
                mode=BehaviorMode.PASSING_LEFT,
                preferred_side=1,
            ),
            route_points=((0.5, 0.0), (1.0, 0.0), (2.0, 0.0)),
        )
    )

    assert result.feasible is True
    assert result.velocity_xy[0] > 0.0
    assert abs(result.velocity_xy[1]) < 0.8
