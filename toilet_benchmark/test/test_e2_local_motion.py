import importlib
import math

import pytest

from toilet_benchmark.domain import AgentSnapshot
from toilet_benchmark.motion import (
    BehaviorMode,
    BehaviorPolicyRequest,
    LocalMotionRequest,
    RoutePlan,
)


def _motion_api():
    module = importlib.import_module("toilet_benchmark.motion")
    return (
        getattr(module, "ContextualBehaviorPolicy"),
        getattr(module, "ContextualBehaviorConfig"),
        getattr(module, "SampledRvoLocalMotion"),
        getattr(module, "SampledRvoConfig"),
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
    body_half_length_m: float = 0.0,
    box_half_length_m: float = 0.0,
    box_half_width_m: float = 0.0,
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
        body_half_length_m=body_half_length_m,
        box_half_length_m=box_half_length_m,
        box_half_width_m=box_half_width_m,
        timestamp_sec=10.0,
        source="test",
    )


def _route(
    agent_id: str,
    *,
    start: tuple[float, float] = (0.0, 0.0),
    goal: tuple[float, float] = (2.0, 0.0),
) -> RoutePlan:
    return RoutePlan.from_points(
        agent_id=agent_id,
        points=((*start, 0.0), (*goal, 0.0)),
        planner_id="test",
    )


def _behavior_request(
    *,
    agent: AgentSnapshot,
    peers: tuple[AgentSnapshot, ...] = (),
    diagnostics: dict | None = None,
) -> BehaviorPolicyRequest:
    route = _route(agent.agent_id)
    if diagnostics is not None:
        route = RoutePlan(
            agent_id=route.agent_id,
            points=route.points,
            planner_id=route.planner_id,
            map_version=route.map_version,
            dynamic_obstacle_count=route.dynamic_obstacle_count,
            diagnostics=diagnostics,
        )
    return BehaviorPolicyRequest(
        timestamp_sec=agent.timestamp_sec,
        agent=agent,
        route=route,
        peers=peers,
        preferred_speed_mps=0.8,
    )


def _local_motion_request(
    *,
    agent: AgentSnapshot,
    peers: tuple[AgentSnapshot, ...] = (),
    behavior_mode: BehaviorMode = BehaviorMode.WALKING,
    hold_position: bool = False,
    preferred_side: int = 0,
    diagnostics: dict | None = None,
    route_goal: tuple[float, float] = (2.0, 0.0),
) -> LocalMotionRequest:
    _, _, _, _ = _motion_api()
    from toilet_benchmark.motion import BehaviorDecision

    route = _route(
        agent.agent_id,
        start=(agent.x, agent.y),
        goal=route_goal,
    )
    if diagnostics is not None:
        route = RoutePlan(
            agent_id=route.agent_id,
            points=route.points,
            planner_id=route.planner_id,
            map_version=route.map_version,
            dynamic_obstacle_count=route.dynamic_obstacle_count,
            diagnostics=diagnostics,
        )
    return LocalMotionRequest(
        timestamp_sec=agent.timestamp_sec,
        dt_sec=0.2,
        agent=agent,
        peers=peers,
        robot=None,
        route=route,
        behavior=BehaviorDecision(
            mode=behavior_mode,
            speed_scale=0.0 if hold_position else 1.0,
            hold_position=hold_position,
            preferred_side=preferred_side,
        ),
        preferred_speed_mps=0.8,
    )


def test_clear_route_returns_walking():
    ContextualBehaviorPolicy, ContextualBehaviorConfig, _, _ = _motion_api()
    policy = ContextualBehaviorPolicy(ContextualBehaviorConfig())

    decision = policy.decide(
        _behavior_request(
            agent=_agent("agent_01", x=0.0, y=0.0),
        )
    )

    assert decision.mode is BehaviorMode.WALKING
    assert decision.hold_position is False


def test_imminent_conflict_selects_side_with_greater_corridor_clearance():
    ContextualBehaviorPolicy, ContextualBehaviorConfig, _, _ = _motion_api()
    policy = ContextualBehaviorPolicy(ContextualBehaviorConfig())
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=0.9, y=0.0, vx=-0.8, vy=0.0)

    decision = policy.decide(
        _behavior_request(
            agent=agent,
            peers=(peer,),
            diagnostics={
                "corridor_clearance_by_side_m": {-1: 0.25, 1: 0.70},
                "predicted_conflict_horizon_sec": 1.2,
            },
        )
    )

    assert decision.mode is BehaviorMode.PASSING_LEFT
    assert decision.preferred_side == 1


def test_no_passable_side_yields_and_holds():
    ContextualBehaviorPolicy, ContextualBehaviorConfig, _, _ = _motion_api()
    policy = ContextualBehaviorPolicy(ContextualBehaviorConfig())
    agent = _agent("agent_01", x=0.0, y=0.0, vx=0.8, vy=0.0)
    peer = _agent("agent_02", x=0.7, y=0.0, vx=-0.8, vy=0.0)

    decision = policy.decide(
        _behavior_request(
            agent=agent,
            peers=(peer,),
            diagnostics={
                "corridor_clearance_by_side_m": {-1: 0.0, 1: 0.0},
                "predicted_conflict_horizon_sec": 0.8,
            },
        )
    )

    assert decision.mode is BehaviorMode.YIELDING
    assert decision.hold_position is True


def test_hold_behavior_outputs_zero_velocity():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())

    result = planner.step(
        _local_motion_request(
            agent=_agent("agent_01", x=0.0, y=0.0),
            hold_position=True,
            behavior_mode=BehaviorMode.YIELDING,
        )
    )

    assert result.velocity_xy == (0.0, 0.0)


def test_hard_dynamic_clearance_uses_capsule_against_oriented_robot_box():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig(clearance_m=0.0))
    pedestrian = _agent(
        "agent_01",
        x=0.0,
        y=0.67,
        yaw=-0.5 * math.pi,
        radius_m=0.22,
        body_half_length_m=0.16,
    )
    robot = _agent(
        "robot",
        x=0.0,
        y=0.0,
        radius_m=0.0,
        box_half_length_m=0.40,
        box_half_width_m=0.30,
    )

    clearance = planner._pair_clearance(
        pedestrian,
        (pedestrian.x, pedestrian.y),
        pedestrian.yaw,
        robot,
        (robot.x, robot.y),
        robot.yaw,
    )

    assert clearance < 0.0


def test_head_on_two_agents_produce_non_colliding_side_biased_velocities():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    left = planner.step(
        _local_motion_request(
            agent=_agent("agent_01", x=-0.6, y=0.0, vx=0.8, vy=0.0),
            peers=(_agent("agent_02", x=0.6, y=0.0, vx=-0.8, vy=0.0),),
            preferred_side=1,
            diagnostics={"neighbor_radius_m": 1.5},
        )
    )
    right = planner.step(
        _local_motion_request(
            agent=_agent("agent_02", x=0.6, y=0.0, vx=-0.8, vy=0.0),
            peers=(_agent("agent_01", x=-0.6, y=0.0, vx=0.8, vy=0.0),),
            preferred_side=1,
            route_goal=(-2.0, 0.0),
            diagnostics={"neighbor_radius_m": 1.5},
        )
    )

    next_left = (
        -0.6 + left.velocity_xy[0] * 0.2,
        0.0 + left.velocity_xy[1] * 0.2,
    )
    next_right = (
        0.6 + right.velocity_xy[0] * 0.2,
        0.0 + right.velocity_xy[1] * 0.2,
    )

    assert left.velocity_xy[1] > 0.0
    assert right.velocity_xy[1] < 0.0
    assert math.dist(next_left, next_right) > 0.60


def test_crossing_avoids_predicted_collision():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    result = planner.step(
        _local_motion_request(
            agent=_agent("agent_01", x=-0.7, y=0.0, vx=0.8, vy=0.0),
            peers=(_agent("agent_02", x=0.0, y=-0.7, vx=0.0, vy=0.8),),
            diagnostics={"prediction_horizon_sec": 1.0},
        )
    )

    direct_collision_velocity = (0.8, 0.0)
    assert result.velocity_xy != direct_collision_velocity
    assert result.feasible is True
    assert result.diagnostics["avoidance_active"] is True
    assert result.diagnostics["velocity_deviation_mps"] > 0.05
    assert result.diagnostics["minimum_dynamic_clearance_m"] >= 0.0


def test_impossible_encirclement_returns_zero_and_infeasible():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    result = planner.step(
        _local_motion_request(
            agent=_agent("agent_01", x=0.0, y=0.0),
            peers=(
                _agent("agent_02", x=0.35, y=0.0),
                _agent("agent_03", x=-0.35, y=0.0),
                _agent("agent_04", x=0.0, y=0.35),
                _agent("agent_05", x=0.0, y=-0.35),
            ),
            diagnostics={"prediction_horizon_sec": 1.5},
        )
    )

    assert result.velocity_xy == (0.0, 0.0)
    assert result.feasible is False


def test_non_overlapping_blocker_yields_instead_of_reversing():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(
        SampledRvoConfig(
            angular_samples=8,
            minimum_forward_speed_fraction=0.8,
        )
    )

    result = planner.step(
        _local_motion_request(
            agent=_agent("agent_01", x=0.0, y=0.0, vx=0.8),
            peers=(_agent("agent_02", x=0.8, y=0.0),),
        )
    )

    assert result.velocity_xy == (0.0, 0.0)
    assert result.feasible is False
    assert result.diagnostics["yielding_without_reverse"] is True


def test_existing_overlap_allows_only_route_compatible_separation():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    right_agent = _agent("agent_01", x=2.65, y=0.58)
    left_agent = _agent("agent_04", x=2.24, y=0.92)

    left_result = planner.step(
        _local_motion_request(
            agent=left_agent,
            peers=(right_agent,),
            route_goal=(-3.8, -0.91),
        )
    )
    right_result = planner.step(
        _local_motion_request(
            agent=right_agent,
            peers=(left_agent,),
            route_goal=(-3.8, -0.91),
        )
    )

    assert left_result.feasible is True
    assert left_result.velocity_xy[0] < 0.0
    assert left_result.diagnostics["overlap_recovery_neighbor_count"] == 1
    assert right_result.feasible is False
    assert right_result.velocity_xy == (0.0, 0.0)


def test_repeated_identical_input_returns_identical_result():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    request = _local_motion_request(
        agent=_agent("agent_01", x=-0.4, y=0.0, vx=0.8, vy=0.0),
        peers=(_agent("agent_02", x=0.4, y=0.0, vx=-0.8, vy=0.0),),
        preferred_side=1,
        diagnostics={"prediction_horizon_sec": 1.0},
    )

    first = planner.step(request)
    second = planner.step(request)

    assert first == second


def test_partial_static_overlap_recovery_is_not_executed_as_a_full_step():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    from toilet_benchmark.motion import GeometrySafetyResult

    class PartialRecoveryGeometry:
        def project(self, request):
            del request
            return GeometrySafetyResult(
                position_xy=(0.01, 0.0),
                applied_fraction=0.25,
                clipped=True,
                minimum_clearance_m=-0.05,
                recovering_overlap=True,
            )

    planner = SampledRvoLocalMotion(
        SampledRvoConfig(),
        geometry_safety=PartialRecoveryGeometry(),
    )

    result = planner.step(
        _local_motion_request(agent=_agent("agent_01", x=0.0, y=0.0))
    )

    assert result.feasible is False
    assert result.velocity_xy == (0.0, 0.0)
    assert result.diagnostics["static_rejected_count"] > 0


def test_invalid_solver_configuration_is_rejected():
    _, _, _, SampledRvoConfig = _motion_api()

    with pytest.raises(ValueError, match="angular_samples"):
        SampledRvoConfig(angular_samples=4)


def test_head_on_side_commitment_remains_collision_free_over_time():
    _, _, SampledRvoLocalMotion, SampledRvoConfig = _motion_api()
    planner = SampledRvoLocalMotion(SampledRvoConfig())
    first = _agent("agent_01", x=-2.0, y=0.0)
    second = _agent("agent_02", x=2.0, y=0.0)
    minimum_distance = math.inf

    for step in range(50):
        first_result = planner.step(
            _local_motion_request(
                agent=first,
                peers=(second,),
                preferred_side=1,
                route_goal=(2.0, 0.0),
            )
        )
        second_result = planner.step(
            _local_motion_request(
                agent=second,
                peers=(first,),
                preferred_side=1,
                route_goal=(-2.0, 0.0),
            )
        )
        first = _agent(
            "agent_01",
            x=first.x + first_result.velocity_xy[0] * 0.1,
            y=first.y + first_result.velocity_xy[1] * 0.1,
            vx=first_result.velocity_xy[0],
            vy=first_result.velocity_xy[1],
        )
        second = _agent(
            "agent_02",
            x=second.x + second_result.velocity_xy[0] * 0.1,
            y=second.y + second_result.velocity_xy[1] * 0.1,
            vx=second_result.velocity_xy[0],
            vy=second_result.velocity_xy[1],
        )
        minimum_distance = min(
            minimum_distance,
            math.dist((first.x, first.y), (second.x, second.y)),
        )

    assert minimum_distance >= first.radius_m + second.radius_m
    assert first.x > 1.5
    assert second.x < -1.5
