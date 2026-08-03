import math

import pytest

from toilet_benchmark.motion import ResumePolicy, RouteEdit, SubgoalCommand


def test_route_edit_requires_new_version_and_matching_generation():
    edit = RouteEdit.from_points(
        agent_id="toilet_agent_01",
        base_generation=3,
        route_version=8,
        phase="EXITING",
        waypoints=[[-2.0, -0.9, 0.0], [-3.8, -0.9, 0.0]],
    )

    assert edit.can_apply(current_generation=3, current_route_version=7)
    assert not edit.can_apply(current_generation=2, current_route_version=7)
    assert not edit.can_apply(current_generation=3, current_route_version=8)
    assert edit.to_dict()["waypoints"][0] == [-2.0, -0.9, 0.0]


def test_route_edit_rejects_empty_or_non_finite_waypoints():
    with pytest.raises(ValueError, match="at least one waypoint"):
        RouteEdit.from_points(
            agent_id="agent_01",
            base_generation=1,
            route_version=1,
            phase="WALK_TO_URINAL",
            waypoints=[],
        )

    with pytest.raises(ValueError, match="finite"):
        RouteEdit.from_points(
            agent_id="agent_01",
            base_generation=1,
            route_version=1,
            phase="WALK_TO_URINAL",
            waypoints=[[math.nan, 0.0, 0.0]],
        )


def test_subgoal_is_bound_to_current_route_and_expires_deterministically():
    command = SubgoalCommand.create(
        agent_id="toilet_agent_01",
        base_generation=3,
        route_version=8,
        target_xy=[-2.4, -0.7],
        issued_at_sec=10.0,
        ttl_sec=2.5,
        reason="pass_robot_left",
        resume_policy=ResumePolicy.REJOIN_ROUTE,
        speed_limit_mps=0.55,
        priority=2,
    )

    assert command.can_apply(current_generation=3, current_route_version=8)
    assert not command.can_apply(current_generation=3, current_route_version=7)
    assert not command.is_expired(12.49)
    assert command.is_expired(12.5)
    assert command.to_dict()["resume_policy"] == "rejoin_route"


def test_subgoal_rejects_invalid_lifetime_and_reason():
    with pytest.raises(ValueError, match="ttl_sec must be positive"):
        SubgoalCommand.create(
            agent_id="agent_01",
            base_generation=1,
            route_version=1,
            target_xy=[0.0, 0.0],
            issued_at_sec=0.0,
            ttl_sec=0.0,
            reason="wait",
        )

    with pytest.raises(ValueError, match="reason must not be empty"):
        SubgoalCommand.create(
            agent_id="agent_01",
            base_generation=1,
            route_version=1,
            target_xy=[0.0, 0.0],
            issued_at_sec=0.0,
            ttl_sec=1.0,
            reason=" ",
        )

