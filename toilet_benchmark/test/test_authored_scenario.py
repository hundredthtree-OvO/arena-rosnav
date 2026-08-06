import math

import pytest

from toilet_benchmark.episodes.schema import (
    CollisionPolicy,
    EpisodeSpec,
    PedestrianBehaviorSpec,
    PedestrianEpisodeSpec,
    PedestrianHoldSpec,
    RobotEpisodeSpec,
    TerminationSpec,
    TrackType,
)
from toilet_benchmark.tracks.authored_scenario_core import (
    RouteRuntime,
    activation_gate_status,
    expand_authored_route,
    initial_character_root_yaw,
    route_heading_to_character_root_yaw,
    with_agent_id_suffix,
)


def _spec(*, terminal_hold: bool = False) -> PedestrianEpisodeSpec:
    holds = [PedestrianHoldSpec(1, 2.0)]
    if terminal_hold:
        holds.append(PedestrianHoldSpec(2, 1.0))
    return PedestrianEpisodeSpec(
        agent_id="agent_01",
        semantic_goal="route_terminal",
        start_pose=(0.0, 0.0, 0.0),
        route_waypoints=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        holds=tuple(holds),
        behavior=PedestrianBehaviorSpec(walking_speed_mps=0.7),
    )


def test_route_runtime_splits_route_at_hold_and_resumes() -> None:
    runtime = RouteRuntime.create(_spec())
    assert runtime.boundary_indices == (1, 2)
    assert runtime.segment() == ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert runtime.execution_segment() == ((1.0, 0.0, 0.0),)
    runtime.state = "MOVING"
    assert runtime.arrive(10.0) == "hold"
    assert runtime.release_hold(11.9) is None
    assert runtime.release_hold(12.0) == "dispatch"
    assert runtime.segment() == ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert runtime.execution_segment() == ((2.0, 0.0, 0.0),)
    assert runtime.arrive(13.0) == "complete"


def test_terminal_hold_completes_only_after_duration() -> None:
    runtime = RouteRuntime.create(_spec(terminal_hold=True))
    runtime.state = "MOVING"
    assert runtime.arrive(0.0) == "hold"
    assert runtime.release_hold(2.0) == "dispatch"
    assert runtime.arrive(3.0) == "hold"
    assert not runtime.complete
    assert runtime.release_hold(4.0) == "complete"
    assert runtime.complete


def test_hold_at_spawn_is_rejected() -> None:
    spec = PedestrianEpisodeSpec(
        agent_id="agent_01",
        semantic_goal="route_terminal",
        route_waypoints=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
        holds=(PedestrianHoldSpec(0, 1.0),),
    )
    try:
        RouteRuntime.create(spec)
    except ValueError as exc:
        assert "spawn waypoint" in str(exc)
    else:
        raise AssertionError("expected a hold-at-spawn validation error")


def test_authored_targets_expand_through_planner_and_remap_holds() -> None:
    spec = _spec()

    def planner(start, goal):
        midpoint = (
            0.5 * (start[0] + goal[0]),
            0.5 * (start[1] + goal[1]),
            0.0,
        )
        return [list(start), list(midpoint), list(goal)]

    expanded = expand_authored_route(spec, planner)

    assert expanded.route_waypoints == (
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.5, 0.0, 0.0),
        (2.0, 0.0, 0.0),
    )
    assert expanded.holds[0].waypoint_index == 2
    assert expanded.start_yaw == pytest.approx(math.pi / 2.0)


def test_authored_route_removes_short_sharp_terminal_planner_kink() -> None:
    spec = PedestrianEpisodeSpec(
        agent_id="agent_01",
        semantic_goal="route_terminal",
        start_pose=(0.0, 0.0, 0.0),
        route_waypoints=((1.0, 0.0, 0.0),),
    )

    def planner(start, goal):
        return [start, (0.98, 0.08, 0.0), goal]

    expanded = expand_authored_route(spec, planner)

    assert expanded.route_waypoints == (
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )


def test_authored_route_removes_short_sharp_initial_planner_kink() -> None:
    spec = PedestrianEpisodeSpec(
        agent_id="agent_01",
        semantic_goal="route_terminal",
        start_pose=(0.0, 0.0, 0.0),
        route_waypoints=((1.0, 0.0, 0.0),),
    )

    def planner(start, goal):
        return [start, (0.02, 0.08, 0.0), goal]

    expanded = expand_authored_route(spec, planner)

    assert expanded.route_waypoints == (
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )


def test_authored_route_preserves_short_collinear_terminal_segment() -> None:
    spec = PedestrianEpisodeSpec(
        agent_id="agent_01",
        semantic_goal="route_terminal",
        start_pose=(0.0, 0.0, 0.0),
        route_waypoints=((1.0, 0.0, 0.0),),
    )

    def planner(start, goal):
        return [start, (0.95, 0.0, 0.0), goal]

    expanded = expand_authored_route(spec, planner)

    assert expanded.route_waypoints[-2:] == (
        (0.95, 0.0, 0.0),
        (1.0, 0.0, 0.0),
    )


def test_character_root_yaw_uses_isaac_people_heading_convention() -> None:
    assert route_heading_to_character_root_yaw(0.0) == pytest.approx(math.pi / 2.0)
    assert route_heading_to_character_root_yaw(math.pi) == pytest.approx(-math.pi / 2.0)
    assert initial_character_root_yaw(
        (1.0, 2.0, 0.0),
        ((1.0, 2.0, 0.0), (0.0, 2.0, 0.0)),
    ) == pytest.approx(-math.pi / 2.0)


def test_runtime_agent_suffix_preserves_route_and_changes_identity() -> None:
    source = _spec()
    episode = EpisodeSpec(
        episode_id="test",
        scene_id="scene",
        task_type="authored_route",
        track=TrackType.INTERACTIVE,
        seed=1,
        robot=RobotEpisodeSpec("robot", (0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
        pedestrians=(source,),
        termination=TerminationSpec(30.0, 0.2, CollisionPolicy.TERMINATE),
    )

    runtime = with_agent_id_suffix(episode, "__inc007")

    assert runtime.pedestrians[0].agent_id == "agent_01__inc007"
    assert runtime.pedestrians[0].route_waypoints == source.route_waypoints
    assert episode.pedestrians[0].agent_id == "agent_01"


def test_activation_requires_consecutive_ready_idle_stable_samples() -> None:
    runtime = RouteRuntime.create(_spec())
    metadata = {
        "embodiment_generation": "2",
        "reactivation_ready": "true",
        "motion_state": "idle",
        "yaw_rad": "0.01",
    }

    def observe(pose=(0.0, 0.0, 0.0), values=metadata):
        return runtime.observe_activation(
            pose,
            values,
            position_tolerance_m=0.15,
            position_stability_m=0.02,
            yaw_tolerance_rad=0.20,
            yaw_stability_rad=0.08,
            required_samples=4,
        )

    assert not observe()
    assert not observe((0.005, 0.0, 0.0))
    assert not observe((0.006, 0.0, 0.0))
    assert observe((0.007, 0.0, 0.0))
    assert runtime.generation == 2


def test_activation_instability_or_not_ready_restarts_barrier() -> None:
    runtime = RouteRuntime.create(_spec())
    ready = {
        "embodiment_generation": "3",
        "reactivation_ready": "true",
        "motion_state": "idle",
        "yaw_rad": "0.0",
    }
    kwargs = {
        "position_tolerance_m": 0.15,
        "position_stability_m": 0.02,
        "yaw_tolerance_rad": 0.20,
        "yaw_stability_rad": 0.08,
        "required_samples": 2,
    }

    assert not runtime.observe_activation((0.0, 0.0, 0.0), ready, **kwargs)
    assert not runtime.observe_activation((0.05, 0.0, 0.0), ready, **kwargs)
    assert runtime.activation_stable_samples == 1
    not_ready = dict(ready, reactivation_ready="false")
    assert not runtime.observe_activation((0.05, 0.0, 0.0), not_ready, **kwargs)
    assert runtime.activation_stable_samples == 0


def test_activation_rejects_stale_heading() -> None:
    runtime = RouteRuntime.create(_spec())
    metadata = {
        "embodiment_generation": "2",
        "reactivation_ready": "true",
        "motion_state": "idle",
        "yaw_rad": "-1.57",
    }

    assert not runtime.observe_activation(
        (0.0, 0.0, 0.0),
        metadata,
        position_tolerance_m=0.15,
        position_stability_m=0.02,
        yaw_tolerance_rad=0.20,
        yaw_stability_rad=0.08,
        required_samples=1,
    )


def test_activation_gate_status_explains_every_failed_condition() -> None:
    status = activation_gate_status(
        _spec(),
        (0.3, 0.0, 0.0),
        {
            "embodiment_generation": "0",
            "reactivation_ready": "false",
            "motion_state": "accepted",
            "yaw_rad": "nan",
        },
        position_tolerance_m=0.15,
        yaw_tolerance_rad=0.20,
    )

    assert not status.eligible
    assert status.reasons == (
        "generation_missing",
        "reactivation_not_ready",
        "motion_state_not_idle",
        "yaw_invalid",
        "position_out_of_tolerance",
    )
    assert status.position_error_m == pytest.approx(0.3)
