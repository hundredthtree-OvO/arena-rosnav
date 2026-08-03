from toilet_benchmark.episodes.schema import (
    PedestrianBehaviorSpec,
    PedestrianEpisodeSpec,
    PedestrianHoldSpec,
)
from toilet_benchmark.tracks.authored_scenario_core import (
    RouteRuntime,
    expand_authored_route,
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
    runtime.state = "MOVING"
    assert runtime.arrive(10.0) == "hold"
    assert runtime.release_hold(11.9) is None
    assert runtime.release_hold(12.0) == "dispatch"
    assert runtime.segment() == ((1.0, 0.0, 0.0), (2.0, 0.0, 0.0))
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
