import json
import math

from toilet_benchmark.animgraph_phase_probe import (
    ProbeSample,
    analyze_probe_samples,
    build_probe_episode,
    time_aligned_root_rmse,
)
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


def _episode() -> EpisodeSpec:
    return EpisodeSpec(
        episode_id="probe_source",
        scene_id="scene",
        task_type="authored_route",
        track=TrackType.INTERACTIVE,
        seed=1,
        robot=RobotEpisodeSpec(
            model="robot",
            start_pose=(0.0, -1.0, 0.0, 0.0),
            goal_pose=(0.0, -1.0, 0.0),
        ),
        pedestrians=(
            PedestrianEpisodeSpec(
                agent_id="agent",
                semantic_goal="route_terminal",
                start_pose=(0.0, 0.0, 0.0),
                route_waypoints=((1.0, 0.0, 0.0), (2.0, 1.0, 0.0)),
                holds=(PedestrianHoldSpec(0, 1.0),),
                behavior=PedestrianBehaviorSpec(walking_speed_mps=0.8),
            ),
        ),
        termination=TerminationSpec(
            timeout_sec=30.0,
            goal_tolerance_m=0.2,
            collision_policy=CollisionPolicy.TERMINATE,
        ),
        assets={"walkable_map": "/tmp/map.json"},
    )


def test_probe_cases_keep_only_the_requested_motion_boundary() -> None:
    source = _episode()
    straight = build_probe_episode(
        source, case="straight", source_agent_id="agent", runtime_agent_id="fresh_01"
    )
    stop_resume = build_probe_episode(
        source, case="stop_resume", source_agent_id="agent", runtime_agent_id="agent"
    )

    assert straight.pedestrians[0].agent_id == "fresh_01"
    assert straight.pedestrians[0].route_waypoints == ((2.0, 1.0, 0.0),)
    assert straight.pedestrians[0].holds == ()
    assert len(stop_resume.pedestrians[0].holds) == 1


def test_probe_episode_serializes_through_the_episode_schema() -> None:
    probe = build_probe_episode(
        _episode(),
        case="turn",
        source_agent_id="agent",
        runtime_agent_id="agent",
    )

    payload = json.loads(json.dumps(probe.to_dict()))
    restored = EpisodeSpec.from_mapping(payload)

    assert restored == probe
    assert payload["pedestrians"][0]["agent_id"] == "agent"


def test_probe_metrics_measure_first_window_lateral_error() -> None:
    samples = [
        ProbeSample(0.0, 0.0, 0.0, 0.0, 0.0, "idle", 2, 3),
        ProbeSample(0.2, 0.1, 0.02, 0.0, 0.5, "executing", 2, 4),
        ProbeSample(1.0, 0.5, 0.08, 0.1, 0.5, "executing", 2, 4),
        ProbeSample(2.1, 1.0, 0.12, 0.2, 0.4, "executing", 2, 4),
        ProbeSample(2.4, 1.2, 0.1, 0.1, 0.0, "idle", 2, 5),
    ]

    result = analyze_probe_samples(samples, ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)))

    assert result["valid"]
    assert math.isclose(result["first_2s_max_lateral_m"], 0.12)
    assert result["generation"] == 2
    assert result["animgraph_clip_phase_observable"] is False


def test_time_aligned_rmse_ignores_process_start_delay() -> None:
    reference = [
        ProbeSample(1.0, 0.0, 0.0, 0.0, 0.5, "executing", 1, 1),
        ProbeSample(2.0, 1.0, 0.0, 0.0, 0.5, "executing", 1, 1),
    ]
    candidate = [
        ProbeSample(5.0, 0.0, 0.1, 0.0, 0.5, "executing", 2, 2),
        ProbeSample(6.0, 1.0, 0.1, 0.0, 0.5, "executing", 2, 2),
    ]

    assert math.isclose(time_aligned_root_rmse(reference, candidate), 0.1)
