import math

import pytest

from toilet_benchmark.episodes.schema import (
    PedestrianHoldSpec,
    PedestrianTerminalBehavior,
)

from toilet_benchmark_ui.editor_model import (
    ActorDraft,
    HoldDraft,
    MapSnapshot,
    ScenarioDraft,
)


def _map() -> MapSnapshot:
    return MapSnapshot(
        frame_id="map",
        resolution=0.5,
        width=4,
        height=3,
        origin=(-1.0, -0.5),
        data=(0, 0, 100, 0, 0, 0, -1, 0, 0, 0, 0, 0),
    )


def test_world_pixel_roundtrip_and_occupancy() -> None:
    grid = _map()
    pixel = grid.world_to_pixel(-0.25, 0.0)
    assert pixel == pytest.approx((1.5, 2.0))
    assert grid.pixel_to_world(*pixel) == pytest.approx((-0.25, 0.0))
    assert grid.is_free(-0.25, 0.0)
    assert not grid.is_free(0.25, -0.25)
    assert grid.is_free(0.25, 0.0) is False


def test_route_validation_rejects_wall_and_accepts_free_segment() -> None:
    grid = _map()
    assert grid.validate_polyline(((-0.75, -0.25), (-0.25, -0.25))).valid
    result = grid.validate_polyline(((-0.75, -0.25), (0.75, -0.25)), radius_m=0.0)
    assert not result.valid
    assert "occupied" in result.reason


def test_scenario_draft_exports_two_independent_pedestrians() -> None:
    scenario = ScenarioDraft.default()
    first = scenario.add_pedestrian("toilet_agent_01")
    first.spawn_pose = (-3.8, -0.9, 0.0, 0.0)
    first.route = [(-3.8, -0.9, 0.0), (-2.5, -0.9, 0.0), (-1.5, -0.9, 0.0)]
    first.holds = [HoldDraft(waypoint_index=1, duration_sec=2.0)]
    second = scenario.add_pedestrian("toilet_agent_02")
    second.spawn_pose = (-1.5, -0.9, 0.0, math.pi)
    second.route = [(-1.5, -0.9, 0.0), (-2.5, -0.9, 0.0), (-3.8, -0.9, 0.0)]
    scenario.robot.spawn_pose = (0.0, -1.5, 0.03, 0.0)
    scenario.robot.goal_pose = (-3.8, -0.91, 0.0)

    episode = scenario.to_episode_spec(map_path="/tmp/test.walkable.json")
    restored = ScenarioDraft.from_episode_spec(episode)

    assert [actor.actor_id for actor in restored.pedestrians] == [
        "toilet_agent_01",
        "toilet_agent_02",
    ]
    assert restored.actor("toilet_agent_01").holds[0].duration_sec == 2.0
    assert restored.actor("toilet_agent_01").holds[0].waypoint_index == 1
    assert list(episode.pedestrians[0].route_waypoints) == first.route
    assert episode.task_type == "authored_route"
    assert restored.robot.goal_pose == pytest.approx((-3.8, -0.91, 0.0))
    assert episode.pedestrians[0].start_yaw == pytest.approx(math.pi / 2.0)
    assert episode.pedestrians[1].start_yaw == pytest.approx(-math.pi / 2.0)


def test_scenario_validation_reports_missing_spawn_and_invalid_hold() -> None:
    scenario = ScenarioDraft.default()
    actor = ActorDraft.pedestrian("toilet_agent_01")
    actor.route = [(-0.75, -0.25, 0.0), (-0.25, -0.25, 0.0)]
    actor.holds = [HoldDraft(waypoint_index=3, duration_sec=1.0)]
    scenario.pedestrians.append(actor)

    messages = scenario.validate(_map(), radius_m=0.0)

    assert any("spawn" in message for message in messages)
    assert any("hold" in message for message in messages)


def test_scenario_accepts_one_safe_target_without_requiring_a_drawn_segment() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    scenario.robot.goal_pose = (0.75, 0.75, 0.0)
    actor = scenario.add_pedestrian("toilet_agent_01")
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.0)
    actor.route = [(0.75, 0.75, 0.0)]

    assert scenario.validate(_map(), radius_m=0.0) == []


def test_scenario_validation_requires_distinct_robot_goal() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    actor = scenario.add_pedestrian("toilet_agent_01")
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.0)
    actor.route = [(0.75, 0.75, 0.0)]

    assert any("robot goal" in message for message in scenario.validate(_map(), radius_m=0.0))

    scenario.robot.goal_pose = (-0.55, 0.75, 0.0)
    assert any("goal tolerance" in message for message in scenario.validate(_map(), radius_m=0.0))


def test_pedestrian_authored_points_include_spawn_only_for_display() -> None:
    actor = ActorDraft.pedestrian("toilet_agent_01")
    actor.spawn_pose = (1.0, 2.0, 0.0, 0.25)
    actor.route = [(2.0, 2.0, 0.0), (3.0, 2.0, 0.0)]

    assert actor.authored_points() == [
        (1.0, 2.0, 0.0),
        (2.0, 2.0, 0.0),
        (3.0, 2.0, 0.0),
    ]
    assert actor.route == [(2.0, 2.0, 0.0), (3.0, 2.0, 0.0)]


def test_live_pedestrian_heading_uses_first_non_degenerate_target() -> None:
    actor = ActorDraft.pedestrian("toilet_agent_01")
    actor.spawn_pose = (1.0, 2.0, 0.0, -1.0)
    actor.route = [(1.0, 2.0, 0.0), (1.0, 3.0, 0.0)]

    assert actor.sync_walking_heading()
    assert actor.spawn_pose[3] == pytest.approx(math.pi / 2.0)


def test_live_pedestrian_heading_keeps_previous_value_without_effective_target() -> None:
    actor = ActorDraft.pedestrian("toilet_agent_01")
    actor.spawn_pose = (1.0, 2.0, 0.0, 0.75)
    actor.route = [(1.0, 2.0, 0.0)]

    assert not actor.sync_walking_heading()
    assert actor.spawn_pose[3] == pytest.approx(0.75)


def test_persistent_pedestrian_needs_only_spawn_and_preserves_manual_facing() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    scenario.robot.goal_pose = (0.75, 0.75, 0.0)
    actor = scenario.add_pedestrian("standing_agent")
    actor.auto_start_yaw = False
    actor.start_hold_duration_sec = None
    actor.terminal_behavior = PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.35)

    assert scenario.validate(_map(), radius_m=0.0) == []
    episode = scenario.to_episode_spec(map_path="/tmp/test.walkable.json")
    restored = ScenarioDraft.from_episode_spec(episode)

    assert not episode.pedestrians[0].auto_start_yaw
    assert episode.pedestrians[0].start_hold_duration_sec is None
    assert episode.pedestrians[0].route_waypoints == ()
    assert restored.pedestrians[0].terminal_behavior == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
    assert restored.pedestrians[0].spawn_pose[3] == pytest.approx(0.35)


def test_terminal_yaw_round_trip_is_separate_from_intermediate_holds() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    scenario.robot.goal_pose = (0.75, 0.75, 0.0)
    actor = scenario.add_pedestrian("walking_agent")
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.0)
    actor.route = [(-0.25, -0.25, 0.0), (0.75, 0.75, 0.0)]
    actor.terminal_behavior = PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
    actor.terminal_yaw = 0.4

    episode = scenario.to_episode_spec(map_path="/tmp/test.walkable.json")
    restored = ScenarioDraft.from_episode_spec(episode)

    assert episode.pedestrians[0].terminal_yaw == pytest.approx(
        0.4 + math.pi / 2.0
    )
    assert restored.pedestrians[0].terminal_yaw == pytest.approx(0.4)
    assert restored.pedestrians[0].holds == []


def test_legacy_terminal_infinite_hold_is_normalized_into_terminal_yaw() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    scenario.robot.goal_pose = (0.75, 0.75, 0.0)
    actor = scenario.add_pedestrian("walking_agent")
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.0)
    actor.route = [(-0.25, -0.25, 0.0), (0.75, 0.75, 0.0)]
    actor.terminal_behavior = PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
    episode = scenario.to_episode_spec(map_path="/tmp/test.walkable.json")
    spec = episode.pedestrians[0]
    legacy = spec.__class__(
        **{
            **spec.__dict__,
            "holds": (PedestrianHoldSpec(1, None, yaw=1.3),),
        }
    )

    restored = ScenarioDraft.from_episode_spec(
        episode.__class__(**{**episode.__dict__, "pedestrians": (legacy,)})
    )

    assert restored.pedestrians[0].holds == []
    assert restored.pedestrians[0].terminal_yaw == pytest.approx(1.3 - math.pi / 2.0)


def test_validation_rejects_unreachable_or_ambiguous_hold_combinations() -> None:
    scenario = ScenarioDraft.default()
    scenario.robot.spawn_pose = (-0.75, 0.75, 0.03, 0.0)
    scenario.robot.goal_pose = (0.75, 0.75, 0.0)
    actor = scenario.add_pedestrian("walking_agent")
    actor.spawn_pose = (-0.75, -0.25, 0.0, 0.0)
    actor.route = [(-0.25, -0.25, 0.0), (0.75, 0.75, 0.0)]
    actor.start_hold_duration_sec = None
    actor.terminal_behavior = PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
    actor.holds = [HoldDraft(0, None), HoldDraft(1, 2.0)]

    errors = scenario.validate(_map(), radius_m=0.0)

    assert any("unreachable" in error for error in errors)
    assert any("finite duration" in error for error in errors)
    assert any("terminal point" in error for error in errors)
