import json
import unittest

from toilet_benchmark.collection_scenarios import (
    ScenarioSelector,
    load_manual_collection_config,
)
from toilet_benchmark.episodes.schema import (
    BENCHMARK_SCHEMA_VERSION,
    CollisionPolicy,
    EpisodeSpec,
    PedestrianBehaviorSpec,
    PedestrianEpisodeSpec,
    PedestrianHoldSpec,
    RobotEpisodeSpec,
    TerminationSpec,
    TrackType,
    episode_from_manual_selection,
)


class TestEpisodeSchema(unittest.TestCase):
    def _manual_config(self):
        return load_manual_collection_config(
            {
                "session": {
                    "output_root": "/tmp/toilet_manual",
                    "operator_id": "operator_01",
                    "seed": 42,
                    "selection_mode": "fixed",
                    "fixed_scenario_id": "doorway",
                    "max_episodes": 1,
                },
                "episode": {
                    "timeout_sec": 120.0,
                    "goal_tolerance_m": 0.35,
                    "goal_stop_speed_mps": 0.08,
                    "settle_timeout_sec": 8.0,
                    "settle_linear_speed_mps": 0.02,
                    "settle_angular_speed_rps": 0.03,
                },
                "pedestrian": {
                    "agent_id": "toilet_agent_01",
                    "count": 3,
                    "character_pool": ["character_a", "character_b"],
                },
                "scenarios": [
                    {
                        "id": "doorway",
                        "enabled": True,
                        "weight": 1.0,
                        "robot_start": [1.0, 2.0, 0.03, -1.57],
                        "robot_goal": [-3.8, -0.91, 0.0],
                        "pedestrian_target_urinal_ids": ["urinal_1", "urinal_3"],
                    }
                ],
            }
        )

    def test_manual_scenario_converts_to_versioned_episode(self):
        config = self._manual_config()
        selection = ScenarioSelector(
            config.scenarios,
            selection_mode="fixed",
            seed=42,
            fixed_scenario_id="doorway",
        ).select()

        episode = episode_from_manual_selection(
            config,
            selection,
            episode_id="dataset_000001",
            scene_id="shenxinfu_841837",
            track=TrackType.DATASET,
        )

        self.assertEqual(episode.schema_version, BENCHMARK_SCHEMA_VERSION)
        self.assertEqual(episode.episode_id, "dataset_000001")
        self.assertEqual(episode.track, TrackType.DATASET)
        self.assertEqual(episode.robot.start_pose, (1.0, 2.0, 0.03, -1.57))
        self.assertEqual(episode.termination.timeout_sec, 120.0)
        self.assertEqual(episode.termination.collision_policy, CollisionPolicy.TERMINATE)
        self.assertEqual(
            [pedestrian.semantic_goal for pedestrian in episode.pedestrians],
            ["urinal_1", "urinal_3", "urinal_1"],
        )
        self.assertEqual(
            [pedestrian.character for pedestrian in episode.pedestrians],
            ["character_a", "character_b", "character_a"],
        )
        self.assertTrue(
            all(pedestrian.start_reference == "entrance_main" for pedestrian in episode.pedestrians)
        )

    def test_episode_dict_round_trip_is_stable(self):
        config = self._manual_config()
        selection = ScenarioSelector(
            config.scenarios,
            selection_mode="fixed",
            seed=42,
            fixed_scenario_id="doorway",
        ).select()
        episode = episode_from_manual_selection(
            config,
            selection,
            episode_id="dataset_000001",
            scene_id="shenxinfu_841837",
            track="dataset",
        )

        payload = episode.to_dict()
        restored = EpisodeSpec.from_mapping(payload)

        self.assertEqual(restored, episode)
        self.assertEqual(
            json.dumps(restored.to_dict(), sort_keys=True),
            json.dumps(payload, sort_keys=True),
        )

    def test_episode_rejects_unknown_track(self):
        config = self._manual_config()
        selection = ScenarioSelector(
            config.scenarios,
            selection_mode="fixed",
            seed=42,
            fixed_scenario_id="doorway",
        ).select()

        with self.assertRaises(ValueError):
            episode_from_manual_selection(
                config,
                selection,
                episode_id="dataset_000001",
                scene_id="shenxinfu_841837",
                track="unknown",
            )

    def test_authored_routes_and_holds_round_trip(self):
        episode = EpisodeSpec(
            episode_id="narrow_head_on",
            scene_id="shenxinfu_841837",
            task_type="authored_route",
            track=TrackType.INTERACTIVE,
            seed=42,
            robot=RobotEpisodeSpec(
                model="xms_mecanum",
                start_pose=(0.0, -1.5, 0.03, 0.0),
                goal_pose=(0.0, -1.5, 0.0),
            ),
            pedestrians=(
                PedestrianEpisodeSpec(
                    agent_id="toilet_agent_01",
                    semantic_goal="route_terminal",
                    character="character_a",
                    start_pose=(-3.8, -0.9, 0.0),
                    start_yaw=0.0,
                    route_waypoints=(
                        (-3.8, -0.9, 0.0),
                        (-2.5, -0.9, 0.0),
                        (-1.5, -0.9, 0.0),
                    ),
                    holds=(PedestrianHoldSpec(waypoint_index=1, duration_sec=2.0),),
                    constrain_to_path=True,
                    behavior=PedestrianBehaviorSpec(walking_speed_mps=0.8),
                ),
            ),
            termination=TerminationSpec(timeout_sec=60.0, goal_tolerance_m=0.3),
            assets={"walkable_map": "shenxinfu_841837.walkable.json"},
        )

        payload = episode.to_dict()
        restored = EpisodeSpec.from_mapping(payload)

        self.assertEqual(restored, episode)
        self.assertEqual(payload["pedestrians"][0]["start_yaw"], 0.0)
        self.assertEqual(payload["pedestrians"][0]["holds"][0]["waypoint_index"], 1)
        self.assertTrue(payload["pedestrians"][0]["constrain_to_path"])


if __name__ == "__main__":
    unittest.main()
