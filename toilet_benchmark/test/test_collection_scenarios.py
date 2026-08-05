import tempfile
import unittest
import json
from pathlib import Path

import yaml

from toilet_benchmark.collection_scenarios import (
    ManualCollectionConfig,
    ScenarioSelector,
    load_manual_collection_config,
)


class TestCollectionScenarios(unittest.TestCase):
    def _write_config(self, payload):
        tmpdir = tempfile.TemporaryDirectory()
        path = Path(tmpdir.name) / "manual_collection.yaml"
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        self.addCleanup(tmpdir.cleanup)
        return path

    def _base_payload(self):
        return {
            "session": {
                "output_root": "/tmp/toilet_manual",
                "operator_id": "operator_01",
                "seed": 42,
                "selection_mode": "round_robin",
                "max_episodes": 0,
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
                "parking_pose": [1000.0, 1000.0, 0.0],
            },
            "scenarios": [
                {
                    "id": "left",
                    "enabled": True,
                    "weight": 1.0,
                    "robot_start": [1.8, 0.25, 0.0, 3.14159],
                    "robot_goal": [-3.8, -0.91, 3.14159],
                    "pedestrian_target_urinal_id": "urinal_3",
                },
                {
                    "id": "center",
                    "enabled": False,
                    "weight": 2.0,
                    "robot_start": [0.7, 0.20, 0.0, 3.14159],
                    "robot_goal": [-3.8, -0.91, 3.14159],
                    "pedestrian_target_urinal_id": "urinal_5",
                },
                {
                    "id": "right",
                    "enabled": True,
                    "weight": 1.0,
                    "robot_start": [0.1, 0.20, 0.0, 3.14159],
                    "robot_goal": [-3.8, -0.91, 3.14159],
                    "pedestrian_target_urinal_id": "urinal_1",
                },
            ],
        }

    def test_loader_accepts_expected_pose_shapes_and_filters_disabled(self):
        config = load_manual_collection_config(self._write_config(self._base_payload()))

        self.assertIsInstance(config, ManualCollectionConfig)
        self.assertEqual(config.session.selection_mode, "round_robin")
        self.assertEqual(config.pedestrian.count, 1)
        self.assertEqual(config.pedestrian.agent_ids, ("toilet_agent_01",))
        self.assertEqual(config.scenarios[0].robot_start, (1.8, 0.25, 0.0, 3.14159))
        self.assertEqual(config.scenarios[0].robot_goal, (-3.8, -0.91, 3.14159))
        self.assertEqual(config.scenarios[0].pedestrian_target_urinal_ids, ("urinal_3",))
        self.assertEqual([scenario.id for scenario in config.enabled_scenarios()], ["left", "right"])

    def test_session_defaults_to_ten_episodes(self):
        payload = self._base_payload()
        del payload["session"]["max_episodes"]

        config = load_manual_collection_config(self._write_config(payload))

        self.assertEqual(config.session.max_episodes, 10)

    def test_negative_session_episode_limit_is_rejected(self):
        payload = self._base_payload()
        payload["session"]["max_episodes"] = -1

        with self.assertRaisesRegex(ValueError, "session.max_episodes must be >= 0"):
            load_manual_collection_config(self._write_config(payload))

    def test_loader_accepts_multiple_pedestrians_and_target_resources(self):
        payload = self._base_payload()
        payload["pedestrian"].update(
            {
                "count": 3,
                "character_pool": [
                    "original_female_adult_business_02",
                    "original_female_adult_medical_01",
                ],
            }
        )
        scenario = payload["scenarios"][0]
        scenario.pop("pedestrian_target_urinal_id")
        scenario["pedestrian_target_urinal_ids"] = ["urinal_3", "urinal_4"]

        config = load_manual_collection_config(self._write_config(payload))

        self.assertEqual(config.pedestrian.count, 3)
        self.assertEqual(
            config.pedestrian.agent_ids,
            ("toilet_agent_01", "toilet_agent_02", "toilet_agent_03"),
        )
        self.assertEqual(
            config.pedestrian.character_pool,
            ("original_female_adult_business_02", "original_female_adult_medical_01"),
        )
        self.assertEqual(config.scenarios[0].pedestrian_target_urinal_ids, ("urinal_3", "urinal_4"))

    def test_loader_rejects_non_positive_pedestrian_count(self):
        payload = self._base_payload()
        payload["pedestrian"]["count"] = 0

        with self.assertRaises(ValueError):
            load_manual_collection_config(self._write_config(payload))

    def test_loader_rejects_empty_target_resource_list(self):
        payload = self._base_payload()
        scenario = payload["scenarios"][0]
        scenario.pop("pedestrian_target_urinal_id")
        scenario["pedestrian_target_urinal_ids"] = []

        with self.assertRaises(ValueError):
            load_manual_collection_config(self._write_config(payload))

    def test_fixed_selector_is_deterministic(self):
        config = load_manual_collection_config(self._write_config(self._base_payload()))
        selector = ScenarioSelector(
            config.scenarios,
            selection_mode="fixed",
            seed=config.session.seed,
            fixed_scenario_id="right",
        )

        draws = [selector.select().scenario.id for _ in range(4)]

        self.assertEqual(draws, ["right", "right", "right", "right"])
        self.assertEqual(selector.coverage_counts, {"left": 0, "right": 4})

    def test_round_robin_skips_disabled_and_cycles_enabled_only(self):
        config = load_manual_collection_config(self._write_config(self._base_payload()))
        selector = ScenarioSelector(config.scenarios, selection_mode="round_robin", seed=config.session.seed)

        draws = [selector.select() for _ in range(5)]

        self.assertEqual([draw.scenario.id for draw in draws], ["left", "right", "left", "right", "left"])
        self.assertEqual([draw.selection_index for draw in draws], [1, 2, 3, 4, 5])

    def test_seeded_random_repeats_for_same_seed(self):
        config = load_manual_collection_config(self._write_config(self._base_payload()))
        first = ScenarioSelector(config.scenarios, selection_mode="seeded_random", seed=7)
        second = ScenarioSelector(config.scenarios, selection_mode="seeded_random", seed=7)

        first_draws = [first.select().scenario.id for _ in range(6)]
        second_draws = [second.select().scenario.id for _ in range(6)]

        self.assertEqual(first_draws, second_draws)
        self.assertTrue(all(draw in {"left", "right"} for draw in first_draws))

    def test_loader_rejects_invalid_robot_goal_shape(self):
        payload = self._base_payload()
        payload["scenarios"][0]["robot_goal"] = [1.0, 2.0]

        with self.assertRaises(ValueError):
            load_manual_collection_config(self._write_config(payload))

    def test_loader_rejects_all_disabled_scenarios(self):
        payload = self._base_payload()
        for scenario in payload["scenarios"]:
            scenario["enabled"] = False

        with self.assertRaises(ValueError):
            load_manual_collection_config(self._write_config(payload))

    def test_authored_route_derives_runtime_contract_from_episode(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        episode_path = Path(tmpdir.name) / "authored.json"
        episode_path.write_text(
            json.dumps(
                {
                    "task_type": "authored_route",
                    "robot": {
                        "start_pose": [1.0, 2.0, 0.03, 1.57],
                        "goal_pose": [-3.8, -0.9, 0.0],
                    },
                    "pedestrians": [
                        {"agent_id": "toilet_agent_01"},
                        {"agent_id": "toilet_agent_02"},
                    ],
                }
            ),
            encoding="utf-8",
        )
        payload = self._base_payload()
        payload["scenario_source"] = {"mode": "authored_route"}
        payload["scenarios"] = [
            {
                "id": "authored",
                "enabled": True,
                "weight": 1.0,
                "episode_path": str(episode_path),
            }
        ]

        config = load_manual_collection_config(self._write_config(payload))
        scenario = config.scenarios[0]

        self.assertEqual(scenario.source_mode, "authored_route")
        self.assertEqual(scenario.robot_start, (1.0, 2.0, 0.03, 1.57))
        self.assertEqual(scenario.robot_goal, (-3.8, -0.9, 0.0))
        self.assertEqual(scenario.pedestrian_agent_ids, ("toilet_agent_01", "toilet_agent_02"))
        self.assertEqual(len(scenario.episode_sha256), 64)
        self.assertEqual(scenario.pedestrian_target_urinal_ids, ())
