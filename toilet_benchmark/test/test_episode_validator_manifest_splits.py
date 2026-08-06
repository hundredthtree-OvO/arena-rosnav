import json
import math
import tempfile
import unittest
from pathlib import Path

from toilet_benchmark.collection_scenarios import ScenarioSelector, load_manual_collection_config
from toilet_benchmark.episodes import (
    EPISODE_MANIFEST_SCHEMA_VERSION,
    EpisodeManifest,
    ManifestEpisode,
    ValidationErrorCode,
    build_episode_manifest,
    compute_episode_hash,
    dump_episode_manifest,
    fill_episode_hashes,
    load_episode_manifest,
    validate_episode,
    episodes_from_manual_config,
)
from toilet_benchmark.episodes.splits import SplitValidationError, assign_episodes_to_splits
from toilet_benchmark.episodes.schema import episode_from_manual_selection


class TestEpisodeValidatorManifestSplits(unittest.TestCase):
    def _manual_config(self):
        return load_manual_collection_config(
            {
                "session": {
                    "output_root": "/tmp/toilet_manual",
                    "operator_id": "operator_01",
                    "seed": 42,
                    "selection_mode": "round_robin",
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
                        "id": "left",
                        "enabled": True,
                        "weight": 1.0,
                        "robot_start": [1.0, 2.0, 0.03, -1.57],
                        "robot_goal": [-3.8, -0.91, 0.0],
                        "pedestrian_target_urinal_ids": ["urinal_1", "urinal_2", "urinal_3"],
                    },
                    {
                        "id": "right",
                        "enabled": True,
                        "weight": 1.0,
                        "robot_start": [0.1, 0.20, 0.0, 3.14159],
                        "robot_goal": [-3.8, -0.91, 3.14159],
                        "pedestrian_target_urinal_ids": ["urinal_4", "urinal_5", "urinal_6"],
                    },
                ],
            }
        )

    def _new_episode(self, episode_id: str, overrides: dict[str, object] | None = None):
        config = self._manual_config()
        selection = ScenarioSelector(config.scenarios, selection_mode="fixed", seed=config.session.seed, fixed_scenario_id="left").select()
        episode = episode_from_manual_selection(
            config,
            selection,
            episode_id=episode_id,
            scene_id="shenxinfu_841837",
            track="dataset",
        )
        payload = episode.to_dict()
        if overrides:
            payload = {**payload, **overrides}
        return episode.__class__.from_mapping(payload)

    def test_validator_is_deterministic_and_reports_schema_version(self):
        episode = self._new_episode("e_bad_schema", {})
        episode = self._new_episode("e_bad_schema", {"schema_version": "wrong.version"})
        errors = validate_episode(episode)
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0].code, ValidationErrorCode.SCHEMA_VERSION_MISMATCH)
        self.assertEqual(errors[0].path, ("schema_version",))

    def test_validator_reports_pose_ids_and_resource_count_issues(self):
        episode = self._new_episode(
            "e_pose_issue",
            {
                "robot": {
                    "model": "xms_mecanum",
                    "start_pose": [math.nan, 0.0, 0.0, 1.0],
                    "goal_pose": [1.0, 2.0, 3.0],
                },
                "pedestrians": [
                    {
                        "agent_id": "ped_1",
                        "semantic_goal": "missing_resource",
                        "behavior": {"type": "regular"},
                    },
                    {
                        "agent_id": "ped_1",
                        "semantic_goal": "urinal_1",
                        "behavior": {"type": "regular"},
                    },
                ],
                "difficulty": {"pedestrian_count": 3},
                "assets": {"urinal_1": {}},
            },
        )
        errors = validate_episode(
            episode,
            semantic_goal_ids={"urinal_1", "urinal_2", "urinal_3"},
        )
        self.assertGreaterEqual(len(errors), 4)
        codes = tuple(error.code for error in errors)
        self.assertIn(ValidationErrorCode.POSE_NOT_FINITE, codes)
        self.assertIn(ValidationErrorCode.PED_COUNT_MISMATCH, codes)
        self.assertIn(ValidationErrorCode.PED_RESOURCE_MISSING, codes)
        self.assertIn(ValidationErrorCode.AGENT_ID_DUPLICATED, codes)

        self.assertEqual(
            tuple(error.path for error in errors),
            tuple(sorted((error.path for error in errors))),
        )

    def test_split_assignment_is_deterministic_and_non_overlapping(self):
        episode_ids = [f"e_{str(i).zfill(3)}" for i in range(10)]
        ratios = {"train": 0.6, "validation": 0.2, "test": 0.2}
        first = assign_episodes_to_splits(episode_ids, ratios, seed=7)
        second = assign_episodes_to_splits(episode_ids, ratios, seed=7)
        self.assertEqual(first, second)

        assigned = [item for items in first.split_to_episodes.values() for item in items]
        self.assertEqual(len(assigned), len(set(assigned)))
        self.assertEqual(len(assigned), len(episode_ids))

        with self.assertRaises(SplitValidationError):
            assign_episodes_to_splits(["x", "x"], ratios)

        largest_remainder = assign_episodes_to_splits(
            ["a", "b"],
            {"train": 0.6, "validation": 0.3, "test": 0.1},
        )
        self.assertEqual(len(largest_remainder.split_to_episodes["train"]), 1)
        self.assertEqual(len(largest_remainder.split_to_episodes["validation"]), 1)
        self.assertEqual(len(largest_remainder.split_to_episodes["test"]), 0)

    def test_manifest_roundtrip_and_deterministic_content_hash(self):
        assignment = {
            "train": ("a", "c", "d"),
            "test": ("b",),
            "validation": ("e",),
        }
        manifest = build_episode_manifest(assignment, split_seed=1234)
        self.assertEqual(manifest.schema_version, EPISODE_MANIFEST_SCHEMA_VERSION)
        with self.assertRaises(ValueError):
            build_episode_manifest({"train": ("same",), "test": ("same",)})

        payload_hash = manifest.canonical_hash()
        reloaded = EpisodeManifest.from_mapping(manifest.to_dict())
        self.assertEqual(reloaded.canonical_hash(), payload_hash)

        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / "episode_manifest.json"
        dump_episode_manifest(manifest, path)
        self.assertTrue(path.exists())
        self.assertFalse(list(path.parent.glob("*.tmp")))

        loaded = load_episode_manifest(path)
        self.assertEqual(loaded.content_hash, manifest.content_hash or manifest.canonical_hash())

        self.assertEqual(json.loads(path.read_text(encoding="utf-8"))["schema_version"], EPISODE_MANIFEST_SCHEMA_VERSION)

        episodes = {
            "a": self._new_episode("a"),
            "b": self._new_episode("b"),
            "c": self._new_episode("c"),
            "d": self._new_episode("d"),
            "e": self._new_episode("e"),
        }
        filled = fill_episode_hashes(loaded, episodes)
        self.assertEqual(
            {item.episode_id: compute_episode_hash(episodes[item.episode_id]) for item in filled.entries},
            {item.episode_id: item.episode_hash for item in filled.entries},
        )

    def test_repository_manual_config_converts_default_authored_scenario(self):
        config_path = Path(__file__).parents[1] / "config" / "manual_collection.yaml"
        config = load_manual_collection_config(config_path)

        episodes = episodes_from_manual_config(
            config,
            scene_id="shenxinfu_841837",
        )

        self.assertEqual(len(episodes), 1)
        self.assertEqual(
            [episode.episode_id for episode in episodes],
            [
                f"manual_{scenario.id}"
                for scenario in config.enabled_scenarios()
            ],
        )
        self.assertEqual(episodes[0].task_type, "authored_route")
        source_scenario = config.enabled_scenarios()[0]
        source_payload = json.loads(Path(source_scenario.episode_path).read_text(encoding="utf-8"))
        self.assertEqual(
            len(episodes[0].pedestrians),
            len(source_payload["pedestrians"]),
        )
        self.assertEqual(episodes[0].metadata["source"], "manual_collection_authored_route")
        self.assertTrue(all(not validate_episode(episode) for episode in episodes))


if __name__ == "__main__":
    unittest.main()
