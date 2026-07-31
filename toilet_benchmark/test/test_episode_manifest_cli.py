from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

from toilet_benchmark.episodes.cli import main, validate_manifest_bundle
from toilet_benchmark.episodes.manifest import (
    EpisodeManifest,
    ManifestEpisode,
    compute_episode_hash,
    dump_episode_manifest,
    load_episode_manifest,
    load_episode_spec,
)


_CONFIG = """\
session:
  output_root: /tmp/toilet_collection
  operator_id: test_operator
  seed: 11
  selection_mode: round_robin
  max_episodes: 0
episode:
  timeout_sec: 120.0
  goal_tolerance_m: 0.35
  goal_stop_speed_mps: 0.08
  settle_timeout_sec: 8.0
  settle_linear_speed_mps: 0.03
  settle_angular_speed_rps: 0.08
pedestrian:
  agent_id: toilet_agent_01
  count: 2
  character_pool: [character_a, character_b]
scenarios:
  - id: scenario_a
    enabled: true
    weight: 1.0
    robot_start: [1.0, 2.0, 0.0, 0.0]
    robot_goal: [3.0, 4.0, 0.0]
    pedestrian_target_urinal_ids: [urinal_1, urinal_2]
  - id: scenario_disabled
    enabled: false
    weight: 1.0
    robot_start: [1.0, 2.0, 0.0, 0.0]
    robot_goal: [3.0, 4.0, 0.0]
    pedestrian_target_urinal_ids: [urinal_2]
  - id: scenario_b
    enabled: true
    weight: 1.0
    robot_start: [2.0, 2.0, 0.0, 0.0]
    robot_goal: [4.0, 4.0, 0.0]
    pedestrian_target_urinal_ids: [urinal_2, urinal_3]
  - id: scenario_c
    enabled: true
    weight: 1.0
    robot_start: [3.0, 2.0, 0.0, 0.0]
    robot_goal: [5.0, 4.0, 0.0]
    pedestrian_target_urinal_ids: [urinal_3, urinal_1]
"""


class EpisodeManifestCliTest(unittest.TestCase):
    def setUp(self):
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.root = Path(self.temp_dir.name)
        self.config_path = self.root / "manual_collection.yaml"
        self.config_path.write_text(_CONFIG, encoding="utf-8")
        self.manifest_path = self.root / "manifest.json"

    def _generate(self) -> int:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return main(
                [
                    "generate",
                    "--config",
                    str(self.config_path),
                    "--scene-id",
                    "test_scene",
                    "--seed",
                    "37",
                    "--split-ratios",
                    "train=0.5,validation=0.25,test=0.25",
                    "--output",
                    str(self.manifest_path),
                ]
            )

    def test_generate_writes_valid_hashed_bundle_with_fixed_splits(self):
        self.assertEqual(self._generate(), 0)
        manifest = load_episode_manifest(self.manifest_path)
        self.assertEqual(len(manifest.entries), 3)
        self.assertEqual(manifest.split_seed, 37)
        self.assertEqual(len({entry.episode_id for entry in manifest.entries}), 3)

        first_layout = {
            entry.episode_id: (entry.split, entry.episode_path)
            for entry in manifest.entries
        }
        for entry in manifest.entries:
            self.assertIsNotNone(entry.episode_path)
            episode = load_episode_spec(self.manifest_path.parent / entry.episode_path)
            self.assertEqual(episode.seed, 37)
            self.assertEqual(episode.scene_id, "test_scene")
            self.assertEqual(entry.episode_hash, compute_episode_hash(episode))

        self.assertEqual(validate_manifest_bundle(self.manifest_path), ())
        first_manifest_bytes = self.manifest_path.read_bytes()
        first_episode_bytes = {
            entry.episode_id: (
                self.manifest_path.parent / entry.episode_path
            ).read_bytes()
            for entry in manifest.entries
        }
        self.assertEqual(self._generate(), 0)
        regenerated = load_episode_manifest(self.manifest_path)
        self.assertEqual(self.manifest_path.read_bytes(), first_manifest_bytes)
        self.assertEqual(
            {
                entry.episode_id: (
                    self.manifest_path.parent / entry.episode_path
                ).read_bytes()
                for entry in regenerated.entries
            },
            first_episode_bytes,
        )
        self.assertEqual(
            {
                entry.episode_id: (entry.split, entry.episode_path)
                for entry in regenerated.entries
            },
            first_layout,
        )

    def test_validate_command_reports_payload_hash_failure(self):
        self.assertEqual(self._generate(), 0)
        manifest = load_episode_manifest(self.manifest_path)
        episode_path = self.manifest_path.parent / manifest.entries[0].episode_path
        payload = json.loads(episode_path.read_text(encoding="utf-8"))
        payload["seed"] += 1
        episode_path.write_text(json.dumps(payload), encoding="utf-8")

        stderr = io.StringIO()
        with redirect_stdout(io.StringIO()), redirect_stderr(stderr):
            exit_code = main(["validate", str(self.manifest_path)])
        self.assertEqual(exit_code, 1)
        self.assertIn("episode_hash_mismatch", stderr.getvalue())

    def test_validate_detects_content_hash_and_split_overlap(self):
        self.assertEqual(self._generate(), 0)
        payload = json.loads(self.manifest_path.read_text(encoding="utf-8"))
        payload["split_seed"] = 99
        self.manifest_path.write_text(json.dumps(payload), encoding="utf-8")
        issues = validate_manifest_bundle(self.manifest_path)
        self.assertIn(
            "manifest_content_hash_mismatch",
            {issue.code for issue in issues},
        )

        manifest = load_episode_manifest(
            self._regenerate_to(self.root / "overlap_source.json")
        )
        original = manifest.entries[0]
        overlapping = EpisodeManifest(
            schema_version=manifest.schema_version,
            benchmark_version=manifest.benchmark_version,
            split_seed=manifest.split_seed,
            entries=manifest.entries
            + (
                ManifestEpisode(
                    episode_id=original.episode_id,
                    split="test",
                    episode_hash=original.episode_hash,
                    episode_path=original.episode_path,
                ),
            ),
        )
        overlap_path = self.root / "overlap_source.json"
        dump_episode_manifest(overlapping, overlap_path)
        overlap_issues = validate_manifest_bundle(overlap_path)
        self.assertIn("episode_split_overlap", {issue.code for issue in overlap_issues})

    def _regenerate_to(self, output: Path) -> Path:
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = main(
                [
                    "generate",
                    "--config",
                    str(self.config_path),
                    "--scene-id",
                    "test_scene",
                    "--output",
                    str(output),
                ]
            )
        self.assertEqual(result, 0)
        return output


if __name__ == "__main__":
    unittest.main()
