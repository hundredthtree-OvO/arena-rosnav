from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from toilet_benchmark.tracks.replay import require_valid_replay_bundle
from toilet_benchmark.tracks.replay_export import (
    RecordedPedestrianSample,
    build_replay_bundle_from_episode,
    derive_replay_trajectory,
)


class ReplayExportTest(unittest.TestCase):
    def test_parking_samples_are_removed_and_motion_is_derived(self):
        samples = (
            RecordedPedestrianSample(0.0, 1000.0, 1000.0, 0.0),
            RecordedPedestrianSample(1.0, -3.8, -0.9, 0.0),
            RecordedPedestrianSample(2.0, -2.8, -0.9, 0.0),
            RecordedPedestrianSample(3.0, -2.8, 0.1, 0.0),
        )
        trajectory = derive_replay_trajectory(samples)
        self.assertEqual(len(trajectory), 3)
        self.assertEqual(trajectory[0].timestamp_sec, 0.0)
        self.assertAlmostEqual(trajectory[0].yaw, 0.0)
        self.assertAlmostEqual(trajectory[-1].yaw, 1.57079632679)
        self.assertEqual(
            trajectory[0].extras["velocity_source"],
            "position_finite_difference",
        )

    def test_episode_metadata_builds_a_valid_sealed_bundle(self):
        records = (
            RecordedPedestrianSample(10.0, -3.8, -0.9, 0.0),
            RecordedPedestrianSample(10.1, -3.7, -0.9, 0.0),
        )
        with tempfile.TemporaryDirectory() as directory:
            episode = Path(directory)
            (episode / "rosbag2").mkdir()
            (episode / "rosbag2" / "metadata.yaml").write_text("bag: test\n")
            (episode / "metadata.yaml").write_text(
                yaml.safe_dump({
                    "episode_id": "episode_1",
                    "seed": 42,
                    "scenario": {"pedestrian_target_urinal_id": "urinal_3"},
                })
            )
            with patch(
                "toilet_benchmark.tracks.replay_export._read_episode_bag",
                return_value=records,
            ):
                bundle = build_replay_bundle_from_episode(
                    episode,
                    agent_id="toilet_agent_01",
                    scene_id="shenxinfu_841837",
                )
        require_valid_replay_bundle(bundle)
        self.assertEqual(bundle.primary_agent().semantic_goal, "urinal_3")
        self.assertTrue(bundle.provenance.content_hash)
        self.assertAlmostEqual(bundle.primary_agent().sample_rate_hz, 10.0)


if __name__ == "__main__":
    unittest.main()
