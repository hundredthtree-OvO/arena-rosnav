from __future__ import annotations

import json
import math
import tempfile
import unittest
from pathlib import Path

from toilet_benchmark.tracks.replay import (
    REPLAY_SCHEMA_VERSION,
    ReplayBundle,
    ReplayBundleValidationErrorCode,
    ReplayRuntime,
    ReplayTrajectorySample,
    compute_replay_bundle_hash,
    compute_replay_state_sequence_hash,
    load_replay_bundle,
    require_valid_replay_bundle,
    seal_replay_bundle,
    validate_replay_bundle,
)


def _base_payload() -> dict[str, object]:
    return {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "benchmark_version": "0.1.0",
        "episode_id": "replay_episode_01",
        "scene_id": "shenxinfu_841837",
        "task_type": "enter_exit",
        "track": "replay",
        "seed": 7,
        "reference_episode_hash": "a" * 64,
        "coordinate_frame": "world",
        "time_base": {
            "origin": "episode_reset",
            "clock": "physics_monotonic_sec",
            "unit": "sec",
            "zero_sec": 0.0,
            "monotonic": True,
            "sample_policy": "strictly_increasing",
        },
        "agents": [
            {
                "agent_id": "toilet_agent_01",
                "semantic_goal": "urinal_3",
                "start_reference": "entrance_main",
                "start_pose": [0.0, 0.0, 0.0, math.pi - 0.1],
                "sample_rate_hz": 2.0,
                "trajectory": [
                    {
                        "sample_index": 0,
                        "timestamp_sec": 0.0,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": math.pi - 0.1,
                        "vx": 0.0,
                        "vy": 0.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    },
                    {
                        "sample_index": 1,
                        "timestamp_sec": 1.0,
                        "x": 2.0,
                        "y": 2.0,
                        "z": 0.0,
                        "yaw": -math.pi + 0.1,
                        "vx": 2.0,
                        "vy": 2.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    },
                ],
            }
        ],
        "annotations": {
            "termination_reason": "robot_human_collision",
            "termination_timestamp_sec": 1.0,
            "collisions": [
                {
                    "kind": "robot_human_collision",
                    "timestamp_sec": 1.0,
                    "actor_ids": ["robot", "toilet_agent_01"],
                    "contact_frame": "world",
                    "geometry_source": "physx_contact",
                    "action": "terminate",
                }
            ],
        },
        "provenance": {
            "source_episode_manifest": "manifest.json",
            "source_episode_hash": "a" * 64,
            "source_capture": "manual_collection",
            "generated_by": "unit-test",
            "generated_at_sec": 123.0,
            "content_hash": "",
        },
    }


def _bundle_from_payload(payload: dict[str, object]) -> ReplayBundle:
    bundle = ReplayBundle.from_mapping(payload)
    content_hash = compute_replay_bundle_hash(bundle)
    payload["provenance"] = dict(payload["provenance"], content_hash=content_hash)
    return ReplayBundle.from_mapping(payload)


class ReplayBundleTest(unittest.TestCase):
    def test_bundle_hash_is_canonical_and_order_independent(self):
        payload_a = _base_payload()
        payload_b = {
            "provenance": dict(payload_a["provenance"]),
            "annotations": dict(payload_a["annotations"]),
            "agents": list(payload_a["agents"]),
            "reference_episode_hash": payload_a["reference_episode_hash"],
            "track": payload_a["track"],
            "scene_id": payload_a["scene_id"],
            "episode_id": payload_a["episode_id"],
            "schema_version": payload_a["schema_version"],
            "task_type": payload_a["task_type"],
            "benchmark_version": payload_a["benchmark_version"],
            "seed": payload_a["seed"],
            "coordinate_frame": payload_a["coordinate_frame"],
            "time_base": dict(payload_a["time_base"]),
        }
        bundle_a = _bundle_from_payload(payload_a)
        bundle_b = _bundle_from_payload(payload_b)

        self.assertEqual(compute_replay_bundle_hash(bundle_a), compute_replay_bundle_hash(bundle_b))
        self.assertEqual(bundle_a.provenance.content_hash, compute_replay_bundle_hash(bundle_a))

    def test_seal_replay_bundle_does_not_mutate_source(self):
        source = ReplayBundle.from_mapping(_base_payload())

        sealed = seal_replay_bundle(source)

        self.assertEqual(source.provenance.content_hash, "")
        self.assertEqual(
            sealed.provenance.content_hash,
            compute_replay_bundle_hash(sealed),
        )

    def test_runtime_interpolates_shortest_yaw_path_and_marks_completion(self):
        bundle = _bundle_from_payload(_base_payload())
        runtime = ReplayRuntime(bundle)

        midpoint = runtime.sample(0.5)
        self.assertAlmostEqual(midpoint.x, 1.0)
        self.assertAlmostEqual(midpoint.y, 1.0)
        self.assertAlmostEqual(abs(midpoint.yaw), math.pi, places=9)
        self.assertFalse(runtime.is_finished(0.5))
        self.assertTrue(runtime.is_finished(1.0))

        dry_run = runtime.dry_run(step_sec=0.5)
        self.assertEqual([frame.timestamp_sec for frame in dry_run.frames], [0.0, 0.5, 1.0])
        self.assertTrue(dry_run.frames[-1].finished)
        self.assertEqual(
            dry_run.state_sequence_hash,
            compute_replay_state_sequence_hash(dry_run.frames),
        )

    def test_strict_validation_rejects_non_finite_monotonic_duplicate_and_empty_trajectory(self):
        payload = _base_payload()
        payload["agents"] = [
            {
                "agent_id": "toilet_agent_01",
                "semantic_goal": "urinal_3",
                "trajectory": [
                    {
                        "sample_index": 0,
                        "timestamp_sec": 0.0,
                        "x": float("nan"),
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    },
                    {
                        "sample_index": 1,
                        "timestamp_sec": 0.0,
                        "x": 1.0,
                        "y": 1.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    },
                ],
            },
            {
                "agent_id": "toilet_agent_01",
                "semantic_goal": "urinal_4",
                "trajectory": [],
            },
        ]
        payload["provenance"] = dict(payload["provenance"], content_hash="")
        bundle = ReplayBundle.from_mapping(payload)
        errors = validate_replay_bundle(bundle)
        codes = {error.code for error in errors}

        self.assertIn(ReplayBundleValidationErrorCode.AGENT_COUNT_UNSUPPORTED, codes)
        self.assertIn(ReplayBundleValidationErrorCode.AGENT_ID_DUPLICATED, codes)
        self.assertIn(ReplayBundleValidationErrorCode.TRAJECTORY_EMPTY, codes)
        self.assertIn(ReplayBundleValidationErrorCode.TRAJECTORY_VALUE_INVALID, codes)
        self.assertIn(ReplayBundleValidationErrorCode.TRAJECTORY_TIME_NOT_MONOTONIC, codes)

    def test_loader_and_validator_accept_canonical_yaml_bundle(self):
        bundle = _bundle_from_payload(_base_payload())
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        path = Path(tmp.name) / "replay_bundle.yaml"
        import yaml

        path.write_text(yaml.safe_dump(bundle.to_dict()), encoding="utf-8")

        loaded = load_replay_bundle(path)
        require_valid_replay_bundle(loaded)
        self.assertEqual(loaded, bundle)

    def test_single_sample_bundle_is_valid_and_dry_runable(self):
        payload = _base_payload()
        payload["agents"] = [
            {
                "agent_id": "toilet_agent_01",
                "semantic_goal": "urinal_3",
                "start_reference": "entrance_main",
                "start_pose": [0.0, 0.0, 0.0, 0.0],
                "trajectory": [
                    {
                        "sample_index": 0,
                        "timestamp_sec": 0.0,
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    }
                ],
            }
        ]
        payload["annotations"] = {}
        bundle = _bundle_from_payload(payload)
        runtime = ReplayRuntime(bundle)

        result = runtime.dry_run()
        self.assertEqual(len(result.frames), 1)
        self.assertTrue(result.frames[0].finished)
        self.assertEqual(result.frames[0].snapshot.to_dict()["agent_id"], "toilet_agent_01")


if __name__ == "__main__":
    unittest.main()
