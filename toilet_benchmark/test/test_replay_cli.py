from __future__ import annotations

import io
import json
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path

import yaml

from toilet_benchmark.tracks.replay import (
    REPLAY_SCHEMA_VERSION,
    ReplayBundle,
    compute_replay_bundle_hash,
    main,
)


def _bundle_payload() -> dict[str, object]:
    return {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "benchmark_version": "0.1.0",
        "episode_id": "replay_episode_cli",
        "scene_id": "shenxinfu_841837",
        "task_type": "enter_exit",
        "track": "replay",
        "seed": 11,
        "reference_episode_hash": "b" * 64,
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
                    },
                    {
                        "sample_index": 1,
                        "timestamp_sec": 1.0,
                        "x": 1.0,
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "vx": 1.0,
                        "vy": 0.0,
                        "wz": 0.0,
                        "radius_m": 0.3,
                        "source": "replay_reference",
                    },
                ],
            }
        ],
        "annotations": {},
        "provenance": {
            "source_episode_manifest": "manifest.json",
            "source_episode_hash": "b" * 64,
            "generated_by": "unit-test",
            "generated_at_sec": 1.0,
            "content_hash": "",
        },
    }


def _write_bundle(path: Path) -> ReplayBundle:
    payload = _bundle_payload()
    bundle = ReplayBundle.from_mapping(payload)
    payload["provenance"] = dict(payload["provenance"], content_hash=compute_replay_bundle_hash(bundle))
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return ReplayBundle.from_mapping(payload)


class ReplayCliTest(unittest.TestCase):
    def test_cli_validate_and_dry_run_reports_sequence_and_hash(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        bundle_path = root / "bundle.yaml"
        bundle = _write_bundle(bundle_path)

        stdout = io.StringIO()
        stderr = io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            exit_code = main([str(bundle_path), "--step-sec", "0.5"])

        self.assertEqual(exit_code, 0)
        self.assertEqual(stderr.getvalue(), "")
        payload = json.loads(stdout.getvalue())
        self.assertEqual(payload["bundle_hash"], compute_replay_bundle_hash(bundle))
        self.assertEqual(len(payload["frames"]), 3)
        self.assertEqual(payload["frames"][1]["timestamp_sec"], 0.5)
        self.assertEqual(payload["frames"][1]["snapshot"]["agent_id"], "toilet_agent_01")
        self.assertEqual(payload["frames"][-1]["finished"], True)

        second_stdout = io.StringIO()
        with redirect_stdout(second_stdout):
            second_exit_code = main([str(bundle_path), "--step-sec", "0.5"])
        second_payload = json.loads(second_stdout.getvalue())
        self.assertEqual(second_exit_code, 0)
        self.assertEqual(
            payload["state_sequence_hash"],
            second_payload["state_sequence_hash"],
        )
        self.assertEqual(payload["frames"], second_payload["frames"])

    def test_validate_only_mode_returns_json_summary(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        bundle_path = root = Path(tmp.name) / "bundle.yaml"
        _write_bundle(bundle_path)

        stdout = io.StringIO()
        stderr = io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            exit_code = main([str(bundle_path), "--validate-only"])

        self.assertEqual(exit_code, 0)
        self.assertEqual(stderr.getvalue(), "")
        payload = json.loads(stdout.getvalue())
        self.assertTrue(payload["valid"])
        self.assertEqual(payload["episode_id"], "replay_episode_cli")
        self.assertEqual(payload["agent_id"], "toilet_agent_01")

    def test_seal_output_adds_hash_without_overwriting_source(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        root = Path(tmp.name)
        source = root / "authoring.yaml"
        output = root / "sealed.yaml"
        source.write_text(
            yaml.safe_dump(_bundle_payload(), sort_keys=False),
            encoding="utf-8",
        )

        stdout = io.StringIO()
        stderr = io.StringIO()
        with redirect_stdout(stdout), redirect_stderr(stderr):
            exit_code = main(
                [str(source), "--seal-output", str(output)]
            )

        self.assertEqual(exit_code, 0)
        self.assertEqual(stderr.getvalue(), "")
        self.assertEqual(
            yaml.safe_load(source.read_text(encoding="utf-8"))["provenance"][
                "content_hash"
            ],
            "",
        )
        sealed = ReplayBundle.from_mapping(
            yaml.safe_load(output.read_text(encoding="utf-8"))
        )
        self.assertEqual(
            sealed.provenance.content_hash,
            compute_replay_bundle_hash(sealed),
        )


if __name__ == "__main__":
    unittest.main()
