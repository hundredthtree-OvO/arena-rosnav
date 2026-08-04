import json
import tempfile
import unittest
from pathlib import Path

import yaml

from toilet_benchmark.collection_scenarios import load_manual_collection_config, ScenarioSelector
from toilet_benchmark.episode_recorder import EpisodeRecorder


class _FakeTimeoutExpired(Exception):
    pass


class _FakeProcess:
    def __init__(self):
        self.terminated = False
        self.killed = False
        self.wait_calls = 0

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):  # noqa: ARG002 - signature mirrors subprocess
        self.wait_calls += 1
        if self.wait_calls == 1 and not self.killed:
            raise _FakeTimeoutExpired()
        return 0

    def kill(self):
        self.killed = True


class _FakeSubprocess:
    TimeoutExpired = _FakeTimeoutExpired

    def __init__(self):
        self.calls = []
        self.process = _FakeProcess()

    def Popen(self, args, **kwargs):
        self.calls.append((list(args), kwargs))
        return self.process


class TestEpisodeRecorder(unittest.TestCase):
    def _base_payload(self, output_root):
        return {
            "session": {
                "output_root": str(output_root),
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
                    "id": "right",
                    "enabled": True,
                    "weight": 1.0,
                    "robot_start": [0.1, 0.20, 0.0, 3.14159],
                    "robot_goal": [-3.8, -0.91, 3.14159],
                    "pedestrian_target_urinal_id": "urinal_1",
                },
            ],
        }

    def _load_config(self, payload):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        path = Path(tmpdir.name) / "manual_collection.yaml"
        path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
        return load_manual_collection_config(path)

    def test_writes_atomic_manifest_and_jsonl_events(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        config = self._load_config(self._base_payload(Path(tmpdir.name) / "data"))
        selector = ScenarioSelector(config.scenarios, selection_mode="round_robin", seed=config.session.seed)
        selection = selector.select()
        fake_subprocess = _FakeSubprocess()

        recorder = EpisodeRecorder.from_config(
            config,
            session_id="session_20260722_120000_seed42",
            subprocess_module=fake_subprocess,
            rosbag_command_factory=lambda context: ["rosbag2", "record", context.episode_id],
        )
        episode_context = recorder.start_episode(selection)
        self.assertTrue(recorder.has_active_episode)
        self.assertTrue(recorder.bag_process_running)
        recorder.record_event("episode_started", {"scenario_id": selection.scenario.id})
        recorder.record_event("hard_guard_intervention", {"reason": "cmd_clipped"})
        metadata_path = recorder.finalize_episode(status="succeeded")
        session_path = recorder.finalize_session(status="completed")

        self.assertFalse(recorder.has_active_episode)
        self.assertFalse(recorder.bag_process_running)

        self.assertTrue(metadata_path.exists())
        self.assertTrue(session_path.exists())
        self.assertEqual(fake_subprocess.calls[0][0], ["rosbag2", "record", episode_context.episode_id])
        self.assertTrue(fake_subprocess.process.terminated)
        self.assertTrue(fake_subprocess.process.killed)

        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        session_manifest = yaml.safe_load(session_path.read_text(encoding="utf-8"))
        events = metadata_path.with_name("events.jsonl").read_text(encoding="utf-8").splitlines()

        self.assertEqual(metadata["status"], "succeeded")
        self.assertEqual(metadata["scenario"]["id"], selection.scenario.id)
        self.assertEqual(metadata["event_count"], 2)
        self.assertEqual(session_manifest["status"], "completed")
        self.assertIsNotNone(session_manifest["ended_at"])
        self.assertEqual(len(events), 2)
        self.assertEqual(json.loads(events[0])["event_type"], "episode_started")
        self.assertFalse(list(metadata_path.parent.glob("*.tmp")))

    def test_sigint_handler_closes_active_episode_without_rosbag_factory(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        config = self._load_config(self._base_payload(Path(tmpdir.name) / "data"))
        selector = ScenarioSelector(config.scenarios, selection_mode="fixed", seed=config.session.seed)
        selection = selector.select()

        recorder = EpisodeRecorder.from_config(
            config,
            session_id="session_20260722_120001_seed42",
        )
        recorder.start_episode(selection)
        recorder.record_event("episode_started", {"scenario_id": selection.scenario.id})
        recorder.handle_sigint(2, None)

        metadata_path = recorder.session_dir / "episode_000001" / "metadata.yaml"
        session_path = recorder.session_dir / "session_manifest.yaml"

        metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))
        session_manifest = yaml.safe_load(session_path.read_text(encoding="utf-8"))

        self.assertTrue(recorder.interrupted)
        self.assertEqual(metadata["status"], "aborted_by_signal")
        self.assertEqual(metadata["termination_reason"], "sigint")
        self.assertEqual(session_manifest["status"], "interrupted")
        self.assertEqual(metadata["event_count"], 1)

    def test_authored_manifest_uses_episode_contract_without_urinal_fields(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        root = Path(tmpdir.name)
        episode_path = root / "authored.json"
        episode_path.write_text(
            json.dumps(
                {
                    "task_type": "authored_route",
                    "robot": {
                        "start_pose": [1.0, 0.0, 0.03, 0.0],
                        "goal_pose": [-3.8, -0.91, 0.0],
                    },
                    "pedestrians": [
                        {"agent_id": "toilet_agent_01"},
                        {"agent_id": "toilet_agent_02"},
                    ],
                }
            ),
            encoding="utf-8",
        )
        payload = self._base_payload(root / "data")
        payload["scenario_source"] = {"mode": "authored_route"}
        payload["scenarios"] = [
            {
                "id": "authored",
                "enabled": True,
                "weight": 1.0,
                "episode_path": str(episode_path),
            }
        ]
        config = self._load_config(payload)
        selection = ScenarioSelector(
            config.scenarios, selection_mode="fixed", seed=config.session.seed
        ).select()
        recorder = EpisodeRecorder.from_config(config, session_id="authored_session")

        recorder.start_episode(selection)
        metadata_path = recorder.finalize_episode(status="succeeded")
        scenario = yaml.safe_load(metadata_path.read_text(encoding="utf-8"))["scenario"]

        self.assertEqual(scenario["source_mode"], "authored_route")
        self.assertEqual(scenario["pedestrian_count"], 2)
        self.assertEqual(
            scenario["pedestrian_agent_ids"],
            ["toilet_agent_01", "toilet_agent_02"],
        )
        self.assertNotIn("pedestrian_target_urinal_id", scenario)
        self.assertNotIn("pedestrian_target_urinal_ids", scenario)

    def test_start_failure_clears_partial_active_episode(self):
        tmpdir = tempfile.TemporaryDirectory()
        self.addCleanup(tmpdir.cleanup)
        config = self._load_config(self._base_payload(Path(tmpdir.name) / "data"))
        selection = ScenarioSelector(
            config.scenarios, selection_mode="fixed", seed=config.session.seed
        ).select()
        recorder = EpisodeRecorder.from_config(config, session_id="failed_session")
        recorder._build_episode_manifest = lambda **_kwargs: (_ for _ in ()).throw(
            ValueError("manifest failure")
        )

        with self.assertRaisesRegex(ValueError, "manifest failure"):
            recorder.start_episode(selection)

        self.assertFalse(recorder.has_active_episode)
        with self.assertRaisesRegex(RuntimeError, "No active episode"):
            recorder.finalize_episode(status="failed")
