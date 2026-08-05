from __future__ import annotations

import math
import unittest
from types import SimpleNamespace

from toilet_benchmark.tracks.replay import AgentSnapshot
from toilet_benchmark.domain.task import (
    EXTERNAL_MOTION_REPLAY_TRACK,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
)
from toilet_benchmark.tracks.replay import (
    REPLAY_SCHEMA_VERSION,
    ReplayBundle,
    ReplayRuntime,
    ReplayStateFrame,
    seal_replay_bundle,
)
from toilet_benchmark.tracks.replay_isaac import (
    ReplayIsaacPlayback,
    ReplayIsaacRunner,
    _command_spec_from_frame,
)


def _bundle() -> ReplayBundle:
    payload = {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "benchmark_version": "0.1.0",
        "episode_id": "replay_test",
        "scene_id": "scene",
        "task_type": "enter_exit",
        "track": "replay",
        "seed": 1,
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
        "agents": [{
            "agent_id": "toilet_agent_01",
            "semantic_goal": "urinal_1",
            "start_reference": "entrance_main",
            "start_pose": [0.0, 0.0, 0.0, 0.0],
            "sample_rate_hz": 1.0,
            "trajectory": [
                {"sample_index": 0, "timestamp_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 1, "timestamp_sec": 1.0, "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
            ],
        }],
        "annotations": {"termination_reason": "success", "termination_timestamp_sec": 1.0},
        "provenance": {
            "source_episode_manifest": "metadata.yaml",
            "source_episode_hash": "a" * 64,
            "generated_by": "test",
            "generated_at_sec": 0.0,
            "content_hash": "",
        },
    }
    return seal_replay_bundle(ReplayBundle.from_mapping(payload))


def _loop_bundle() -> ReplayBundle:
    payload = {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "benchmark_version": "0.1.0",
        "episode_id": "loop_replay_test",
        "scene_id": "scene",
        "task_type": "enter_exit",
        "track": "replay",
        "seed": 2,
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
        "agents": [{
            "agent_id": "toilet_agent_01",
            "semantic_goal": "urinal_1",
            "start_reference": "entrance_main",
            "start_pose": [0.0, 0.0, 0.0, 0.0],
            "sample_rate_hz": 1.0,
            "trajectory": [
                {"sample_index": 0, "timestamp_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 1, "timestamp_sec": 2.0, "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": -0.5, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 2, "timestamp_sec": 4.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 0.5, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 3, "timestamp_sec": 6.0, "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
            ],
        }],
        "annotations": {"termination_reason": "success", "termination_timestamp_sec": 6.0},
        "provenance": {
            "source_episode_manifest": "metadata.yaml",
            "source_episode_hash": "b" * 64,
            "generated_by": "test",
            "generated_at_sec": 0.0,
            "content_hash": "",
        },
    }
    return seal_replay_bundle(ReplayBundle.from_mapping(payload))


def _metrics_bundle() -> ReplayBundle:
    payload = {
        "schema_version": REPLAY_SCHEMA_VERSION,
        "benchmark_version": "0.1.0",
        "episode_id": "metrics_replay_test",
        "scene_id": "scene",
        "task_type": "enter_exit",
        "track": "replay",
        "seed": 3,
        "reference_episode_hash": "c" * 64,
        "coordinate_frame": "world",
        "time_base": {
            "origin": "episode_reset",
            "clock": "physics_monotonic_sec",
            "unit": "sec",
            "zero_sec": 0.0,
            "monotonic": True,
            "sample_policy": "strictly_increasing",
        },
        "agents": [{
            "agent_id": "toilet_agent_01",
            "semantic_goal": "urinal_1",
            "start_reference": "entrance_main",
            "start_pose": [0.0, 0.0, 0.0, 0.0],
            "sample_rate_hz": 1.0,
            "trajectory": [
                {"sample_index": 0, "timestamp_sec": 0.0, "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 1, "timestamp_sec": 1.0, "x": 1.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 1.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
                {"sample_index": 2, "timestamp_sec": 2.0, "x": 2.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0, "wz": 0.0, "radius_m": 0.3, "source": "replay_reference"},
            ],
        }],
        "annotations": {"termination_reason": "success", "termination_timestamp_sec": 2.0},
        "provenance": {
            "source_episode_manifest": "metadata.yaml",
            "source_episode_hash": "c" * 64,
            "generated_by": "test",
            "generated_at_sec": 0.0,
            "content_hash": "",
        },
    }
    return seal_replay_bundle(ReplayBundle.from_mapping(payload))


class ReplayIsaacTest(unittest.TestCase):
    class RecordingAdapter:
        def __init__(self):
            self.dispatches = []

        def send(self, dispatch, done_callback=None):
            self.dispatches.append(dispatch)
            return dispatch

    def test_frame_maps_to_external_motion_contract(self):
        frame = ReplayStateFrame(
            sample_index=3,
            timestamp_sec=0.5,
            snapshot=AgentSnapshot(
                agent_id="agent",
                x=1.0,
                y=2.0,
                z=0.0,
                yaw=0.4,
                vx=0.3,
                vy=-0.2,
            ),
            finished=False,
        )
        command = _command_spec_from_frame(
            frame, external_timeout_sec=0.35, terminal_mode="align"
        )
        self.assertEqual(command.direct_pose, (1.0, 2.0, 0.0))
        self.assertEqual(command.external_velocity, (0.3, -0.2, 0.0))
        self.assertEqual(
            command.external_motion_mode,
            EXTERNAL_MOTION_REPLAY_TRACK,
        )

    def test_people_callback_parses_yaw_tags(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            monotonic_fn=monotonic,
        )
        person = SimpleNamespace(
            name="toilet_agent_01",
            reliability=1.0,
            position=SimpleNamespace(x=1.0, y=2.0, z=0.0),
            velocity=SimpleNamespace(x=0.1, y=0.2),
            tagnames=["yaw_valid", "yaw_rad"],
            tags=["true", "1.25"],
        )
        runner._people_cb(
            SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(sec=12, nanosec=500_000_000)),
                people=[person],
            )
        )
        self.assertIsNotNone(runner._live_pose)
        self.assertAlmostEqual(runner._live_pose.yaw, 1.25)
        self.assertAlmostEqual(runner._live_pose.source_stamp_sec, 12.5)

    def test_terminal_frame_uses_terminal_align(self):
        playback = ReplayIsaacPlayback(_bundle(), rate_hz=10.0)
        result = playback.dry_run()
        self.assertTrue(result.dispatches[-1].finished)
        self.assertEqual(
            result.dispatches[-1].command_spec.external_motion_mode,
            EXTERNAL_MOTION_TERMINAL_ALIGN,
        )
        self.assertEqual(result.dispatches[-1].command_spec.external_velocity, (0.0, 0.0, 0.0))

    def test_time_scale_changes_dispatch_count_not_terminal_pose(self):
        normal = ReplayIsaacPlayback(_bundle(), rate_hz=10.0, time_scale=1.0).dry_run()
        fast = ReplayIsaacPlayback(_bundle(), rate_hz=10.0, time_scale=2.0).dry_run()
        self.assertLess(len(fast.dispatches), len(normal.dispatches))
        self.assertEqual(
            fast.dispatches[-1].command_spec.direct_pose,
            normal.dispatches[-1].command_spec.direct_pose,
        )

    def test_slow_time_scale_still_reaches_terminal_frame(self):
        result = ReplayIsaacPlayback(
            _bundle(), rate_hz=10.0, time_scale=0.1
        ).dry_run()
        self.assertTrue(result.dispatches[-1].finished)

    def test_terminal_handshake_requires_valid_live_yaw(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            terminal_stable_samples=1,
            terminal_timeout_sec=0.2,
            monotonic_fn=monotonic,
        )
        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        clock["value"] = 0.1
        runner.update_live_pose(
            {
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        self.assertIsNotNone(runner.tick(physics_now_sec=1.0))
        clock["value"] = 0.5
        self.assertIsNone(runner.tick(physics_now_sec=1.2))
        self.assertTrue(runner.finished)
        self.assertEqual(runner.terminal_result.result, "timeout")

    def test_live_catch_up_is_bounded_by_fresh_pose(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            catch_up_sec=0.1,
            monotonic_fn=monotonic,
        )

        first = runner.tick(physics_now_sec=0.0)
        self.assertIsNotNone(first)
        clock["value"] = 0.2
        runner.update_live_pose(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        second = runner.tick(physics_now_sec=1.0)
        self.assertIsNotNone(second)
        self.assertAlmostEqual(second.playback_sec, 0.1)

    def test_live_catch_up_relaxes_after_sustained_progress_stall(self):
        clock = {"value": 0.0}

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            catch_up_sec=0.1,
            monotonic_fn=lambda: clock["value"],
        )
        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        runner.update_live_pose(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=0.0,
        )
        clock["value"] = 0.1
        bounded = runner.tick(physics_now_sec=1.0)
        self.assertAlmostEqual(bounded.playback_sec, 0.1)

        clock["value"] = 1.6
        runner.update_live_pose(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        relaxed = runner.tick(physics_now_sec=1.0)

        self.assertIsNotNone(relaxed)
        self.assertGreater(relaxed.playback_sec, bounded.playback_sec)
        self.assertAlmostEqual(relaxed.playback_sec, 0.6)

    def test_terminal_handshake_converges_after_stable_samples(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            terminal_stable_samples=3,
            terminal_timeout_sec=1.0,
            monotonic_fn=monotonic,
        )

        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        terminal_dispatches = []
        for index, physics_now_sec in enumerate((1.0, 1.1, 1.2), start=1):
            clock["value"] = 0.1 * index
            runner.update_live_pose(
                {
                    "x": 1.0,
                    "y": 0.0,
                    "z": 0.0,
                    "yaw": 0.0,
                    "vx": 0.0,
                    "vy": 0.0,
                },
                agent_id="toilet_agent_01",
                received_monotonic_sec=clock["value"],
            )
            terminal_dispatches.append(runner.tick(physics_now_sec=physics_now_sec))

        self.assertTrue(runner.finished)
        self.assertIsNotNone(runner.terminal_result)
        self.assertEqual(runner.terminal_result.result, "converged")
        self.assertGreaterEqual(runner.terminal_result.stable_samples, 3)
        self.assertIsNotNone(terminal_dispatches[0])
        self.assertIsNotNone(terminal_dispatches[1])
        self.assertIsNone(terminal_dispatches[2])

    def test_live_progress_stays_monotonic_on_out_and_back_loop(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _loop_bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            catch_up_sec=0.1,
            monotonic_fn=monotonic,
        )
        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        clock["value"] = 0.1
        runner.update_live_pose(
            {
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        self.assertIsNotNone(runner.tick(physics_now_sec=1.0))
        clock["value"] = 0.2
        runner.update_live_pose(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        third = runner.tick(physics_now_sec=2.0)
        self.assertIsNotNone(third)
        self.assertGreaterEqual(third.playback_sec, 2.0)
        self.assertGreaterEqual(third.playback_sec, 1.0)

    def test_live_progress_uses_timestamp_span_not_segment_length(self):
        runner = ReplayIsaacRunner(
            _loop_bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            catch_up_sec=0.1,
            monotonic_fn=lambda: 0.0,
        )
        runner.update_live_pose(
            {
                "x": 0.25,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=0.0,
        )

        self.assertAlmostEqual(runner._estimate_live_progress_sec(), 0.5)

    def test_live_progress_can_remain_behind_dispatched_reference(self):
        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            catch_up_sec=0.1,
            monotonic_fn=lambda: 0.0,
        )
        runner._last_playback_sec = 0.8
        runner._last_live_progress_sec = 0.0
        runner.update_live_pose(
            {
                "x": 0.25,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=0.0,
        )

        self.assertAlmostEqual(runner._estimate_live_progress_sec(), 0.25)

    def test_terminal_handshake_times_out(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            terminal_stable_samples=3,
            terminal_timeout_sec=0.25,
            monotonic_fn=monotonic,
        )

        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        clock["value"] = 0.1
        self.assertIsNotNone(runner.tick(physics_now_sec=1.0))
        clock["value"] = 0.5
        self.assertIsNone(runner.tick(physics_now_sec=1.1))

        self.assertTrue(runner.finished)
        self.assertIsNotNone(runner.terminal_result)
        self.assertEqual(runner.terminal_result.result, "timeout")

    def test_final_report_includes_tracking_metrics(self):
        clock = {"value": 0.0}

        def monotonic():
            return clock["value"]

        runner = ReplayIsaacRunner(
            _metrics_bundle(),
            service_name="/isaac/move_pedestrians",
            adapter=self.RecordingAdapter(),
            terminal_stable_samples=1,
            terminal_timeout_sec=1.0,
            monotonic_fn=monotonic,
        )

        self.assertIsNotNone(runner.tick(physics_now_sec=0.0))
        runner.update_live_pose(
            {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 1.0,
                "vy": 0.0,
                "source_stamp_sec": 100.0,
                "reference_playback_sec": 0.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        clock["value"] = 0.1
        self.assertIsNotNone(runner.tick(physics_now_sec=1.0))
        runner.update_live_pose(
            {
                "x": 1.0,
                "y": 0.5,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 1.0,
                "vy": 0.0,
                "source_stamp_sec": 101.0,
                "reference_playback_sec": 1.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        clock["value"] = 0.2
        runner.update_live_pose(
            {
                "x": 2.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "source_stamp_sec": 102.0,
                "reference_playback_sec": 2.0,
            },
            agent_id="toilet_agent_01",
            received_monotonic_sec=clock["value"],
        )
        final_dispatch = runner.tick(physics_now_sec=2.0)
        self.assertIsNotNone(final_dispatch)
        self.assertTrue(final_dispatch.finished)

        metrics = runner.tracking_metrics()
        self.assertEqual(metrics.live_sample_count, 3)
        self.assertEqual(metrics.reference_sample_count, 3)
        self.assertEqual(metrics.time_aligned_sample_count, 3)
        self.assertAlmostEqual(
            metrics.time_aligned_position_rmse_m,
            math.sqrt(0.25 / 3.0),
        )
        self.assertAlmostEqual(
            metrics.path_lateral_error_rmse_m,
            math.sqrt(0.25 / 3.0),
        )
        self.assertAlmostEqual(metrics.path_lateral_error_p95_m, 0.5)
        self.assertAlmostEqual(metrics.path_lateral_error_max_m, 0.5)
        self.assertAlmostEqual(metrics.terminal_position_error_m, 0.0)
        self.assertAlmostEqual(metrics.terminal_yaw_error_rad, 0.0)

        report = runner.final_report()
        self.assertTrue(report["finished"])
        self.assertIn("tracking_metrics", report)
        self.assertEqual(
            report["tracking_metrics"]["time_aligned_sample_count"],
            3,
        )
        self.assertAlmostEqual(
            report["tracking_metrics"]["path_lateral_error_rmse_m"],
            math.sqrt(0.25 / 3.0),
        )


if __name__ == "__main__":
    unittest.main()
