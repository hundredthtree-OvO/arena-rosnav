import json
import tempfile
import unittest
import sys
from pathlib import Path
from unittest.mock import patch

from toilet_benchmark.behavior_matrix_runner import (
    BehaviorMatrixRunSpec,
    BehaviorMatrixSpec,
    build_hunav_interactive_smoke_command,
    expand_behavior_matrix,
    resolve_target_resources,
    run_behavior_matrix,
    summarize_matrix_quality,
)
from toilet_benchmark.interactive_smoke_runner import discover_paths


class BehaviorMatrixRunnerTests(unittest.TestCase):
    def test_build_hunav_interactive_smoke_command_is_reproducible(self):
        paths = discover_paths()
        run_spec = BehaviorMatrixRunSpec(
            behavior="regular",
            initial_agents=2,
            intervention_profile="crossing_conflict",
            seed=12345,
        )

        command = build_hunav_interactive_smoke_command(
            paths,
            run_spec,
            run_output_root=Path("/tmp/matrix/000_regular"),
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource="urinal_1",
        )

        self.assertEqual(command[0], sys.executable)
        self.assertIn("--dynamic-crossing-profile", command)
        self.assertIn("crossing_conflict", command)
        self.assertIn("--initial-agents", command)
        self.assertIn("2", command)
        self.assertIn("--output-root", command)
        self.assertIn("/tmp/matrix/000_regular", command)
        self.assertIn("--benchmark", command)
        self.assertIn(str(paths.benchmark), command)

    def test_fixed_target_layouts_resolve_two_to_four_agents(self):
        self.assertEqual(
            resolve_target_resources(layout="spread", agent_count=2, custom=""),
            "urinal_1,urinal_5",
        )
        self.assertEqual(
            resolve_target_resources(layout="adjacent", agent_count=4, custom=""),
            "urinal_1,urinal_2,urinal_3,urinal_4",
        )
        with self.assertRaises(ValueError):
            resolve_target_resources(layout="spread", agent_count=5, custom="")

    def test_matrix_expansion_freezes_targets_for_each_agent_count(self):
        spec = BehaviorMatrixSpec(
            behaviors=("regular",),
            agent_counts=(2, 3, 4),
            intervention_profiles=("crossing_conflict",),
            seeds=(12345,),
            output_root=Path("/tmp/matrix"),
            dry_run=True,
            stop_on_failure=False,
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource="urinal_1",
            target_layout="spread",
            motion_backend="local_motion",
        )

        runs = expand_behavior_matrix(spec)

        self.assertEqual([run.initial_agents for run in runs], [2, 3, 4])
        self.assertEqual(runs[0].target_resource, "urinal_1,urinal_5")
        self.assertEqual(
            runs[-1].target_resource,
            "urinal_1,urinal_5,urinal_3,urinal_2",
        )

    def test_command_can_reuse_one_spawned_bridge_for_the_matrix(self):
        paths = discover_paths()
        run_spec = BehaviorMatrixRunSpec(
            behavior="regular",
            initial_agents=1,
            intervention_profile="crossing_clear",
            seed=12345,
        )

        command = build_hunav_interactive_smoke_command(
            paths,
            run_spec,
            run_output_root=Path("/tmp/matrix/reused"),
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource="urinal_1",
            reuse_existing_bridge=True,
            skip_spawn=True,
        )

        self.assertIn("--reuse-existing-bridge", command)
        self.assertIn("--skip-spawn", command)

    def test_local_motion_backend_is_forwarded_to_smoke(self):
        paths = discover_paths()
        run_spec = BehaviorMatrixRunSpec(
            behavior="regular",
            initial_agents=2,
            intervention_profile="crossing_conflict",
            seed=12345,
            target_resource="urinal_1,urinal_5",
        )

        command = build_hunav_interactive_smoke_command(
            paths,
            run_spec,
            run_output_root=Path("/tmp/matrix/local"),
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource=run_spec.target_resource,
            motion_backend="local_motion",
        )

        self.assertEqual(command[command.index("--motion-backend") + 1], "local_motion")
        self.assertEqual(
            command[command.index("--target-resource") + 1],
            "urinal_1,urinal_5",
        )

    def test_matrix_quality_aggregates_multi_agent_failures(self):
        quality = summarize_matrix_quality(
            (
                {
                    "subrun_summary": {
                        "completed": True,
                        "visual_envelope_gate_passed": False,
                        "raw_pose_pair_overlap_frames": 2,
                        "raw_pose_pair_min_distance_m": 0.42,
                        "raw_pose_pair_max_simultaneous_stop_duration_sec": 1.5,
                        "raw_pose_peer_activation_max_jump_excess_m": 0.03,
                        "local_motion_infeasible_sample_count": 4,
                        "local_motion_overlap_recovery_sample_count": 2,
                        "local_motion_min_peer_clearance_m": -0.01,
                        "local_motion_min_robot_clearance_m": -0.20,
                        "raw_pose_total_stop_episodes": 3,
                    }
                },
            )
        )

        self.assertEqual(quality["task_completed_run_count"], 1)
        self.assertEqual(quality["pair_overlap_run_count"], 1)
        self.assertAlmostEqual(quality["minimum_pair_distance_m"], 0.42)
        self.assertEqual(quality["local_motion_infeasible_sample_count"], 4)
        self.assertEqual(quality["local_motion_overlap_recovery_sample_count"], 2)
        self.assertEqual(quality["minimum_peer_clearance_m"], -0.01)
        self.assertEqual(quality["minimum_robot_clearance_m"], -0.20)

    def test_dry_run_writes_matrix_summary_without_launching_smoke(self):
        spec = BehaviorMatrixSpec(
            behaviors=("regular",),
            agent_counts=(1,),
            intervention_profiles=("custom",),
            seeds=(12345,),
            output_root=Path(tempfile.mkdtemp()),
            dry_run=True,
            stop_on_failure=False,
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource="urinal_1",
        )

        with patch("toilet_benchmark.behavior_matrix_runner.subprocess.run") as run_mock:
            summary = run_behavior_matrix(spec, paths=discover_paths())

        self.assertFalse(run_mock.called)
        self.assertTrue(summary["dry_run"])
        self.assertEqual(summary["planned_run_count"], 1)
        self.assertEqual(summary["completed_run_count"], 0)
        self.assertEqual(summary["failed_run_count"], 0)
        self.assertEqual(summary["runs"][0]["status"], "planned")
        matrix_summary_path = Path(summary["matrix_run_dir"]) / "matrix_summary.json"
        self.assertTrue(matrix_summary_path.is_file())
        on_disk = json.loads(matrix_summary_path.read_text(encoding="utf-8"))
        self.assertEqual(on_disk["runs"][0]["status"], "planned")

    def test_run_behavior_matrix_reads_summary_and_stops_on_failure(self):
        spec = BehaviorMatrixSpec(
            behaviors=("regular",),
            agent_counts=(1,),
            intervention_profiles=("custom", "crossing_clear"),
            seeds=(1, 2, 3),
            output_root=Path(tempfile.mkdtemp()),
            dry_run=False,
            stop_on_failure=True,
            reaction="disabled",
            legacy_avoidance="disabled",
            target_resource="urinal_1",
        )
        call_state = {"count": 0}

        def fake_run(command, **kwargs):
            call_state["count"] += 1
            output_root = Path(command[command.index("--output-root") + 1])
            summary_root = output_root / "20260730_000000_fake"
            summary_root.mkdir(parents=True, exist_ok=True)
            summary = {
                "completed": call_state["count"] == 1,
                "exit_code": 0 if call_state["count"] == 1 else 1,
                "run_index": call_state["count"],
            }
            (summary_root / "summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            return type(
                "Completed",
                (),
                {
                    "returncode": 0 if call_state["count"] == 1 else 1,
                    "stdout": f"run {call_state['count']}\n",
                },
            )()

        with patch("toilet_benchmark.behavior_matrix_runner.subprocess.run", side_effect=fake_run):
            summary = run_behavior_matrix(spec, paths=discover_paths())

        self.assertEqual(call_state["count"], 2)
        self.assertTrue(summary["stopped_on_failure"])
        self.assertEqual(summary["planned_run_count"], 6)
        self.assertEqual(summary["completed_run_count"], 1)
        self.assertEqual(summary["failed_run_count"], 1)
        self.assertEqual(len(summary["runs"]), 2)
        self.assertEqual(summary["runs"][0]["status"], "succeeded")
        self.assertEqual(summary["runs"][0]["subrun_summary"]["run_index"], 1)
        self.assertEqual(summary["runs"][1]["status"], "failed")
        self.assertEqual(summary["runs"][1]["subrun_summary"]["run_index"], 2)
        self.assertTrue(summary["runs"][1]["summary_path"].endswith("summary.json"))


if __name__ == "__main__":
    unittest.main()
