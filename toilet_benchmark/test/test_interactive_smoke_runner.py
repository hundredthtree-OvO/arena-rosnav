import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import yaml

from toilet_benchmark.interactive_smoke_runner import (
    DYNAMIC_CROSSING_PROFILES,
    SCENARIO_PROFILES,
    _robot_intervention_arg,
    _latest_agent_x,
    _pose_arg,
    _build_parser,
    apply_visual_envelope_gate,
    build_dynamic_crossing_command,
    build_commands,
    build_robot_reset_command,
    discover_paths,
    make_benchmark_variant,
    resolve_dynamic_crossing_profile,
    run_director_with_dynamic_crossing,
    run_director_with_occupied_passage,
    stop_dynamic_crossing,
    summarize_director_log,
    summarize_local_motion_diagnostics,
    summarize_raw_pose_log,
    summarize_visual_envelope_log,
)


class InteractiveSmokeRunnerTests(unittest.TestCase):
    def test_discovers_current_workspace_and_packages(self):
        paths = discover_paths()

        self.assertEqual(paths.workspace.name, "arena_ws")
        self.assertTrue((paths.arena_isaac / "scripts" / "arena_scene_profile.py").is_file())
        self.assertTrue(paths.benchmark.is_file())

    def test_build_commands_expands_distinct_target_resources(self):
        paths = discover_paths()
        _, _, director = build_commands(
            paths,
            benchmark=paths.benchmark,
            behavior="regular",
            target_resource="urinal_1,urinal_5",
            initial_agents=2,
        )

        targets = [
            director[index + 1]
            for index, value in enumerate(director)
            if value == "--target-resource"
        ]
        self.assertEqual(targets, ["urinal_1", "urinal_5"])

    def test_build_commands_forwards_scenario_runtime_override(self):
        paths = discover_paths()
        _, _, director = build_commands(
            paths,
            benchmark=paths.benchmark,
            behavior="regular",
            target_resource="urinal_1",
            initial_agents=1,
            scenario_runtime="takeover",
        )

        self.assertEqual(
            director[director.index("--scenario-runtime") + 1],
            "takeover",
        )

    def test_variant_isolated_diagnostics_and_reaction_override(self):
        source_payload = {
            "motion_backend": {
                "hunav": {
                    "robot_proximity_reaction": {"enabled": True},
                }
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.yaml"
            destination = root / "variant.yaml"
            source.write_text(yaml.safe_dump(source_payload), encoding="utf-8")

            result = make_benchmark_variant(
                source,
                destination,
                diagnostic_dir=root / "diagnostics",
                reaction_override=False,
            )

            hunav = result["motion_backend"]["hunav"]
            self.assertFalse(hunav["robot_proximity_reaction"]["enabled"])
            self.assertEqual(hunav["diagnostic_log_dir"], str(root / "diagnostics"))
            self.assertTrue(destination.is_file())

    def test_variant_can_enable_local_motion_shadow_without_editing_source(self):
        source_payload = {
            "motion_backend": {
                "hunav": {"local_motion_shadow": {"enabled": False}}
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.yaml"
            destination = root / "variant.yaml"
            source.write_text(yaml.safe_dump(source_payload), encoding="utf-8")

            result = make_benchmark_variant(
                source,
                destination,
                diagnostic_dir=root / "diagnostics",
                reaction_override=None,
                local_motion_shadow_override=True,
            )

        self.assertTrue(
            result["motion_backend"]["hunav"]["local_motion_shadow"]["enabled"]
        )

    def test_occupied_passage_variant_freezes_activation_and_service_timing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.yaml"
            destination = root / "variant.yaml"
            source.write_text("{}\n", encoding="utf-8")

            result = make_benchmark_variant(
                source,
                destination,
                diagnostic_dir=root / "diagnostics",
                reaction_override=None,
                scenario_profile=SCENARIO_PROFILES["occupied_passage"],
            )

        self.assertEqual(result["director"]["initial_spawn_interval_sec"], 40.0)
        self.assertEqual(result["service_time_sec"]["urinal"], [55.0, 55.0])

    def test_passable_passage_profiles_share_geometry_but_vary_destination(self):
        first = SCENARIO_PROFILES["passable_passage_u1"]
        second = SCENARIO_PROFILES["passable_passage_u2"]

        self.assertEqual(first.target_resource, "urinal_3,urinal_1")
        self.assertEqual(second.target_resource, "urinal_3,urinal_2")
        self.assertEqual(first.robot_blocking_pose, (0.0, 0.0, 0.03, 0.0))
        self.assertEqual(second.robot_blocking_pose, first.robot_blocking_pose)
        self.assertEqual(first.trigger_marker, "toilet_agent_01 started using urinal_3")
        self.assertEqual(first.initial_spawn_interval_sec, 40.0)
        self.assertEqual(first.urinal_service_time_sec, (70.0, 70.0))

    def test_diagnostic_summary_counts_avoidance_side_commitment(self):
        def diagnostic_line(timestamp, avoidance_side):
            return (
                f"[INFO] [{timestamp:.3f}] [toilet_director_node]: "
                "HuNav diagnostics: agent=toilet_agent_01, generation=1, "
                "phase=WALK_TO_URINAL, pose=(-1.00,0.00), yaw=0.000, "
                "speed=0.500, route_progress_m=1.00, "
                f"avoidance_side={avoidance_side:+d}, "
                "behavior_profile=regular, native_behavior=regular, "
                "native_behavior_state=inactive, pedestrian_pair_contacts=0"
            )

        text = "\n".join(
            (
                diagnostic_line(10.0, avoidance_side=0),
                diagnostic_line(11.0, avoidance_side=1),
                diagnostic_line(12.0, avoidance_side=1),
                diagnostic_line(13.0, avoidance_side=-1),
                diagnostic_line(14.0, avoidance_side=0),
                diagnostic_line(15.0, avoidance_side=-1),
            )
        )

        result = summarize_director_log(text, exit_code=0)
        metrics = result["diagnostic_samples_by_agent"]["toilet_agent_01"]

        self.assertEqual(metrics["avoidance_episode_count"], 2)
        self.assertEqual(metrics["avoidance_active_sample_count"], 4)
        self.assertEqual(metrics["avoidance_side_switch_count"], 1)

    def test_diagnostic_summary_aggregates_local_motion_shadow(self):
        def diagnostic_line(timestamp, feasible, error, behavior_mode):
            return (
                f"[INFO] [{timestamp:.3f}] [toilet_director_node]: "
                "HuNav diagnostics: agent=toilet_agent_01, generation=1, "
                "phase=WALK_TO_URINAL, pose=(-1.00,0.00), yaw=0.000, "
                "speed=0.500, route_progress_m=1.00, avoidance_side=+0, "
                "local_shadow_enabled=True, "
                f"local_shadow_feasible={feasible}, "
                f"local_shadow_velocity_error_mps={error:.3f}, "
                f"local_shadow_behavior_mode={behavior_mode}, "
                "local_shadow_selected_speed_mps=0.300, "
                f"local_shadow_avoidance_active={'True' if feasible == 'False' else 'False'}, "
                "behavior_profile=regular, native_behavior=regular, "
                "native_behavior_state=inactive, pedestrian_pair_contacts=0"
            )

        text = "\n".join(
            (
                diagnostic_line(10.0, "True", 0.10, "walking"),
                diagnostic_line(11.0, "False", 0.40, "yielding"),
                diagnostic_line(12.0, "True", 0.20, "walking"),
            )
        )

        result = summarize_director_log(text, exit_code=0)
        metrics = result["diagnostic_samples_by_agent"]["toilet_agent_01"]

        self.assertEqual(metrics["local_shadow_sample_count"], 3)
        self.assertEqual(metrics["local_shadow_feasible_sample_count"], 2)
        self.assertAlmostEqual(metrics["local_shadow_feasible_ratio"], 2.0 / 3.0)
        self.assertAlmostEqual(metrics["local_shadow_velocity_error_mean_mps"], 0.7 / 3.0)
        self.assertEqual(metrics["local_shadow_velocity_error_max_mps"], 0.4)
        self.assertEqual(metrics["local_shadow_avoidance_active_sample_count"], 1)
        self.assertAlmostEqual(metrics["local_shadow_selected_speed_mean_mps"], 0.3)
        self.assertEqual(
            metrics["local_shadow_behavior_modes"],
            {"walking": 2, "yielding": 1},
        )
        self.assertEqual(result["local_shadow_sample_count"], 3)
        self.assertEqual(result["local_shadow_infeasible_sample_count"], 1)

    def test_pair_distance_uses_planar_euclidean_distance_once(self):
        def diagnostic_line(agent_id, x, y):
            return (
                "[INFO] [10.000] [toilet_director_node]: "
                f"HuNav diagnostics: agent={agent_id}, generation=1, "
                f"phase=WALK_TO_URINAL, pose=({x:.2f},{y:.2f}), yaw=0.000, "
                "speed=0.500, route_progress_m=1.00, avoidance_side=+0, "
                "behavior_profile=regular, native_behavior=regular, "
                "native_behavior_state=inactive, pedestrian_pair_contacts=0"
            )

        result = summarize_director_log(
            "\n".join(
                (
                    diagnostic_line("toilet_agent_01", 0.0, 0.0),
                    diagnostic_line("toilet_agent_02", 0.3, 0.4),
                )
            ),
            exit_code=0,
        )

        self.assertAlmostEqual(result["pedestrian_pair_min_distance_m"], 0.5)

    def test_summary_counts_e1_shadow_mismatches(self):
        text = "\n".join(
            (
                "E1 scenario shadow mismatch: agent=toilet_agent_01",
                "E1 scenario shadow mismatch: agent=toilet_agent_02",
            )
        )

        result = summarize_director_log(text, exit_code=0)

        self.assertEqual(result["scenario_shadow_mismatch_events"], 2)

    def test_variant_preserves_seed_and_behavior_blocks_without_override(self):
        source_payload = {
            "arrival": {"seed": 42},
            "motion_backend": {
                "hunav": {
                    "robot_proximity_reaction": {"enabled": True},
                    "behavior": {
                        "surprised": {"duration": 60.0},
                        "scared": {"duration": 12.0},
                    },
                }
            },
            "hunav_profile": {
                "regular": {"behavior_type": "REGULAR"},
                "surprised": {"behavior_type": "SURPRISED"},
                "scared": {"behavior_type": "SCARED"},
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.yaml"
            destination = root / "variant.yaml"
            source.write_text(yaml.safe_dump(source_payload), encoding="utf-8")

            result = make_benchmark_variant(
                source,
                destination,
                diagnostic_dir=root / "diagnostics",
                reaction_override=None,
            )

            self.assertEqual(result["arrival"]["seed"], 42)
            self.assertTrue(result["motion_backend"]["hunav"]["robot_proximity_reaction"]["enabled"])
            self.assertEqual(
                result["motion_backend"]["hunav"]["behavior"]["surprised"]["duration"],
                60.0,
            )
            self.assertEqual(
                result["hunav_profile"]["scared"]["behavior_type"],
                "SCARED",
            )

    def test_variant_overrides_seed_and_legacy_avoidance(self):
        source_payload = {
            "arrival": {"seed": 1},
            "motion_backend": {
                "hunav": {
                    "regular_avoidance": {"enabled": True},
                }
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "source.yaml"
            destination = root / "variant.yaml"
            source.write_text(yaml.safe_dump(source_payload), encoding="utf-8")

            result = make_benchmark_variant(
                source,
                destination,
                diagnostic_dir=root / "diagnostics",
                reaction_override=None,
                avoidance_override=False,
                seed_override=12345,
            )

            self.assertEqual(result["arrival"]["seed"], 12345)
            self.assertFalse(
                result["motion_backend"]["hunav"]["regular_avoidance"]["enabled"]
            )

    def test_build_commands_passes_behavior_and_run_overrides_to_director(self):
        paths = discover_paths()

        _, _, director = build_commands(
            paths,
            benchmark=Path("/tmp/variant.yaml"),
            behavior="scared",
            target_resource="urinal_4",
            initial_agents=2,
        )

        self.assertEqual(
            director,
            [
                "ros2",
                "run",
                "toilet_benchmark",
                "toilet_director_node",
                "--benchmark",
                "/tmp/variant.yaml",
                "--motion-backend",
                "hunav",
                "--profile",
                "scared",
                "--initial-agents",
                "2",
                "--target-resource",
                "urinal_4",
            ],
        )

    def test_build_commands_can_select_local_motion_backend(self):
        paths = discover_paths()

        _, _, director = build_commands(
            paths,
            benchmark=Path("/tmp/variant.yaml"),
            motion_backend="local_motion",
            behavior="regular",
            target_resource="urinal_1",
            initial_agents=1,
        )

        self.assertEqual(
            director[director.index("--motion-backend") + 1],
            "local_motion",
        )

    def test_build_robot_reset_command_uses_ros_pose_quaternion(self):
        command = build_robot_reset_command((1.0, -2.0, 0.03, 3.141592653589793))

        self.assertEqual(command[:5], [
            "ros2",
            "service",
            "call",
            "/isaac/reset_mecanum_episode",
            "isaacsim_msgs/srv/ResetRobot",
        ])
        request = yaml.safe_load(command[5])
        self.assertEqual(request["name"], "xms_mecanum")
        self.assertEqual(request["pose"]["position"], {"x": 1.0, "y": -2.0, "z": 0.03})
        self.assertAlmostEqual(request["pose"]["orientation"]["z"], 1.0)
        self.assertAlmostEqual(request["pose"]["orientation"]["w"], 0.0, places=6)

    def test_robot_intervention_arg_normalizes_dynamic_crossing_alias(self):
        self.assertEqual(_robot_intervention_arg("dynamic-crossing"), "dynamic_crossing")
        self.assertEqual(_robot_intervention_arg("dynamic_crossing"), "dynamic_crossing")
        self.assertEqual(_robot_intervention_arg("parked-away"), "parked_away")
        self.assertEqual(_robot_intervention_arg("fixed-origin"), "fixed_origin")
        self.assertEqual(
            _robot_intervention_arg("occupied-passage"),
            "occupied_passage",
        )

    def test_dynamic_crossing_command_uses_cmd_vel_pulse_forward_motion(self):
        command = build_dynamic_crossing_command(
            topic="/cmd_vel_gamepad_diff",
            linear_x=0.24,
            duration_sec=2.5,
        )

        self.assertEqual(
            command,
            [
                "ros2",
                "run",
                "ros2isaacsim",
                "cmd_vel_pulse",
                "--topic",
                "/cmd_vel_gamepad_diff",
                "--motion",
                "forward",
                "--duration",
                "2.5",
                "--rate",
                "20.0",
                "--linear",
                "0.24",
            ],
        )

    def test_dynamic_crossing_profile_resolution_uses_custom_defaults(self):
        parsed = _build_parser().parse_args([])
        resolved, resolved_name = resolve_dynamic_crossing_profile(
            parsed.dynamic_crossing_profile,
            reset_pose=getattr(parsed, "dynamic_crossing_reset_pose", None),
            trigger_x=getattr(parsed, "dynamic_crossing_trigger_x", None),
            topic=getattr(parsed, "dynamic_crossing_topic", None),
            linear_x=getattr(parsed, "dynamic_crossing_linear_x", None),
            motion_sec=getattr(parsed, "dynamic_crossing_motion_sec", None),
        )

        self.assertEqual(resolved_name, "custom")
        self.assertEqual(resolved.reset_pose[:3], (0.0, -0.8, 0.03))
        self.assertAlmostEqual(resolved.reset_pose[3], math.pi / 2.0)
        self.assertEqual(resolved.motion_sec, 5.0)

    def test_default_smoke_targets_distinct_resources_for_multi_agent_runs(self):
        parsed = _build_parser().parse_args([])

        self.assertEqual(
            parsed.target_resource,
            "urinal_1,urinal_2,urinal_3,urinal_4",
        )

    def test_dynamic_crossing_profile_resolution_honors_selection_and_overrides(self):
        clear_profile, clear_name = resolve_dynamic_crossing_profile("crossing_clear")
        resolved, resolved_name = resolve_dynamic_crossing_profile(
            "crossing_clear",
            trigger_x=-2.0,
            linear_x=0.18,
        )

        self.assertEqual(clear_name, "crossing_clear")
        self.assertGreater(
            clear_profile.trigger_x,
            DYNAMIC_CROSSING_PROFILES["crossing_conflict"].trigger_x,
        )
        self.assertEqual(
            DYNAMIC_CROSSING_PROFILES["crossing_conflict"].trigger_x,
            -2.0,
        )
        self.assertEqual(resolved_name, "custom")
        self.assertEqual(resolved.reset_pose, DYNAMIC_CROSSING_PROFILES["crossing_clear"].reset_pose)
        self.assertEqual(resolved.post_pose, DYNAMIC_CROSSING_PROFILES["crossing_clear"].post_pose)
        self.assertEqual(resolved.topic, "/cmd_vel_gamepad_diff")
        self.assertEqual(resolved.trigger_x, -2.0)
        self.assertEqual(resolved.linear_x, 0.18)
        self.assertEqual(resolved.motion_sec, DYNAMIC_CROSSING_PROFILES["crossing_clear"].motion_sec)

    def test_dynamic_crossing_resets_pose_before_trigger_and_returns_cleanly(self):
        class FakeProcess:
            def __init__(self):
                self._poll_count = 0
                self.pid = 1234
                self.returncode = 0

            def poll(self):
                self._poll_count += 1
                return None if self._poll_count < 2 else 0

            def wait(self, timeout=None):
                return 0

        reset_poses = []
        command_calls = []

        def fake_run(command, **kwargs):
            command_calls.append(list(command))
            return type("Completed", (), {"returncode": 0, "stdout": "ok\n"})()

        def fake_reset(pose, **kwargs):
            reset_poses.append(tuple(pose))
            return True

        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory) / "director.log"
            event_log = Path(directory) / "robot_intervention.jsonl"
            with patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.Popen",
                return_value=FakeProcess(),
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.run",
                side_effect=fake_run,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.wait_for_robot_reset_service",
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.reset_robot_for_intervention",
                side_effect=fake_reset,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner._latest_agent_x",
                return_value=None,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.stop_process_group"
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.time.sleep"
            ):
                result = run_director_with_dynamic_crossing(
                    ["ros2", "run", "toilet_benchmark", "toilet_director_node"],
                    cwd=Path(directory),
                    env={},
                    log_path=log_path,
                    event_log=event_log,
                    reset_pose=(0.0, -2.0, 0.03, 3.141592653589793 / 2.0),
                    trigger_x=-1.0,
                    topic="/cmd_vel_gamepad_diff",
                    linear_x=0.2,
                    motion_sec=1.0,
                    timeout_sec=2.0,
                )

        self.assertEqual(result, 0)
        self.assertEqual(len(reset_poses), 1)
        self.assertEqual(reset_poses[0][3], 3.141592653589793 / 2.0)
        self.assertEqual(command_calls, [])

    def test_dynamic_crossing_trigger_runs_motion_then_explicit_zero_pulse(self):
        class FakeProcess:
            def __init__(self):
                self._poll_count = 0
                self.pid = 1234
                self.returncode = 0

            def poll(self):
                self._poll_count += 1
                return None if self._poll_count < 3 else 0

            def wait(self, timeout=None):
                return 0

        command_calls = []

        def fake_run(command, **kwargs):
            command_calls.append(list(command))
            return type("Completed", (), {"returncode": 0, "stdout": "ok\n"})()

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            event_log = root / "events.jsonl"
            with patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.Popen",
                return_value=FakeProcess(),
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.run",
                side_effect=fake_run,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.wait_for_robot_reset_service",
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.reset_robot_for_intervention",
                return_value=True,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner._latest_agent_x",
                return_value=-0.5,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.stop_process_group"
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.time.sleep"
            ):
                result = run_director_with_dynamic_crossing(
                    ["director"],
                    cwd=root,
                    env={},
                    log_path=root / "director.log",
                    event_log=event_log,
                    reset_pose=(0.0, -0.8, 0.03, math.pi / 2.0),
                    trigger_x=-1.0,
                    topic="/cmd_vel_gamepad_diff",
                    linear_x=0.2,
                    motion_sec=1.0,
                    timeout_sec=2.0,
                )

            events = [
                json.loads(line)
                for line in event_log.read_text(encoding="utf-8").splitlines()
            ]

        self.assertEqual(result, 0)
        self.assertEqual(len(command_calls), 2)
        self.assertEqual(command_calls[0][-2:], ["--linear", "0.2"])
        self.assertIn("--duration", command_calls[0])
        self.assertEqual(command_calls[1][-2:], ["--linear", "0.0"])
        self.assertEqual(
            [event["event"] for event in events],
            [
                "robot_dynamic_crossing_triggered",
                "robot_dynamic_crossing_cmd_vel_pulse_completed",
                "robot_dynamic_crossing_zero_command",
            ],
        )

    def test_dynamic_crossing_moves_robot_to_post_pose_after_pulse(self):
        class FakeProcess:
            pid = 1234
            returncode = 0

            def __init__(self):
                self.poll_count = 0

            def poll(self):
                self.poll_count += 1
                return None if self.poll_count < 3 else 0

        reset_poses = []
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.Popen",
                return_value=FakeProcess(),
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.run",
                return_value=type("Completed", (), {"returncode": 0, "stdout": "ok\n"})(),
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.wait_for_robot_reset_service",
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.reset_robot_for_intervention",
                side_effect=lambda pose, **kwargs: reset_poses.append(tuple(pose)) or True,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner._latest_agent_x",
                return_value=0.0,
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.stop_process_group"
            ), patch(
                "toilet_benchmark.interactive_smoke_runner.time.sleep"
            ):
                result = run_director_with_dynamic_crossing(
                    ["director"],
                    cwd=root,
                    env={},
                    log_path=root / "director.log",
                    event_log=root / "events.jsonl",
                    reset_pose=(1.0, -0.8, 0.03, math.pi / 2.0),
                    trigger_x=-1.0,
                    topic="/cmd_vel_gamepad_diff",
                    linear_x=0.2,
                    motion_sec=1.0,
                    timeout_sec=2.0,
                    post_pose=(4.0, -3.0, 0.03, 0.0),
                )

        self.assertEqual(result, 0)
        self.assertEqual(
            reset_poses,
            [
                (4.0, -3.0, 0.03, 0.0),
                (1.0, -0.8, 0.03, math.pi / 2.0),
                (4.0, -3.0, 0.03, 0.0),
            ],
        )

    def test_occupied_passage_latches_robot_after_first_agent_service(self):
        class FakeProcess:
            pid = 1234
            returncode = 0

            def __init__(self):
                self.poll_count = 0

            def poll(self):
                self.poll_count += 1
                return None if self.poll_count < 3 else 0

        events = []

        def fake_reset(pose, **kwargs):
            events.append(kwargs["event"])
            return True

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with (
                patch(
                    "toilet_benchmark.interactive_smoke_runner.subprocess.Popen",
                    return_value=FakeProcess(),
                ),
                patch(
                    "toilet_benchmark.interactive_smoke_runner.wait_for_robot_reset_service"
                ),
                patch(
                    "toilet_benchmark.interactive_smoke_runner.reset_robot_for_intervention",
                    side_effect=fake_reset,
                ),
                patch(
                    "toilet_benchmark.interactive_smoke_runner._tail",
                    return_value=(
                        "toilet_agent_01 started using urinal_2\n"
                        "toilet_agent_01 leaving urinal_2"
                    ),
                ),
                patch(
                    "toilet_benchmark.interactive_smoke_runner.stop_process_group"
                ),
                patch("toilet_benchmark.interactive_smoke_runner.time.sleep"),
            ):
                result = run_director_with_occupied_passage(
                    ["director"],
                    cwd=root,
                    env={},
                    log_path=root / "director.log",
                    event_log=root / "events.jsonl",
                    trigger_marker="toilet_agent_01 started using urinal_2",
                    release_marker="toilet_agent_01 leaving urinal_2",
                    blocking_pose=(2.05, 0.18, 0.03, 0.0),
                    away_pose=(4.0, -3.0, 0.03, 0.0),
                    timeout_sec=10.0,
                )

        self.assertEqual(result, 0)
        self.assertEqual(
            events,
            [
                "robot_occupied_passage_initial_away_pose_requested",
                "robot_occupied_passage_blocking_pose_requested",
                "robot_occupied_passage_release_pose_requested",
            ],
        )

    def test_explicit_zero_pulse_reports_failure_without_raising(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            event_log = root / "events.jsonl"
            with patch(
                "toilet_benchmark.interactive_smoke_runner.subprocess.run",
                side_effect=OSError("publisher unavailable"),
            ):
                accepted = stop_dynamic_crossing(
                    cwd=root,
                    env={},
                    event_log=event_log,
                    topic="/cmd_vel_gamepad_diff",
                )

        self.assertFalse(accepted)

    def test_latest_agent_x_reads_last_diagnostic_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "director.log"
            log.write_text(
                "\n".join(
                    (
                        "HuNav diagnostics: agent=a, pose=(-2.100,-0.9), yaw=0.0",
                        "unrelated",
                        "HuNav diagnostics: agent=a, pose=(-1.175,-0.8), yaw=0.0",
                    )
                ),
                encoding="utf-8",
            )

            self.assertAlmostEqual(_latest_agent_x(log), -1.175)

    def test_latest_agent_x_reads_local_motion_diagnostic_sample(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "director.log"
            log.write_text(
                "Local motion diagnostics: agent=a, phase=WALK_TO_URINAL, "
                "feasible=True, pose=(-1.250,-0.8), robot_fresh=True\n",
                encoding="utf-8",
            )

            self.assertAlmostEqual(_latest_agent_x(log), -1.250)

    def test_pose_arg_requires_four_finite_values(self):
        self.assertEqual(_pose_arg("0,-2,0.03,1.57"), (0.0, -2.0, 0.03, 1.57))

        with self.assertRaises(Exception):
            _pose_arg("0,-2,0.03")

    def test_parser_can_reuse_existing_spawned_bridge(self):
        parsed = _build_parser().parse_args(
            ["--reuse-existing-bridge", "--skip-spawn"]
        )

        self.assertTrue(parsed.reuse_existing_bridge)
        self.assertTrue(parsed.skip_spawn)

    def test_parser_preserves_hunav_default_and_accepts_local_motion(self):
        self.assertEqual(_build_parser().parse_args([]).motion_backend, "hunav")
        parsed = _build_parser().parse_args(
            ["--motion-backend", "local_motion"]
        )

        self.assertEqual(parsed.motion_backend, "local_motion")

        with self.assertRaises(SystemExit):
            _build_parser().parse_args(["--motion-backend", "unsupported"])

    def test_summary_reports_completion_and_interventions(self):
        text = "\n".join(
            (
                (
                    "native_behavior=surprised, native_behavior_state=active_1, "
                    "reaction=None, robot_contacts=1"
                ),
                "HuNav hard safety: static_clip=True",
                "toilet_agent_01 reached urinal_1",
                "toilet_agent_01 aligned at EXITING",
                "All toilet benchmark agents completed",
            )
        )

        summary = summarize_director_log(text, exit_code=0)

        self.assertTrue(summary["completed"])
        self.assertEqual(summary["native_surprised_samples"], 1)
        self.assertEqual(summary["native_behavior_active_samples"], 1)
        self.assertEqual(summary["robot_contact_events"], 1)
        self.assertEqual(summary["static_clip_events"], 1)
        self.assertEqual(summary["urinal_arrivals"], 1)
        self.assertEqual(summary["exit_arrivals"], 1)

    def test_local_motion_summary_reports_feasibility_and_tracking(self):
        text = "\n".join(
            (
                (
                    "[INFO] [10.000] [toilet_director_node]: Local motion "
                    "diagnostics: agent=toilet_agent_01, phase=WALK_TO_URINAL, "
                    "feasible=True, command_speed=0.500, actual_speed=0.300, "
                    "remaining_m=2.000, static_clearance_m=inf, "
                    "service_inflight=True, static_rejected=2, dynamic_rejected=3, "
                    "dynamic_clearance_m=0.080, peer_clearance_m=0.120, "
                    "robot_clearance_m=-0.040, soft_clearance_target_m=0.150, "
                    "overlap_recovery_neighbors=1"
                ),
                (
                    "[INFO] [11.000] [toilet_director_node]: Local motion "
                    "diagnostics: agent=toilet_agent_01, phase=WALK_TO_URINAL, "
                    "feasible=False, command_speed=0.000, actual_speed=0.100, "
                    "remaining_m=2.100, static_clearance_m=-0.020, "
                    "service_inflight=False, static_rejected=5, dynamic_rejected=7"
                ),
            )
        )

        summary = summarize_local_motion_diagnostics(text)
        agent = summary["local_motion_diagnostic_samples_by_agent"][
            "toilet_agent_01"
        ]

        self.assertEqual(summary["local_motion_sample_count"], 2)
        self.assertEqual(summary["local_motion_infeasible_sample_count"], 1)
        self.assertEqual(summary["local_motion_feasible_ratio"], 0.5)
        self.assertAlmostEqual(
            summary["local_motion_speed_tracking_error_mean_mps"], 0.15
        )
        self.assertEqual(summary["local_motion_min_static_clearance_m"], -0.02)
        self.assertEqual(summary["local_motion_min_peer_clearance_m"], 0.12)
        self.assertEqual(summary["local_motion_min_robot_clearance_m"], -0.04)
        self.assertEqual(summary["local_motion_overlap_recovery_sample_count"], 1)
        self.assertEqual(summary["local_motion_service_inflight_sample_count"], 1)
        self.assertEqual(agent["remaining_distance_regression_count"], 1)
        self.assertEqual(agent["max_static_rejected_candidates"], 5)
        self.assertEqual(agent["max_dynamic_rejected_candidates"], 7)

    def test_summary_counts_active_state_markers_from_multiple_samples(self):
        text = "\n".join(
            (
                "native_behavior=regular, native_behavior_state=active_1",
                "native_behavior=scared, native_behavior_state=active_2",
                "native_behavior=surprised, native_behavior_state=active_2",
            )
        )

        summary = summarize_director_log(text, exit_code=0)

        self.assertEqual(summary["native_behavior_active_samples"], 3)
        self.assertEqual(summary["native_surprised_samples"], 1)
        self.assertEqual(
            summary["native_behavior_samples"],
            {"regular": 1, "scared": 1, "surprised": 1},
        )
        self.assertEqual(
            summary["native_behavior_active_samples_by_profile"],
            {"regular": 1, "scared": 1, "surprised": 1},
        )

    def test_summary_characterizes_two_agent_spacing_and_progress(self):
        text = "\n".join(
            (
                (
                    "[INFO] [10.000] [toilet_director_node]: Activating "
                    "toilet_agent_01 at shared entrance pose [-3.8, -0.91, 0.0];"
                ),
                (
                    "[INFO] [11.900] [toilet_director_node]: Activating "
                    "toilet_agent_02 at shared entrance pose [-3.8, -0.91, 0.0];"
                ),
                (
                    "[INFO] [11.400] [toilet_director_node]: HuNav diagnostics: "
                    "agent=toilet_agent_01, generation=1, phase=WALK_TO_URINAL, "
                    "pose=(-3.70,-0.90), yaw=0.000, speed=0.000, "
                    "route_progress_m=0.20, route_remaining_m=1.00, "
                    "behavior_profile=regular, native_behavior=regular, "
                    "behavior_configuration=custom, native_behavior_state=inactive, "
                    "pedestrian_pair_contacts=0, static_clip=False"
                ),
                (
                    "[INFO] [11.450] [toilet_director_node]: HuNav diagnostics: "
                    "agent=toilet_agent_02, generation=1, phase=WALK_TO_URINAL, "
                    "pose=(-3.20,-0.90), yaw=0.000, speed=0.000, "
                    "route_progress_m=0.30, route_remaining_m=1.00, "
                    "behavior_profile=regular, native_behavior=regular, "
                    "behavior_configuration=custom, native_behavior_state=inactive, "
                    "pedestrian_pair_contacts=1, static_clip=False"
                ),
                (
                    "[INFO] [12.400] [toilet_director_node]: HuNav diagnostics: "
                    "agent=toilet_agent_01, generation=1, phase=WALK_TO_URINAL, "
                    "pose=(-3.60,-0.90), yaw=0.000, speed=0.000, "
                    "route_progress_m=0.10, route_remaining_m=1.00, "
                    "behavior_profile=regular, native_behavior=regular, "
                    "behavior_configuration=custom, native_behavior_state=inactive, "
                    "pedestrian_pair_contacts=1, static_clip=False"
                ),
                (
                    "[INFO] [12.450] [toilet_director_node]: HuNav diagnostics: "
                    "agent=toilet_agent_02, generation=1, phase=WALK_TO_URINAL, "
                    "pose=(-3.00,-0.90), yaw=0.000, speed=0.000, "
                    "route_progress_m=0.40, route_remaining_m=1.00, "
                    "behavior_profile=regular, native_behavior=regular, "
                    "behavior_configuration=custom, native_behavior_state=inactive, "
                    "pedestrian_pair_contacts=0, static_clip=False"
                ),
            )
        )

        summary = summarize_director_log(text, exit_code=0)

        self.assertEqual(summary["diagnostic_agent_count"], 2)
        self.assertAlmostEqual(summary["pedestrian_pair_min_distance_m"], 0.5)
        self.assertEqual(summary["pedestrian_pair_overlap_samples"], 1)
        self.assertAlmostEqual(summary["simultaneous_stop_duration_sec"], 1.0)
        self.assertEqual(
            summary["diagnostic_samples_by_agent"]["toilet_agent_01"][
                "route_progress_regression_count"
            ],
            1,
        )
        self.assertAlmostEqual(
            summary["diagnostic_samples_by_agent"]["toilet_agent_01"][
                "peer_activation_bracketing_displacement_m"
            ],
            0.1,
        )
        self.assertAlmostEqual(
            summary["diagnostic_samples_by_agent"]["toilet_agent_01"][
                "peer_activation_jump_excess_m"
            ],
            0.0,
        )

    def test_visual_envelope_summary_tracks_sustained_static_overlap(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "visual_envelope.jsonl"
            records = (
                {
                    "agents": [
                        {
                            "agent_id": "toilet_agent_01",
                            "available": True,
                            "overlap_sample_count": 2,
                            "overlap_ratio": 0.25,
                            "overlap_paths": ["/World/partition"],
                        }
                    ]
                },
                {
                    "agents": [
                        {
                            "agent_id": "toilet_agent_01",
                            "available": True,
                            "overlap_sample_count": 1,
                            "overlap_ratio": 0.125,
                            "overlap_paths": ["/World/partition"],
                        }
                    ]
                },
                {
                    "agents": [
                        {
                            "agent_id": "toilet_agent_01",
                            "available": True,
                            "overlap_sample_count": 0,
                            "overlap_ratio": 0.0,
                            "overlap_paths": [],
                        }
                    ]
                },
            )
            path.write_text(
                "\n".join(json.dumps(record) for record in records) + "\n",
                encoding="utf-8",
            )

            summary = summarize_visual_envelope_log(path)

        self.assertEqual(summary["visual_envelope_frame_samples"], 3)
        self.assertEqual(summary["visual_envelope_agent_samples"], 3)
        self.assertEqual(summary["visual_envelope_available_samples"], 3)
        self.assertEqual(summary["visual_envelope_overlap_samples"], 2)
        self.assertEqual(summary["visual_envelope_max_overlap_ratio"], 0.25)
        self.assertEqual(
            summary["visual_envelope_max_consecutive_overlap_samples"],
            2,
        )
        self.assertEqual(
            summary["visual_envelope_overlap_paths"],
            {"/World/partition": 2},
        )
        self.assertFalse(apply_visual_envelope_gate(summary))
        self.assertFalse(summary["visual_envelope_collision_free"])
        self.assertEqual(
            summary["visual_envelope_failure_reasons"],
            ["visual_envelope_overlap"],
        )

    def test_visual_envelope_gate_requires_observed_skeletons(self):
        summary = summarize_visual_envelope_log(
            Path("/tmp/does-not-exist.jsonl")
        )

        self.assertFalse(apply_visual_envelope_gate(summary))
        self.assertFalse(summary["visual_envelope_validated"])
        self.assertEqual(
            summary["visual_envelope_failure_reasons"],
            ["visual_envelope_unavailable"],
        )

    def test_visual_envelope_summary_ignores_unactivated_parked_agents(self):
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "visual-envelope.jsonl"
            path.write_text(
                json.dumps(
                    {
                        "agents": [
                            {
                                "agent_id": "toilet_agent_01",
                                "available": True,
                                "overlap_sample_count": 0,
                                "overlap_ratio": 0.0,
                                "overlap_paths": [],
                            },
                            {
                                "agent_id": "toilet_agent_02",
                                "available": True,
                                "overlap_sample_count": 1,
                                "overlap_ratio": 0.5,
                                "overlap_paths": ["/World/parking_obstacle"],
                            },
                        ]
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            summary = summarize_visual_envelope_log(
                path,
                expected_agent_ids=("toilet_agent_01",),
            )

        self.assertEqual(summary["visual_envelope_agent_samples"], 1)
        self.assertEqual(
            summary["visual_envelope_available_samples_by_agent"],
            {"toilet_agent_01": 1},
        )
        self.assertEqual(summary["visual_envelope_overlap_samples"], 0)
        self.assertEqual(summary["visual_envelope_overlap_paths"], {})
        self.assertTrue(
            apply_visual_envelope_gate(
                summary,
                expected_agent_ids=("toilet_agent_01",),
            )
        )

    def test_visual_envelope_gate_requires_every_activated_agent(self):
        summary = {
            "visual_envelope_frame_samples": 4,
            "visual_envelope_available_samples": 4,
            "visual_envelope_available_samples_by_agent": {
                "toilet_agent_01": 4,
            },
            "visual_envelope_overlap_samples": 0,
        }

        self.assertFalse(
            apply_visual_envelope_gate(
                summary,
                expected_agent_ids=("toilet_agent_01", "toilet_agent_02"),
            )
        )
        self.assertEqual(
            summary["visual_envelope_missing_agents"],
            ["toilet_agent_02"],
        )
        self.assertEqual(
            summary["visual_envelope_failure_reasons"],
            ["visual_envelope_agent_missing"],
        )

    def test_raw_pose_summary_filters_parking_and_tracks_active_pairs(self):
        director_text = "\n".join(
            (
                (
                    "[INFO] [10.000] [toilet_director_node]: Activating "
                    "toilet_agent_01 at shared entrance pose [-3.8, -0.91, 0.0];"
                ),
                (
                    "[INFO] [11.000] [toilet_director_node]: Activating "
                    "toilet_agent_02 at shared entrance pose [-3.8, -0.91, 0.0];"
                ),
                (
                    "[INFO] [13.000] [toilet_director_node]: "
                    "toilet_agent_01 aligned at EXITING: pose=[]"
                ),
                (
                    "[INFO] [13.000] [toilet_director_node]: "
                    "toilet_agent_02 aligned at EXITING: pose=[]"
                ),
            )
        )
        frames = (
            {
                "source_stamp_sec": 9.0,
                "agents": [
                    {"name": "toilet_agent_01", "x": 1000.0, "y": 1000.0},
                    {"name": "toilet_agent_02", "x": 1000.0, "y": 1000.0},
                ],
            },
            {
                "source_stamp_sec": 10.9,
                "agents": [
                    {
                        "name": "toilet_agent_01",
                        "x": 0.0,
                        "y": 0.0,
                        "vx": 0.5,
                        "vy": 0.0,
                    }
                ],
            },
            {
                "source_stamp_sec": 11.1,
                "agents": [
                    {
                        "name": "toilet_agent_01",
                        "x": 0.1,
                        "y": 0.0,
                        "vx": 0.5,
                        "vy": 0.0,
                    },
                    {
                        "name": "toilet_agent_02",
                        "x": 0.55,
                        "y": 0.0,
                        "vx": 0.0,
                        "vy": 0.0,
                    },
                ],
            },
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.jsonl"
            path.write_text(
                "\n".join(json.dumps(frame) for frame in frames) + "\n",
                encoding="utf-8",
            )

            summary = summarize_raw_pose_log(
                path,
                director_text=director_text,
            )

        self.assertEqual(summary["raw_pose_frame_count"], 2)
        self.assertEqual(summary["raw_pose_agent_sample_count"], 3)
        self.assertAlmostEqual(summary["raw_pose_pair_min_distance_m"], 0.45)
        self.assertEqual(summary["raw_pose_pair_overlap_frames"], 1)
        pair = summary["raw_pose_pair_metrics"][
            "toilet_agent_01|toilet_agent_02"
        ]
        self.assertAlmostEqual(pair["min_distance_m"], 0.45)
        self.assertEqual(pair["overlap_frame_count"], 1)
        self.assertAlmostEqual(
            summary["raw_pose_peer_activation_max_jump_excess_m"],
            0.0,
        )

    def test_raw_pose_summary_counts_repeated_turn_direction_changes(self):
        director_text = "\n".join(
            (
                (
                    "[INFO] [10.000] [toilet_director_node]: Activating "
                    "toilet_agent_01 at shared entrance pose [0.0, 0.0, 0.0];"
                ),
                (
                    "[INFO] [12.000] [toilet_director_node]: "
                    "toilet_agent_01 aligned at WALK_TO_URINAL: pose=[]"
                ),
                (
                    "[INFO] [13.000] [toilet_director_node]: "
                    "toilet_agent_01 aligned at EXITING: pose=[]"
                ),
            )
        )
        points = (
            (10.1, 0.0, 0.0),
            (10.2, 0.1, 0.1),
            (10.3, 0.2, 0.0),
            (10.4, 0.3, 0.1),
            (10.5, 0.4, 0.0),
        )
        frames = [
            {
                "source_stamp_sec": stamp,
                "agents": [
                    {
                        "name": "toilet_agent_01",
                        "x": x,
                        "y": y,
                        "vx": 1.0,
                        "vy": 0.0,
                    }
                ],
            }
            for stamp, x, y in points
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.jsonl"
            path.write_text(
                "\n".join(json.dumps(frame) for frame in frames) + "\n",
                encoding="utf-8",
            )
            summary = summarize_raw_pose_log(
                path,
                director_text=director_text,
            )

        locomotion = summary["raw_pose_locomotion_by_agent"]["toilet_agent_01"]
        self.assertEqual(locomotion["turn_direction_reversal_count"], 2)
        self.assertEqual(summary["raw_pose_max_turn_direction_reversals"], 2)

    def test_raw_pose_summary_isolates_pedestrian_yield_stability(self):
        director_text = "\n".join(
            (
                (
                    "[INFO] [10.000] [toilet_director_node]: Activating "
                    "toilet_agent_02 at shared entrance pose [0.0, 0.0, 0.0];"
                ),
                (
                    "[INFO] [11.000] [toilet_director_node]: HuNav pedestrian "
                    "corridor blocked: agent=toilet_agent_02, "
                    "blocker=toilet_agent_01"
                ),
                (
                    "[INFO] [12.000] [toilet_director_node]: HuNav pedestrian "
                    "corridor released: agent=toilet_agent_02, "
                    "blocker=toilet_agent_01"
                ),
                (
                    "[INFO] [13.000] [toilet_director_node]: "
                    "toilet_agent_02 aligned at WALK_TO_URINAL: pose=[]"
                ),
            )
        )
        frames = [
            {
                "source_stamp_sec": stamp,
                "agents": [
                    {
                        "name": "toilet_agent_02",
                        "x": x,
                        "y": 0.0,
                        "vx": velocity,
                        "vy": 0.0,
                        "yaw": yaw,
                    }
                ],
            }
            for stamp, x, velocity, yaw in (
                (10.5, 0.0, 0.4, 0.0),
                (11.0, 0.2, 0.0, 0.1),
                (11.5, 0.21, 0.01, 0.11),
                (12.0, 0.22, 0.0, 0.09),
                (12.5, 0.4, 0.4, 0.0),
            )
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.jsonl"
            path.write_text(
                "\n".join(json.dumps(frame) for frame in frames) + "\n",
                encoding="utf-8",
            )
            summary = summarize_raw_pose_log(
                path,
                director_text=director_text,
            )

        metrics = summary["raw_pose_pedestrian_yield_by_agent"][
            "toilet_agent_02"
        ]
        self.assertEqual(metrics["window_count"], 1)
        self.assertEqual(metrics["sample_count"], 3)
        self.assertAlmostEqual(metrics["max_xy_span_m"], 0.02)
        self.assertAlmostEqual(metrics["max_yaw_span_rad"], 0.01)
        self.assertAlmostEqual(metrics["max_speed_mps"], 0.01)

    def test_visual_envelope_summary_is_empty_when_log_is_missing(self):
        summary = summarize_visual_envelope_log(Path("/tmp/does-not-exist.jsonl"))

        self.assertEqual(summary["visual_envelope_frame_samples"], 0)
        self.assertEqual(summary["visual_envelope_overlap_paths"], {})
        self.assertEqual(summary["visual_envelope_unavailable_reasons"], {})


if __name__ == "__main__":
    unittest.main()
