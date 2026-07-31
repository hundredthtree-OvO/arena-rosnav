import json
from dataclasses import replace
from pathlib import Path
import tempfile
import time
import unittest

from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent

from toilet_benchmark.hunav_motion_backend import HuNavMotionBackend
from toilet_benchmark.motion_backend import MotionCommand
from toilet_benchmark.polyline_lookahead import PolylineLookaheadTracker
from toilet_benchmark.robot_reaction import (
    ReactionDecision,
    RobotProximityReactionController,
)


class _IsaacBackend:
    def __init__(self):
        self.commands = []
        self.callbacks = []

    def send(self, command, done_callback=None):
        self.commands.append(command)
        self.callbacks.append(done_callback)
        return object()


class _Logger:
    def info(self, message):
        pass

    def error(self, message):
        pass

    def warning(self, message):
        pass


class _WalkablePlanner:
    def __init__(self, *, segment_free=False, clearance=None, route_filter=None):
        self.segment_free = segment_free
        self.clearance = clearance
        self.route_filter = route_filter
        self.plan_calls = []

    def polyline_is_free(self, points):
        if self.route_filter is not None:
            return bool(self.route_filter(points))
        return self.segment_free

    def plan(self, start, goal, *, z=0.0, dynamic_obstacles=None):
        self.plan_calls.append((start, goal, z, dynamic_obstacles))
        return [[0.25, 0.35, z], [goal[0], goal[1], z]]

    def clearance_at(self, point):
        if self.clearance is not None:
            return float(self.clearance(point))
        return 1.0

    def clip_step(self, start, goal):
        if self.segment_free:
            return goal, 1.0, False
        return start, 0.0, True

    resolution = 0.1


def _walking_command(agent_id="toilet_agent_01", *, orientation=0.0):
    return MotionCommand(
        agent_id=agent_id,
        goal_pose=[1.0, 0.0, 0.0],
        path_points=[[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]],
        velocity=0.6,
        orientation=orientation,
    )


class TestHuNavMotionBackend(unittest.TestCase):
    def setUp(self):
        self.backend = object.__new__(HuNavMotionBackend)
        self.backend._isaac = _IsaacBackend()
        self.backend._logger = _Logger()
        self.backend._command = None
        self.backend._shadow = None
        self.backend._generation = 0
        self.backend._active_generation = 0
        self.backend._hunav_active = False
        self.backend._settled_generation = 0
        self.backend._settled_agent_id = ""
        self.backend._terminal_align_active = False
        self.backend._terminal_align_stable_since = 0.0
        self.backend._frozen_present = False
        self.backend._terminal_align_yaw_tolerance_rad = 0.15
        self.backend._terminal_align_max_speed_mps = 0.05
        self.backend._terminal_align_stable_sec = 0.4
        self.backend._settled_arrival_tolerance_m = 0.35
        self.backend._lookahead_distance_m = 0.80
        self.backend._lookahead_projection_window_m = 1.50
        self.backend._lookahead_progress_slack_m = 0.15
        self.backend._lookahead_max_cross_track_m = 0.80
        self.backend._lookahead_tracker = None
        self.backend._lookahead_target = None
        self.backend._last_visible_route_target = None
        self.backend._route_visibility_lost_since = 0.0
        self.backend._route_visibility_grace_sec = 0.5
        self.backend._route_splice_max_cross_track_m = 0.35
        self.backend._walkable_planner = None
        self.backend._constrained_yaw_speed_threshold_mps = 0.12
        self.backend._static_projection_enabled = True
        self.backend._static_projection_max_deflection_deg = 80.0
        self.backend._static_projection_angle_step_deg = 20.0
        self.backend._regular_avoidance_enabled = True
        self.backend._avoidance_trigger_distance_m = 1.40
        self.backend._avoidance_side_clearance_m = 0.18
        self.backend._avoidance_clearance_samples_m = (0.18, 0.06, 0.02)
        self.backend._avoidance_forward_offset_m = 0.30
        self.backend._avoidance_forward_offset_samples_m = (0.30, 0.50)
        self.backend._avoidance_release_distance_m = 1.60
        self.backend._avoidance_prediction_horizon_sec = 1.50
        self.backend._avoidance_sample_spacing_m = 0.10
        self.backend._avoidance_min_dynamic_clearance_m = 0.01
        self.backend._avoidance_suppress_backward_motion = True
        self.backend._avoidance_geometry_override_enabled = True
        self.backend._avoidance_narrow_space_fallback = "yielding"
        self.backend._pedestrian_yield_enabled = True
        self.backend._pedestrian_yield_trigger_distance_m = 0.95
        self.backend._pedestrian_yield_release_distance_m = 1.10
        self.backend._pedestrian_yield_side_clearance_m = 0.08
        self.backend._pedestrian_yield_release_ticks = 3
        self.backend._pedestrian_yield_forward_offset_m = 0.45
        self.backend._avoidance_active_route_infeasible_confirm_ticks = 4
        self.backend._avoidance_active_route_infeasible_ticks = 0
        self.backend._avoidance_stall_timeout_sec = 2.0
        self.backend._avoidance_stall_progress_epsilon_m = 0.03
        self.backend._avoidance_max_recovery_attempts = 1
        self.backend._avoidance_recovery_attempts = 0
        self.backend._avoidance_side = 0
        self.backend._avoidance_target = None
        self.backend._avoidance_encounter_active = False
        self.backend._last_avoidance_candidates = ()
        self.backend._local_route_mode = None
        self.backend._local_route_points = []
        self.backend._local_route_index = 0
        self.backend._local_route_best_distance = None
        self.backend._local_route_last_progress_at = 0.0
        self.backend._avoidance_failed_side = 0
        self.backend._local_waypoint_tolerance_m = 0.22
        self.backend._static_hold_yaw = None
        self.backend._static_clear_ticks = 0
        self.backend._last_global_replan_at = 0.0
        self.backend._global_replan_min_interval_sec = 1.0
        self.backend._pending_rebase_replan = False
        self.backend._yield_hold_yaw = None
        self.backend._pedestrian_yield_blocker = None
        self.backend._pedestrian_yield_clear_ticks = 0
        self.backend._last_external_motion_mode = 0
        self.backend._agent_radius = 0.30
        self.backend._social_radius = 0.30
        self.backend._planning_radius = 0.30
        self.backend._hard_radius = 0.30
        self.backend._max_compute_dt_sec = 0.20
        self.backend._robot_radius = 0.45
        self.backend._stationary_robot_radius = 0.36
        self.backend._stationary_robot_linear_speed_threshold_mps = 0.05
        self.backend._stationary_robot_angular_speed_threshold_rps = 0.10
        self.backend._robot_footprint_half_length_m = 0.36
        self.backend._robot_footprint_half_width_m = 0.27
        self.backend._goal_radius = 0.12
        self.backend._behavior = {}
        self.backend._external_timeout_sec = 0.35
        self.backend._max_hunav_step_speed_factor = 2.0
        self.backend._max_hunav_step_min_m = 0.03
        self.backend._state_timeout_sec = 0.5
        self.backend._external_future = None
        self.backend._actual = {}
        self.backend._reaction_controller = RobotProximityReactionController(
            {"enabled": False},
            seed=42,
        )
        self.backend._reaction_decision = ReactionDecision(
            state="WALKING",
            reaction=None,
            speed_scale=1.0,
            distance_m=None,
            ttc_sec=None,
        )
        self.backend._native_behavior_profile_by_agent = {}
        self.backend._native_behavior_response_state_by_agent = {}
        self.backend._route_publisher = None
        self.backend._robot = None
        self.backend._route_hold_active = False
        self.backend._route_hold_yaw = None

    def test_fixed_pedestrian_in_closed_corridor_triggers_stable_yield(self):
        now = time.monotonic()
        self.backend.send(_walking_command("toilet_agent_01"))
        self.backend.send(
            MotionCommand(
                agent_id="toilet_agent_01",
                goal_pose=[0.8, 0.0, 0.0],
                path_points=[],
                velocity=0.0,
                orientation=0.0,
                stop=True,
            )
        )
        self.backend.send(_walking_command("toilet_agent_02"))
        self.backend._select_runtime("toilet_agent_02")
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 1.0, "y": 0.0},
        )()
        self.backend._actual = {
            "toilet_agent_01": {
                "x": 0.75,
                "y": 0.0,
                "received_at": now,
            },
            "toilet_agent_02": {
                "x": 0.0,
                "y": 0.0,
                "received_at": now,
            },
        }
        self.backend._pedestrian_bypass_available = lambda *args: False

        yielding = self.backend._update_pedestrian_corridor_yield(
            {"x": 0.0, "y": 0.0, "yaw": 0.2}
        )

        self.assertTrue(yielding)
        self.assertEqual(
            self.backend._reaction_decision.state,
            "YIELDING_TO_PEDESTRIAN",
        )
        self.assertEqual(
            self.backend._pedestrian_yield_blocker,
            "toilet_agent_01",
        )
        self.assertAlmostEqual(self.backend._yield_hold_yaw, 0.2)

    def test_pedestrian_corridor_yield_releases_after_blocker_moves(self):
        self.test_fixed_pedestrian_in_closed_corridor_triggers_stable_yield()
        self.backend._runtime_states["toilet_agent_01"][
            "_frozen_present"
        ] = False
        command = self.backend._runtime_states["toilet_agent_01"]["_command"]
        self.backend._runtime_states["toilet_agent_01"]["_command"] = replace(
            command,
            stop=False,
        )

        results = [
            self.backend._update_pedestrian_corridor_yield(
                {"x": 0.0, "y": 0.0, "yaw": 0.2}
            )
            for _ in range(self.backend._pedestrian_yield_release_ticks)
        ]

        self.assertEqual(results, [True, True, False])
        self.assertIsNone(self.backend._pedestrian_yield_blocker)
        self.assertTrue(self.backend._pending_rebase_replan)
        self.assertEqual(self.backend._reaction_decision.state, "WALKING")

    def test_walking_command_is_accepted_without_forwarding_isaac_path(self):
        callback_results = []

        future = self.backend.send(
            _walking_command(),
            done_callback=lambda done: callback_results.append(done.result().ret),
        )

        self.assertTrue(future.result().ret)
        self.assertEqual(callback_results, [True])
        self.assertEqual(self.backend._isaac.commands, [])
        self.assertEqual(self.backend._command.agent_id, "toilet_agent_01")

    def test_hunav_backend_owns_stall_recovery(self):
        self.assertFalse(
            self.backend.allows_director_stall_recovery("toilet_agent_01")
        )

    def test_native_behavior_transition_is_written_only_when_state_changes(self):
        self.backend._command = _walking_command()
        self.backend._generation = 3
        self.backend._last_native_behavior_signature = None
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "behavior.jsonl"
            self.backend._behavior_event_path = path
            active = {
                "profile": "surprised",
                "native_type": "surprised",
                "state": "active_1",
            }
            inactive = {**active, "state": "inactive"}

            self.backend._record_native_behavior_transition(active)
            self.backend._record_native_behavior_transition(active)
            self.backend._record_native_behavior_transition(inactive)

            events = [
                json.loads(line)
                for line in path.read_text(encoding="utf-8").splitlines()
            ]
        self.assertEqual(len(events), 2)
        self.assertEqual(events[0]["native_behavior_state"], "active_1")
        self.assertEqual(events[1]["previous_native_behavior_state"], "active_1")
        self.assertEqual(events[1]["native_behavior_state"], "inactive")

    def test_second_agent_joins_shared_hunav_world(self):
        self.backend.send(_walking_command("toilet_agent_01"))

        future = self.backend.send(_walking_command("toilet_agent_02"))

        self.assertTrue(future.result().ret)
        self.assertEqual(
            set(self.backend._commands),
            {"toilet_agent_01", "toilet_agent_02"},
        )

    def test_direct_pose_preserves_hunav_session_and_uses_compatibility_adapter(self):
        self.backend.send(_walking_command())
        previous_command = self.backend._command
        direct = MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[-3.8, -0.9, 0.0],
            path_points=[],
            velocity=0.0,
            stop=True,
            use_direct_pose=True,
            direct_pose=[-3.8, -0.9, 0.0],
        )
        callback = object()

        returned = self.backend.send(direct, done_callback=callback)

        self.assertIs(self.backend._command, previous_command)
        self.assertIn("toilet_agent_01", self.backend._runtime_states)
        self.assertEqual(self.backend._isaac.commands, [direct])
        self.assertEqual(self.backend._isaac.callbacks, [callback])
        self.assertIsNotNone(returned)

    def test_goal_settlement_requires_current_generation_and_bounded_pose_error(self):
        self.backend.send(_walking_command())
        self.backend._settled_generation = self.backend._generation
        self.backend._settled_agent_id = "toilet_agent_01"

        self.assertTrue(
            self.backend.is_goal_settled(
                "toilet_agent_01", [1.25, 0.0, 0.0], [1.0, 0.0, 0.0]
            )
        )
        self.assertFalse(
            self.backend.is_goal_settled(
                "toilet_agent_01", [1.5, 0.0, 0.0], [1.0, 0.0, 0.0]
            )
        )

    def test_hunav_agent_receives_one_rolling_lookahead_goal(self):
        self.backend.send(
            MotionCommand(
                agent_id="toilet_agent_01",
                goal_pose=[2.0, 2.0, 0.0],
                path_points=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                velocity=0.6,
            )
        )
        actual = {
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.backend._initialize_lookahead(actual)

        agent = self.backend._build_agent(actual)

        self.assertEqual(len(agent.goals), 1)
        self.assertAlmostEqual(agent.goals[0].position.x, 0.8)
        self.assertAlmostEqual(agent.goals[0].position.y, 0.0)

    def test_yield_hold_preserves_pose_and_sends_zero_velocity(self):
        self.backend.send(_walking_command())

        self.backend._send_yield_hold(
            {"x": 0.4, "y": -0.2, "z": 0.0, "yaw": 1.2}
        )

        hold = self.backend._isaac.commands[-1]
        self.assertTrue(hold.use_external_motion)
        self.assertTrue(hold.external_freeze_pose)
        self.assertEqual(hold.external_motion_mode, 1)
        self.assertEqual(hold.external_velocity, (0.0, 0.0, 0.0))
        self.assertEqual(hold.direct_pose, (0.4, -0.2, 0.0))
        self.assertAlmostEqual(hold.orientation, 1.2)

    def test_yield_hold_latches_heading_across_pose_updates(self):
        self.backend.send(_walking_command())
        self.backend._send_yield_hold(
            {"x": 0.4, "y": -0.2, "z": 0.0, "yaw": 1.2}
        )
        self.backend._external_future = None

        self.backend._send_yield_hold(
            {"x": 0.4, "y": -0.2, "z": 0.0, "yaw": 0.7}
        )

        first_hold, second_hold = self.backend._isaac.commands[-2:]
        self.assertAlmostEqual(first_hold.orientation, 1.2)
        self.assertAlmostEqual(second_hold.orientation, 1.2)

    def test_robot_reaction_reset_clears_latched_heading(self):
        self.backend._yield_hold_yaw = 1.2

        self.backend._reset_robot_reaction()

        self.assertIsNone(self.backend._yield_hold_yaw)

    def test_motion_mode_transition_is_recorded_once(self):
        self.backend.send(_walking_command())
        actual = {"x": 0.4, "y": -0.2, "z": 0.0, "yaw": 1.2}

        self.backend._record_motion_mode(1, actual, target_yaw=1.2)
        self.backend._record_motion_mode(1, actual, target_yaw=1.2)

        self.assertEqual(self.backend._last_external_motion_mode, 1)

    def test_terminal_alignment_does_not_settle_before_yaw_converges(self):
        self.backend.send(_walking_command(orientation=3.14159))
        self.backend._begin_terminal_alignment()
        actual = {
            "x": 1.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.backend._actual["toilet_agent_01"] = actual

        self.backend._tick_terminal_alignment(actual)

        self.assertEqual(self.backend._settled_generation, 0)
        self.assertTrue(self.backend._terminal_align_active)
        self.assertEqual(
            self.backend._isaac.commands[-1].external_motion_mode,
            2,
        )

    def test_final_lookahead_captures_step_before_hunav_consumes_goal(self):
        self.backend.send(_walking_command(orientation=1.57))
        self.backend._lookahead_target = type(
            "Lookahead",
            (),
            {"is_final": True},
        )()
        self.backend._actual["toilet_agent_01"] = {
            "x": 0.80,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.5,
            "vy": 0.0,
            "received_at": time.monotonic(),
        }
        previous = Agent()
        previous.position.position.x = 0.72
        proposed = Agent()
        proposed.position.position.x = 1.08
        proposed.velocity.linear.x = 0.5
        proposed.linear_vel = 0.5
        proposed.goals = [Pose()]
        self.backend._publish_live_diagnostics = lambda *args: None
        self.backend._set_runtime_shadow = lambda agent: None

        self.backend._finish_computed_agent(
            previous,
            proposed,
            {"enabled": True},
        )

        self.assertTrue(self.backend._terminal_align_active)
        self.assertEqual(proposed.goals, [])
        self.assertEqual(proposed.linear_vel, 0.0)
        self.assertEqual(
            self.backend._isaac.commands[-1].external_motion_mode,
            2,
        )
        self.assertEqual(
            self.backend._isaac.commands[-1].direct_pose[:2],
            (0.8, 0.0),
        )

    def test_terminal_alignment_does_not_settle_outside_position_envelope(self):
        self.backend.send(_walking_command())
        self.backend._begin_terminal_alignment()
        self.backend._terminal_align_stable_since = (
            time.monotonic() - self.backend._terminal_align_stable_sec - 0.1
        )
        actual = {
            "x": 0.60,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.backend._actual["toilet_agent_01"] = actual

        self.backend._tick_terminal_alignment(actual)

        self.assertTrue(self.backend._terminal_align_active)
        self.assertEqual(self.backend._settled_generation, 0)

    def test_terminal_alignment_retries_after_busy_external_request(self):
        self.backend.send(_walking_command(orientation=3.14159))
        self.backend._begin_terminal_alignment()
        busy = __import__("concurrent.futures").futures.Future()
        self.backend._external_future = busy
        actual = {
            "x": 1.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.backend._actual["toilet_agent_01"] = actual

        self.backend._tick_terminal_alignment(actual)
        self.assertEqual(self.backend._isaac.commands, [])
        self.assertTrue(self.backend._terminal_align_active)

        busy.set_result(type("Result", (), {"ret": True})())
        self.backend._external_future = None
        self.backend._tick_terminal_alignment(actual)

        self.assertEqual(len(self.backend._isaac.commands), 1)
        self.assertEqual(
            self.backend._isaac.commands[0].external_motion_mode,
            2,
        )

    def test_terminal_alignment_settles_only_after_stable_live_pose(self):
        import time

        self.backend.send(_walking_command())
        self.backend._begin_terminal_alignment()
        self.backend._terminal_align_stable_since = (
            time.monotonic() - self.backend._terminal_align_stable_sec - 0.1
        )
        actual = {
            "x": 1.0,
            "y": 0.0,
            "z": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
        }
        self.backend._actual["toilet_agent_01"] = actual

        self.backend._tick_terminal_alignment(actual)

        self.assertEqual(
            self.backend._settled_generation,
            self.backend._generation,
        )
        self.assertFalse(self.backend._terminal_align_active)

    def test_regular_avoidance_latches_one_side_of_blocking_robot(self):
        self.backend.send(_walking_command())
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "received_at": __import__("time").monotonic(),
        }
        nominal = type(
            "Target",
            (),
            {"x": 1.0, "y": 0.0},
        )()

        first = self.backend._start_regular_avoidance((0.0, 0.0), nominal)
        second = self.backend._active_local_route_goal((0.1, 0.0))

        self.assertIsNotNone(first)
        self.assertEqual(first, second)
        self.assertNotEqual(first[1], 0.0)
        self.assertEqual(self.backend._local_route_mode, "AVOIDANCE")
        self.assertEqual(len(self.backend._local_route_points), 2)

    def test_regular_avoidance_prefers_wider_static_corridor(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(
            segment_free=True,
            clearance=lambda point: 1.0 if point[1] > 0.0 else 0.20,
        )
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "received_at": time.monotonic(),
        }

        self.backend._start_regular_avoidance(
            (0.0, 0.0),
            type("Target", (), {"x": 1.0, "y": 0.0})(),
        )

        self.assertEqual(self.backend._avoidance_side, 1)
        self.assertGreater(self.backend._avoidance_target[1], 0.0)

    def test_regular_avoidance_uses_predicted_robot_motion(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=True)
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.40,
            "received_at": time.monotonic(),
        }

        self.backend._start_regular_avoidance(
            (0.0, 0.0),
            type("Target", (), {"x": 1.0, "y": 0.0})(),
        )

        self.assertEqual(self.backend._avoidance_side, -1)
        self.assertLess(self.backend._avoidance_target[1], 0.0)

    def test_regular_avoidance_rejects_blocked_side_corridor(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(
            segment_free=True,
            route_filter=lambda points: not any(
                float(point[1]) > 0.05 for point in points[1:]
            ),
        )
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "received_at": time.monotonic(),
        }

        self.backend._start_regular_avoidance(
            (0.0, 0.0),
            type("Target", (), {"x": 1.0, "y": 0.0})(),
        )

        self.assertEqual(self.backend._avoidance_side, -1)
        rejected = [
            item
            for item in self.backend._last_avoidance_candidates
            if item.side == 1
        ]
        self.assertEqual(rejected[0].reason, "static corridor blocked")

    def test_regular_avoidance_samples_tighter_safe_corridor(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(
            segment_free=True,
            route_filter=lambda points: all(
                abs(float(point[1])) <= 0.65 for point in points[1:]
            ),
        )
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "yaw": 0.0,
            "received_at": time.monotonic(),
        }

        goal = self.backend._start_regular_avoidance(
            (0.0, 0.0),
            type("Target", (), {"x": 1.0, "y": 0.0})(),
        )

        self.assertIsNotNone(goal)
        self.assertLessEqual(abs(self.backend._avoidance_target[1]), 0.65)

    def test_completed_avoidance_handoff_does_not_force_yielding(self):
        import time

        self.backend.send(_walking_command())
        self.backend._reaction_controller = RobotProximityReactionController(
            {
                "enabled": True,
                "mode": "fixed",
                "reaction": "regular",
                "trigger": {
                    "distance_m": 1.1,
                    "time_to_collision_sec": 1.5,
                    "front_half_angle_deg": 180.0,
                },
                "release": {"distance_m": 1.25, "stable_sec": 0.8},
                "reactions": {"regular": {"speed_scale": 1.0}},
            },
            seed=42,
        )
        self.backend._robot = {
            "x": 0.7,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "received_at": time.monotonic(),
        }
        self.backend._avoidance_encounter_active = True
        self.backend._local_route_mode = None
        self.backend._lookahead_target = type(
            "Target", (), {"x": 1.0, "y": 0.0}
        )()
        self.backend._avoidance_candidates = lambda *args: (True, [], 0.7)

        self.backend._update_robot_reaction(
            {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "vx": 0.2,
                "vy": 0.0,
            }
        )

        self.assertEqual(self.backend._reaction_decision.reaction, "regular")

    def test_regular_avoidance_releases_after_robot_is_passed(self):
        self.backend.send(_walking_command())
        self.backend._reaction_decision = ReactionDecision(
            state="REGULAR_TO_ROBOT",
            reaction="regular",
            speed_scale=1.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "received_at": __import__("time").monotonic(),
        }
        nominal = type("Target", (), {"x": 1.0, "y": 0.0})()
        self.backend._start_regular_avoidance((0.0, 0.0), nominal)
        self.backend._local_route_mode = None
        self.backend._local_route_points = []

        goal = self.backend._start_regular_avoidance(
            (1.0, 0.0),
            type("Target", (), {"x": 2.0, "y": 0.0})(),
        )

        self.assertIsNone(goal)
        self.assertIsNone(self.backend._avoidance_target)
        self.assertFalse(self.backend._avoidance_encounter_active)

    def test_reaction_release_cancels_avoidance_and_rejoins_global_route(self):
        self.backend._walkable_planner = _WalkablePlanner(segment_free=True)
        self.backend.send(
            MotionCommand(
                agent_id="toilet_agent_01",
                goal_pose=[2.0, 0.0, 0.0],
                path_points=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                velocity=0.6,
            )
        )
        actual = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0}
        self.backend._initialize_lookahead(actual)
        self.backend._avoidance_encounter_active = True
        self.backend._avoidance_side = 1
        self.backend._avoidance_target = (0.8, 0.9)
        self.backend._set_local_route("AVOIDANCE", [self.backend._avoidance_target])
        self.backend._reaction_decision = ReactionDecision(
            state="WALKING",
            reaction=None,
            speed_scale=1.0,
            distance_m=1.0,
            ttc_sec=None,
        )
        agent = self.backend._build_agent(actual)

        self.backend._update_lookahead_goal(agent, 0.1, 0.0)

        self.assertIsNone(self.backend._local_route_mode)
        self.assertIsNone(self.backend._avoidance_target)
        self.assertFalse(self.backend._avoidance_encounter_active)
        self.assertEqual(len(self.backend._walkable_planner.plan_calls), 1)
        self.assertNotAlmostEqual(agent.goals[0].position.y, 0.9)

    def test_fixed_regular_does_not_relatch_after_reaction_release(self):
        import time

        self.backend.send(_walking_command())
        self.backend._reaction_controller = RobotProximityReactionController(
            {
                "enabled": True,
                "mode": "fixed",
                "reaction": "regular",
            },
            seed=12345,
        )
        self.backend._reaction_decision = ReactionDecision(
            state="WALKING",
            reaction=None,
            speed_scale=1.0,
            distance_m=0.9,
            ttc_sec=None,
        )
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "received_at": time.monotonic(),
        }
        nominal = type("Target", (), {"x": 1.0, "y": 0.0})()

        goal = self.backend._start_regular_avoidance((0.0, 0.0), nominal)

        self.assertIsNone(goal)
        self.assertIsNone(self.backend._local_route_mode)
        self.assertIsNone(self.backend._avoidance_target)
        self.assertFalse(self.backend._avoidance_encounter_active)

    def test_regular_avoidance_removes_backward_but_keeps_lateral_motion(self):
        self.backend._avoidance_encounter_active = True
        self.backend._avoidance_target = (1.0, 0.0)

        projected = self.backend._suppress_backward_avoidance_step(
            (0.0, 0.0),
            (-0.1, 0.2),
        )

        self.assertAlmostEqual(projected[0], 0.0)
        self.assertAlmostEqual(projected[1], 0.2)

    def test_narrow_space_overrides_regular_reaction_with_yielding(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=False)
        self.backend._reaction_controller = RobotProximityReactionController(
            {
                "enabled": True,
                "mode": "fixed",
                "reaction": "regular",
                "trigger": {"distance_m": 1.0},
            },
            seed=42,
        )
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 2.0, "y": 0.0},
        )()
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "received_at": time.monotonic(),
        }

        self.backend._update_robot_reaction(
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.3, "vy": 0.0}
        )

        self.assertEqual(
            self.backend._reaction_decision.state,
            "YIELDING_TO_ROBOT",
        )
        self.assertEqual(self.backend._reaction_controller.reaction, "yielding")

    def test_active_avoidance_remains_latched_when_prediction_temporarily_fails(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=False)
        self.backend._reaction_controller = RobotProximityReactionController(
            {
                "enabled": True,
                "mode": "fixed",
                "reaction": "regular",
                "trigger": {"distance_m": 1.0},
            },
            seed=42,
        )
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 2.0, "y": 0.0},
        )()
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "received_at": time.monotonic(),
        }
        self.backend._avoidance_encounter_active = True
        self.backend._avoidance_side = 1
        self.backend._avoidance_target = (1.0, 0.8)
        self.backend._set_local_route("AVOIDANCE", [(1.0, 0.8)])
        actual = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.3, "vy": 0.0}

        for expected_ticks in range(1, 6):
            self.backend._update_robot_reaction(actual)
            self.assertEqual(
                self.backend._reaction_decision.state,
                "REGULAR_TO_ROBOT",
            )
            self.assertEqual(
                self.backend._avoidance_active_route_infeasible_ticks,
                expected_ticks,
            )

        self.assertEqual(self.backend._local_route_mode, "AVOIDANCE")

    def test_hunav_step_is_clipped_to_commanded_speed_envelope(self):
        self.backend.send(_walking_command())
        previous = Agent()
        proposed = Agent()
        proposed.position.position.x = 3.0

        clipped = self.backend._clip_hunav_step(previous, proposed, 0.1)

        self.assertTrue(clipped)
        self.assertAlmostEqual(proposed.position.position.x, 0.12)
        self.assertAlmostEqual(proposed.linear_vel, 1.2)

    def test_stalled_avoidance_yields_after_recovery_is_exhausted(self):
        from toilet_benchmark.hunav_motion_backend import AvoidanceCandidate

        candidate = AvoidanceCandidate(
            side=1,
            target=(1.0, 0.5),
            waypoints=((1.0, 0.5),),
            accepted=True,
            reason="accepted",
            static_clearance_m=0.5,
            dynamic_clearance_m=0.2,
            path_length_m=1.2,
            forward_progress_m=1.0,
            turn_angle_rad=0.2,
        )
        self.backend.send(_walking_command())
        self.backend._robot = {"x": 0.8, "y": 0.0}
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 1.0, "y": 0.0},
        )()
        self.backend._avoidance_side = 1
        self.backend._avoidance_encounter_active = True
        self.backend._avoidance_recovery_attempts = 1
        self.backend._avoidance_candidates = (
            lambda *_args: (True, [candidate], 0.8)
        )

        recovered = self.backend._recover_stalled_avoidance((0.0, 0.0))

        self.assertEqual(recovered, (0.0, 0.0))
        self.assertFalse(self.backend._avoidance_encounter_active)
        self.assertEqual(self.backend._reaction_controller.reaction, "yielding")

    def test_yield_release_discards_shadow_and_schedules_hunav_rebase(self):
        class _ReleaseController:
            reaction = None

            def update(self, **_kwargs):
                return ReactionDecision(
                    state="WALKING",
                    reaction=None,
                    speed_scale=1.0,
                    distance_m=1.2,
                    ttc_sec=None,
                    transition="YIELDING_TO_ROBOT->WALKING",
                )

        self.backend.send(_walking_command())
        self.backend._reaction_controller = _ReleaseController()
        self.backend._reaction_decision = ReactionDecision(
            state="YIELDING_TO_ROBOT",
            reaction="yielding",
            speed_scale=0.0,
            distance_m=0.8,
            ttc_sec=1.0,
        )
        self.backend._shadow = object()
        self.backend._active_generation = self.backend._generation
        self.backend._lookahead_tracker = object()
        self.backend._lookahead_target = object()

        self.backend._update_robot_reaction(
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0}
        )

        self.assertIsNone(self.backend._shadow)
        self.assertEqual(self.backend._active_generation, 0)
        self.assertIsNone(self.backend._lookahead_tracker)
        self.assertIsNone(self.backend._lookahead_target)
        self.assertTrue(self.backend._pending_rebase_replan)

    def test_dynamic_replan_uses_current_and_predicted_robot_pose(self):
        import time

        self.backend._avoidance_prediction_horizon_sec = 1.5
        self.backend._robot = {
            "x": 1.0,
            "y": 2.0,
            "yaw": 0.0,
            "vx": 0.4,
            "vy": -0.2,
            "wz": 0.0,
            "received_at": time.monotonic(),
        }

        obstacles = self.backend._dynamic_replan_obstacles()

        self.assertEqual(len(obstacles), 2)
        self.assertAlmostEqual(obstacles[0][0], 1.0)
        self.assertAlmostEqual(obstacles[1][0], 1.6)
        self.assertAlmostEqual(obstacles[1][1], 1.7)

    def test_dynamic_replan_rejects_a_route_that_starts_backward(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner()
        self.backend._walkable_planner.plan = (
            lambda *_args, **_kwargs: [
                [-0.40, 0.0, 0.0],
                [1.0, 0.0, 0.0],
            ]
        )
        self.backend._robot = {
            "x": 0.4,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
            "received_at": time.monotonic(),
        }

        replanned = self.backend._replan_to_final_goal(
            (0.0, 0.0),
            reason="test",
            force=True,
        )

        self.assertFalse(replanned)
        self.assertIsNone(self.backend._lookahead_tracker)

    def test_completed_avoidance_keeps_encounter_latched(self):
        self.backend.send(_walking_command())
        self.backend._avoidance_encounter_active = True
        self.backend._avoidance_side = 1
        self.backend._avoidance_target = (0.5, 0.2)
        self.backend._local_route_mode = "AVOIDANCE"
        self.backend._local_route_points = [(0.0, 0.0)]
        self.backend._local_route_index = 0
        self.backend._replan_to_final_goal = lambda *_args, **_kwargs: True

        goal = self.backend._active_local_route_goal((0.0, 0.0))

        self.assertIsNone(goal)
        self.assertTrue(self.backend._avoidance_encounter_active)
        self.assertEqual(self.backend._avoidance_side, 0)

    def test_successful_global_recovery_keeps_encounter_latched(self):
        self.backend.send(_walking_command())
        self.backend._avoidance_encounter_active = True
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 0.8, "y": 0.0},
        )()
        self.backend._replan_to_final_goal = lambda *_args, **_kwargs: True

        goal = self.backend._replan_or_yield_after_avoidance((0.0, 0.0))

        self.assertEqual(goal, (0.8, 0.0))
        self.assertTrue(self.backend._avoidance_encounter_active)

    def test_candidate_scoring_ignores_millimetric_margin_advantage(self):
        from toilet_benchmark.hunav_motion_backend import AvoidanceCandidate

        short = AvoidanceCandidate(
            side=-1,
            target=(1.0, -0.5),
            waypoints=((1.0, -0.5),),
            accepted=True,
            reason="accepted",
            static_clearance_m=0.40,
            dynamic_clearance_m=0.100,
            path_length_m=1.60,
            forward_progress_m=1.40,
            turn_angle_rad=0.20,
        )
        long = AvoidanceCandidate(
            side=1,
            target=(1.0, 0.5),
            waypoints=((1.0, 0.5),),
            accepted=True,
            reason="accepted",
            static_clearance_m=1.00,
            dynamic_clearance_m=0.103,
            path_length_m=2.40,
            forward_progress_m=1.40,
            turn_angle_rad=0.60,
        )

        selected = min(
            (short, long),
            key=lambda item: self.backend._avoidance_candidate_sort_key(
                item,
                preferred_side=1,
            ),
        )

        self.assertEqual(selected.side, -1)

    def test_stationary_robot_uses_smaller_hard_safety_radius(self):
        stationary = {
            "vx": 0.01,
            "vy": 0.0,
            "wz": 0.01,
        }
        moving = {
            "vx": 0.20,
            "vy": 0.0,
            "wz": 0.0,
        }

        self.assertAlmostEqual(
            self.backend._effective_robot_radius(stationary),
            0.36,
        )
        self.assertAlmostEqual(
            self.backend._effective_robot_radius(moving),
            0.45,
        )

    def test_visible_route_target_keeps_short_required_corner(self):
        self.backend._walkable_planner = _WalkablePlanner(
            route_filter=lambda points: (
                0.09
                <= float(points[-1][0])
                <= 0.14
            ),
        )
        self.backend._lookahead_tracker = PolylineLookaheadTracker(
            [(0.0, 0.0), (0.10, 0.10), (0.50, 0.10)],
            lookahead_m=0.80,
        )
        nominal = self.backend._lookahead_tracker.update(0.0, 0.0)

        visible = self.backend._visible_route_target((0.0, 0.0), nominal)

        self.assertGreater(visible.target_progress_m, 0.09)
        self.assertLess(visible.target_progress_m, 0.15)
        self.assertFalse(self.backend._route_hold_active)

    def test_no_visible_route_target_latches_explicit_yaw_hold(self):
        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=False)
        self.backend._lookahead_tracker = PolylineLookaheadTracker(
            [(0.0, 0.0), (0.5, 0.0)],
            lookahead_m=0.80,
        )
        nominal = self.backend._lookahead_tracker.update(0.0, 0.0)
        self.backend._last_global_replan_at = time.monotonic()
        self.backend._route_visibility_lost_since = time.monotonic() - 1.0

        visible = self.backend._visible_route_target((0.0, 0.0), nominal)

        self.assertEqual((visible.x, visible.y), (0.0, 0.0))
        self.assertTrue(self.backend._route_hold_active)

    def test_transient_visibility_loss_keeps_last_target_without_freezing(self):
        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=False)
        self.backend._lookahead_tracker = PolylineLookaheadTracker(
            [(0.0, 0.0), (1.0, 0.0)],
            lookahead_m=0.80,
        )
        nominal = self.backend._lookahead_tracker.update(0.0, 0.0)
        self.backend._last_visible_route_target = nominal
        self.backend._last_global_replan_at = time.monotonic()

        visible = self.backend._visible_route_target((0.0, 0.0), nominal)

        self.assertEqual((visible.x, visible.y), (nominal.x, nominal.y))
        self.assertFalse(self.backend._route_hold_active)
        self.assertGreater(self.backend._route_visibility_lost_since, 0.0)

    def test_route_rebuild_splices_nearby_previous_lookahead(self):
        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=True)
        self.backend._lookahead_tracker = PolylineLookaheadTracker(
            [(0.0, 0.0), (2.0, 0.0)],
            lookahead_m=0.80,
        )
        previous = self.backend._lookahead_tracker.update(0.0, 0.0)
        self.backend._lookahead_target = previous
        self.backend._last_visible_route_target = previous

        rebuilt = self.backend._reset_lookahead_route(
            (0.2, 0.0),
            [[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            reason="test splice",
        )

        self.assertTrue(rebuilt)
        self.assertEqual(self.backend._lookahead_tracker._points[1], (0.8, 0.0))

    def test_route_splice_discards_new_route_points_behind_splice(self):
        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=True)
        self.backend._lookahead_tracker = PolylineLookaheadTracker(
            [(0.0, 0.0), (2.0, 0.0)],
            lookahead_m=0.80,
        )
        previous = self.backend._lookahead_tracker.update(0.0, 0.0)
        self.backend._last_visible_route_target = previous

        self.backend._reset_lookahead_route(
            (0.2, 0.0),
            [[0.3, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            reason="test splice trim",
        )

        self.assertEqual(
            self.backend._lookahead_tracker._points,
            [(0.2, 0.0), (0.8, 0.0), (1.0, 0.0), (2.0, 0.0)],
        )

    def test_oriented_robot_footprint_reduces_lateral_avoidance_offset(self):
        robot = {
            "x": 0.8,
            "y": 0.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
        }

        along_route = self.backend._robot_support_radius(robot, (1.0, 0.0))
        across_route = self.backend._robot_support_radius(robot, (0.0, 1.0))

        self.assertAlmostEqual(along_route, 0.36)
        self.assertAlmostEqual(across_route, 0.27)
        self.assertLess(
            self.backend._agent_radius
            + across_route
            + self.backend._avoidance_side_clearance_m,
            self.backend._agent_radius
            + self.backend._stationary_robot_radius
            + self.backend._avoidance_side_clearance_m,
        )

    def test_open_space_keeps_sampled_regular_reaction(self):
        import time

        self.backend.send(_walking_command())
        self.backend._walkable_planner = _WalkablePlanner(segment_free=True)
        self.backend._reaction_controller = RobotProximityReactionController(
            {
                "enabled": True,
                "mode": "fixed",
                "reaction": "regular",
                "trigger": {"distance_m": 1.0},
            },
            seed=42,
        )
        self.backend._lookahead_target = type(
            "Target",
            (),
            {"x": 2.0, "y": 0.0},
        )()
        self.backend._robot = {
            "x": 0.8,
            "y": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "received_at": time.monotonic(),
        }

        self.backend._update_robot_reaction(
            {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.3, "vy": 0.0}
        )

        self.assertEqual(
            self.backend._reaction_decision.state,
            "REGULAR_TO_ROBOT",
        )

    def test_blocked_lookahead_uses_visibility_grace_before_hold(self):
        self.backend._walkable_planner = _WalkablePlanner(segment_free=False)
        self.backend.send(
            MotionCommand(
                agent_id="toilet_agent_01",
                goal_pose=[2.0, 0.0, 0.0],
                path_points=[[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                velocity=0.6,
            )
        )
        actual = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0}
        self.backend._initialize_lookahead(actual)
        agent = self.backend._build_agent(actual)

        self.backend._update_lookahead_goal(agent, 0.0, 0.0)

        self.assertIsNone(self.backend._local_route_mode)
        self.assertAlmostEqual(agent.goals[0].position.x, 0.8)
        self.assertAlmostEqual(agent.goals[0].position.y, 0.0)
        self.assertFalse(self.backend._route_hold_active)
        self.assertEqual(len(self.backend._walkable_planner.plan_calls), 1)

    def test_static_clip_latches_yaw_until_motion_is_clear(self):
        previous = Agent()
        previous.yaw = 1.2
        clipped = Agent()

        self.backend._apply_motion_yaw(
            previous,
            clipped,
            {"static_clip": True, "robot_contacts": 0, "static_contacts": 0},
            0.0,
        )
        self.assertAlmostEqual(clipped.yaw, 1.2)

        changed_feedback = Agent()
        changed_feedback.yaw = -1.0
        clipped_again = Agent()
        self.backend._apply_motion_yaw(
            changed_feedback,
            clipped_again,
            {"static_clip": True, "robot_contacts": 0, "static_contacts": 0},
            0.0,
        )
        self.assertAlmostEqual(clipped_again.yaw, 1.2)

        for _ in range(3):
            self.backend._apply_motion_yaw(
                changed_feedback,
                Agent(),
                {"static_clip": False, "robot_contacts": 0, "static_contacts": 0},
                0.0,
            )
        self.assertIsNone(self.backend._static_hold_yaw)


if __name__ == "__main__":
    unittest.main()
