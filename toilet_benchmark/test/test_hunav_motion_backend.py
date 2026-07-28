import unittest

from toilet_benchmark.hunav_motion_backend import HuNavMotionBackend
from toilet_benchmark.motion_backend import MotionCommand
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


def _walking_command(agent_id="toilet_agent_01"):
    return MotionCommand(
        agent_id=agent_id,
        goal_pose=[1.0, 0.0, 0.0],
        path_points=[[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]],
        velocity=0.6,
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
        self.backend._settled_generation = 0
        self.backend._settled_agent_id = ""
        self.backend._settled_arrival_tolerance_m = 0.35
        self.backend._lookahead_distance_m = 0.80
        self.backend._lookahead_tracker = None
        self.backend._lookahead_target = None
        self.backend._yield_hold_yaw = None
        self.backend._last_external_motion_mode = 0
        self.backend._agent_radius = 0.30
        self.backend._goal_radius = 0.12
        self.backend._behavior = {}
        self.backend._external_timeout_sec = 0.35
        self.backend._external_future = None
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
        self.backend._route_publisher = None

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

    def test_second_agent_is_rejected_while_first_owns_backend(self):
        self.backend.send(_walking_command("toilet_agent_01"))

        future = self.backend.send(_walking_command("toilet_agent_02"))

        self.assertFalse(future.result().ret)
        self.assertEqual(self.backend._command.agent_id, "toilet_agent_01")

    def test_direct_pose_disables_hunav_and_uses_compatibility_adapter(self):
        self.backend.send(_walking_command())
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

        self.assertIsNone(self.backend._command)
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


if __name__ == "__main__":
    unittest.main()
