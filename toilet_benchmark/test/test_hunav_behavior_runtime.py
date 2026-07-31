import unittest
import time
from types import SimpleNamespace

from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior, Agents

from toilet_benchmark.motion.hunav.behavior import (
    HUNAV_BEHAVIOR_DEFAULTS,
    HUNAV_BEHAVIOR_ORDER,
    HUNAV_BEHAVIOR_TYPE_BY_NAME,
    build_hunav_behavior,
    describe_hunav_behavior,
)
from toilet_benchmark.motion_backend import MotionCommand
import test_hunav_motion_backend as single_agent_tests
from toilet_benchmark.hunav_motion_backend import HuNavMotionBackend


class TestHuNavBehaviorProfiles(unittest.TestCase):
    def test_behavior_order_covers_all_s2a_types(self):
        self.assertEqual(
            tuple(HUNAV_BEHAVIOR_ORDER),
            (
                "regular",
                "impassive",
                "surprised",
                "scared",
                "curious",
                "threatening",
            ),
        )

    def test_build_behavior_applies_defaults_when_no_profile(self):
        behavior = build_hunav_behavior(
            behavior_name="regular",
            command_velocity=0.6,
            speed_scale=1.0,
            behavior_config={},
        )

        self.assertEqual(behavior.type, AgentBehavior.BEH_REGULAR)
        self.assertEqual(behavior.state, AgentBehavior.BEH_NO_ACTIVE)
        self.assertEqual(behavior.configuration, AgentBehavior.BEH_CONF_CUSTOM)
        self.assertEqual(behavior.vel, 0.6)
        self.assertEqual(behavior.dist, 1.0)
        self.assertEqual(
            behavior.social_force_factor,
            HUNAV_BEHAVIOR_DEFAULTS["social_force_factor"],
        )
        self.assertEqual(
            behavior.goal_force_factor,
            HUNAV_BEHAVIOR_DEFAULTS["goal_force_factor"],
        )

    def test_build_behavior_supports_per_behavior_overrides(self):
        behavior = build_hunav_behavior(
            behavior_name="threatening",
            command_velocity=0.8,
            speed_scale=1.0,
            behavior_config={
                "social_force_factor": 1.0,
                "threatening": {
                    "goal_force_factor": 9.0,
                    "obstacle_force_factor": 11.0,
                },
            },
        )

        self.assertEqual(behavior.type, AgentBehavior.BEH_THREATENING)
        self.assertEqual(behavior.social_force_factor, 1.0)
        self.assertEqual(behavior.goal_force_factor, 9.0)
        self.assertEqual(behavior.obstacle_force_factor, 11.0)
        self.assertEqual(behavior.other_force_factor, 20.0)

    def test_build_behavior_maps_complete_native_profile(self):
        behavior = build_hunav_behavior(
            behavior_name="surprised",
            command_velocity=0.8,
            speed_scale=0.5,
            behavior_config={
                "surprised": {
                    "configuration": "random_normal",
                    "duration": 3.5,
                    "once": True,
                    "vel": 0.6,
                    "dist": 1.4,
                    "social_force_factor": 1.1,
                    "goal_force_factor": 2.2,
                    "obstacle_force_factor": 3.3,
                    "other_force_factor": 4.4,
                }
            },
        )

        self.assertEqual(behavior.type, AgentBehavior.BEH_SURPRISED)
        self.assertEqual(
            behavior.configuration,
            AgentBehavior.BEH_CONF_RANDOM_NORMAL,
        )
        self.assertEqual(behavior.duration, 3.5)
        self.assertTrue(behavior.once)
        self.assertAlmostEqual(behavior.vel, 0.3)
        self.assertAlmostEqual(behavior.dist, 1.4)
        self.assertAlmostEqual(behavior.social_force_factor, 1.1)
        self.assertAlmostEqual(behavior.goal_force_factor, 2.2)
        self.assertAlmostEqual(behavior.obstacle_force_factor, 3.3)
        self.assertAlmostEqual(behavior.other_force_factor, 4.4)

    def test_build_behavior_rejects_unknown_configuration(self):
        with self.assertRaises(ValueError):
            build_hunav_behavior(
                behavior_name="regular",
                command_velocity=0.5,
                speed_scale=1.0,
                behavior_config={"regular": {"configuration": "invalid"}},
            )

    def test_backend_uses_behavior_carried_by_motion_command(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        backend: HuNavMotionBackend = fixture.backend
        command = MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[1.0, 0.0, 0.0],
            path_points=[[1.0, 0.0, 0.0]],
            velocity=0.6,
            behavior="scared",
        )
        backend.send(command)

        with backend._runtime_context(command.agent_id):
            agent = Agent()
            backend._restore_agent_fields(agent)

        self.assertEqual(agent.behavior.type, AgentBehavior.BEH_SCARED)

    def test_returned_native_state_survives_compute_and_is_reused_on_next_request(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        backend: HuNavMotionBackend = fixture.backend
        command = MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[1.0, 0.0, 0.0],
            path_points=[[1.0, 0.0, 0.0]],
            velocity=0.6,
            behavior="surprised",
        )
        backend.send(command)

        previous = Agent()
        previous.name = command.agent_id
        previous.position.position.x = 0.0
        previous.position.position.y = 0.0
        previous.yaw = 0.0
        previous.velocity.linear.x = 0.0
        previous.velocity.linear.y = 0.0
        previous.linear_vel = 0.0

        updated = Agent()
        updated.name = command.agent_id
        updated.position.position.x = 0.1
        updated.position.position.y = 0.0
        updated.yaw = 0.0
        updated.velocity.linear.x = 0.2
        updated.velocity.linear.y = 0.0
        updated.linear_vel = 0.2
        updated.goals = [Pose()]
        updated.behavior = build_hunav_behavior(
            behavior_name="surprised",
            command_velocity=0.6,
            speed_scale=1.0,
            behavior_config={},
        )
        updated.behavior.state = AgentBehavior.BEH_ACTIVE_1

        backend._compute_generation = 1
        backend._batch_generation = 1
        backend._compute_previous_by_id = {command.agent_id: previous}
        backend._compute_runtime_generations = {command.agent_id: 1}
        backend._compute_agent_ids = (command.agent_id,)
        backend._eligible_agent_ids = (command.agent_id,)
        backend._last_batch_compute_at = time.monotonic()
        backend._publish_live_diagnostics = lambda prev, agent, safety: diag_states.append(
            int(agent.behavior.state)
        )
        backend._set_runtime_shadow = lambda agent: None
        backend._send_external_motion = lambda *args, **kwargs: None
        backend._clip_hunav_step = lambda previous, agent, dt: None
        backend._apply_hard_safety = lambda previous, agent, dt: {"enabled": False}
        backend._apply_batch_pedestrian_safety = lambda *args, **kwargs: None
        diag_states: list[int] = []
        updated_agents = Agents()
        updated_agents.agents = [updated]

        backend._compute_done(
            SimpleNamespace(
                result=lambda: SimpleNamespace(updated_agents=updated_agents)
            )
        )

        self.assertEqual(diag_states, [AgentBehavior.BEH_ACTIVE_1])
        self.assertEqual(
            backend._native_behavior_profile_by_agent[command.agent_id],
            "surprised",
        )
        self.assertEqual(
            backend._native_behavior_response_state_by_agent[command.agent_id],
            AgentBehavior.BEH_ACTIVE_1,
        )

        with backend._runtime_context(command.agent_id):
            restored = Agent()
            backend._restore_agent_fields(restored)

        self.assertEqual(restored.behavior.state, AgentBehavior.BEH_ACTIVE_1)

    def test_behavior_diagnostics_expose_native_and_custom_layers(self):
        behavior = build_hunav_behavior(
            behavior_name="curious",
            command_velocity=0.7,
            speed_scale=1.0,
            behavior_config={
                "curious": {
                    "configuration": "random_uniform",
                    "duration": 2.0,
                    "once": True,
                }
            },
        )

        diagnostics = describe_hunav_behavior("curious", behavior)

        self.assertEqual(diagnostics["profile"], "curious")
        self.assertEqual(diagnostics["native_type"], "curious")
        self.assertEqual(diagnostics["configuration"], "random_uniform")
        self.assertEqual(diagnostics["state"], "inactive")
        self.assertEqual(diagnostics["duration"], 2.0)
        self.assertTrue(diagnostics["once"])

    def test_unknown_behavior_name_falls_back_to_regular(self):
        behavior = build_hunav_behavior(
            behavior_name="ghost",
            command_velocity=0.4,
            speed_scale=0.5,
            behavior_config={},
        )
        self.assertEqual(behavior.type, AgentBehavior.BEH_REGULAR)
        self.assertEqual(behavior.vel, 0.2)

    def test_behavior_type_mapping_covers_all_s2a_names(self):
        expected_types = (
            AgentBehavior.BEH_REGULAR,
            AgentBehavior.BEH_IMPASSIVE,
            AgentBehavior.BEH_SURPRISED,
            AgentBehavior.BEH_SCARED,
            AgentBehavior.BEH_CURIOUS,
            AgentBehavior.BEH_THREATENING,
        )
        observed_types = tuple(
            HUNAV_BEHAVIOR_TYPE_BY_NAME[name]
            for name in HUNAV_BEHAVIOR_ORDER
        )
        self.assertEqual(observed_types, expected_types)


class TestHuNavRuntimeRegistry(unittest.TestCase):
    def test_registry_captures_and_restores_session_isolated(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        backend: HuNavMotionBackend = fixture.backend

        backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        backend.send(single_agent_tests._walking_command("toilet_agent_02"))

        with backend._runtime_context("toilet_agent_01"):
            backend._generation = 10
            backend._settled_agent_id = "toilet_agent_01"
            backend._yield_hold_yaw = 1.2

        with backend._runtime_context("toilet_agent_02"):
            backend._generation = 20
            backend._settled_agent_id = "toilet_agent_02"
            backend._yield_hold_yaw = 2.4

        with backend._runtime_context("toilet_agent_01"):
            self.assertEqual(backend._generation, 10)
            self.assertEqual(backend._settled_agent_id, "toilet_agent_01")
            self.assertEqual(backend._yield_hold_yaw, 1.2)

        with backend._runtime_context("toilet_agent_02"):
            self.assertEqual(backend._generation, 20)
            self.assertEqual(backend._settled_agent_id, "toilet_agent_02")
            self.assertEqual(backend._yield_hold_yaw, 2.4)

    def test_registry_remove_agent_is_isolated_by_id(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        backend: HuNavMotionBackend = fixture.backend

        backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        backend.send(single_agent_tests._walking_command("toilet_agent_02"))

        with backend._runtime_context("toilet_agent_01"):
            backend._generation = 7

        backend.remove_agent("toilet_agent_01")

        self.assertNotIn("toilet_agent_01", backend._runtime_states)
        self.assertIn("toilet_agent_02", backend._runtime_states)
        with backend._runtime_context("toilet_agent_02"):
            self.assertEqual(backend._generation, 1)
            backend._generation = 11

        backend.send(
            MotionCommand(
                agent_id="toilet_agent_03",
                goal_pose=[1.0, 0.0, 0.0],
                path_points=[[0.5, 0.0, 0.0], [1.0, 0.0, 0.0]],
                velocity=0.6,
            )
        )

        self.assertIn("toilet_agent_03", backend._runtime_states)
        self.assertEqual(backend._runtime_states["toilet_agent_03"]["_active_generation"], 0)

    def test_registry_isolates_hold_and_alignment_state_between_agents(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        backend: HuNavMotionBackend = fixture.backend

        backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        backend.send(single_agent_tests._walking_command("toilet_agent_02"))

        with backend._runtime_context("toilet_agent_01"):
            backend._route_hold_active = True
            backend._route_hold_yaw = 1.1
            backend._yield_hold_yaw = 2.2
            backend._terminal_align_active = True
            backend._terminal_align_stable_since = 11.1

        with backend._runtime_context("toilet_agent_02"):
            backend._route_hold_active = False
            backend._route_hold_yaw = None
            backend._yield_hold_yaw = None
            backend._terminal_align_active = False
            backend._terminal_align_stable_since = 0.0

        with backend._runtime_context("toilet_agent_01"):
            self.assertTrue(backend._route_hold_active)
            self.assertEqual(backend._route_hold_yaw, 1.1)
            self.assertEqual(backend._yield_hold_yaw, 2.2)
            self.assertTrue(backend._terminal_align_active)
            self.assertEqual(backend._terminal_align_stable_since, 11.1)

        with backend._runtime_context("toilet_agent_02"):
            self.assertFalse(backend._route_hold_active)
            self.assertIsNone(backend._route_hold_yaw)
            self.assertIsNone(backend._yield_hold_yaw)
            self.assertFalse(backend._terminal_align_active)
            self.assertEqual(backend._terminal_align_stable_since, 0.0)


if __name__ == "__main__":
    unittest.main()
