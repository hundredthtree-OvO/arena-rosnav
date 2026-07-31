import time
import unittest
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from hunav_msgs.msg import Agent
import test_hunav_motion_backend as single_agent_tests

from toilet_benchmark.hunav_phase0_safety import SafetyConfig
from toilet_benchmark.motion_backend import MotionCommand


class _ImmediateFuture:
    def __init__(self, result):
        self._result = result
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return self._result


class _ResetClient:
    def __init__(self):
        self.requests = []
        self.future = _ImmediateFuture(SimpleNamespace(ok=True))

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class _Clock:
    def now(self):
        return self

    def to_msg(self):
        return Time()


class _Node:
    def get_clock(self):
        return _Clock()


def _stop_command(agent_id):
    return MotionCommand(
        agent_id=agent_id,
        goal_pose=[0.0, 0.0, 0.0],
        path_points=[],
        velocity=0.0,
        stop=True,
    )


class TestHuNavMultiAgentBackend(unittest.TestCase):
    def setUp(self):
        fixture = single_agent_tests.TestHuNavMotionBackend()
        fixture.setUp()
        self.backend = fixture.backend

    def test_accepts_two_commands_concurrently_for_different_agents(self):
        first = self.backend.send(
            single_agent_tests._walking_command("toilet_agent_01")
        )
        second = self.backend.send(
            single_agent_tests._walking_command("toilet_agent_02")
        )

        self.assertTrue(first.result().ret)
        self.assertTrue(second.result().ret)
        self.assertEqual(
            set(self.backend._commands),
            {"toilet_agent_01", "toilet_agent_02"},
        )

    def test_pre_registered_session_enters_roster_only_on_activation(self):
        holds = [
            _stop_command("toilet_agent_01"),
            _stop_command("toilet_agent_02"),
        ]

        self.backend.register_agents(holds)
        roster_generation = self.backend._batch_generation
        self.backend.send(
            single_agent_tests._walking_command("toilet_agent_01")
        )

        self.assertEqual(self.backend._batch_generation, roster_generation + 1)
        self.assertEqual(
            set(self.backend._runtime_states),
            {"toilet_agent_01", "toilet_agent_02"},
        )
        self.assertTrue(
            self.backend._runtime_states["toilet_agent_01"]["_hunav_active"]
        )
        self.assertFalse(
            self.backend._runtime_states["toilet_agent_02"]["_hunav_active"]
        )
        self.assertFalse(
            self.backend._runtime_states["toilet_agent_01"]["_frozen_present"]
        )
        self.assertTrue(
            self.backend._runtime_states["toilet_agent_02"]["_frozen_present"]
        )

    def test_pre_registered_hold_preserves_future_desired_speed(self):
        command = MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[1000.0, 1000.0, 0.0],
            path_points=[],
            velocity=0.75,
            stop=True,
        )
        self.backend.register_agents([command])
        now = time.monotonic()
        self.backend._actual = {
            command.agent_id: {
                "x": 1000.0,
                "y": 1000.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            }
        }

        with self.backend._runtime_context(command.agent_id):
            built = self.backend._build_agent(
                self.backend._actual[command.agent_id]
            )

        self.assertEqual(built.goals, [])
        self.assertEqual(built.linear_vel, 0.0)
        self.assertAlmostEqual(built.desired_velocity, 0.75)
        self.assertAlmostEqual(built.behavior.vel, 0.75)

    def test_inactive_prespawn_session_is_excluded_from_shared_reset(self):
        self.backend.register_agents(
            [
                _stop_command("toilet_agent_01"),
                _stop_command("toilet_agent_02"),
            ]
        )
        self.backend.send(
            single_agent_tests._walking_command("toilet_agent_01")
        )
        self.backend._node = _Node()
        self.backend._reset_client = _ResetClient()
        self.backend._reset_future = None
        self.backend._compute_future = None
        now = time.monotonic()
        self.backend._state_timeout_sec = 5.0
        self.backend._actual = {
            "toilet_agent_01": {
                "x": -2.0,
                "y": -0.8,
                "z": 0.0,
                "vx": 0.4,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            },
            "toilet_agent_02": {
                "x": 1002.0,
                "y": 1000.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            },
        }

        self.backend._tick()

        request = self.backend._reset_client.requests[0]
        self.assertEqual(
            [agent.name for agent in request.current_agents.agents],
            ["toilet_agent_01"],
        )

    def test_replanning_existing_agent_does_not_invalidate_shared_roster(self):
        self.backend.register_agents(
            [
                _stop_command("toilet_agent_01"),
                _stop_command("toilet_agent_02"),
            ]
        )
        self.backend.send(
            single_agent_tests._walking_command("toilet_agent_01")
        )
        roster_generation = self.backend._batch_generation

        self.backend.send(
            single_agent_tests._walking_command("toilet_agent_01")
        )

        self.assertEqual(self.backend._batch_generation, roster_generation)

    def test_activation_direct_pose_keeps_pre_registered_member(self):
        agent_id = "toilet_agent_01"
        self.backend.register_agents([_stop_command(agent_id)])
        command = MotionCommand(
            agent_id=agent_id,
            goal_pose=[-3.8, -0.91, 0.0],
            path_points=[],
            velocity=0.0,
            stop=True,
            use_direct_pose=True,
            direct_pose=[-3.8, -0.91, 0.0],
        )

        self.backend.send(command)

        self.assertIn(agent_id, self.backend._runtime_states)

    def test_dispatch_reset_batches_all_active_agents(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        self.backend._node = _Node()
        self.backend._reset_client = _ResetClient()
        now = time.monotonic()
        self.backend._state_timeout_sec = 5.0
        self.backend._actual = {
            "toilet_agent_01": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            },
            "toilet_agent_02": {
                "x": 1.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            },
        }

        self.backend._dispatch_reset()

        self.assertEqual(len(self.backend._reset_client.requests), 1)
        request = self.backend._reset_client.requests[0]
        self.assertEqual(
            {agent.name for agent in request.current_agents.agents},
            {"toilet_agent_01", "toilet_agent_02"},
        )
        self.assertEqual(
            len({agent.id for agent in request.current_agents.agents}),
            2,
        )

    def test_stopping_one_agent_freezes_it_in_shared_world(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))

        self.backend.send(_stop_command("toilet_agent_01"))

        self.assertEqual(
            set(self.backend._commands),
            {"toilet_agent_01", "toilet_agent_02"},
        )
        self.assertTrue(
            self.backend._runtime_states["toilet_agent_01"]["_frozen_present"]
        )

    def test_frozen_agent_is_included_as_stationary_reset_member(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        self.backend.send(_stop_command("toilet_agent_01"))
        self.backend._node = _Node()
        self.backend._reset_client = _ResetClient()
        now = time.monotonic()
        self.backend._state_timeout_sec = 5.0
        self.backend._actual = {
            agent_id: {
                "x": float(index),
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            }
            for index, agent_id in enumerate(
                ("toilet_agent_01", "toilet_agent_02")
            )
        }

        self.backend._dispatch_reset()

        request = self.backend._reset_client.requests[0]
        frozen = next(
            agent
            for agent in request.current_agents.agents
            if agent.name == "toilet_agent_01"
        )
        self.assertEqual(frozen.goals, [])
        self.assertEqual(frozen.desired_velocity, 0.0)
        self.assertEqual(frozen.behavior.vel, 0.0)

    def test_remove_agent_only_removes_requested_shared_world_member(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))

        self.backend.remove_agent("toilet_agent_01")

        self.assertEqual(set(self.backend._commands), {"toilet_agent_02"})
        self.assertEqual(set(self.backend._runtime_states), {"toilet_agent_02"})

    def test_resuming_frozen_agent_restores_motion_state(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        self.backend.send(_stop_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))

        self.assertFalse(
            self.backend._runtime_states["toilet_agent_01"]["_frozen_present"]
        )
        self.assertFalse(self.backend._commands["toilet_agent_01"].stop)

    def test_compute_result_is_dispatched_to_each_agent_session(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        previous = {}
        updated = []
        for index, agent_id in enumerate(
            ("toilet_agent_01", "toilet_agent_02"),
            start=1,
        ):
            old = Agent()
            old.id = index
            old.name = agent_id
            new = Agent()
            new.id = index
            new.name = agent_id
            previous[agent_id] = old
            updated.append(new)
        processed = []
        self.backend._compute_generation = self.backend._batch_generation
        self.backend._compute_agent_ids = (
            "toilet_agent_01",
            "toilet_agent_02",
        )
        self.backend._eligible_agent_ids = self.backend._compute_agent_ids
        self.backend._compute_previous_by_id = previous
        self.backend._last_batch_compute_at = time.monotonic() - 0.1
        self.backend._compute_future = object()
        self.backend._safety_enabled = False
        self.backend._finish_computed_agent = (
            lambda old, new, safety: processed.append(
                (self.backend._command.agent_id, new.name)
            )
        )
        result = SimpleNamespace(
            updated_agents=SimpleNamespace(agents=updated)
        )

        self.backend._compute_done(_ImmediateFuture(result))

        self.assertEqual(
            processed,
            [
                ("toilet_agent_01", "toilet_agent_01"),
                ("toilet_agent_02", "toilet_agent_02"),
            ],
        )

    def test_partial_compute_response_invalidates_batch(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        previous = {}
        for index, agent_id in enumerate(
            ("toilet_agent_01", "toilet_agent_02"),
            start=1,
        ):
            agent = Agent()
            agent.id = index
            agent.name = agent_id
            previous[agent_id] = agent
        returned = Agent()
        returned.id = 1
        returned.name = "toilet_agent_01"
        self.backend._compute_generation = self.backend._batch_generation
        self.backend._compute_agent_ids = (
            "toilet_agent_01",
            "toilet_agent_02",
        )
        self.backend._eligible_agent_ids = self.backend._compute_agent_ids
        self.backend._compute_previous_by_id = previous
        self.backend._active_batch_generation = self.backend._batch_generation
        self.backend._active_batch_agent_ids = self.backend._compute_agent_ids
        result = SimpleNamespace(
            updated_agents=SimpleNamespace(agents=[returned])
        )

        self.backend._compute_done(_ImmediateFuture(result))

        self.assertEqual(self.backend._active_batch_generation, 0)
        self.assertEqual(self.backend._active_batch_agent_ids, ())

    def test_reset_warmup_does_not_dispatch_zero_motion_response(self):
        agent_id = "toilet_agent_01"
        self.backend.send(single_agent_tests._walking_command(agent_id))
        previous = Agent()
        previous.id = 1
        previous.name = agent_id
        returned = Agent()
        returned.id = 1
        returned.name = agent_id
        self.backend._compute_generation = self.backend._batch_generation
        self.backend._compute_agent_ids = (agent_id,)
        self.backend._compute_runtime_generations = {
            agent_id: self.backend._runtime_states[agent_id]["_generation"]
        }
        self.backend._eligible_agent_ids = (agent_id,)
        self.backend._compute_previous_by_id = {agent_id: previous}
        self.backend._active_batch_generation = self.backend._batch_generation
        self.backend._active_batch_agent_ids = (agent_id,)
        self.backend._batch_warmup_pending = True
        dispatched = []
        self.backend._finish_computed_agent = (
            lambda *args, **kwargs: dispatched.append(args)
        )
        result = SimpleNamespace(
            updated_agents=SimpleNamespace(agents=[returned])
        )

        self.backend._compute_done(_ImmediateFuture(result))

        self.assertEqual(dispatched, [])
        self.assertFalse(self.backend._batch_warmup_pending)
        self.assertIsNotNone(
            self.backend._runtime_states[agent_id]["_shadow"]
        )

    def test_reset_warmup_keeps_measured_snapshot_not_integrated_response(self):
        agent_id = "toilet_agent_01"
        self.backend.send(single_agent_tests._walking_command(agent_id))
        previous = Agent()
        previous.id = 1
        previous.name = agent_id
        previous.position.position.x = 1.0
        returned = Agent()
        returned.id = 1
        returned.name = agent_id
        returned.position.position.x = 2.0
        self.backend._compute_generation = self.backend._batch_generation
        self.backend._compute_agent_ids = (agent_id,)
        self.backend._compute_runtime_generations = {
            agent_id: self.backend._runtime_states[agent_id]["_generation"]
        }
        self.backend._eligible_agent_ids = (agent_id,)
        self.backend._compute_previous_by_id = {agent_id: previous}
        self.backend._active_batch_generation = self.backend._batch_generation
        self.backend._active_batch_agent_ids = (agent_id,)
        self.backend._batch_warmup_pending = True

        self.backend._compute_done(
            _ImmediateFuture(
                SimpleNamespace(
                    updated_agents=SimpleNamespace(agents=[returned])
                )
            )
        )

        shadow = self.backend._runtime_states[agent_id]["_shadow"].agents[0]
        self.assertEqual(shadow.position.position.x, 1.0)

    def test_roster_reset_refreshes_existing_external_motion_reference(self):
        first = "toilet_agent_01"
        second = "toilet_agent_02"
        self.backend.send(single_agent_tests._walking_command(first))
        self.backend._active_batch_agent_ids = (first,)
        self.backend.send(single_agent_tests._walking_command(second))
        self.backend._reset_future = object()
        self.backend._state_timeout_sec = 5.0
        self.backend._actual[first] = {
            "x": -2.5,
            "y": -0.9,
            "z": 0.0,
            "vx": 0.4,
            "vy": 0.1,
            "yaw": 0.25,
            "received_at": time.monotonic(),
        }
        sent = []
        self.backend._isaac.send = lambda command, callback=None: (
            sent.append(command)
            or _ImmediateFuture(SimpleNamespace(ret=True))
        )

        self.backend._refresh_existing_motion_during_reset()

        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0].agent_id, first)
        self.assertEqual(tuple(sent[0].direct_pose), (-2.5, -0.9, 0.0))
        self.assertEqual(tuple(sent[0].external_velocity), (0.4, 0.1, 0.0))

    def test_pair_safety_uses_hard_radius_not_social_radius(self):
        self.backend._social_radius = 0.30
        self.backend._agent_radius = self.backend._social_radius
        self.backend._hard_radius = 0.26
        self.backend._safety_enabled = True
        self.backend._safety_config = SafetyConfig(clearance_m=0.01)
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        previous = {}
        proposed = {}
        for index, (agent_id, x) in enumerate(
            (("toilet_agent_01", 0.0), ("toilet_agent_02", 0.531)),
            start=1,
        ):
            old = Agent()
            old.id = index
            old.name = agent_id
            old.position.position.x = x
            new = Agent()
            new.id = index
            new.name = agent_id
            new.position.position.x = x
            previous[agent_id] = old
            proposed[agent_id] = new

        safety = {}
        self.backend._apply_batch_pedestrian_safety(
            previous, proposed, safety, 0.1
        )

        self.assertAlmostEqual(
            abs(
                proposed["toilet_agent_01"].position.position.x
                - proposed["toilet_agent_02"].position.position.x
            ),
            0.531,
        )
        self.assertFalse(
            safety["toilet_agent_01"]["pedestrian_pair_intervened"]
        )

    def test_agent_without_fresh_pose_does_not_block_ready_agent_reset(self):
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        self.backend._node = _Node()
        self.backend._reset_client = _ResetClient()
        now = time.monotonic()
        self.backend._state_timeout_sec = 5.0
        self.backend._actual = {
            "toilet_agent_01": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "yaw": 0.0,
                "received_at": now,
            },
        }
        self.backend._reset_future = None
        self.backend._compute_future = None

        self.backend._tick()

        request = self.backend._reset_client.requests[0]
        self.assertEqual(
            [agent.name for agent in request.current_agents.agents],
            ["toilet_agent_01"],
        )

    def test_stale_external_callback_cannot_clear_recreated_session_future(self):
        agent_id = "toilet_agent_01"
        self.backend.send(single_agent_tests._walking_command(agent_id))
        agent = Agent()
        agent.name = agent_id
        agent.id = 1
        self.backend._send_external_motion(agent)
        stale_callback = self.backend._isaac.callbacks[-1]
        self.backend.send(_stop_command(agent_id))
        self.backend.send(single_agent_tests._walking_command(agent_id))
        sentinel = object()
        with self.backend._runtime_context(agent_id):
            self.backend._external_future = sentinel

        stale_callback(_ImmediateFuture(SimpleNamespace(ret=True)))

        with self.backend._runtime_context(agent_id):
            self.assertIs(self.backend._external_future, sentinel)

    def test_pair_safety_prevents_two_agents_from_crossing_through_each_other(self):
        self.backend._safety_enabled = True
        self.backend._safety_config = SafetyConfig()
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        previous_left = Agent()
        previous_left.id = 1
        previous_left.name = "toilet_agent_01"
        previous_left.position.position.x = -0.4
        previous_right = Agent()
        previous_right.id = 2
        previous_right.name = "toilet_agent_02"
        previous_right.position.position.x = 0.4
        proposed_left = Agent()
        proposed_left.id = 1
        proposed_left.name = "toilet_agent_01"
        proposed_left.position.position.x = 0.4
        proposed_right = Agent()
        proposed_right.id = 2
        proposed_right.name = "toilet_agent_02"
        proposed_right.position.position.x = -0.4
        safety = {}

        self.backend._apply_batch_pedestrian_safety(
            {
                "toilet_agent_01": previous_left,
                "toilet_agent_02": previous_right,
            },
            {
                "toilet_agent_01": proposed_left,
                "toilet_agent_02": proposed_right,
            },
            safety,
            0.1,
        )

        separation = abs(
            proposed_left.position.position.x
            - proposed_right.position.position.x
        )
        self.assertGreaterEqual(separation, 0.60)
        self.assertTrue(
            safety["toilet_agent_01"]["pedestrian_pair_intervened"]
        )
        self.assertTrue(
            safety["toilet_agent_02"]["pedestrian_pair_intervened"]
        )

    def test_pair_safety_keeps_yielding_agent_fixed(self):
        self.backend._safety_enabled = True
        self.backend._safety_config = SafetyConfig()
        self.backend.send(single_agent_tests._walking_command("toilet_agent_01"))
        self.backend.send(single_agent_tests._walking_command("toilet_agent_02"))
        with self.backend._runtime_context("toilet_agent_01"):
            self.backend._reaction_decision = SimpleNamespace(reaction="yielding")
        previous = {}
        proposed = {}
        for index, (agent_id, x) in enumerate(
            (("toilet_agent_01", 0.0), ("toilet_agent_02", 1.0)),
            start=1,
        ):
            old = Agent()
            old.id = index
            old.name = agent_id
            old.position.position.x = x
            new = Agent()
            new.id = index
            new.name = agent_id
            new.position.position.x = 0.5
            previous[agent_id] = old
            proposed[agent_id] = new

        self.backend._apply_batch_pedestrian_safety(
            previous,
            proposed,
            {},
            0.1,
        )

        self.assertAlmostEqual(
            proposed["toilet_agent_01"].position.position.x,
            0.0,
        )


if __name__ == "__main__":
    unittest.main()
