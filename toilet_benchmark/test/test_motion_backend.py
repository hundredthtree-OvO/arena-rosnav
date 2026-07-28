import importlib
import sys
import types
import unittest


class _FakeFuture:
    def __init__(self):
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)


class _FakeClient:
    def __init__(self, future=None, ready=True):
        self.future = future or _FakeFuture()
        self.ready = ready
        self.requests = []
        self.wait_calls = []

    def wait_for_service(self, timeout_sec):
        self.wait_calls.append(timeout_sec)
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


class _FakeNode:
    def __init__(self, client):
        self.client = client
        self.create_client_calls = []

    def create_client(self, service_type, service_name):
        self.create_client_calls.append((service_type, service_name))
        return self.client


def _install_fake_isaacsim_msgs():
    isaacsim_msgs = types.ModuleType("isaacsim_msgs")
    msg = types.ModuleType("isaacsim_msgs.msg")
    srv = types.ModuleType("isaacsim_msgs.srv")

    class NavPed:
        def __init__(self):
            self.path = ""
            self.goal_pose = []
            self.path_points_flat = []
            self.velocity = None
            self.orientation = None
            self.stop = None
            self.use_direct_pose = None
            self.direct_pose = []
            self.use_external_motion = None
            self.external_velocity = []
            self.external_timeout_sec = None
            self.external_freeze_pose = None
            self.constrain_to_path = None
            self.loop_path = None

    class MovePed:
        class Request:
            def __init__(self):
                self.nav_list = []

    msg.NavPed = NavPed
    srv.MovePed = MovePed
    isaacsim_msgs.msg = msg
    isaacsim_msgs.srv = srv

    sys.modules["isaacsim_msgs"] = isaacsim_msgs
    sys.modules["isaacsim_msgs.msg"] = msg
    sys.modules["isaacsim_msgs.srv"] = srv


def _load_motion_backend_module():
    _install_fake_isaacsim_msgs()
    sys.modules.pop("toilet_benchmark.motion_backend", None)
    return importlib.import_module("toilet_benchmark.motion_backend")


class TestMotionBackend(unittest.TestCase):
    def test_motion_command_exposes_phase1_backend_contract_fields(self):
        module = _load_motion_backend_module()

        command = module.MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[1.0, 2.0, 0.0],
            path_points=[[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]],
            velocity=0.0,
            orientation=1.25,
            stop=True,
            use_direct_pose=True,
            direct_pose=[-3.0, 4.0, 0.0],
            use_external_motion=True,
            external_velocity=[0.2, -0.1, 0.0],
            external_timeout_sec=0.4,
            external_freeze_pose=True,
            constrain_to_path=True,
        )

        self.assertEqual(command.agent_id, "toilet_agent_01")
        self.assertEqual(command.goal_pose, [1.0, 2.0, 0.0])
        self.assertEqual(command.path_points, [[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]])
        self.assertEqual(command.velocity, 0.0)
        self.assertEqual(command.orientation, 1.25)
        self.assertTrue(command.stop)
        self.assertTrue(command.use_direct_pose)
        self.assertEqual(command.direct_pose, [-3.0, 4.0, 0.0])
        self.assertTrue(command.use_external_motion)
        self.assertEqual(command.external_velocity, [0.2, -0.1, 0.0])
        self.assertEqual(command.external_timeout_sec, 0.4)
        self.assertTrue(command.external_freeze_pose)
        self.assertTrue(command.constrain_to_path)

    def test_backend_wait_for_service_uses_move_client_timeout(self):
        module = _load_motion_backend_module()
        client = _FakeClient(ready=False)
        node = _FakeNode(client)
        backend = module.IsaacPeopleBackend(node, "/isaac/move_pedestrians")

        ready = backend.wait_for_service(0.75)

        self.assertFalse(ready)
        self.assertEqual(len(node.create_client_calls), 1)
        self.assertEqual(node.create_client_calls[0][1], "/isaac/move_pedestrians")
        self.assertEqual(client.wait_calls, [0.75])

    def test_isaac_backend_allows_legacy_director_stall_recovery(self):
        module = _load_motion_backend_module()
        backend = module.IsaacPeopleBackend(
            _FakeNode(_FakeClient()),
            "/isaac/move_pedestrians",
        )

        self.assertTrue(
            backend.allows_director_stall_recovery("toilet_agent_01")
        )

    def test_send_converts_command_to_one_move_request_with_flattened_path(self):
        module = _load_motion_backend_module()
        future = _FakeFuture()
        client = _FakeClient(future=future)
        backend = module.IsaacPeopleBackend(_FakeNode(client), "/isaac/move_pedestrians")
        command = module.MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[4.0, 5.0, 0.0],
            path_points=[[1.0, 2.0, 0.0], [3.0, 4.0, 0.0]],
            velocity=0.0,
            orientation=-0.5,
            stop=False,
            use_direct_pose=True,
            direct_pose=[9.0, 8.0, 0.0],
            use_external_motion=True,
            external_velocity=[0.3, 0.4, 0.0],
            external_timeout_sec=0.6,
            external_freeze_pose=True,
            constrain_to_path=True,
        )

        returned_future = backend.send(command)

        self.assertIs(returned_future, future)
        self.assertEqual(len(client.requests), 1)
        request = client.requests[0]
        self.assertEqual(len(request.nav_list), 1)
        nav = request.nav_list[0]
        self.assertEqual(nav.path, "toilet_agent_01")
        self.assertEqual(nav.goal_pose, [4.0, 5.0, 0.0])
        self.assertEqual(nav.path_points_flat, [1.0, 2.0, 0.0, 3.0, 4.0, 0.0])
        self.assertEqual(nav.velocity, 0.0)
        self.assertEqual(nav.orientation, -0.5)
        self.assertFalse(nav.stop)
        self.assertTrue(nav.use_direct_pose)
        self.assertEqual(nav.direct_pose, [9.0, 8.0, 0.0])
        self.assertTrue(nav.use_external_motion)
        self.assertEqual(nav.external_velocity, [0.3, 0.4, 0.0])
        self.assertEqual(nav.external_timeout_sec, 0.6)
        self.assertTrue(nav.external_freeze_pose)
        self.assertTrue(nav.constrain_to_path)
        self.assertFalse(nav.loop_path)

    def test_send_attaches_provided_done_callback_to_future(self):
        module = _load_motion_backend_module()
        future = _FakeFuture()
        client = _FakeClient(future=future)
        backend = module.IsaacPeopleBackend(_FakeNode(client), "/isaac/move_pedestrians")
        callback = object()
        command = module.MotionCommand(
            agent_id="toilet_agent_02",
            goal_pose=[0.0, 0.0, 0.0],
            path_points=[],
            velocity=0.3,
            orientation=0.0,
            stop=True,
            use_direct_pose=False,
            direct_pose=None,
            constrain_to_path=False,
        )

        returned_future = backend.send(command, done_callback=callback)

        self.assertIs(returned_future, future)
        self.assertEqual(future.callbacks, [callback])


if __name__ == "__main__":
    unittest.main()
