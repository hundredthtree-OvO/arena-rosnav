import json
from types import SimpleNamespace

from std_msgs.msg import String

from toilet_benchmark.manual_collection_node import ManualCollectionNode


class _Recorder:
    def __init__(self):
        self.events = []

    def record_event(self, event, payload):
        self.events.append((event, payload))


class _Harness:
    _event_payload = ManualCollectionNode._event_payload

    def __init__(self):
        self._state = "RUNNING"
        self._finishing = False
        self.robot_name = "xms_mecanum"
        self.recorder = _Recorder()
        self.finished = []

    def _finish_episode(
        self,
        status,
        reason,
        *,
        extra=None,
        abort_pedestrian_runtime=False,
    ):
        self.finished.append(
            (status, reason, extra, abort_pedestrian_runtime)
        )

    def get_logger(self):
        return type("Logger", (), {"error": lambda _self, _message: None})()


def _message(robot="xms_mecanum"):
    return String(
        data=json.dumps(
            {
                "event": "pedestrian_robot_contact",
                "robot": robot,
                "pedestrian": "/World/Characters/toilet_agent_01",
                "penetration_m": 0.01,
            }
        )
    )


def test_contact_fails_active_episode_and_preserves_payload():
    node = _Harness()

    ManualCollectionNode._pedestrian_contact_cb(node, _message())

    assert node.recorder.events[0][0] == "robot_human_collision"
    assert node.finished[0][0:2] == ("failed", "robot_human_collision")
    assert node.finished[0][2]["collision"]["penetration_m"] == 0.01
    assert node.finished[0][3] is True


def test_contact_ignores_other_robot_or_inactive_episode():
    node = _Harness()
    ManualCollectionNode._pedestrian_contact_cb(node, _message("other_robot"))
    node._state = "BETWEEN_EPISODES"
    ManualCollectionNode._pedestrian_contact_cb(node, _message())

    assert node.recorder.events == []
    assert node.finished == []


class _RunningHarness:
    def __init__(self):
        self._episode_started_at = 10.0
        self.config = SimpleNamespace(
            episode=SimpleNamespace(
                timeout_sec=90.0,
                goal_tolerance_m=0.3,
                goal_stop_speed_mps=0.08,
            )
        )
        self._pedestrian_process = None
        self._selection = SimpleNamespace(
            scenario=SimpleNamespace(robot_goal=(1.0, 2.0))
        )
        self._latest_odom = (1.0, 2.0, 0.0, 0.0, 0.0, 0.0)
        self.finished = []

    def _finish_episode(self, status, reason, **kwargs):
        self.finished.append((status, reason, kwargs))


def test_robot_goal_cancels_unfinished_pedestrians_instead_of_waiting_for_route():
    node = _RunningHarness()

    ManualCollectionNode._advance_running(node, 20.0)

    assert node.finished == [
        ("succeeded", "robot_reached_goal", {"abort_pedestrian_runtime": True})
    ]


class _PedestrianReadyHarness:
    _event_payload = ManualCollectionNode._event_payload
    _pedestrian_status_cb = ManualCollectionNode._pedestrian_status_cb

    def __init__(self):
        self._state = "WAIT_PEDESTRIAN"
        self._runtime_agent_ids = ("agent_01__run", "agent_02__run")
        self._active_pedestrian_generations = {}
        self._selection = SimpleNamespace(
            scenario=SimpleNamespace(source_mode="authored_route")
        )
        self.release_requests = 0

    def _active_agent_ids(self):
        return self._runtime_agent_ids

    def _request_control_release(self):
        self.release_requests += 1

    def get_logger(self):
        return type("Logger", (), {"info": lambda _self, _message: None})()


def _active_message(agent_id, *, heartbeat=False):
    return String(data=json.dumps({
        "event": "pedestrian_active",
        "agent_id": agent_id,
        "generation": 1,
        "heartbeat": heartbeat,
    }))


def test_repeated_active_heartbeat_is_idempotent_and_recovers_missing_agent():
    node = _PedestrianReadyHarness()

    node._pedestrian_status_cb(_active_message("agent_01__run"))
    node._pedestrian_status_cb(_active_message("agent_01__run", heartbeat=True))
    node._pedestrian_status_cb(_active_message("agent_02__run", heartbeat=True))

    assert node._active_pedestrian_generations == {
        "agent_01__run": 1,
        "agent_02__run": 1,
    }
    assert node.release_requests == 1
