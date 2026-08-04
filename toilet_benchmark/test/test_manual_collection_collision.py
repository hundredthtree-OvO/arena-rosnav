import json

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
