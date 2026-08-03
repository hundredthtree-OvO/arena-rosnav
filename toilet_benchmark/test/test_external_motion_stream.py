import json

from toilet_benchmark.domain.task import (
    EXTERNAL_MOTION_LOCOMOTION,
    MotionCommand,
)
from toilet_benchmark.motion_backend import ExternalMotionStreamPublisher


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Node:
    def __init__(self):
        self.publisher = _Publisher()

    def create_publisher(self, _message_type, topic, qos_profile):
        assert topic == "/isaac/pedestrian_external_motion"
        assert qos_profile.depth == 1
        return self.publisher


def _command(agent_id, x):
    return MotionCommand(
        agent_id=agent_id,
        goal_pose=(2.0, 0.0, 0.0),
        path_points=(),
        direct_pose=(x, 0.0, 0.0),
        velocity=0.3,
        orientation=0.0,
        use_external_motion=True,
        external_velocity=(0.3, 0.0, 0.0),
        external_timeout_sec=1.0,
        external_motion_mode=EXTERNAL_MOTION_LOCOMOTION,
    )


def test_external_motion_stream_publishes_one_latest_only_batch_per_tick():
    node = _Node()
    stream = ExternalMotionStreamPublisher(node)

    stream.publish((_command("agent_01", 0.1), _command("agent_02", 0.2)))

    assert len(node.publisher.messages) == 1
    payload = json.loads(node.publisher.messages[0].data)
    assert payload["version"] == 1
    assert payload["sequence"] == 1
    assert [item["agent_id"] for item in payload["commands"]] == [
        "agent_01",
        "agent_02",
    ]
    assert payload["commands"][1]["position"] == [0.2, 0.0, 0.0]


def test_external_motion_stream_sequence_increases_between_batches():
    node = _Node()
    stream = ExternalMotionStreamPublisher(node)

    stream.publish((_command("agent_01", 0.1),))
    stream.publish((_command("agent_01", 0.2),))

    sequences = [json.loads(message.data)["sequence"] for message in node.publisher.messages]
    assert sequences == [1, 2]


def test_external_motion_stream_cancel_invalidates_agents_in_next_sequence():
    node = _Node()
    stream = ExternalMotionStreamPublisher(node)

    stream.publish((_command("agent_01", 0.1),))
    stream.cancel(("agent_01", "agent_01", "agent_02"))

    payload = json.loads(node.publisher.messages[-1].data)
    assert payload["sequence"] == 2
    assert payload["commands"] == [
        {"agent_id": "agent_01", "cancel": True},
        {"agent_id": "agent_02", "cancel": True},
    ]
