"""Latest-only transport for benchmark-owned pedestrian motion references."""

from __future__ import annotations

import json
import time
from typing import Sequence
import uuid

try:
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
except Exception:  # pragma: no cover - pure contract tests do not require ROS.
    class HistoryPolicy:
        KEEP_LAST = "KEEP_LAST"

    class ReliabilityPolicy:
        BEST_EFFORT = "BEST_EFFORT"

    class QoSProfile:
        def __init__(self, *, history=None, depth=1, reliability=None):
            self.history = history
            self.depth = depth
            self.reliability = reliability

try:
    from std_msgs.msg import String
except Exception:  # pragma: no cover
    class String:
        def __init__(self, data: str = ""):
            self.data = data

from .domain.task import MotionCommand


class ExternalMotionStreamPublisher:
    """Publish one latest-only batch for all high-rate pedestrian commands."""

    def __init__(
        self,
        node,
        topic_name: str = "/isaac/pedestrian_external_motion",
    ):
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._publisher = node.create_publisher(String, str(topic_name), qos)
        self._stream_id = uuid.uuid4().hex
        self._sequence = 0

    def publish(self, commands: Sequence[MotionCommand]) -> None:
        commands = tuple(commands)
        if not commands:
            return
        self._sequence += 1
        payload = {
            "version": 1,
            "stream_id": self._stream_id,
            "sequence": self._sequence,
            "sent_monotonic_sec": time.monotonic(),
            "commands": [self._serialize(command) for command in commands],
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._publisher.publish(message)

    def cancel(self, agent_ids: Sequence[str]) -> None:
        unique_agent_ids = tuple(dict.fromkeys(str(value) for value in agent_ids if value))
        if not unique_agent_ids:
            return
        self._sequence += 1
        payload = {
            "version": 1,
            "stream_id": self._stream_id,
            "sequence": self._sequence,
            "sent_monotonic_sec": time.monotonic(),
            "commands": [
                {"agent_id": agent_id, "cancel": True}
                for agent_id in unique_agent_ids
            ],
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._publisher.publish(message)

    @staticmethod
    def _serialize(command: MotionCommand) -> dict[str, object]:
        if not command.use_external_motion or command.direct_pose is None:
            raise ValueError("external-motion stream only accepts external commands")
        if command.external_velocity is None:
            raise ValueError("external-motion stream command needs a velocity")
        return {
            "agent_id": str(command.agent_id),
            "position": [float(value) for value in command.direct_pose],
            "velocity": [float(value) for value in command.external_velocity],
            "yaw": float(command.orientation),
            "timeout_sec": float(command.external_timeout_sec),
            "freeze_pose": bool(command.external_freeze_pose),
            "motion_mode": int(command.external_motion_mode),
        }
