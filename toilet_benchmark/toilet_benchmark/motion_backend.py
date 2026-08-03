"""Backend-neutral pedestrian motion commands and the Isaac service adapter."""

from __future__ import annotations

import json
import time
from typing import Callable, Protocol, Sequence
import uuid

from isaacsim_msgs.msg import NavPed
from isaacsim_msgs.srv import MovePed
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .domain.task import (
    EXTERNAL_MOTION_FREEZE,
    EXTERNAL_MOTION_LOCOMOTION,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
    MotionCommand,
)


class MotionBackend(Protocol):
    def register_agents(self, commands: Sequence[MotionCommand]) -> None:
        """Register a batch before any member starts moving."""
        ...

    def wait_for_service(self, timeout_sec: float) -> bool:
        ...

    def send(
        self,
        command: MotionCommand,
        done_callback: Callable | None = None,
    ):
        ...

    def is_goal_settled(
        self,
        agent_id: str,
        current_pose: Sequence[float],
        target_pose: Sequence[float],
    ) -> bool:
        """Return whether the backend has consumed the current terminal goal."""
        ...

    def allows_director_stall_recovery(self, agent_id: str) -> bool:
        """Return whether the director may redispatch a stalled command."""
        ...

    def remove_agent(self, agent_id: str) -> None:
        """Remove an agent from backend-owned runtime state."""
        ...


def _flatten_path_points(points: Sequence[Sequence[float]]) -> list[float]:
    return [
        float(value)
        for point in points
        for value in (point[0], point[1], point[2])
    ]


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
        """Invalidate cached external commands before service-owned motion takes over."""
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


class IsaacPeopleBackend:
    """Translate motion commands into the existing Isaac MovePed service."""

    def __init__(self, node, service_name: str):
        self.service_name = str(service_name)
        self._client = node.create_client(MovePed, self.service_name)

    def wait_for_service(self, timeout_sec: float) -> bool:
        return bool(self._client.wait_for_service(timeout_sec=float(timeout_sec)))

    def register_agents(self, commands: Sequence[MotionCommand]) -> None:
        """Isaac owns spawned pedestrians directly; no local roster is needed."""
        return None

    def send(
        self,
        command: MotionCommand,
        done_callback: Callable | None = None,
    ):
        nav = NavPed()
        nav.path = str(command.agent_id)
        nav.goal_pose = [float(value) for value in command.goal_pose]
        nav.path_points_flat = _flatten_path_points(command.path_points)
        nav.loop_path = False
        nav.velocity = float(command.velocity)
        nav.orientation = float(command.orientation)
        nav.stop = bool(command.stop)
        nav.constrain_to_path = bool(command.constrain_to_path)
        nav.use_direct_pose = bool(command.use_direct_pose)
        if command.direct_pose is not None:
            nav.direct_pose = [float(value) for value in command.direct_pose]
        nav.use_external_motion = bool(command.use_external_motion)
        if command.external_velocity is not None:
            nav.external_velocity = [
                float(value) for value in command.external_velocity
            ]
        nav.external_timeout_sec = float(command.external_timeout_sec)
        nav.external_freeze_pose = bool(command.external_freeze_pose)
        nav.external_motion_mode = int(command.external_motion_mode)

        request = MovePed.Request()
        request.nav_list = [nav]
        future = self._client.call_async(request)
        if done_callback is not None:
            future.add_done_callback(done_callback)
        return future

    def remove_agent(self, agent_id: str) -> None:
        """Isaac owns no local per-agent motion state."""
        return None

    def is_goal_settled(
        self,
        agent_id: str,
        current_pose: Sequence[float],
        target_pose: Sequence[float],
    ) -> bool:
        """The compatibility backend has no independent completion signal."""
        return False

    def allows_director_stall_recovery(self, agent_id: str) -> bool:
        """The legacy Isaac path executor relies on director recovery."""
        return True
