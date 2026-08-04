"""Backend-neutral pedestrian motion commands and the Isaac service adapter."""

from __future__ import annotations

from typing import Callable, Protocol, Sequence

from isaacsim_msgs.msg import NavPed
from isaacsim_msgs.srv import MovePed
from .domain.task import (
    EXTERNAL_MOTION_FREEZE,
    EXTERNAL_MOTION_LOCOMOTION,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
    MotionCommand,
)
from .external_motion_stream import ExternalMotionStreamPublisher


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
