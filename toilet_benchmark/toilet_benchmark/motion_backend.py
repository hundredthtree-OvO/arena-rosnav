"""Backend-neutral pedestrian motion commands and the Isaac service adapter."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from isaacsim_msgs.msg import NavPed
from isaacsim_msgs.srv import MovePed

EXTERNAL_MOTION_LOCOMOTION = 0
EXTERNAL_MOTION_FREEZE = 1
EXTERNAL_MOTION_TERMINAL_ALIGN = 2


@dataclass(frozen=True)
class MotionCommand:
    agent_id: str
    goal_pose: Sequence[float]
    path_points: Sequence[Sequence[float]]
    velocity: float
    orientation: float = 0.0
    stop: bool = False
    use_direct_pose: bool = False
    direct_pose: Sequence[float] | None = None
    use_external_motion: bool = False
    external_velocity: Sequence[float] | None = None
    external_timeout_sec: float = 0.5
    external_freeze_pose: bool = False
    external_motion_mode: int = EXTERNAL_MOTION_LOCOMOTION
    constrain_to_path: bool = False
    # Semantic phase is diagnostics-only. Motion backends must not infer FSM
    # transitions from it.
    phase: str = ""


class MotionBackend(Protocol):
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
