"""Task lifecycle and motion command contracts."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Sequence


class StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class TaskPhase(StringEnum):
    UNSPECIFIED = ""
    ACTIVATING = "ACTIVATING"
    WALK_TO_ENTRY_CLEARANCE = "WALK_TO_ENTRY_CLEARANCE"
    WALK_TO_URINAL = "WALK_TO_URINAL"
    QUEUEING = "QUEUEING"
    FINAL_ALIGN_URINAL = "FINAL_ALIGN_URINAL"
    USING_URINAL = "USING_URINAL"
    WALK_TO_EXIT_STAGING = "WALK_TO_EXIT_STAGING"
    EXITING = "EXITING"
    DESPAWNING = "DESPAWNING"
    DONE = "DONE"


class TerminationReason(StringEnum):
    SUCCESS = "success"
    ROBOT_HUMAN_COLLISION = "robot_human_collision"
    ROBOT_SCENE_COLLISION = "robot_scene_collision"
    TIMEOUT = "timeout"
    DEADLOCK = "deadlock"
    INVALID_SIMULATION = "invalid_simulation"
    OPERATOR_ABORT = "operator_abort"
    RUNTIME_ERROR = "runtime_error"


EXTERNAL_MOTION_LOCOMOTION = 0
EXTERNAL_MOTION_FREEZE = 1
EXTERNAL_MOTION_TERMINAL_ALIGN = 2
EXTERNAL_MOTION_REPLAY_TRACK = 3


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
    phase: str | TaskPhase = ""
    behavior: str = "regular"
