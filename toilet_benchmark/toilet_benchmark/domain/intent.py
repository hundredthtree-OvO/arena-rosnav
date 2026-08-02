"""Scenario directives that describe task intent without motion details."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .task import TaskPhase


class DirectiveType(str, Enum):
    MOVE_TO_OBJECT = "move_to_object"
    WAIT_AT_QUEUE = "wait_at_queue"
    START_ACTIVITY = "start_activity"
    MOVE_TO_EXIT = "move_to_exit"
    RETIRE = "retire"


@dataclass(frozen=True)
class TaskDirective:
    """Current semantic action requested by an agent executive."""

    agent_id: str
    directive_type: DirectiveType
    phase: TaskPhase
    target_id: str | None = None
    object_id: str | None = None
    slot_id: str | None = None

    def __post_init__(self) -> None:
        if not self.agent_id:
            raise ValueError("agent_id must not be empty")

