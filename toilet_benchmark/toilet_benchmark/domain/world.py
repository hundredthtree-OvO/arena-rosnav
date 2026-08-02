"""Immutable world feedback consumed by the scenario event runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Mapping

from .agent import AgentSnapshot


@dataclass(frozen=True)
class AgentTaskFeedback:
    """Semantic acknowledgements produced by adapters, not geometry heuristics."""

    reached_target_id: str | None = None
    activity_complete: bool = False


@dataclass(frozen=True)
class WorldSnapshot:
    timestamp_sec: float
    agents: Mapping[str, AgentSnapshot] = field(default_factory=dict)
    task_feedback: Mapping[str, AgentTaskFeedback] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.timestamp_sec)):
            raise ValueError("timestamp_sec must be finite")

    def feedback_for(self, agent_id: str) -> AgentTaskFeedback:
        return self.task_feedback.get(agent_id, AgentTaskFeedback())

