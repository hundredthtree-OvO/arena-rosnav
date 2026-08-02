"""Stable domain contracts shared by runtime and benchmark tooling."""

from .agent import AgentSnapshot, DirectedAgentState
from .events import BenchmarkEvent, decode_json_payload, encode_event_payload
from .intent import DirectiveType, TaskDirective
from .task import MotionCommand, TaskPhase, TerminationReason
from .world import AgentTaskFeedback, WorldSnapshot

__all__ = [
    "AgentSnapshot",
    "AgentTaskFeedback",
    "BenchmarkEvent",
    "DirectiveType",
    "DirectedAgentState",
    "MotionCommand",
    "TaskPhase",
    "TaskDirective",
    "TerminationReason",
    "WorldSnapshot",
    "decode_json_payload",
    "encode_event_payload",
]
