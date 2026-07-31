"""Stable domain contracts shared by runtime and benchmark tooling."""

from .agent import AgentSnapshot, DirectedAgentState
from .events import BenchmarkEvent, decode_json_payload, encode_event_payload
from .task import MotionCommand, TaskPhase, TerminationReason

__all__ = [
    "AgentSnapshot",
    "BenchmarkEvent",
    "DirectedAgentState",
    "MotionCommand",
    "TaskPhase",
    "TerminationReason",
    "decode_json_payload",
    "encode_event_payload",
]
