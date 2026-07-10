from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DirectedAgentState:
    agent_id: str
    status: str
    goal_pose: list[float]
    velocity: float


class HunavAdapter:
    """Thin translation layer placeholder from director state to Hunav-friendly commands."""

    def to_debug_dict(self, state: DirectedAgentState) -> dict:
        return {
            "agent_id": state.agent_id,
            "status": state.status,
            "goal_pose": list(state.goal_pose),
            "velocity": float(state.velocity),
        }
