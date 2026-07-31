from __future__ import annotations

from .domain.agent import DirectedAgentState


class HunavAdapter:
    """Thin translation layer placeholder from director state to Hunav-friendly commands."""

    def to_debug_dict(self, state: DirectedAgentState) -> dict:
        return {
            "agent_id": state.agent_id,
            "status": state.status,
            "goal_pose": list(state.goal_pose),
            "velocity": float(state.velocity),
        }
