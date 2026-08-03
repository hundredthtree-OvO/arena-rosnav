"""Pedestrian state contracts independent of ROS and simulator backends."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


@dataclass(frozen=True)
class AgentSnapshot:
    """A timestamped 2D pedestrian state used at subsystem boundaries."""

    agent_id: str
    x: float
    y: float
    z: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    radius_m: float = 0.3
    body_half_length_m: float = 0.0
    box_half_length_m: float = 0.0
    box_half_width_m: float = 0.0
    timestamp_sec: float = 0.0
    source: str = "unknown"

    def __post_init__(self) -> None:
        if not self.agent_id:
            raise ValueError("agent_id must not be empty")
        if not self.source:
            raise ValueError("source must not be empty")
        numeric_values = (
            self.x,
            self.y,
            self.z,
            self.yaw,
            self.vx,
            self.vy,
            self.wz,
            self.radius_m,
            self.body_half_length_m,
            self.box_half_length_m,
            self.box_half_width_m,
            self.timestamp_sec,
        )
        if not all(math.isfinite(float(value)) for value in numeric_values):
            raise ValueError("agent snapshot values must be finite")
        if min(
            self.radius_m,
            self.body_half_length_m,
            self.box_half_length_m,
            self.box_half_width_m,
        ) < 0.0:
            raise ValueError("agent geometry dimensions must be non-negative")

    def to_dict(self) -> dict[str, Any]:
        return {
            "agent_id": self.agent_id,
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "yaw": float(self.yaw),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "wz": float(self.wz),
            "radius_m": float(self.radius_m),
            "body_half_length_m": float(self.body_half_length_m),
            "box_half_length_m": float(self.box_half_length_m),
            "box_half_width_m": float(self.box_half_width_m),
            "timestamp_sec": float(self.timestamp_sec),
            "source": self.source,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "AgentSnapshot":
        return cls(
            agent_id=str(value["agent_id"]),
            x=float(value["x"]),
            y=float(value["y"]),
            z=float(value["z"]),
            yaw=float(value["yaw"]),
            vx=float(value.get("vx", 0.0)),
            vy=float(value.get("vy", 0.0)),
            wz=float(value.get("wz", 0.0)),
            radius_m=float(value.get("radius_m", 0.3)),
            body_half_length_m=float(value.get("body_half_length_m", 0.0)),
            box_half_length_m=float(value.get("box_half_length_m", 0.0)),
            box_half_width_m=float(value.get("box_half_width_m", 0.0)),
            timestamp_sec=float(value.get("timestamp_sec", 0.0)),
            source=str(value.get("source", "unknown")),
        )


@dataclass
class DirectedAgentState:
    """Current director intent consumed by HuNav adapters."""

    agent_id: str
    status: str
    goal_pose: list[float]
    velocity: float
