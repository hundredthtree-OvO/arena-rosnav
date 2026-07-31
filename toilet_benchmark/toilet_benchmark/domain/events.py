"""Version-neutral event serialization used across ROS topic boundaries."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from typing import Any, Mapping

from .task import TaskPhase


_RESERVED_FIELDS = {
    "event",
    "timestamp_sec",
    "episode_id",
    "agent_id",
    "phase",
    "generation",
}


@dataclass(frozen=True)
class BenchmarkEvent:
    event_type: str
    timestamp_sec: float | None = None
    episode_id: str | None = None
    agent_id: str | None = None
    phase: str | TaskPhase | None = None
    generation: int | None = None
    payload: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.event_type:
            raise ValueError("event_type must not be empty")
        conflicts = _RESERVED_FIELDS.intersection(self.payload)
        if conflicts:
            names = ", ".join(sorted(conflicts))
            raise ValueError(f"payload contains reserved event fields: {names}")

    def to_payload(self) -> dict[str, Any]:
        value = dict(self.payload)
        value["event"] = self.event_type
        optional_fields = (
            ("timestamp_sec", self.timestamp_sec),
            ("episode_id", self.episode_id),
            ("agent_id", self.agent_id),
            ("phase", self.phase),
            ("generation", self.generation),
        )
        for name, field_value in optional_fields:
            if field_value is not None:
                value[name] = str(field_value) if isinstance(field_value, TaskPhase) else field_value
        return value

    @classmethod
    def from_payload(cls, value: Mapping[str, Any]) -> "BenchmarkEvent":
        event_type = str(value.get("event", ""))
        if not event_type:
            raise ValueError("event payload must contain a non-empty event field")
        payload = {key: item for key, item in value.items() if key not in _RESERVED_FIELDS}
        return cls(
            event_type=event_type,
            timestamp_sec=(
                float(value["timestamp_sec"]) if value.get("timestamp_sec") is not None else None
            ),
            episode_id=(
                str(value["episode_id"]) if value.get("episode_id") is not None else None
            ),
            agent_id=str(value["agent_id"]) if value.get("agent_id") is not None else None,
            phase=str(value["phase"]) if value.get("phase") is not None else None,
            generation=(
                int(value["generation"]) if value.get("generation") is not None else None
            ),
            payload=payload,
        )


def encode_event_payload(value: BenchmarkEvent | Mapping[str, Any]) -> str:
    payload = value.to_payload() if isinstance(value, BenchmarkEvent) else dict(value)
    return json.dumps(payload, sort_keys=True)


def decode_json_payload(raw: str) -> dict[str, Any]:
    try:
        value = json.loads(raw)
    except (TypeError, json.JSONDecodeError):
        return {"raw": str(raw)}
    return value if isinstance(value, dict) else {"value": value}
