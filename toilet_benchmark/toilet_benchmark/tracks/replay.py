"""Frozen replay track bundle loading, validation, interpolation, and dry-run CLI.

This module is intentionally backend-neutral. It validates replay bundles, samples
one frozen pedestrian trajectory deterministically, and exposes a read-only
runtime contract that can be connected to a simulator backend later.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence

import yaml

from toilet_benchmark.domain.task import TerminationReason


REPLAY_SCHEMA_VERSION = "toilet-replay-track-0.1"
REPLAY_BENCHMARK_VERSION = "0.1.0"
_REPLAY_TRACK_NAME = "replay"
_WORLD_COORDINATE_FRAME = "world"
_TIME_BASE_ORIGIN = "episode_reset"
_TIME_BASE_CLOCK = "physics_monotonic_sec"
_TIME_BASE_UNIT = "sec"
_TIME_BASE_SAMPLE_POLICY = "strictly_increasing"
_REPLAY_REFERENCE_SOURCE = "replay_reference"
_JSON_SEPARATORS = (",", ":")
_HEX_HASH_PATTERN = re.compile(r"^[0-9a-f]{64}$")
_IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")
_ALLOWED_COLLISION_KINDS = {
    TerminationReason.ROBOT_HUMAN_COLLISION.value,
    TerminationReason.ROBOT_SCENE_COLLISION.value,
}


@dataclass(frozen=True)
class AgentSnapshot:
    """Timestamped pedestrian state owned by the Replay Track."""

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
            self.box_half_width_m,
            self.box_half_length_m,
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


class ReplayBundleValidationErrorCode:
    SCHEMA_VERSION_MISMATCH = "schema_version_mismatch"
    BENCHMARK_VERSION_INVALID = "benchmark_version_invalid"
    EPISODE_ID_INVALID = "episode_id_invalid"
    SCENE_ID_INVALID = "scene_id_invalid"
    TASK_TYPE_INVALID = "task_type_invalid"
    TRACK_INVALID = "track_invalid"
    COORDINATE_FRAME_INVALID = "coordinate_frame_invalid"
    REFERENCE_EPISODE_HASH_INVALID = "reference_episode_hash_invalid"
    REFERENCE_EPISODE_HASH_MISMATCH = "reference_episode_hash_mismatch"
    TIME_BASE_INVALID = "time_base_invalid"
    PROVENANCE_INVALID = "provenance_invalid"
    CONTENT_HASH_MISSING = "content_hash_missing"
    CONTENT_HASH_MISMATCH = "content_hash_mismatch"
    AGENT_COUNT_UNSUPPORTED = "agent_count_unsupported"
    AGENT_ID_INVALID = "agent_id_invalid"
    AGENT_ID_DUPLICATED = "agent_id_duplicated"
    AGENT_GOAL_INVALID = "agent_goal_invalid"
    AGENT_START_REFERENCE_INVALID = "agent_start_reference_invalid"
    AGENT_START_POSE_INVALID = "agent_start_pose_invalid"
    AGENT_START_POSE_MISMATCH = "agent_start_pose_mismatch"
    AGENT_SAMPLE_RATE_INVALID = "agent_sample_rate_invalid"
    TRAJECTORY_EMPTY = "trajectory_empty"
    TRAJECTORY_INDEX_INVALID = "trajectory_index_invalid"
    TRAJECTORY_TIME_INVALID = "trajectory_time_invalid"
    TRAJECTORY_TIME_NOT_MONOTONIC = "trajectory_time_not_monotonic"
    TRAJECTORY_VALUE_INVALID = "trajectory_value_invalid"
    TRAJECTORY_SOURCE_INVALID = "trajectory_source_invalid"
    TERMINATION_REASON_INVALID = "termination_reason_invalid"
    TERMINATION_TIMESTAMP_INVALID = "termination_timestamp_invalid"
    COLLISION_INVALID = "collision_invalid"


@dataclass(frozen=True)
class ReplayBundleValidationError:
    code: str
    path: tuple[str, ...]
    message: str


def _add_error(
    errors: list[ReplayBundleValidationError],
    code: str,
    path: Sequence[str],
    message: str,
) -> None:
    errors.append(ReplayBundleValidationError(code=code, path=tuple(path), message=message))


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _is_non_empty_string(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _validate_identifier(
    errors: list[ReplayBundleValidationError],
    value: Any,
    path: tuple[str, ...],
    code: str,
) -> None:
    if not _is_non_empty_string(value):
        _add_error(errors, code, path, f"{'.'.join(path)} must be a non-empty string")
        return
    if not _IDENTIFIER_PATTERN.fullmatch(value):
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must match [A-Za-z0-9._-]+",
        )


def _validate_hash(
    errors: list[ReplayBundleValidationError],
    value: Any,
    path: tuple[str, ...],
    code: str,
) -> None:
    if not _is_non_empty_string(value) or not _HEX_HASH_PATTERN.fullmatch(value):
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must be a lowercase sha256 hex digest",
        )


def _validate_numeric_tuple(
    errors: list[ReplayBundleValidationError],
    value: Any,
    expected_length: int,
    path: tuple[str, ...],
    code: str,
) -> None:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must contain {expected_length} numeric values",
        )
        return
    if len(value) != expected_length:
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must contain {expected_length} values",
        )
        return
    if not all(_is_finite(item) for item in value):
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must contain finite numeric values",
        )


def _canonical_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=_JSON_SEPARATORS)


def _split_known_mapping(
    value: Mapping[str, Any],
    known_keys: Sequence[str],
) -> tuple[dict[str, Any], dict[str, Any]]:
    known = {}
    extras = {}
    for key, item in value.items():
        if key in known_keys:
            known[key] = item
        else:
            extras[key] = item
    return known, extras


@dataclass(frozen=True)
class ReplayTimeBase:
    origin: str = ""
    clock: str = ""
    unit: str = ""
    zero_sec: float = 0.0
    monotonic: bool = True
    sample_policy: str = ""
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "origin": self.origin,
            "clock": self.clock,
            "unit": self.unit,
            "zero_sec": float(self.zero_sec),
            "monotonic": bool(self.monotonic),
            "sample_policy": self.sample_policy,
        }
        payload.update(dict(self.extras))
        return payload

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayTimeBase":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            ("origin", "clock", "unit", "zero_sec", "monotonic", "sample_policy"),
        )
        return cls(
            origin=str(known.get("origin", "")),
            clock=str(known.get("clock", "")),
            unit=str(known.get("unit", "")),
            zero_sec=float(known.get("zero_sec", 0.0)),
            monotonic=bool(known.get("monotonic", True)),
            sample_policy=str(known.get("sample_policy", "")),
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayTrajectorySample:
    sample_index: int = 0
    timestamp_sec: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    radius_m: float = 0.3
    source: str = _REPLAY_REFERENCE_SOURCE
    confidence: float | None = None
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "sample_index": int(self.sample_index),
            "timestamp_sec": float(self.timestamp_sec),
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "yaw": float(self.yaw),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "wz": float(self.wz),
            "radius_m": float(self.radius_m),
            "source": self.source,
        }
        if self.confidence is not None:
            payload["confidence"] = float(self.confidence)
        payload.update(dict(self.extras))
        return payload

    def to_snapshot(self, *, agent_id: str) -> AgentSnapshot:
        return AgentSnapshot(
            agent_id=agent_id,
            x=self.x,
            y=self.y,
            z=self.z,
            yaw=self.yaw,
            vx=self.vx,
            vy=self.vy,
            wz=self.wz,
            radius_m=self.radius_m,
            timestamp_sec=self.timestamp_sec,
            source=self.source,
        )

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayTrajectorySample":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            (
                "sample_index",
                "timestamp_sec",
                "x",
                "y",
                "z",
                "yaw",
                "vx",
                "vy",
                "wz",
                "radius_m",
                "source",
                "confidence",
            ),
        )
        confidence = known.get("confidence")
        return cls(
            sample_index=int(known.get("sample_index", 0)),
            timestamp_sec=float(known.get("timestamp_sec", 0.0)),
            x=float(known.get("x", 0.0)),
            y=float(known.get("y", 0.0)),
            z=float(known.get("z", 0.0)),
            yaw=float(known.get("yaw", 0.0)),
            vx=float(known.get("vx", 0.0)),
            vy=float(known.get("vy", 0.0)),
            wz=float(known.get("wz", 0.0)),
            radius_m=float(known.get("radius_m", 0.3)),
            source=str(known.get("source", _REPLAY_REFERENCE_SOURCE)),
            confidence=float(confidence) if confidence is not None else None,
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayTrackAgent:
    agent_id: str = ""
    semantic_goal: str = ""
    character: str | None = None
    start_reference: str | None = None
    start_pose: tuple[float, float, float, float] | None = None
    sample_rate_hz: float | None = None
    trajectory: tuple[ReplayTrajectorySample, ...] = field(default_factory=tuple)
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "agent_id": self.agent_id,
            "semantic_goal": self.semantic_goal,
            "trajectory": [sample.to_dict() for sample in self.trajectory],
        }
        if self.character is not None:
            payload["character"] = self.character
        if self.start_reference is not None:
            payload["start_reference"] = self.start_reference
        if self.start_pose is not None:
            payload["start_pose"] = [float(value) for value in self.start_pose]
        if self.sample_rate_hz is not None:
            payload["sample_rate_hz"] = float(self.sample_rate_hz)
        payload.update(dict(self.extras))
        return payload

    @property
    def terminal_timestamp_sec(self) -> float:
        return self.trajectory[-1].timestamp_sec

    @property
    def start_snapshot(self) -> AgentSnapshot:
        return self.trajectory[0].to_snapshot(agent_id=self.agent_id)

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayTrackAgent":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            (
                "agent_id",
                "semantic_goal",
                "character",
                "start_reference",
                "start_pose",
                "sample_rate_hz",
                "trajectory",
            ),
        )
        start_pose = known.get("start_pose")
        trajectory_raw = known.get("trajectory", ())
        trajectory_items = []
        if isinstance(trajectory_raw, Sequence) and not isinstance(
            trajectory_raw, (str, bytes, bytearray)
        ):
            trajectory_items = [
                ReplayTrajectorySample.from_mapping(item if isinstance(item, Mapping) else {})
                for item in trajectory_raw
            ]
        return cls(
            agent_id=str(known.get("agent_id", "")),
            semantic_goal=str(known.get("semantic_goal", "")),
            character=(
                str(known["character"]) if known.get("character") is not None else None
            ),
            start_reference=(
                str(known["start_reference"])
                if known.get("start_reference") is not None
                else None
            ),
            start_pose=(
                tuple(start_pose)
                if isinstance(start_pose, Sequence)
                and not isinstance(start_pose, (str, bytes, bytearray))
                else None
            ),
            sample_rate_hz=(
                float(known["sample_rate_hz"])
                if known.get("sample_rate_hz") is not None
                else None
            ),
            trajectory=tuple(trajectory_items),
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayCollisionAnnotation:
    kind: str = ""
    timestamp_sec: float = 0.0
    actor_ids: tuple[str, ...] = field(default_factory=tuple)
    contact_frame: str | None = None
    geometry_source: str | None = None
    action: str | None = None
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "kind": self.kind,
            "timestamp_sec": float(self.timestamp_sec),
            "actor_ids": list(self.actor_ids),
        }
        if self.contact_frame is not None:
            payload["contact_frame"] = self.contact_frame
        if self.geometry_source is not None:
            payload["geometry_source"] = self.geometry_source
        if self.action is not None:
            payload["action"] = self.action
        payload.update(dict(self.extras))
        return payload

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayCollisionAnnotation":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            ("kind", "timestamp_sec", "actor_ids", "contact_frame", "geometry_source", "action"),
        )
        actor_ids_raw = known.get("actor_ids", ())
        actor_ids = ()
        if isinstance(actor_ids_raw, Sequence) and not isinstance(
            actor_ids_raw, (str, bytes, bytearray)
        ):
            actor_ids = tuple(str(item) for item in actor_ids_raw)
        return cls(
            kind=str(known.get("kind", "")),
            timestamp_sec=float(known.get("timestamp_sec", 0.0)),
            actor_ids=actor_ids,
            contact_frame=(
                str(known["contact_frame"])
                if known.get("contact_frame") is not None
                else None
            ),
            geometry_source=(
                str(known["geometry_source"])
                if known.get("geometry_source") is not None
                else None
            ),
            action=str(known["action"]) if known.get("action") is not None else None,
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayAnnotations:
    termination_reason: str | None = None
    termination_timestamp_sec: float | None = None
    collisions: tuple[ReplayCollisionAnnotation, ...] = field(default_factory=tuple)
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        if self.termination_reason is not None:
            payload["termination_reason"] = self.termination_reason
        if self.termination_timestamp_sec is not None:
            payload["termination_timestamp_sec"] = float(self.termination_timestamp_sec)
        if self.collisions:
            payload["collisions"] = [collision.to_dict() for collision in self.collisions]
        payload.update(dict(self.extras))
        return payload

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayAnnotations":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            ("termination_reason", "termination_timestamp_sec", "collisions"),
        )
        collisions_raw = known.get("collisions", ())
        collisions = ()
        if isinstance(collisions_raw, Sequence) and not isinstance(
            collisions_raw, (str, bytes, bytearray)
        ):
            collisions = tuple(
                ReplayCollisionAnnotation.from_mapping(item if isinstance(item, Mapping) else {})
                for item in collisions_raw
            )
        return cls(
            termination_reason=(
                str(known["termination_reason"])
                if known.get("termination_reason") is not None
                else None
            ),
            termination_timestamp_sec=(
                float(known["termination_timestamp_sec"])
                if known.get("termination_timestamp_sec") is not None
                else None
            ),
            collisions=collisions,
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayProvenance:
    source_episode_manifest: str = ""
    source_episode_hash: str = ""
    source_capture: str | None = None
    generated_by: str = ""
    generated_at_sec: float = 0.0
    content_hash: str = ""
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self, *, include_content_hash: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "source_episode_manifest": self.source_episode_manifest,
            "source_episode_hash": self.source_episode_hash,
            "generated_by": self.generated_by,
            "generated_at_sec": float(self.generated_at_sec),
        }
        if self.source_capture is not None:
            payload["source_capture"] = self.source_capture
        if include_content_hash:
            payload["content_hash"] = self.content_hash
        payload.update(dict(self.extras))
        return payload

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayProvenance":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            (
                "source_episode_manifest",
                "source_episode_hash",
                "source_capture",
                "generated_by",
                "generated_at_sec",
                "content_hash",
            ),
        )
        return cls(
            source_episode_manifest=str(known.get("source_episode_manifest", "")),
            source_episode_hash=str(known.get("source_episode_hash", "")),
            source_capture=(
                str(known["source_capture"])
                if known.get("source_capture") is not None
                else None
            ),
            generated_by=str(known.get("generated_by", "")),
            generated_at_sec=float(known.get("generated_at_sec", 0.0)),
            content_hash=str(known.get("content_hash", "")),
            extras=extras,
        )


@dataclass(frozen=True)
class ReplayBundle:
    schema_version: str = ""
    benchmark_version: str = REPLAY_BENCHMARK_VERSION
    episode_id: str = ""
    scene_id: str = ""
    task_type: str = ""
    track: str = _REPLAY_TRACK_NAME
    seed: int = 0
    reference_episode_hash: str = ""
    coordinate_frame: str = _WORLD_COORDINATE_FRAME
    time_base: ReplayTimeBase = field(default_factory=ReplayTimeBase)
    agents: tuple[ReplayTrackAgent, ...] = field(default_factory=tuple)
    annotations: ReplayAnnotations = field(default_factory=ReplayAnnotations)
    provenance: ReplayProvenance = field(default_factory=ReplayProvenance)
    extras: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self, *, include_content_hash: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.schema_version,
            "benchmark_version": self.benchmark_version,
            "episode_id": self.episode_id,
            "scene_id": self.scene_id,
            "task_type": self.task_type,
            "track": self.track,
            "seed": int(self.seed),
            "reference_episode_hash": self.reference_episode_hash,
            "coordinate_frame": self.coordinate_frame,
            "time_base": self.time_base.to_dict(),
            "agents": [agent.to_dict() for agent in self.agents],
            "annotations": self.annotations.to_dict(),
            "provenance": self.provenance.to_dict(include_content_hash=include_content_hash),
        }
        payload.update(dict(self.extras))
        return payload

    def canonical_payload(self) -> str:
        return _canonical_json(self.to_dict(include_content_hash=False))

    def primary_agent(self) -> ReplayTrackAgent:
        if len(self.agents) != 1:
            raise ValueError("replay foundation currently supports exactly one agent")
        return self.agents[0]

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any] | None) -> "ReplayBundle":
        payload = dict(value or {})
        known, extras = _split_known_mapping(
            payload,
            (
                "schema_version",
                "benchmark_version",
                "episode_id",
                "scene_id",
                "task_type",
                "track",
                "seed",
                "reference_episode_hash",
                "coordinate_frame",
                "time_base",
                "agents",
                "annotations",
                "provenance",
            ),
        )
        agents_raw = known.get("agents", ())
        agents = ()
        if isinstance(agents_raw, Sequence) and not isinstance(
            agents_raw, (str, bytes, bytearray)
        ):
            agents = tuple(
                ReplayTrackAgent.from_mapping(item if isinstance(item, Mapping) else {})
                for item in agents_raw
            )
        return cls(
            schema_version=str(known.get("schema_version", "")),
            benchmark_version=str(known.get("benchmark_version", REPLAY_BENCHMARK_VERSION)),
            episode_id=str(known.get("episode_id", "")),
            scene_id=str(known.get("scene_id", "")),
            task_type=str(known.get("task_type", "")),
            track=str(known.get("track", _REPLAY_TRACK_NAME)),
            seed=int(known.get("seed", 0)),
            reference_episode_hash=str(known.get("reference_episode_hash", "")),
            coordinate_frame=str(known.get("coordinate_frame", _WORLD_COORDINATE_FRAME)),
            time_base=ReplayTimeBase.from_mapping(known.get("time_base")),
            agents=agents,
            annotations=ReplayAnnotations.from_mapping(known.get("annotations")),
            provenance=ReplayProvenance.from_mapping(known.get("provenance")),
            extras=extras,
        )


def compute_replay_bundle_hash(bundle: ReplayBundle) -> str:
    return hashlib.sha256(bundle.canonical_payload().encode("utf-8")).hexdigest()


def seal_replay_bundle(bundle: ReplayBundle) -> ReplayBundle:
    """Return a canonical bundle carrying its own deterministic content hash."""

    unsigned = replace(
        bundle,
        provenance=replace(bundle.provenance, content_hash=""),
    )
    return replace(
        unsigned,
        provenance=replace(
            unsigned.provenance,
            content_hash=compute_replay_bundle_hash(unsigned),
        ),
    )


def _state_frame_payload(frame: "ReplayStateFrame") -> dict[str, Any]:
    return {
        "sample_index": frame.sample_index,
        "timestamp_sec": frame.timestamp_sec,
        "finished": frame.finished,
        "snapshot": frame.snapshot.to_dict(),
    }


def compute_replay_state_sequence_hash(frames: Sequence["ReplayStateFrame"]) -> str:
    payload = [_state_frame_payload(frame) for frame in frames]
    return hashlib.sha256(
        _canonical_json({"frames": payload}).encode("utf-8")
    ).hexdigest()


@dataclass(frozen=True)
class ReplayStateFrame:
    sample_index: int
    timestamp_sec: float
    snapshot: AgentSnapshot
    finished: bool

    def to_dict(self) -> dict[str, Any]:
        return _state_frame_payload(self)


@dataclass(frozen=True)
class ReplayDryRunResult:
    bundle_hash: str
    state_sequence_hash: str
    frames: tuple[ReplayStateFrame, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "bundle_hash": self.bundle_hash,
            "state_sequence_hash": self.state_sequence_hash,
            "frames": [frame.to_dict() for frame in self.frames],
        }


class ReplayActorSink(Protocol):
    def publish(self, frame: ReplayStateFrame) -> None:
        """Consume a sampled replay frame without mutating bundle state."""


def load_replay_bundle(path: str | Path) -> ReplayBundle:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise TypeError("replay bundle payload must be a mapping")
    return ReplayBundle.from_mapping(payload)


def _validate_time_base(
    errors: list[ReplayBundleValidationError],
    time_base: ReplayTimeBase,
) -> None:
    if (
        time_base.origin != _TIME_BASE_ORIGIN
        or time_base.clock != _TIME_BASE_CLOCK
        or time_base.unit != _TIME_BASE_UNIT
        or time_base.sample_policy != _TIME_BASE_SAMPLE_POLICY
        or not time_base.monotonic
        or not _is_finite(time_base.zero_sec)
        or float(time_base.zero_sec) != 0.0
    ):
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TIME_BASE_INVALID,
            ("time_base",),
            "time_base must use the episode_reset/physics_monotonic_sec/sec/strictly_increasing contract",
        )


def _validate_annotations(
    errors: list[ReplayBundleValidationError],
    annotations: ReplayAnnotations,
) -> None:
    if annotations.termination_reason is not None:
        if annotations.termination_reason not in {item.value for item in TerminationReason}:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.TERMINATION_REASON_INVALID,
                ("annotations", "termination_reason"),
                f"unsupported termination_reason: {annotations.termination_reason}",
            )
    if annotations.termination_timestamp_sec is not None and (
        not _is_finite(annotations.termination_timestamp_sec)
        or float(annotations.termination_timestamp_sec) < 0.0
    ):
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TERMINATION_TIMESTAMP_INVALID,
            ("annotations", "termination_timestamp_sec"),
            "termination_timestamp_sec must be a non-negative finite number",
        )
    for index, collision in enumerate(annotations.collisions):
        if collision.kind not in _ALLOWED_COLLISION_KINDS:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.COLLISION_INVALID,
                ("annotations", "collisions", str(index), "kind"),
                f"unsupported collision kind: {collision.kind}",
            )
        if not _is_finite(collision.timestamp_sec) or float(collision.timestamp_sec) < 0.0:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.COLLISION_INVALID,
                ("annotations", "collisions", str(index), "timestamp_sec"),
                "collision timestamp_sec must be a non-negative finite number",
            )
        if len(collision.actor_ids) < 2 or not all(_is_non_empty_string(value) for value in collision.actor_ids):
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.COLLISION_INVALID,
                ("annotations", "collisions", str(index), "actor_ids"),
                "collision actor_ids must contain at least two non-empty ids",
            )
        if collision.action is not None and collision.action not in {"terminate", "contain"}:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.COLLISION_INVALID,
                ("annotations", "collisions", str(index), "action"),
                f"unsupported collision action: {collision.action}",
            )


def _validate_sample(
    errors: list[ReplayBundleValidationError],
    sample: ReplayTrajectorySample,
    *,
    agent_index: int,
    sample_index: int,
    previous_timestamp_sec: float | None,
) -> None:
    path = ("agents", str(agent_index), "trajectory", str(sample_index))
    if sample.sample_index != sample_index:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRAJECTORY_INDEX_INVALID,
            path + ("sample_index",),
            f"sample_index must equal {sample_index}",
        )
    if not _is_finite(sample.timestamp_sec):
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRAJECTORY_TIME_INVALID,
            path + ("timestamp_sec",),
            "timestamp_sec must be finite",
        )
    elif previous_timestamp_sec is not None and float(sample.timestamp_sec) <= previous_timestamp_sec:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRAJECTORY_TIME_NOT_MONOTONIC,
            path + ("timestamp_sec",),
            "trajectory timestamps must be strictly increasing",
        )
    for field_name, value in (
        ("x", sample.x),
        ("y", sample.y),
        ("z", sample.z),
        ("yaw", sample.yaw),
        ("vx", sample.vx),
        ("vy", sample.vy),
        ("wz", sample.wz),
        ("radius_m", sample.radius_m),
    ):
        if not _is_finite(value):
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.TRAJECTORY_VALUE_INVALID,
                path + (field_name,),
                f"{field_name} must be finite",
            )
    if sample.radius_m < 0.0:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRAJECTORY_VALUE_INVALID,
            path + ("radius_m",),
            "radius_m must be non-negative",
        )
    if sample.source != _REPLAY_REFERENCE_SOURCE:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRAJECTORY_SOURCE_INVALID,
            path + ("source",),
            f"source must be '{_REPLAY_REFERENCE_SOURCE}'",
        )


def validate_replay_bundle(
    bundle: ReplayBundle,
    *,
    expected_schema_version: str = REPLAY_SCHEMA_VERSION,
    expected_benchmark_version: str = REPLAY_BENCHMARK_VERSION,
) -> tuple[ReplayBundleValidationError, ...]:
    errors: list[ReplayBundleValidationError] = []

    if bundle.schema_version != expected_schema_version:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.SCHEMA_VERSION_MISMATCH,
            ("schema_version",),
            f"schema_version must be '{expected_schema_version}'",
        )
    if bundle.benchmark_version != expected_benchmark_version:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.BENCHMARK_VERSION_INVALID,
            ("benchmark_version",),
            f"benchmark_version must be '{expected_benchmark_version}'",
        )

    _validate_identifier(
        errors,
        bundle.episode_id,
        ("episode_id",),
        ReplayBundleValidationErrorCode.EPISODE_ID_INVALID,
    )
    _validate_identifier(
        errors,
        bundle.scene_id,
        ("scene_id",),
        ReplayBundleValidationErrorCode.SCENE_ID_INVALID,
    )
    _validate_identifier(
        errors,
        bundle.task_type,
        ("task_type",),
        ReplayBundleValidationErrorCode.TASK_TYPE_INVALID,
    )
    if bundle.track != _REPLAY_TRACK_NAME:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.TRACK_INVALID,
            ("track",),
            "track must be 'replay'",
        )
    if bundle.coordinate_frame != _WORLD_COORDINATE_FRAME:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.COORDINATE_FRAME_INVALID,
            ("coordinate_frame",),
            "coordinate_frame must be 'world'",
        )
    _validate_hash(
        errors,
        bundle.reference_episode_hash,
        ("reference_episode_hash",),
        ReplayBundleValidationErrorCode.REFERENCE_EPISODE_HASH_INVALID,
    )
    _validate_time_base(errors, bundle.time_base)
    _validate_annotations(errors, bundle.annotations)

    provenance = bundle.provenance
    if (
        not _is_non_empty_string(provenance.source_episode_manifest)
        or not _is_non_empty_string(provenance.generated_by)
        or not _is_finite(provenance.generated_at_sec)
    ):
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.PROVENANCE_INVALID,
            ("provenance",),
            "provenance must define source_episode_manifest, generated_by, and generated_at_sec",
        )
    _validate_hash(
        errors,
        provenance.source_episode_hash,
        ("provenance", "source_episode_hash"),
        ReplayBundleValidationErrorCode.PROVENANCE_INVALID,
    )
    if provenance.content_hash and not _HEX_HASH_PATTERN.fullmatch(provenance.content_hash):
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.CONTENT_HASH_MISSING,
            ("provenance", "content_hash"),
            "content_hash must be a lowercase sha256 hex digest",
        )

    if provenance.source_episode_hash and bundle.reference_episode_hash:
        if provenance.source_episode_hash != bundle.reference_episode_hash:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.REFERENCE_EPISODE_HASH_MISMATCH,
                ("reference_episode_hash",),
                "reference_episode_hash must match provenance.source_episode_hash",
            )

    if len(bundle.agents) != 1:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.AGENT_COUNT_UNSUPPORTED,
            ("agents",),
            "replay foundation currently supports exactly one agent",
        )

    seen_agent_ids: set[str] = set()
    for agent_index, agent in enumerate(bundle.agents):
        path = ("agents", str(agent_index))
        _validate_identifier(
            errors,
            agent.agent_id,
            path + ("agent_id",),
            ReplayBundleValidationErrorCode.AGENT_ID_INVALID,
        )
        if agent.agent_id in seen_agent_ids:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.AGENT_ID_DUPLICATED,
                path + ("agent_id",),
                f"agent_id duplicated: {agent.agent_id}",
            )
        seen_agent_ids.add(agent.agent_id)
        if not _is_non_empty_string(agent.semantic_goal):
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.AGENT_GOAL_INVALID,
                path + ("semantic_goal",),
                "semantic_goal must be a non-empty string",
            )
        if agent.start_reference is not None and not _is_non_empty_string(agent.start_reference):
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.AGENT_START_REFERENCE_INVALID,
                path + ("start_reference",),
                "start_reference must be a non-empty string when present",
            )
        if agent.sample_rate_hz is not None and (
            not _is_finite(agent.sample_rate_hz) or float(agent.sample_rate_hz) <= 0.0
        ):
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.AGENT_SAMPLE_RATE_INVALID,
                path + ("sample_rate_hz",),
                "sample_rate_hz must be a positive finite number",
            )
        if agent.start_pose is not None:
            _validate_numeric_tuple(
                errors,
                agent.start_pose,
                4,
                path + ("start_pose",),
                ReplayBundleValidationErrorCode.AGENT_START_POSE_INVALID,
            )

        if not agent.trajectory:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.TRAJECTORY_EMPTY,
                path + ("trajectory",),
                "trajectory must contain at least one sample",
            )
            continue

        if agent.trajectory[0].timestamp_sec != 0.0:
            _add_error(
                errors,
                ReplayBundleValidationErrorCode.TRAJECTORY_TIME_INVALID,
                path + ("trajectory", "0", "timestamp_sec"),
                "first sample timestamp_sec must be 0.0",
            )
        if (
            agent.start_pose is not None
            and len(agent.start_pose) == 4
            and all(_is_finite(value) for value in agent.start_pose)
        ):
            start = agent.trajectory[0]
            expected = tuple(float(value) for value in agent.start_pose)
            actual = (start.x, start.y, start.z, start.yaw)
            if any(not math.isclose(exp, act, rel_tol=0.0, abs_tol=1e-9) for exp, act in zip(expected, actual)):
                _add_error(
                    errors,
                    ReplayBundleValidationErrorCode.AGENT_START_POSE_MISMATCH,
                    path + ("start_pose",),
                    "start_pose must match the first trajectory sample",
                )

        previous_timestamp: float | None = None
        for sample_index, sample in enumerate(agent.trajectory):
            _validate_sample(
                errors,
                sample,
                agent_index=agent_index,
                sample_index=sample_index,
                previous_timestamp_sec=previous_timestamp,
            )
            previous_timestamp = float(sample.timestamp_sec)

    computed_hash = compute_replay_bundle_hash(bundle)
    if not provenance.content_hash:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.CONTENT_HASH_MISSING,
            ("provenance", "content_hash"),
            "content_hash is required",
        )
    elif provenance.content_hash != computed_hash:
        _add_error(
            errors,
            ReplayBundleValidationErrorCode.CONTENT_HASH_MISMATCH,
            ("provenance", "content_hash"),
            "content_hash does not match the canonical bundle payload",
        )

    return tuple(sorted(errors, key=lambda item: (item.path, item.code, item.message)))


def require_valid_replay_bundle(
    bundle: ReplayBundle,
    *,
    expected_schema_version: str = REPLAY_SCHEMA_VERSION,
    expected_benchmark_version: str = REPLAY_BENCHMARK_VERSION,
) -> None:
    errors = validate_replay_bundle(
        bundle,
        expected_schema_version=expected_schema_version,
        expected_benchmark_version=expected_benchmark_version,
    )
    if errors:
        messages = "; ".join(f"{error.code}@{'.'.join(error.path)}: {error.message}" for error in errors)
        raise ValueError(f"invalid replay bundle: {messages}")


def _yaw_interpolate(start: float, end: float, ratio: float) -> float:
    delta = math.atan2(math.sin(end - start), math.cos(end - start))
    return math.atan2(math.sin(start + delta * ratio), math.cos(start + delta * ratio))


def _linear_interpolate(start: float, end: float, ratio: float) -> float:
    return start + ((end - start) * ratio)


def _interpolate_snapshot(
    start: ReplayTrajectorySample,
    end: ReplayTrajectorySample,
    timestamp_sec: float,
    *,
    agent_id: str,
) -> AgentSnapshot:
    if math.isclose(start.timestamp_sec, end.timestamp_sec, rel_tol=0.0, abs_tol=1e-12):
        return start.to_snapshot(agent_id=agent_id)
    ratio = (timestamp_sec - start.timestamp_sec) / (end.timestamp_sec - start.timestamp_sec)
    ratio = min(1.0, max(0.0, ratio))
    return AgentSnapshot(
        agent_id=agent_id,
        x=_linear_interpolate(start.x, end.x, ratio),
        y=_linear_interpolate(start.y, end.y, ratio),
        z=_linear_interpolate(start.z, end.z, ratio),
        yaw=_yaw_interpolate(start.yaw, end.yaw, ratio),
        vx=_linear_interpolate(start.vx, end.vx, ratio),
        vy=_linear_interpolate(start.vy, end.vy, ratio),
        wz=_linear_interpolate(start.wz, end.wz, ratio),
        radius_m=_linear_interpolate(start.radius_m, end.radius_m, ratio),
        timestamp_sec=timestamp_sec,
        source=_REPLAY_REFERENCE_SOURCE,
    )


class ReplayRuntime:
    """Read-only runtime for a frozen replay trajectory.

    The runtime samples one frozen pedestrian track deterministically. A backend
    adapter can be attached later by consuming :class:`ReplayStateFrame` values
    through a backend-neutral sink.
    """

    def __init__(self, bundle: ReplayBundle):
        require_valid_replay_bundle(bundle)
        self._bundle = bundle
        self._agent = bundle.primary_agent()

    @property
    def bundle(self) -> ReplayBundle:
        return self._bundle

    @property
    def agent(self) -> ReplayTrackAgent:
        return self._agent

    @property
    def terminal_timestamp_sec(self) -> float:
        return self._agent.terminal_timestamp_sec

    def is_finished(self, timestamp_sec: float) -> bool:
        return float(timestamp_sec) >= self.terminal_timestamp_sec

    def sample(self, timestamp_sec: float) -> AgentSnapshot:
        timestamp = float(timestamp_sec)
        trajectory = self._agent.trajectory
        if timestamp <= trajectory[0].timestamp_sec:
            return trajectory[0].to_snapshot(agent_id=self._agent.agent_id)
        if timestamp >= trajectory[-1].timestamp_sec:
            return trajectory[-1].to_snapshot(agent_id=self._agent.agent_id)
        for left, right in zip(trajectory, trajectory[1:]):
            if left.timestamp_sec <= timestamp <= right.timestamp_sec:
                return _interpolate_snapshot(
                    left,
                    right,
                    timestamp,
                    agent_id=self._agent.agent_id,
                )
        return trajectory[-1].to_snapshot(agent_id=self._agent.agent_id)

    def _sample_times(self, step_sec: float | None) -> tuple[float, ...]:
        if step_sec is None:
            return tuple(sample.timestamp_sec for sample in self._agent.trajectory)
        if not _is_finite(step_sec) or float(step_sec) <= 0.0:
            raise ValueError("step_sec must be a positive finite number")
        step = float(step_sec)
        end = self.terminal_timestamp_sec
        times = [0.0]
        current_index = 1
        while True:
            current = current_index * step
            if current >= end:
                break
            times.append(current)
            current_index += 1
        if not math.isclose(times[-1], end, rel_tol=0.0, abs_tol=1e-12):
            times.append(end)
        return tuple(times)

    def dry_run(self, *, step_sec: float | None = None) -> ReplayDryRunResult:
        frames = []
        for sample_index, timestamp_sec in enumerate(self._sample_times(step_sec)):
            snapshot = self.sample(timestamp_sec)
            frames.append(
                ReplayStateFrame(
                    sample_index=sample_index,
                    timestamp_sec=timestamp_sec,
                    snapshot=snapshot,
                    finished=self.is_finished(timestamp_sec),
                )
            )
        result_frames = tuple(frames)
        return ReplayDryRunResult(
            bundle_hash=compute_replay_bundle_hash(self._bundle),
            state_sequence_hash=compute_replay_state_sequence_hash(result_frames),
            frames=result_frames,
        )

    def stream(
        self,
        *,
        step_sec: float | None = None,
        sink: ReplayActorSink | None = None,
    ) -> tuple[ReplayStateFrame, ...]:
        frames = []
        for frame in self.dry_run(step_sec=step_sec).frames:
            if sink is not None:
                sink.publish(frame)
            frames.append(frame)
        return tuple(frames)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_replay", description=__doc__)
    parser.add_argument("bundle", help="replay bundle JSON or YAML path")
    parser.add_argument(
        "--step-sec",
        type=float,
        default=None,
        help="optional dry-run sampling step in seconds",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="validate the replay bundle without sampling it",
    )
    parser.add_argument(
        "--seal-output",
        type=Path,
        default=None,
        help="validate authoring fields, add content_hash, and write a canonical bundle",
    )
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _build_parser().parse_args(args)
    try:
        bundle = load_replay_bundle(namespace.bundle)
        if namespace.seal_output is not None:
            ignored_codes = {
                ReplayBundleValidationErrorCode.CONTENT_HASH_MISSING,
                ReplayBundleValidationErrorCode.CONTENT_HASH_MISMATCH,
            }
            errors = tuple(
                error
                for error in validate_replay_bundle(bundle)
                if error.code not in ignored_codes
            )
            if errors:
                messages = "; ".join(
                    f"{error.code}@{'.'.join(error.path)}: {error.message}"
                    for error in errors
                )
                raise ValueError(f"invalid replay bundle: {messages}")
            sealed = seal_replay_bundle(bundle)
            output_path = namespace.seal_output.expanduser().resolve()
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                yaml.safe_dump(
                    sealed.to_dict(),
                    sort_keys=False,
                    allow_unicode=True,
                ),
                encoding="utf-8",
            )
            print(
                json.dumps(
                    {
                        "bundle_hash": sealed.provenance.content_hash,
                        "output": str(output_path),
                        "valid": True,
                    },
                    sort_keys=True,
                    ensure_ascii=False,
                )
            )
            return 0
        require_valid_replay_bundle(bundle)
        if namespace.validate_only:
            print(
                json.dumps(
                    {
                        "bundle_hash": compute_replay_bundle_hash(bundle),
                        "valid": True,
                        "episode_id": bundle.episode_id,
                        "agent_id": bundle.primary_agent().agent_id,
                    },
                    sort_keys=True,
                    ensure_ascii=False,
                )
            )
            return 0
        runtime = ReplayRuntime(bundle)
        result = runtime.dry_run(step_sec=namespace.step_sec)
        print(json.dumps(result.to_dict(), sort_keys=True, ensure_ascii=False))
        return 0
    except (OSError, TypeError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
