"""Normalize supported pedestrian state messages before director processing."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping


@dataclass(frozen=True)
class PedestrianObservation:
    identifier: str
    position: tuple[float, float, float]
    observed_at: float
    reliability: float
    metadata: Mapping[str, str]
    velocity: tuple[float, float, float] | None = None


def observations_from_message(msg, *, observed_at: float) -> list[PedestrianObservation]:
    people = getattr(msg, "people", None) or getattr(msg, "pedestrians", None)
    if not people:
        return []
    observations = []
    for person in people:
        reliability = getattr(person, "reliability", 1.0)
        reliability = 1.0 if reliability is None else float(reliability)
        if reliability <= 0.0:
            continue
        identifier = (
            getattr(person, "stage_prefix", None)
            or getattr(person, "name", None)
            or getattr(person, "id", None)
        )
        if identifier is None:
            continue
        pose_field = getattr(person, "pose", None) or getattr(person, "position", None)
        xyz = extract_xyz(pose_field)
        if xyz is None:
            continue
        velocity = extract_xyz(getattr(person, "velocity", None))
        names = list(getattr(person, "tagnames", []) or [])
        values = list(getattr(person, "tags", []) or [])
        metadata = {str(name): str(value) for name, value in zip(names, values)}
        observations.append(
            PedestrianObservation(
                identifier=str(identifier),
                position=(xyz[0], xyz[1], xyz[2]),
                observed_at=float(observed_at),
                reliability=reliability,
                metadata=metadata,
                velocity=velocity,
            )
        )
    return observations


def extract_xyz(field) -> tuple[float, float, float] | None:
    if field is None:
        return None
    if hasattr(field, "x") and hasattr(field, "y") and hasattr(field, "z"):
        return (float(field.x), float(field.y), float(field.z))
    inner = getattr(field, "position", None)
    if inner is not None and hasattr(inner, "x") and hasattr(inner, "y") and hasattr(inner, "z"):
        return (float(inner.x), float(inner.y), float(inner.z))
    pose = getattr(field, "pose", None)
    if pose is not None:
        inner_pose = getattr(pose, "position", None)
        if (
            inner_pose is not None
            and hasattr(inner_pose, "x")
            and hasattr(inner_pose, "y")
            and hasattr(inner_pose, "z")
        ):
            return (float(inner_pose.x), float(inner_pose.y), float(inner_pose.z))
    return None
