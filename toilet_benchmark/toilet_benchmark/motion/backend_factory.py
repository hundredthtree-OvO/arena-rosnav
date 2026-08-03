"""Construct the selected pedestrian motion backend behind one boundary."""

from __future__ import annotations

from typing import Mapping


def build_motion_backend(
    node,
    *,
    backend_name: str,
    move_service_name: str,
    hunav_namespace: str,
    hunav_config: dict,
    motion_config: Mapping[str, object],
    arrival_seed: int,
):
    """Build a compatibility or continuous backend without leaking selection into the director."""
    name = str(backend_name).strip().lower()
    if name == "isaac_people":
        from ..motion_backend import IsaacPeopleBackend

        return IsaacPeopleBackend(node, move_service_name)

    if name == "hunav":
        from ..hunav_motion_backend import HuNavMotionBackend

        config = dict(hunav_config)
        config.setdefault("reaction_seed", int(arrival_seed))
        return HuNavMotionBackend(
            node,
            move_service_name=move_service_name,
            namespace=hunav_namespace,
            config=config,
        )

    if name == "local_motion":
        from ..local_motion_backend import LocalMotionBackend

        config = dict(motion_config.get("local_motion", {}) or {})
        config.setdefault("safety", dict(hunav_config.get("safety", {}) or {}))
        return LocalMotionBackend(
            node,
            move_service_name=move_service_name,
            config=config,
        )

    raise ValueError(f"Unsupported pedestrian motion backend: {backend_name!r}")
