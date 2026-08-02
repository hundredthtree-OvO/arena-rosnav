"""Composable scenario templates."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class EnterUseExit:
    """Enter, acquire one semantic resource, use it, release it, and exit."""

    exit_target_id: str = "exit_main"
    activity_duration_sec: float | None = None

    def __post_init__(self) -> None:
        if not self.exit_target_id:
            raise ValueError("exit_target_id must not be empty")
        if self.activity_duration_sec is not None and self.activity_duration_sec < 0.0:
            raise ValueError("activity_duration_sec must be non-negative")
