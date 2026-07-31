"""Episode schema validation utilities for S2-B."""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from typing import Any, Collection, Sequence

from .schema import BENCHMARK_SCHEMA_VERSION, EpisodeSpec


_ID_PATTERN = re.compile(r"^[A-Za-z0-9._-]+$")


class ValidationErrorCode:
    SCHEMA_VERSION_MISMATCH = "schema_version_mismatch"
    EPISODE_ID_INVALID = "episode_id_invalid"
    SCENE_ID_INVALID = "scene_id_invalid"
    TASK_TYPE_INVALID = "task_type_invalid"
    TRACK_INVALID = "track_invalid"
    AGENT_ID_INVALID = "agent_id_invalid"
    AGENT_ID_DUPLICATED = "agent_id_duplicated"
    PED_COUNT_MISMATCH = "pedestrian_count_mismatch"
    PED_RESOURCE_MISSING = "pedestrian_resource_missing"
    TIMEOUT_INVALID = "timeout_invalid"
    TOLERANCE_INVALID = "goal_tolerance_invalid"
    POSE_NOT_FINITE = "pose_not_finite"
    POSE_DIMENSION_INVALID = "pose_dimension_invalid"


@dataclass(frozen=True)
class EpisodeValidationError:
    code: str
    path: tuple[str, ...]
    message: str


def _add_error(
    errors: list[EpisodeValidationError],
    code: str,
    path: Sequence[str],
    message: str,
) -> None:
    errors.append(EpisodeValidationError(code=code, path=tuple(path), message=message))


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _validate_pose_tuple(
    errors: list[EpisodeValidationError],
    values: Any,
    expected_len: int,
    path: tuple[str, ...],
) -> None:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes, bytearray)):
        _add_error(
            errors,
            ValidationErrorCode.POSE_DIMENSION_INVALID,
            path,
            f"{'.'.join(path)} must be a finite numeric tuple of length {expected_len}",
        )
        return
    if len(values) != expected_len:
        _add_error(
            errors,
            ValidationErrorCode.POSE_DIMENSION_INVALID,
            path,
            f"{'.'.join(path)} must have exactly {expected_len} values",
        )
        return
    if not all(_is_finite(item) for item in values):
        _add_error(
            errors,
            ValidationErrorCode.POSE_NOT_FINITE,
            path,
            f"{'.'.join(path)} contains non-finite values",
        )


def _validate_identifier(
    errors: list[EpisodeValidationError],
    value: Any,
    path: tuple[str, ...],
    code: str,
) -> None:
    if not isinstance(value, str) or not value.strip():
        _add_error(errors, code, path, f"{'.'.join(path)} must be a non-empty string")
        return
    if not _ID_PATTERN.fullmatch(value):
        _add_error(
            errors,
            code,
            path,
            f"{'.'.join(path)} must match [A-Za-z0-9._-]+",
        )


def validate_episode(
    episode: EpisodeSpec,
    *,
    expected_schema_version: str = BENCHMARK_SCHEMA_VERSION,
    semantic_goal_ids: Collection[str] | None = None,
) -> tuple[EpisodeValidationError, ...]:
    """Return a deterministic list of validation errors for an episode."""

    errors: list[EpisodeValidationError] = []

    if episode.schema_version != expected_schema_version:
        _add_error(
            errors,
            ValidationErrorCode.SCHEMA_VERSION_MISMATCH,
            ("schema_version",),
            f"schema_version must be '{expected_schema_version}'",
        )

    _validate_identifier(errors, episode.episode_id, ("episode_id",), ValidationErrorCode.EPISODE_ID_INVALID)
    _validate_identifier(errors, episode.scene_id, ("scene_id",), ValidationErrorCode.SCENE_ID_INVALID)

    if not episode.task_type.strip():
        _add_error(errors, ValidationErrorCode.TASK_TYPE_INVALID, ("task_type",), "task_type must be non-empty")

    if not episode.track:
        _add_error(errors, ValidationErrorCode.TRACK_INVALID, ("track",), "track must be defined")
    _validate_pose_tuple(errors, episode.robot.start_pose, 4, ("robot", "start_pose"))
    _validate_pose_tuple(errors, episode.robot.goal_pose, 3, ("robot", "goal_pose"))
    if not isinstance(episode.robot.model, str) or not episode.robot.model.strip():
        _add_error(errors, ValidationErrorCode.TASK_TYPE_INVALID, ("robot", "model"), "robot.model must be non-empty")

    if not _is_finite(episode.termination.timeout_sec) or episode.termination.timeout_sec <= 0:
        _add_error(
            errors,
            ValidationErrorCode.TIMEOUT_INVALID,
            ("termination", "timeout_sec"),
            "termination.timeout_sec must be a positive finite number",
        )
    if not _is_finite(episode.termination.goal_tolerance_m) or episode.termination.goal_tolerance_m < 0:
        _add_error(
            errors,
            ValidationErrorCode.TOLERANCE_INVALID,
            ("termination", "goal_tolerance_m"),
            "termination.goal_tolerance_m must be a non-negative finite number",
        )

    agent_ids: list[str] = []
    for index, pedestrian in enumerate(episode.pedestrians):
        _validate_identifier(
            errors,
            pedestrian.agent_id,
            ("pedestrians", str(index), "agent_id"),
            ValidationErrorCode.AGENT_ID_INVALID,
        )
        agent_ids.append(str(pedestrian.agent_id))
        if not isinstance(pedestrian.semantic_goal, str) or not pedestrian.semantic_goal.strip():
            _add_error(
                errors,
                ValidationErrorCode.PED_RESOURCE_MISSING,
                ("pedestrians", str(index), "semantic_goal"),
                f"pedestrians[{index}].semantic_goal must be non-empty",
            )
        if pedestrian.start_pose is not None:
            _validate_pose_tuple(
                errors,
                pedestrian.start_pose,
                3,
                ("pedestrians", str(index), "start_pose"),
            )

    if len(agent_ids) != len(set(agent_ids)):
        seen: set[str] = set()
        for index, agent_id in enumerate(agent_ids):
            if agent_id in seen:
                _add_error(
                    errors,
                    ValidationErrorCode.AGENT_ID_DUPLICATED,
                    ("pedestrians", str(index), "agent_id"),
                    f"pedestrians[{index}].agent_id duplicated",
                )
            seen.add(agent_id)

    if "pedestrian_count" in episode.difficulty:
        expected = episode.difficulty["pedestrian_count"]
        if not isinstance(expected, int) or expected < 0:
            _add_error(
                errors,
                ValidationErrorCode.PED_COUNT_MISMATCH,
                ("difficulty", "pedestrian_count"),
                "difficulty.pedestrian_count must be a non-negative int",
            )
        elif expected != len(agent_ids):
            _add_error(
                errors,
                ValidationErrorCode.PED_COUNT_MISMATCH,
                ("difficulty", "pedestrian_count"),
                f"count mismatch: difficulty.pedestrian_count={expected}, actual={len(agent_ids)}",
            )

    if semantic_goal_ids is not None:
        known_goals = {str(goal_id) for goal_id in semantic_goal_ids}
        for index, pedestrian in enumerate(episode.pedestrians):
            if pedestrian.semantic_goal and pedestrian.semantic_goal not in known_goals:
                _add_error(
                    errors,
                    ValidationErrorCode.PED_RESOURCE_MISSING,
                    ("pedestrians", str(index), "semantic_goal"),
                    f"pedestrian semantic_goal '{pedestrian.semantic_goal}' is not declared",
                )

    return tuple(sorted(errors, key=lambda item: (item.path, item.code, item.message)))


def require_valid_episode(
    episode: EpisodeSpec,
    *,
    expected_schema_version: str = BENCHMARK_SCHEMA_VERSION,
    semantic_goal_ids: Collection[str] | None = None,
) -> None:
    errors = validate_episode(
        episode,
        expected_schema_version=expected_schema_version,
        semantic_goal_ids=semantic_goal_ids,
    )
    if errors:
        messages = "; ".join(f"{error.code}@{'.'.join(error.path)}: {error.message}" for error in errors)
        raise ValueError(f"invalid episode: {messages}")
