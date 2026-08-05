"""Simulator-neutral schema for reviewed manual-collection episodes."""

from __future__ import annotations

from enum import Enum


DATASET_SCHEMA_VERSION = "toilet-policy-dataset-0.1"
ANNOTATION_SCHEMA_VERSION = "toilet-policy-annotations-0.1"


class QualityLabel(str, Enum):
    PENDING_REVIEW = "pending_review"
    CLEAN_SUCCESS = "clean_success"
    RECOVERED_SCENE_CONTACT = "recovered_scene_contact"
    HUMAN_COLLISION = "human_collision"
    TASK_FAILURE = "task_failure"
    INVALID_SIMULATION = "invalid_simulation"


def parse_quality_label(value: str) -> QualityLabel:
    try:
        return QualityLabel(str(value).strip())
    except ValueError as exc:
        choices = ", ".join(item.value for item in QualityLabel)
        raise ValueError(f"unknown quality label {value!r}; expected one of: {choices}") from exc
