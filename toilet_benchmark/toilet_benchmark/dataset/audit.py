"""Audit manual-collection bags and build a leakage-safe dataset index."""

from __future__ import annotations

import hashlib
import json
import random
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import yaml

from .schema import (
    ANNOTATION_SCHEMA_VERSION,
    DATASET_SCHEMA_VERSION,
    QualityLabel,
    parse_quality_label,
)


REQUIRED_TOPICS = (
    "/cmd_vel_gamepad_diff",
    "/cmd_vel_applied",
    "/odom",
    "/front_scan",
    "/rear_scan",
    "/isaac/pedestrian_states",
)


@dataclass(frozen=True)
class Annotation:
    quality: QualityLabel
    notes: str = ""


def episode_key(session_dir: Path, episode_dir: Path) -> str:
    return f"{session_dir.name}/{episode_dir.name}"


def load_annotations(path: Path) -> dict[str, Annotation]:
    if not path.exists():
        return {}
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if payload.get("schema_version") != ANNOTATION_SCHEMA_VERSION:
        raise ValueError(
            f"annotation schema mismatch in {path}: "
            f"expected {ANNOTATION_SCHEMA_VERSION!r}"
        )
    result = {}
    for key, value in (payload.get("episodes") or {}).items():
        item = value or {}
        result[str(key)] = Annotation(
            quality=parse_quality_label(item.get("quality", "pending_review")),
            notes=str(item.get("notes", "")),
        )
    return result


def _automatic_quality(metadata: Mapping[str, Any], issues: Sequence[str]) -> QualityLabel:
    if issues:
        return QualityLabel.INVALID_SIMULATION
    reason = str(metadata.get("termination_reason", ""))
    status = str(metadata.get("status", ""))
    if reason == "robot_human_collision":
        return QualityLabel.HUMAN_COLLISION
    if reason == "scene_collision":
        return QualityLabel.RECOVERED_SCENE_CONTACT
    if status == "succeeded" and reason == "robot_reached_goal":
        return QualityLabel.PENDING_REVIEW
    return QualityLabel.TASK_FAILURE


def _bag_information(path: Path) -> tuple[dict[str, int], float, list[str]]:
    issues = []
    if not path.exists():
        return {}, 0.0, ["missing_rosbag_metadata"]
    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        info = payload["rosbag2_bagfile_information"]
        duration_sec = float(info["duration"]["nanoseconds"]) / 1e9
        counts = {
            str(item["topic_metadata"]["name"]): int(item["message_count"])
            for item in info.get("topics_with_message_count", ())
        }
    except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        return {}, 0.0, [f"invalid_rosbag_metadata:{exc}"]
    if duration_sec <= 0.0:
        issues.append("non_positive_bag_duration")
    for topic in REQUIRED_TOPICS:
        if counts.get(topic, 0) <= 0:
            issues.append(f"missing_required_topic:{topic}")
    return counts, duration_sec, issues


def _episode_hash(episode_dir: Path) -> str:
    digest = hashlib.sha256()
    inputs = [episode_dir / "metadata.yaml", episode_dir / "events.jsonl"]
    inputs.extend(sorted((episode_dir / "rosbag2").glob("*")))
    for path in inputs:
        if not path.is_file():
            continue
        digest.update(path.relative_to(episode_dir).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return digest.hexdigest()


def _allocate_session_splits(
    session_names: Sequence[str],
    *,
    seed: int,
) -> dict[str, str]:
    names = sorted(set(session_names))
    if not names:
        return {}
    random.Random(int(seed)).shuffle(names)
    if len(names) == 1:
        return {names[0]: "train"}
    if len(names) == 2:
        return {names[0]: "train", names[1]: "test"}
    validation_count = max(1, round(len(names) * 0.17))
    test_count = max(1, round(len(names) * 0.17))
    while validation_count + test_count >= len(names):
        if test_count > 1:
            test_count -= 1
        elif validation_count > 1:
            validation_count -= 1
        else:
            break
    train_count = len(names) - validation_count - test_count
    assignments = {}
    for index, name in enumerate(names):
        if index < train_count:
            assignments[name] = "train"
        elif index < train_count + validation_count:
            assignments[name] = "validation"
        else:
            assignments[name] = "test"
    return assignments


def audit_dataset(
    data_root: Path,
    *,
    session_globs: Sequence[str],
    annotations: Mapping[str, Annotation],
    split_seed: int,
    accept_unreviewed_success: bool = False,
) -> tuple[list[dict[str, Any]], dict[str, str]]:
    session_dirs = sorted(
        {
            path.resolve()
            for pattern in session_globs
            for path in data_root.expanduser().glob(pattern)
            if path.is_dir()
        }
    )
    split_by_session = _allocate_session_splits(
        [path.name for path in session_dirs], seed=split_seed
    )
    records = []
    for session_dir in session_dirs:
        for episode_dir in sorted(session_dir.glob("episode_*")):
            if not episode_dir.is_dir():
                continue
            key = episode_key(session_dir, episode_dir)
            issues = []
            metadata_path = episode_dir / "metadata.yaml"
            try:
                metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            except (OSError, yaml.YAMLError) as exc:
                metadata = {}
                issues.append(f"invalid_episode_metadata:{exc}")
            counts, duration_sec, bag_issues = _bag_information(
                episode_dir / "rosbag2" / "metadata.yaml"
            )
            issues.extend(bag_issues)
            if not metadata.get("rosbag_started", False):
                issues.append("rosbag_not_started")
            automatic = _automatic_quality(metadata, issues)
            annotation = annotations.get(key)
            protected = {
                QualityLabel.INVALID_SIMULATION,
                QualityLabel.HUMAN_COLLISION,
                QualityLabel.RECOVERED_SCENE_CONTACT,
                QualityLabel.TASK_FAILURE,
            }
            if automatic in protected:
                quality = automatic
                quality_source = "automatic_protected"
            elif annotation is not None:
                quality = annotation.quality
                quality_source = "manual"
            else:
                quality = automatic
                quality_source = "automatic"
            records.append(
                {
                    "schema_version": DATASET_SCHEMA_VERSION,
                    "episode_key": key,
                    "session_id": session_dir.name,
                    "episode_id": episode_dir.name,
                    "episode_dir": str(episode_dir),
                    "source_hash": _episode_hash(episode_dir),
                    "split": split_by_session[session_dir.name],
                    "status": str(metadata.get("status", "unknown")),
                    "termination_reason": str(metadata.get("termination_reason", "unknown")),
                    "quality": quality.value,
                    "quality_source": quality_source,
                    "notes": annotation.notes if annotation is not None else "",
                    "bc_eligible": (
                        quality is QualityLabel.CLEAN_SUCCESS
                        or (
                            accept_unreviewed_success
                            and quality is QualityLabel.PENDING_REVIEW
                        )
                    )
                    and not issues,
                    "training_admission": (
                        "unreviewed_success_override"
                        if accept_unreviewed_success
                        and quality is QualityLabel.PENDING_REVIEW
                        and not issues
                        else "reviewed_clean"
                        if quality is QualityLabel.CLEAN_SUCCESS and not issues
                        else "excluded"
                    ),
                    "duration_sec": round(duration_sec, 6),
                    "topic_counts": counts,
                    "issues": issues,
                }
            )
    return records, split_by_session


def annotation_template(records: Iterable[Mapping[str, Any]]) -> dict[str, Any]:
    episodes = {}
    for record in records:
        quality = str(record["quality"])
        episodes[str(record["episode_key"])] = {
            "quality": quality,
            "notes": str(record.get("notes", "")),
        }
    return {
        "schema_version": ANNOTATION_SCHEMA_VERSION,
        "episodes": episodes,
    }


def dataset_stats(records: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    return {
        "schema_version": DATASET_SCHEMA_VERSION,
        "episode_count": len(records),
        "duration_sec": round(sum(float(item["duration_sec"]) for item in records), 3),
        "status_counts": dict(sorted(Counter(item["status"] for item in records).items())),
        "termination_counts": dict(
            sorted(Counter(item["termination_reason"] for item in records).items())
        ),
        "quality_counts": dict(sorted(Counter(item["quality"] for item in records).items())),
        "split_counts": dict(sorted(Counter(item["split"] for item in records).items())),
        "bc_eligible_count": sum(bool(item["bc_eligible"]) for item in records),
        "issue_episode_count": sum(bool(item["issues"]) for item in records),
    }


def write_jsonl(path: Path, records: Iterable[Mapping[str, Any]]) -> None:
    path.write_text(
        "".join(json.dumps(dict(item), sort_keys=True) + "\n" for item in records),
        encoding="utf-8",
    )
