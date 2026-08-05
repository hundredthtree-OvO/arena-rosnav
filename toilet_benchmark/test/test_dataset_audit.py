from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import yaml

from toilet_benchmark.dataset.audit import (
    Annotation,
    REQUIRED_TOPICS,
    audit_dataset,
    dataset_stats,
)
from toilet_benchmark.dataset.schema import QualityLabel


def _write_episode(
    root: Path,
    session: str,
    episode: str,
    *,
    status: str = "succeeded",
    reason: str = "robot_reached_goal",
) -> Path:
    path = root / session / episode
    bag = path / "rosbag2"
    bag.mkdir(parents=True)
    (path / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "status": status,
                "termination_reason": reason,
                "rosbag_started": True,
            }
        ),
        encoding="utf-8",
    )
    (path / "events.jsonl").write_text(
        json.dumps({"event_type": "episode_finished"}) + "\n", encoding="utf-8"
    )
    topics = [
        {
            "topic_metadata": {"name": topic, "type": "test/msg/Type"},
            "message_count": 10,
        }
        for topic in REQUIRED_TOPICS
    ]
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump(
            {
                "rosbag2_bagfile_information": {
                    "duration": {"nanoseconds": 1_000_000_000},
                    "topics_with_message_count": topics,
                }
            }
        ),
        encoding="utf-8",
    )
    (bag / "rosbag2_0.db3").write_bytes(b"test bag")
    return path


class DatasetAuditTest(unittest.TestCase):
    def test_manual_clean_review_makes_success_bc_eligible(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_episode(root, "session_a", "episode_000001")
            records, splits = audit_dataset(
                root,
                session_globs=("session_*",),
                annotations={
                    "session_a/episode_000001": Annotation(QualityLabel.CLEAN_SUCCESS)
                },
                split_seed=42,
            )

        self.assertEqual(splits, {"session_a": "train"})
        self.assertEqual(records[0]["quality"], "clean_success")
        self.assertTrue(records[0]["bc_eligible"])
        self.assertFalse(records[0]["issues"])

    def test_unreviewed_success_can_be_admitted_explicitly(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_episode(root, "session_a", "episode_000001")
            records, _ = audit_dataset(
                root,
                session_globs=("session_*",),
                annotations={},
                split_seed=42,
                accept_unreviewed_success=True,
            )

        self.assertEqual(records[0]["quality"], "pending_review")
        self.assertEqual(
            records[0]["training_admission"], "unreviewed_success_override"
        )
        self.assertTrue(records[0]["bc_eligible"])

    def test_explicit_human_collision_cannot_be_relabeled_as_clean(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            _write_episode(
                root,
                "session_a",
                "episode_000001",
                status="failed",
                reason="robot_human_collision",
            )
            records, _ = audit_dataset(
                root,
                session_globs=("session_*",),
                annotations={
                    "session_a/episode_000001": Annotation(QualityLabel.CLEAN_SUCCESS)
                },
                split_seed=42,
            )

        self.assertEqual(records[0]["quality"], "human_collision")
        self.assertEqual(records[0]["quality_source"], "automatic_protected")
        self.assertFalse(records[0]["bc_eligible"])

    def test_splits_are_disjoint_at_session_level(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for index in range(6):
                session = f"session_{index}"
                _write_episode(root, session, "episode_000001")
                _write_episode(root, session, "episode_000002")
            records, splits = audit_dataset(
                root,
                session_globs=("session_*",),
                annotations={},
                split_seed=7,
            )

        self.assertEqual(len(splits), 6)
        self.assertEqual(set(splits.values()), {"train", "validation", "test"})
        for record in records:
            self.assertEqual(record["split"], splits[record["session_id"]])
        self.assertEqual(dataset_stats(records)["episode_count"], 12)


if __name__ == "__main__":
    unittest.main()
