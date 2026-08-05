"""Convert reviewed manual-collection rosbag episodes into aligned NPZ shards."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from .alignment import nearest_index, previous_index, relative_goal
from .preprocessing import normalize_scan, quaternion_yaw


@dataclass
class TopicSeries:
    timestamps: list[float]
    values: list[Any]


TOPICS = {
    "/front_scan",
    "/rear_scan",
    "/odom",
    "/cmd_vel_gamepad_diff",
    "/cmd_vel_applied",
    "/toilet_benchmark/collection_status",
}


def _read_topics(bag_dir: Path) -> dict[str, TopicSeries]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = TOPICS - set(types)
    if missing:
        raise ValueError(f"missing topics in {bag_dir}: {sorted(missing)}")
    message_types = {topic: get_message(types[topic]) for topic in TOPICS}
    result = {topic: TopicSeries([], []) for topic in TOPICS}
    while reader.has_next():
        topic, payload, timestamp_ns = reader.read_next()
        if topic not in TOPICS:
            continue
        result[topic].timestamps.append(float(timestamp_ns) / 1e9)
        result[topic].values.append(deserialize_message(payload, message_types[topic]))
    return result


def _running_start(series: TopicSeries) -> float:
    for timestamp, message in zip(series.timestamps, series.values):
        try:
            if json.loads(message.data).get("state") == "RUNNING":
                return timestamp
        except (AttributeError, json.JSONDecodeError, TypeError):
            continue
    raise ValueError("collection status never entered RUNNING")


def export_episode(
    episode_dir: Path,
    output_path: Path,
    *,
    sample_rate_hz: float,
    beam_count: int,
) -> dict[str, Any]:
    metadata = yaml.safe_load((episode_dir / "metadata.yaml").read_text()) or {}
    goal = (metadata.get("scenario") or {}).get("robot_goal")
    if not isinstance(goal, list) or len(goal) != 3:
        raise ValueError(f"{episode_dir}: scenario.robot_goal must be [x, y, yaw]")
    series = _read_topics(episode_dir / "rosbag2")
    start = _running_start(series["/toilet_benchmark/collection_status"])
    required = [
        series["/front_scan"],
        series["/rear_scan"],
        series["/odom"],
        series["/cmd_vel_gamepad_diff"],
        series["/cmd_vel_applied"],
    ]
    end = min(items.timestamps[-1] for items in required if items.timestamps)
    if end <= start:
        raise ValueError(f"{episode_dir}: no aligned RUNNING interval")
    step = 1.0 / max(1.0, float(sample_rate_hz))
    targets = np.arange(start, end, step, dtype=np.float64)
    scans = np.empty((len(targets), 2, beam_count), dtype=np.float32)
    state = np.empty((len(targets), 8), dtype=np.float32)
    action = np.empty((len(targets), 2), dtype=np.float32)
    applied = np.empty((len(targets), 2), dtype=np.float32)
    previous_action = np.zeros(2, dtype=np.float32)
    max_scan_age = 0.35
    for row, target in enumerate(targets):
        front_index = nearest_index(series["/front_scan"].timestamps, target)
        rear_index = nearest_index(series["/rear_scan"].timestamps, target)
        if abs(series["/front_scan"].timestamps[front_index] - target) > max_scan_age:
            raise ValueError(f"{episode_dir}: stale front scan near {target:.3f}")
        if abs(series["/rear_scan"].timestamps[rear_index] - target) > max_scan_age:
            raise ValueError(f"{episode_dir}: stale rear scan near {target:.3f}")
        scans[row, 0] = normalize_scan(series["/front_scan"].values[front_index], beam_count)
        scans[row, 1] = normalize_scan(series["/rear_scan"].values[rear_index], beam_count)
        odom = series["/odom"].values[nearest_index(series["/odom"].timestamps, target)]
        pose = odom.pose.pose
        twist = odom.twist.twist
        goal_state = relative_goal(
            pose.position.x,
            pose.position.y,
            quaternion_yaw(pose.orientation),
            goal[0],
            goal[1],
            goal[2],
        )
        state[row] = np.asarray(
            [*goal_state, twist.linear.x, twist.angular.z, *previous_action],
            dtype=np.float32,
        )
        command = series["/cmd_vel_gamepad_diff"].values[
            previous_index(series["/cmd_vel_gamepad_diff"].timestamps, target)
        ]
        action[row] = (float(command.linear.x), float(command.angular.z))
        executed = series["/cmd_vel_applied"].values[
            previous_index(series["/cmd_vel_applied"].timestamps, target)
        ]
        applied[row] = (float(executed.linear.x), float(executed.angular.z))
        previous_action = action[row]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_path,
        timestamp_sec=(targets - start).astype(np.float32),
        scan=scans,
        state=state,
        action=action,
        applied_action=applied,
    )
    return {
        "sample_count": len(targets),
        "duration_sec": round(float(targets[-1] - targets[0]), 6),
        "scan_shape": list(scans.shape[1:]),
        "state_size": int(state.shape[1]),
        "action_size": int(action.shape[1]),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_policy_export", description=__doc__)
    parser.add_argument("--index", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-rate-hz", type=float, default=10.0)
    parser.add_argument("--beam-count", type=int, default=721)
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _parser().parse_args(args)
    namespace.output.mkdir(parents=True, exist_ok=True)
    records = [json.loads(line) for line in namespace.index.read_text().splitlines() if line]
    selected = [item for item in records if item.get("bc_eligible")]
    manifest = []
    try:
        for index, item in enumerate(selected, start=1):
            shard_name = item["episode_key"].replace("/", "__") + ".npz"
            relative = Path("episodes") / shard_name
            summary = export_episode(
                Path(item["episode_dir"]),
                namespace.output / relative,
                sample_rate_hz=namespace.sample_rate_hz,
                beam_count=namespace.beam_count,
            )
            manifest.append({**item, "shard": str(relative), **summary})
            print(f"[{index}/{len(selected)}] {item['episode_key']}: {summary['sample_count']} samples")
        payload = {
            "schema_version": "toilet-policy-aligned-0.1",
            "sample_rate_hz": namespace.sample_rate_hz,
            "beam_count": namespace.beam_count,
            "episode_count": len(manifest),
            "sample_count": sum(item["sample_count"] for item in manifest),
            "episodes": manifest,
        }
        (namespace.output / "manifest.json").write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps({key: payload[key] for key in ("episode_count", "sample_count")}))
        return 0
    except (OSError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
