"""Export one recorded pedestrian trajectory into a sealed Replay bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Sequence

import yaml

from .replay import (
    REPLAY_SCHEMA_VERSION,
    ReplayAnnotations,
    ReplayBundle,
    ReplayProvenance,
    ReplayTimeBase,
    ReplayTrackAgent,
    ReplayTrajectorySample,
    require_valid_replay_bundle,
    seal_replay_bundle,
)


@dataclass(frozen=True)
class RecordedPedestrianSample:
    timestamp_sec: float
    x: float
    y: float
    z: float


def _wrap_angle(value: float) -> float:
    return (float(value) + math.pi) % (2.0 * math.pi) - math.pi


def _episode_hash(episode_dir: Path) -> str:
    digest = hashlib.sha256()
    inputs = [episode_dir / "metadata.yaml"]
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


def derive_replay_trajectory(
    records: Sequence[RecordedPedestrianSample],
    *,
    radius_m: float = 0.3,
    parking_bound_m: float = 100.0,
    motion_epsilon_m: float = 0.002,
) -> tuple[ReplayTrajectorySample, ...]:
    active = [
        item
        for item in records
        if abs(item.x) <= parking_bound_m and abs(item.y) <= parking_bound_m
    ]
    if len(active) < 2:
        raise ValueError("trajectory needs at least two non-parking samples")
    origin = active[0].timestamp_sec
    times = [item.timestamp_sec - origin for item in active]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError("recorded timestamps must be strictly increasing")

    velocities: list[tuple[float, float]] = []
    yaws: list[float] = []
    last_yaw = 0.0
    for index, item in enumerate(active):
        if index == 0:
            other = active[1]
            dt = other.timestamp_sec - item.timestamp_sec
            dx, dy = other.x - item.x, other.y - item.y
        elif index == len(active) - 1:
            other = active[index - 1]
            dt = item.timestamp_sec - other.timestamp_sec
            dx, dy = item.x - other.x, item.y - other.y
        else:
            before, after = active[index - 1], active[index + 1]
            dt = after.timestamp_sec - before.timestamp_sec
            dx, dy = after.x - before.x, after.y - before.y
        vx, vy = dx / dt, dy / dt
        velocities.append((vx, vy))
        if math.hypot(dx, dy) >= motion_epsilon_m:
            last_yaw = math.atan2(dy, dx)
        yaws.append(last_yaw)

    samples = []
    for index, item in enumerate(active):
        if index == 0:
            wz = 0.0
        else:
            dt = times[index] - times[index - 1]
            wz = _wrap_angle(yaws[index] - yaws[index - 1]) / dt
        samples.append(
            ReplayTrajectorySample(
                sample_index=index,
                timestamp_sec=times[index],
                x=item.x,
                y=item.y,
                z=item.z,
                yaw=yaws[index],
                vx=velocities[index][0],
                vy=velocities[index][1],
                wz=wz,
                radius_m=radius_m,
                source="replay_reference",
                extras={
                    "velocity_source": "position_finite_difference",
                    "yaw_source": "position_finite_difference",
                },
            )
        )
    return tuple(samples)


def _read_episode_bag(
    episode_dir: Path,
    *,
    agent_id: str,
    topic: str,
) -> tuple[RecordedPedestrianSample, ...]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError("rosbag2_py and ROS message support are required") from exc

    bag_dir = episode_dir / "rosbag2"
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"topic {topic!r} is not present in {bag_dir}")
    message_type = get_message(topic_types[topic])
    records = []
    while reader.has_next():
        current_topic, payload, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(payload, message_type)
        for person in message.people:
            if person.name == agent_id:
                records.append(
                    RecordedPedestrianSample(
                        timestamp_sec=float(timestamp_ns) / 1e9,
                        x=float(person.position.x),
                        y=float(person.position.y),
                        z=float(person.position.z),
                    )
                )
    if not records:
        raise ValueError(f"agent {agent_id!r} has no samples on {topic}")
    return tuple(records)


def build_replay_bundle_from_episode(
    episode_dir: Path,
    *,
    agent_id: str,
    scene_id: str,
    task_type: str = "enter_exit",
    topic: str = "/isaac/pedestrian_states",
    radius_m: float = 0.3,
    parking_bound_m: float = 100.0,
) -> ReplayBundle:
    episode_dir = episode_dir.expanduser().resolve()
    metadata_path = episode_dir / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
    records = _read_episode_bag(episode_dir, agent_id=agent_id, topic=topic)
    trajectory = derive_replay_trajectory(
        records,
        radius_m=radius_m,
        parking_bound_m=parking_bound_m,
    )
    deltas = [
        b.timestamp_sec - a.timestamp_sec for a, b in zip(trajectory, trajectory[1:])
    ]
    sample_rate_hz = 1.0 / median(deltas)
    source_hash = _episode_hash(episode_dir)
    scenario = metadata.get("scenario") or {}
    semantic_goal = str(
        scenario.get("pedestrian_target_urinal_id")
        or metadata.get("pedestrian_target_urinal_id")
        or "unspecified_goal"
    )
    episode_id = str(metadata.get("episode_id") or episode_dir.name)
    seed = int(metadata.get("seed", 0))
    bundle = ReplayBundle(
        schema_version=REPLAY_SCHEMA_VERSION,
        episode_id=f"{episode_id}_replay",
        scene_id=scene_id,
        task_type=task_type,
        seed=seed,
        reference_episode_hash=source_hash,
        time_base=ReplayTimeBase(
            origin="episode_reset",
            clock="physics_monotonic_sec",
            unit="sec",
            zero_sec=0.0,
            monotonic=True,
            sample_policy="strictly_increasing",
        ),
        agents=(
            ReplayTrackAgent(
                agent_id=agent_id,
                semantic_goal=semantic_goal,
                start_reference="entrance_main",
                start_pose=(
                    trajectory[0].x,
                    trajectory[0].y,
                    trajectory[0].z,
                    trajectory[0].yaw,
                ),
                sample_rate_hz=sample_rate_hz,
                trajectory=trajectory,
                extras={
                    "source_topic": topic,
                    "parking_bound_m": parking_bound_m,
                },
            ),
        ),
        annotations=ReplayAnnotations(
            termination_reason="success",
            termination_timestamp_sec=trajectory[-1].timestamp_sec,
        ),
        provenance=ReplayProvenance(
            source_episode_manifest=str(metadata_path),
            source_episode_hash=source_hash,
            source_capture="manual_collection_rosbag2",
            generated_by="toilet_replay_export",
            generated_at_sec=0.0,
            extras={
                "velocity_source": "position_finite_difference",
                "yaw_source": "position_finite_difference",
            },
        ),
    )
    sealed = seal_replay_bundle(bundle)
    require_valid_replay_bundle(sealed)
    return sealed


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_replay_export", description=__doc__)
    parser.add_argument("episode_dir", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--agent-id", default="toilet_agent_01")
    parser.add_argument("--scene-id", default="shenxinfu_841837")
    parser.add_argument("--task-type", default="enter_exit")
    parser.add_argument("--topic", default="/isaac/pedestrian_states")
    parser.add_argument("--radius-m", type=float, default=0.3)
    parser.add_argument("--parking-bound-m", type=float, default=100.0)
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _build_parser().parse_args(args)
    try:
        bundle = build_replay_bundle_from_episode(
            namespace.episode_dir,
            agent_id=namespace.agent_id,
            scene_id=namespace.scene_id,
            task_type=namespace.task_type,
            topic=namespace.topic,
            radius_m=namespace.radius_m,
            parking_bound_m=namespace.parking_bound_m,
        )
        namespace.output.parent.mkdir(parents=True, exist_ok=True)
        namespace.output.write_text(
            yaml.safe_dump(bundle.to_dict(), sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "output": str(namespace.output),
                    "content_hash": bundle.provenance.content_hash,
                    "sample_count": len(bundle.primary_agent().trajectory),
                    "duration_sec": bundle.primary_agent().terminal_timestamp_sec,
                },
                sort_keys=True,
            )
        )
        return 0
    except (OSError, RuntimeError, TypeError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
