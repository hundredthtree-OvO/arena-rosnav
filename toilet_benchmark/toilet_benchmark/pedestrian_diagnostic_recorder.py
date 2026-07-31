"""Record live pedestrian poses and visual envelopes as JSONL."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


def _stamp_sec(message) -> float | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    if stamp is None:
        return None
    return float(getattr(stamp, "sec", 0.0)) + float(
        getattr(stamp, "nanosec", 0.0)
    ) * 1e-9


class PedestrianDiagnosticRecorder:
    def __init__(self, node, *, pose_output: Path, envelope_output: Path):
        from people_msgs.msg import People
        from std_msgs.msg import String

        from toilet_benchmark.hunav_isaac_mirror_core import person_yaw

        self._node = node
        self._pose_output = pose_output
        self._envelope_output = envelope_output
        self._person_yaw = person_yaw
        self._pose_stream = pose_output.open("w", encoding="utf-8")
        self._envelope_stream = envelope_output.open("w", encoding="utf-8")
        self._pose_subscription = node.create_subscription(
            People,
            "/isaac/pedestrian_states",
            self._pose_cb,
            20,
        )
        self._envelope_subscription = node.create_subscription(
            String,
            "/isaac/pedestrian_visual_envelopes",
            self._envelope_cb,
            10,
        )

    def close(self) -> None:
        self._pose_stream.close()
        self._envelope_stream.close()

    def _pose_cb(self, message) -> None:
        agents = []
        for person in getattr(message, "people", ()) or ():
            position = getattr(person, "position", None)
            velocity = getattr(person, "velocity", None)
            agents.append(
                {
                    "name": str(getattr(person, "name", "") or ""),
                    "x": float(getattr(position, "x", 0.0)),
                    "y": float(getattr(position, "y", 0.0)),
                    "z": float(getattr(position, "z", 0.0)),
                    "yaw": self._person_yaw(person),
                    "vx": float(getattr(velocity, "x", 0.0)),
                    "vy": float(getattr(velocity, "y", 0.0)),
                    "reliability": float(getattr(person, "reliability", 0.0)),
                }
            )
        payload = {
            "source_stamp_sec": _stamp_sec(message),
            "received_wall_time": time.time(),
            "received_monotonic_time": time.monotonic(),
            "agents": agents,
        }
        self._pose_stream.write(json.dumps(payload, sort_keys=True) + "\n")
        self._pose_stream.flush()

    def _envelope_cb(self, message) -> None:
        try:
            payload = json.loads(str(getattr(message, "data", "") or ""))
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        payload.setdefault("received_wall_time", time.time())
        payload.setdefault("received_monotonic_time", time.monotonic())
        self._envelope_stream.write(json.dumps(payload, sort_keys=True) + "\n")
        self._envelope_stream.flush()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pose-output", type=Path, required=True)
    parser.add_argument("--envelope-output", type=Path, required=True)
    return parser


def main(args=None) -> int:
    namespace = _build_parser().parse_args(args)
    namespace.pose_output.expanduser().resolve().parent.mkdir(
        parents=True, exist_ok=True
    )
    namespace.envelope_output.expanduser().resolve().parent.mkdir(
        parents=True, exist_ok=True
    )

    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node("pedestrian_diagnostic_recorder")
    recorder = PedestrianDiagnosticRecorder(
        node,
        pose_output=namespace.pose_output.expanduser().resolve(),
        envelope_output=namespace.envelope_output.expanduser().resolve(),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
