"""Load, validate, and publish exported pedestrian walkable maps."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def load_walkable_map(path: str | Path) -> dict[str, Any]:
    resolved = Path(path).expanduser()
    if not resolved.is_file():
        raise FileNotFoundError(
            f"walkable map does not exist: {resolved}. "
            "Start the updated bridge and run "
            "'ros2 run ros2isaacsim export_walkable_map' first."
        )
    payload = json.loads(resolved.read_text(encoding="utf-8"))
    if payload.get("schema") != "arena.walkable_map.v1":
        raise ValueError(f"unsupported walkable-map schema: {payload.get('schema')}")
    width = int(payload["width"])
    height = int(payload["height"])
    data = [int(value) for value in payload["data"]]
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError(
            f"invalid walkable-map dimensions: {width}x{height}, data={len(data)}"
        )
    payload["width"] = width
    payload["height"] = height
    payload["data"] = data
    return payload


def cell_for_world(payload: dict[str, Any], x: float, y: float) -> tuple[int, int] | None:
    resolution = float(payload["resolution"])
    origin = payload["origin"]
    ix = math.floor((float(x) - float(origin[0])) / resolution)
    iy = math.floor((float(y) - float(origin[1])) / resolution)
    if ix < 0 or iy < 0 or ix >= payload["width"] or iy >= payload["height"]:
        return None
    return int(ix), int(iy)


def value_at_world(payload: dict[str, Any], x: float, y: float) -> int | None:
    cell = cell_for_world(payload, x, y)
    if cell is None:
        return None
    ix, iy = cell
    return int(payload["data"][iy * payload["width"] + ix])


def load_validation_anchors(path: str | Path | None) -> list[dict[str, Any]]:
    if not path:
        return []
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        config = yaml.safe_load(handle) or {}
    anchors = []
    for item in config.get("anchors", []) or []:
        pose = item.get("pose", [])
        if len(pose) < 2:
            continue
        anchors.append(
            {
                "id": str(item.get("id", f"anchor_{len(anchors)}")),
                "pose": [float(pose[0]), float(pose[1])],
                "expected": str(item.get("expected", "free")),
            }
        )
    return anchors


def default_validation_anchors_path() -> str:
    try:
        return str(
            Path(get_package_share_directory("toilet_benchmark"))
            / "config"
            / "walkable_map_validation.yaml"
        )
    except Exception:
        return str(
            Path(__file__).resolve().parents[1]
            / "config"
            / "walkable_map_validation.yaml"
        )


class WalkableMapPublisher(Node):
    def __init__(
        self,
        *,
        map_path: str,
        anchors_path: str = "",
        map_topic: str = "/toilet_benchmark/walkable_map",
        marker_topic: str = "/toilet_benchmark/walkable_map_anchors",
        frame_id: str = "odom",
    ):
        super().__init__("toilet_walkable_map")
        self._payload = load_walkable_map(map_path)
        self._anchors = load_validation_anchors(anchors_path)
        self._frame_id = str(frame_id)
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_publisher = self.create_publisher(OccupancyGrid, map_topic, qos)
        self._marker_publisher = self.create_publisher(MarkerArray, marker_topic, qos)
        self._publish()

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.frame_id = self._frame_id
        message.header.stamp = stamp
        message.info.resolution = float(self._payload["resolution"])
        message.info.width = int(self._payload["width"])
        message.info.height = int(self._payload["height"])
        message.info.origin.position.x = float(self._payload["origin"][0])
        message.info.origin.position.y = float(self._payload["origin"][1])
        message.info.origin.orientation.w = 1.0
        message.data = self._payload["data"]
        self._map_publisher.publish(message)

        markers = MarkerArray()
        failures = []
        for index, anchor in enumerate(self._anchors):
            x, y = anchor["pose"]
            value = value_at_world(self._payload, x, y)
            state = "outside" if value is None else "free" if value == 0 else "occupied" if value == 100 else "unknown"
            expected = anchor["expected"]
            passed = state == expected
            if not passed:
                failures.append(f"{anchor['id']}={state}, expected={expected}")
            marker = Marker()
            marker.header.frame_id = self._frame_id
            marker.header.stamp = stamp
            marker.ns = "walkable_map_anchor"
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.08
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.16
            marker.color.r = 0.1 if passed else 1.0
            marker.color.g = 0.9 if passed else 0.1
            marker.color.b = 0.2
            marker.color.a = 0.95
            markers.markers.append(marker)
        self._marker_publisher.publish(markers)

        counts = self._payload.get("counts", {})
        self.get_logger().info(
            f"Published walkable map: dimensions={message.info.width}x{message.info.height}, "
            f"resolution={message.info.resolution:.3f}, counts={counts}, "
            f"fingerprint={self._payload.get('scene_fingerprint', '')[:12]}"
        )
        if failures:
            self.get_logger().error(
                "Walkable-map anchor validation failed: " + "; ".join(failures)
            )
        elif self._anchors:
            self.get_logger().info(
                f"Walkable-map anchor validation passed: {len(self._anchors)} anchors."
            )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map",
        default="/home/stardust/resources/arena_ws/arena_assets/navigation/"
        "shenxinfu_841837.walkable.json",
    )
    parser.add_argument(
        "--anchors",
        default=default_validation_anchors_path(),
    )
    parser.add_argument("--map-topic", default="/toilet_benchmark/walkable_map")
    parser.add_argument(
        "--marker-topic",
        default="/toilet_benchmark/walkable_map_anchors",
    )
    parser.add_argument("--frame-id", default="odom")
    parsed, ros_args = parser.parse_known_args(argv)
    rclpy.init(args=ros_args)
    node = None
    try:
        node = WalkableMapPublisher(
            map_path=parsed.map,
            anchors_path=parsed.anchors,
            map_topic=parsed.map_topic,
            marker_topic=parsed.marker_topic,
            frame_id=parsed.frame_id,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        return 0
    except (FileNotFoundError, ValueError, json.JSONDecodeError) as exc:
        print(f"walkable_map_publisher: {exc}")
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
