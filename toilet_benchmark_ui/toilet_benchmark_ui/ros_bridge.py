"""Read-only ROS 2 state bridge for the standalone editor."""

from __future__ import annotations

import math
import threading

from python_qt_binding import QtCore


def _xyz(field):
    if field is None:
        return None
    if all(hasattr(field, axis) for axis in ("x", "y", "z")):
        return float(field.x), float(field.y), float(field.z)
    nested = getattr(field, "position", None)
    if nested is not None:
        return _xyz(nested)
    nested = getattr(field, "pose", None)
    if nested is not None:
        return _xyz(nested)
    return None


def _metadata(person) -> dict[str, str]:
    names = list(getattr(person, "tagnames", []) or [])
    values = list(getattr(person, "tags", []) or [])
    return {str(name): str(value) for name, value in zip(names, values)}


def _yaw(velocity, metadata: dict[str, str]) -> float:
    for key in ("yaw_rad", "heading_rad"):
        try:
            value = float(metadata[key])
        except (KeyError, TypeError, ValueError):
            continue
        if math.isfinite(value):
            return value
    if velocity is not None and math.hypot(velocity[0], velocity[1]) > 0.05:
        return math.atan2(velocity[1], velocity[0])
    return 0.0


class RosWorker(QtCore.QThread):
    map_ready = QtCore.Signal(object)
    people_ready = QtCore.Signal(object)
    robot_ready = QtCore.Signal(object)
    anchors_ready = QtCore.Signal(object)
    ros_error = QtCore.Signal(str)

    def __init__(
        self,
        *,
        people_topic: str = "/isaac/pedestrian_states",
        fallback_people_topic: str = "/task_generator_node/people",
        map_topic: str = "/toilet_benchmark/walkable_map",
        anchors_topic: str = "/toilet_benchmark/walkable_map_anchors",
        robot_odom_topic: str = "/odom",
    ) -> None:
        super().__init__()
        self._people_topic = str(people_topic)
        self._fallback_people_topic = str(fallback_people_topic)
        self._map_topic = str(map_topic)
        self._anchors_topic = str(anchors_topic)
        self._robot_odom_topic = str(robot_odom_topic)
        self._stop_event = threading.Event()

    def stop(self) -> None:
        self._stop_event.set()
        self.wait(2000)

    def run(self) -> None:
        import rclpy
        from nav_msgs.msg import OccupancyGrid, Odometry
        from people_msgs.msg import People
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from visualization_msgs.msg import MarkerArray

        node = None
        try:
            rclpy.init(args=None)
            node = Node("toilet_route_editor_ros")
            map_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            state_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
            node.create_subscription(OccupancyGrid, self._map_topic, self._map_cb, map_qos)
            node.create_subscription(People, self._people_topic, self._people_cb, state_qos)
            if self._fallback_people_topic and self._fallback_people_topic != self._people_topic:
                node.create_subscription(
                    People,
                    self._fallback_people_topic,
                    self._people_cb,
                    state_qos,
                )
            node.create_subscription(Odometry, self._robot_odom_topic, self._robot_cb, state_qos)
            node.create_subscription(MarkerArray, self._anchors_topic, self._anchors_cb, map_qos)
            while not self._stop_event.is_set():
                rclpy.spin_once(node, timeout_sec=0.05)
        except Exception as exc:
            self.ros_error.emit(f"ROS editor bridge stopped: {type(exc).__name__}: {exc}")
        finally:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

    def _map_cb(self, message) -> None:
        from .editor_model import MapSnapshot

        try:
            self.map_ready.emit(MapSnapshot.from_occupancy_grid(message))
        except (TypeError, ValueError) as exc:
            self.ros_error.emit(f"Invalid walkable map: {exc}")

    def _people_cb(self, message) -> None:
        people = []
        for index, person in enumerate(getattr(message, "people", []) or []):
            position = _xyz(getattr(person, "position", None) or getattr(person, "pose", None))
            if position is None:
                continue
            velocity = _xyz(getattr(person, "velocity", None)) or (0.0, 0.0, 0.0)
            metadata = _metadata(person)
            identifier = str(
                getattr(person, "name", None)
                or getattr(person, "id", None)
                or f"agent_{index}"
            )
            people.append(
                {
                    "id": identifier,
                    "position": position,
                    "velocity": velocity,
                    "yaw": _yaw(velocity, metadata),
                    "speed": math.hypot(velocity[0], velocity[1]),
                    "phase": metadata.get("phase", ""),
                    "generation": metadata.get("generation", ""),
                    "frame_id": str(message.header.frame_id or "map"),
                }
            )
        self.people_ready.emit(people)

    def _robot_cb(self, message) -> None:
        position = _xyz(getattr(message.pose, "pose", None))
        if position is None:
            return
        self.robot_ready.emit(
            {
                "position": position,
                "frame_id": str(message.header.frame_id or "odom"),
            }
        )

    def _anchors_cb(self, message) -> None:
        self.anchors_ready.emit(
            [
                {
                    "id": int(marker.id),
                    "x": float(marker.pose.position.x),
                    "y": float(marker.pose.position.y),
                    "color": (
                        float(marker.color.r),
                        float(marker.color.g),
                        float(marker.color.b),
                    ),
                }
                for marker in message.markers
                if int(marker.action) != 2
            ]
        )
