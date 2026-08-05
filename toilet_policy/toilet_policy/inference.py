"""ROS 2 closed-loop inference for the dual-lidar behavior-cloning policy."""

from __future__ import annotations

import json
import math
from collections import deque
from pathlib import Path

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .alignment import relative_goal
from .model import LidarGruPolicy
from .preprocessing import normalize_scan, quaternion_yaw


class ToiletPolicyInference(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "toilet_policy_inference",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("checkpoint", "")
        self.declare_parameter("front_scan_topic", "/front_scan")
        self.declare_parameter("rear_scan_topic", "/rear_scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("preview_topic", "/toilet_policy/cmd_vel_preview")
        self.declare_parameter("command_topic", "/cmd_vel_gamepad_diff")
        self.declare_parameter("diagnostics_topic", "/toilet_policy/diagnostics")
        self.declare_parameter("goal_pose", [-3.8, -0.91, 0.0])
        self.declare_parameter("closed_loop", False)
        self.declare_parameter("goal_tolerance_m", 0.35)
        self.declare_parameter("max_linear_mps", 0.4)
        self.declare_parameter("max_angular_rps", 0.8)
        self.declare_parameter("sensor_timeout_sec", 0.35)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("device", "auto")

        checkpoint_path = Path(str(self.get_parameter("checkpoint").value)).expanduser()
        if not checkpoint_path.is_file():
            raise FileNotFoundError(f"policy checkpoint does not exist: {checkpoint_path}")
        requested_device = str(self.get_parameter("device").value).strip().lower()
        if requested_device == "auto":
            requested_device = "cuda" if torch.cuda.is_available() else "cpu"
        self._device = torch.device(requested_device)
        checkpoint = torch.load(checkpoint_path, map_location=self._device, weights_only=True)
        model_config = checkpoint.get("model", {})
        self._model = LidarGruPolicy(
            state_size=int(model_config.get("state_size", 8)),
            hidden_size=int(model_config.get("hidden_size", 128)),
        ).to(self._device)
        self._model.load_state_dict(checkpoint["model_state_dict"])
        self._model.eval()
        self._sequence_length = int(checkpoint.get("sequence_length", 8))
        self._beam_count = int(checkpoint.get("beam_count", 721))
        self._goal = tuple(float(value) for value in self.get_parameter("goal_pose").value)
        if len(self._goal) != 3:
            raise ValueError("goal_pose must be [x, y, yaw]")

        self._closed_loop = bool(self.get_parameter("closed_loop").value)
        self._goal_tolerance = float(self.get_parameter("goal_tolerance_m").value)
        self._max_linear = float(self.get_parameter("max_linear_mps").value)
        self._max_angular = float(self.get_parameter("max_angular_rps").value)
        self._timeout = float(self.get_parameter("sensor_timeout_sec").value)
        self._front = None
        self._rear = None
        self._odom = None
        self._front_time = None
        self._rear_time = None
        self._odom_time = None
        self._previous_action = np.zeros(2, dtype=np.float32)
        self._scan_history = deque(maxlen=self._sequence_length)
        self._state_history = deque(maxlen=self._sequence_length)
        self._last_status = None

        self._preview_publisher = self.create_publisher(
            Twist, str(self.get_parameter("preview_topic").value), 10
        )
        self._command_publisher = self.create_publisher(
            Twist, str(self.get_parameter("command_topic").value), 10
        )
        self._diagnostics_publisher = self.create_publisher(
            String, str(self.get_parameter("diagnostics_topic").value), 10
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("front_scan_topic").value),
            self._on_front_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            str(self.get_parameter("rear_scan_topic").value),
            self._on_rear_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_odom,
            qos_profile_sensor_data,
        )
        self.create_service(SetBool, "~/set_enabled", self._set_enabled)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"Loaded policy {checkpoint_path}: device={self._device}, "
            f"sequence_length={self._sequence_length}, beams={self._beam_count}, "
            f"goal={self._goal}, closed_loop={self._closed_loop}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_front_scan(self, message: LaserScan) -> None:
        self._front = message
        self._front_time = self._now()

    def _on_rear_scan(self, message: LaserScan) -> None:
        self._rear = message
        self._rear_time = self._now()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_time = self._now()

    def _set_enabled(self, request, response):
        self.set_closed_loop(bool(request.data))
        response.success = True
        response.message = f"closed_loop={self._closed_loop}"
        self.get_logger().info(response.message)
        return response

    def set_closed_loop(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self._closed_loop and not enabled:
            self._command_publisher.publish(self._twist((0.0, 0.0)))
        self._closed_loop = enabled

    def _status(self, state: str, **fields) -> None:
        payload = {"state": state, "closed_loop": self._closed_loop, **fields}
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._diagnostics_publisher.publish(message)
        if state != self._last_status:
            self.get_logger().info(message.data)
            self._last_status = state

    def _inputs_fresh(self, now: float) -> bool:
        stamps = (self._front_time, self._rear_time, self._odom_time)
        return all(stamp is not None and 0.0 <= now - stamp <= self._timeout for stamp in stamps)

    @staticmethod
    def _twist(action) -> Twist:
        message = Twist()
        message.linear.x = float(action[0])
        message.angular.z = float(action[1])
        return message

    def _publish_command(self, action, *, command: bool) -> None:
        message = self._twist(action)
        self._preview_publisher.publish(message)
        if command and self._closed_loop:
            self._command_publisher.publish(message)

    def _stop(self, state: str) -> None:
        zero = np.zeros(2, dtype=np.float32)
        self._previous_action = zero
        self._publish_command(zero, command=True)
        self._status(state, history_size=len(self._scan_history))

    def _tick(self) -> None:
        now = self._now()
        if not self._inputs_fresh(now):
            self._scan_history.clear()
            self._state_history.clear()
            self._stop("WAITING_FOR_FRESH_INPUT")
            return
        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        yaw = quaternion_yaw(pose.orientation)
        goal_state = relative_goal(
            pose.position.x, pose.position.y, yaw, *self._goal
        )
        goal_distance = math.hypot(goal_state[0], goal_state[1])
        if goal_distance <= self._goal_tolerance:
            self._stop("GOAL_REACHED")
            return
        observation = np.stack(
            (
                normalize_scan(self._front, self._beam_count),
                normalize_scan(self._rear, self._beam_count),
            )
        )
        state = np.asarray(
            [
                *goal_state,
                float(twist.linear.x),
                float(twist.angular.z),
                *self._previous_action,
            ],
            dtype=np.float32,
        )
        self._scan_history.append(observation)
        self._state_history.append(state)
        if len(self._scan_history) < self._sequence_length:
            self._stop("WARMING_UP")
            return
        scan_tensor = torch.from_numpy(np.stack(self._scan_history)[None]).to(self._device)
        state_tensor = torch.from_numpy(np.stack(self._state_history)[None]).to(self._device)
        with torch.inference_mode():
            prediction, _ = self._model(scan_tensor, state_tensor)
        action = prediction[0, -1].detach().cpu().numpy().astype(np.float32)
        action[0] = np.clip(action[0], -self._max_linear, self._max_linear)
        action[1] = np.clip(action[1], -self._max_angular, self._max_angular)
        self._previous_action = action.copy()
        self._publish_command(action, command=True)
        self._status(
            "ACTIVE" if self._closed_loop else "PREVIEW",
            goal_distance_m=round(goal_distance, 4),
            linear_mps=round(float(action[0]), 4),
            angular_rps=round(float(action[1]), 4),
        )

    def stop(self) -> None:
        zero = np.zeros(2, dtype=np.float32)
        self._preview_publisher.publish(self._twist(zero))
        if self._closed_loop:
            self._command_publisher.publish(self._twist(zero))


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    try:
        node = ToiletPolicyInference()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
