"""Record a single-pedestrian HuNav shadow beside the live Isaac executor."""

from __future__ import annotations

import argparse
from contextlib import suppress
from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import ComputeAgents, ResetAgents
from nav_msgs.msg import Odometry
from people_msgs.msg import People
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
import yaml

from toilet_benchmark.hunav_isaac_mirror_core import (
    MotionIntent,
    normalize_angle,
    parse_motion_intent,
    person_yaw,
    point_to_polyline_distance,
    summarize_samples,
    tag_mapping,
    velocity_heading,
)
from toilet_benchmark.hunav_phase0_smoke import ManagedHuNavProcess
from toilet_benchmark.hunav_phase0_safety import (
    SafetyConfig,
    project_safe_step,
)


def _default_config_path() -> Path:
    with suppress(Exception):
        return (
            Path(get_package_share_directory("toilet_benchmark"))
            / "config"
            / "hunav_isaac_mirror.yaml"
        )
    return (
        Path(__file__).resolve().parents[1]
        / "config"
        / "hunav_isaac_mirror.yaml"
    )


def _load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError(f"mirror config must be a mapping: {path}")
    return config


def _stamp_sec(stamp) -> float:
    return float(getattr(stamp, "sec", 0)) + float(
        getattr(stamp, "nanosec", 0)
    ) * 1e-9


def _stamp_message(stamp_sec: float) -> Time:
    seconds = int(math.floor(float(stamp_sec)))
    nanoseconds = int(round((float(stamp_sec) - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    return Time(sec=seconds, nanosec=nanoseconds)


def _yaw_from_quaternion(orientation) -> float:
    x = float(orientation.x)
    y = float(orientation.y)
    z = float(orientation.z)
    w = float(orientation.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _set_agent_pose(agent: Agent, x: float, y: float, yaw: float) -> None:
    agent.position.position.x = float(x)
    agent.position.position.y = float(y)
    agent.position.position.z = 0.0
    agent.position.orientation.z = math.sin(float(yaw) * 0.5)
    agent.position.orientation.w = math.cos(float(yaw) * 0.5)
    agent.yaw = float(yaw)


class HuNavIsaacMirror(Node):
    def __init__(
        self,
        *,
        config: Mapping[str, Any],
        namespace: str,
        log_dir: Path,
    ):
        super().__init__("hunav_isaac_mirror", namespace=namespace)
        self.config = dict(config)
        self.log_dir = log_dir
        self.agent_name = str(config.get("agent_name", "toilet_agent_01"))
        self.agent_numeric_id = int(config.get("agent_numeric_id", 1))
        self.actual_topic = str(
            config.get("actual_topic", "/isaac/pedestrian_states")
        )
        self.intent_topic = str(
            config.get("intent_topic", "/toilet_benchmark/director_status")
        )
        self.robot_odom_topic = str(config.get("robot_odom_topic", "/odom"))
        mirror_cfg = config.get("mirror", {}) or {}
        self.compute_hz = max(1.0, float(mirror_cfg.get("compute_hz", 10.0)))
        self.max_actual_age_sec = max(
            0.05, float(mirror_cfg.get("max_actual_age_sec", 0.5))
        )
        self.exit_on_retire = bool(mirror_cfg.get("exit_on_retire", True))
        self.service_timeout_sec = max(
            0.1, float(mirror_cfg.get("service_timeout_sec", 2.0))
        )
        self.agent_radius = max(
            0.01, float(mirror_cfg.get("agent_radius_m", 0.30))
        )
        self.goal_radius = max(
            0.01, float(mirror_cfg.get("goal_radius_m", 0.20))
        )
        self.robot_radius = max(
            0.01, float(mirror_cfg.get("robot_radius_m", 0.45))
        )
        self.motion_heading_min_speed = max(
            0.01,
            float(mirror_cfg.get("motion_heading_min_speed_mps", 0.10)),
        )
        self.near_target_radius = max(
            self.goal_radius,
            float(mirror_cfg.get("near_target_radius_m", 0.35)),
        )
        self.settled_speed = max(
            0.0,
            float(mirror_cfg.get("settled_speed_mps", 0.05)),
        )
        self.settled_hold_sec = max(
            0.0,
            float(mirror_cfg.get("settled_hold_sec", 1.0)),
        )
        self.behavior_cfg = dict(config.get("behavior", {}) or {})
        safety_cfg = dict(config.get("safety", {}) or {})
        self.safety_enabled = bool(safety_cfg.get("enabled", True))
        self.safety_config = SafetyConfig(
            clearance_m=float(safety_cfg.get("clearance_m", 0.01)),
            contact_epsilon_m=float(
                safety_cfg.get("contact_epsilon_m", 0.0001)
            ),
            max_iterations=int(safety_cfg.get("max_iterations", 8)),
        )
        if bool(config.get("use_sim_time", True)):
            self.set_parameters(
                [Parameter("use_sim_time", Parameter.Type.BOOL, True)]
            )

        self.compute_client = self.create_client(
            ComputeAgents, "compute_agents"
        )
        self.reset_client = self.create_client(ResetAgents, "reset_agents")
        self.create_subscription(
            People, self.actual_topic, self._people_cb, 20
        )
        self.create_subscription(
            String, self.intent_topic, self._intent_cb, 20
        )
        self.create_subscription(
            Odometry, self.robot_odom_topic, self._robot_odom_cb, 20
        )
        self.create_timer(1.0 / self.compute_hz, self._tick)

        self._actual = None
        self._actual_sequence = 0
        self._last_compute_actual_sequence = -1
        self._robot = None
        self._pending_intent: MotionIntent | None = None
        self._active_intent: MotionIntent | None = None
        self._shadow: Agents | None = None
        self._reset_future = None
        self._reset_generation: int | None = None
        self._compute_future = None
        self._compute_actual = None
        self._compute_robot = None
        self._compute_previous_shadow = None
        self._compute_dt = 1.0 / self.compute_hz
        self._compute_intent_generation: int | None = None
        self._reset_robot = None
        self._safety_robot_previous = None
        self._last_shadow_stamp_sec = None
        self._segment_index = 0
        self._sample_index = 0
        self._segment_start_stamp = 0.0
        self._intent_count = 0
        self._compute_failures = 0
        self._should_stop = False
        self._samples: list[dict[str, Any]] = []
        self._trace_stream = (log_dir / "trace.jsonl").open(
            "w", encoding="utf-8"
        )
        self._event_stream = (log_dir / "events.jsonl").open(
            "w", encoding="utf-8"
        )
        self.get_logger().info(
            f"HuNav-Isaac mirror ready: agent={self.agent_name}, "
            f"actual={self.actual_topic}, intents={self.intent_topic}, "
            f"compute_hz={self.compute_hz:.1f}; "
            f"shadow mode sends no Isaac commands."
        )

    @property
    def should_stop(self) -> bool:
        return self._should_stop

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + float(timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if self.compute_client.wait_for_service(
                timeout_sec=min(0.5, remaining)
            ) and self.reset_client.wait_for_service(timeout_sec=0.1):
                return True
        return False

    def _write_event(self, payload: Mapping[str, Any]) -> None:
        record = {
            "received_monotonic_sec": time.monotonic(),
            **dict(payload),
        }
        self._event_stream.write(json.dumps(record, sort_keys=True) + "\n")
        self._event_stream.flush()

    def _people_cb(self, message: People) -> None:
        for person in message.people:
            name = str(getattr(person, "name", "") or "")
            if name != self.agent_name and not name.rstrip("/").endswith(
                "/" + self.agent_name
            ):
                continue
            reliability = float(getattr(person, "reliability", 0.0))
            if reliability <= 0.0:
                return
            tags = tag_mapping(person)
            stamp_sec = _stamp_sec(message.header.stamp)
            if stamp_sec <= 0.0:
                stamp_sec = float(self.get_clock().now().nanoseconds) * 1e-9
            self._actual = {
                "source_stamp_sec": stamp_sec,
                "received_monotonic_sec": time.monotonic(),
                "x": float(person.position.x),
                "y": float(person.position.y),
                "z": float(person.position.z),
                "vx": float(person.velocity.x),
                "vy": float(person.velocity.y),
                "vz": float(person.velocity.z),
                "yaw": person_yaw(person),
                "reliability": reliability,
                "motion_state": tags.get("motion_state", "unknown"),
                "command_generation": int(
                    tags.get("command_generation", "0") or 0
                ),
                "guard_blocked": (
                    tags.get("guard_blocked", "false").lower() == "true"
                ),
                "guard_block_reason": tags.get("guard_block_reason", ""),
            }
            self._actual_sequence += 1
            return

    def _intent_cb(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            self.get_logger().warning(
                "Ignoring malformed director status JSON."
            )
            return
        self._write_event(payload)
        if str(payload.get("agent_id", "")) != self.agent_name:
            return
        if payload.get("event") == "pedestrian_retiring":
            if self.exit_on_retire:
                self._should_stop = True
            return
        try:
            intent = parse_motion_intent(payload)
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(f"Ignoring invalid motion intent: {exc}")
            return
        if intent is None:
            return
        latest_generation = max(
            self._pending_intent.generation if self._pending_intent else 0,
            self._active_intent.generation if self._active_intent else 0,
        )
        if intent.generation <= latest_generation:
            return
        self._intent_count += 1
        self._pending_intent = intent
        if not intent.shadow_enabled:
            self._active_intent = None
            self._shadow = None
            self.get_logger().info(
                f"Mirror observed non-walking intent generation "
                f"{intent.generation}; shadow paused."
            )
            return
        self.get_logger().info(
            f"Mirror queued intent generation {intent.generation}: "
            f"phase={intent.phase}, goals={len(intent.goals())}, "
            f"velocity={intent.velocity:.2f}"
        )

    def _robot_odom_cb(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        yaw = _yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        velocity_x = (
            cos_yaw * float(twist.linear.x)
            - sin_yaw * float(twist.linear.y)
        )
        velocity_y = (
            sin_yaw * float(twist.linear.x)
            + cos_yaw * float(twist.linear.y)
        )
        self._robot = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
            "vx": velocity_x,
            "vy": velocity_y,
            "wz": float(twist.angular.z),
            "source_stamp_sec": _stamp_sec(message.header.stamp),
            "received_monotonic_sec": time.monotonic(),
        }

    def _actual_is_fresh(self) -> bool:
        return (
            self._actual is not None
            and time.monotonic() - self._actual["received_monotonic_sec"]
            <= self.max_actual_age_sec
        )

    def _build_agent(self, intent: MotionIntent) -> Agent:
        actual = self._actual
        first_goal = intent.goals()[0]
        yaw = actual["yaw"]
        if yaw is None:
            yaw = math.atan2(
                first_goal[1] - actual["y"],
                first_goal[0] - actual["x"],
            )
        agent = Agent()
        agent.id = self.agent_numeric_id
        agent.type = Agent.PERSON
        agent.name = self.agent_name
        agent.group_id = -1
        _set_agent_pose(agent, actual["x"], actual["y"], yaw)
        agent.velocity.linear.x = float(actual["vx"])
        agent.velocity.linear.y = float(actual["vy"])
        agent.linear_vel = math.hypot(actual["vx"], actual["vy"])
        agent.desired_velocity = float(intent.velocity)
        agent.radius = self.agent_radius
        agent.goal_radius = self.goal_radius
        agent.cyclic_goals = False
        goals = []
        for point in intent.goals():
            goal = Pose()
            goal.position.x = float(point[0])
            goal.position.y = float(point[1])
            goal.position.z = 0.0
            goal.orientation.w = 1.0
            goals.append(goal)
        agent.goals = goals
        agent.closest_obs = []
        agent.behavior = self._build_behavior(intent.velocity)
        return agent

    def _build_behavior(self, velocity: float) -> AgentBehavior:
        behavior = AgentBehavior()
        behavior.type = AgentBehavior.BEH_REGULAR
        behavior.state = AgentBehavior.BEH_NO_ACTIVE
        behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
        behavior.duration = 0.0
        behavior.once = False
        behavior.vel = float(velocity)
        behavior.dist = 1.0
        behavior.social_force_factor = float(
            self.behavior_cfg.get("social_force_factor", 5.0)
        )
        behavior.goal_force_factor = float(
            self.behavior_cfg.get("goal_force_factor", 2.0)
        )
        behavior.obstacle_force_factor = float(
            self.behavior_cfg.get("obstacle_force_factor", 10.0)
        )
        behavior.other_force_factor = float(
            self.behavior_cfg.get("other_force_factor", 20.0)
        )
        return behavior

    def _robot_snapshot(self) -> dict[str, Any]:
        if self._robot is None:
            return {
                "available": False,
                "age_sec": None,
                "source_stamp_sec": None,
                "x": 100.0,
                "y": 100.0,
                "yaw": 0.0,
                "vx": 0.0,
                "vy": 0.0,
                "wz": 0.0,
            }
        state = deepcopy(self._robot)
        state["available"] = True
        state["age_sec"] = max(
            0.0,
            time.monotonic() - float(state["received_monotonic_sec"]),
        )
        state.pop("received_monotonic_sec", None)
        return state

    def _build_robot(self, state: Mapping[str, Any]) -> Agent:
        state = state or {
            "x": 100.0,
            "y": 100.0,
            "yaw": 0.0,
            "vx": 0.0,
            "vy": 0.0,
            "wz": 0.0,
        }
        robot = Agent()
        robot.id = 0
        robot.type = Agent.ROBOT
        robot.name = "mirror_robot"
        robot.group_id = -1
        _set_agent_pose(robot, state["x"], state["y"], state["yaw"])
        robot.velocity.linear.x = state["vx"]
        robot.velocity.linear.y = state["vy"]
        robot.velocity.angular.z = state["wz"]
        robot.desired_velocity = math.hypot(state["vx"], state["vy"])
        robot.radius = self.robot_radius
        return robot

    def _begin_pending_intent(self) -> None:
        intent = self._pending_intent
        if (
            intent is None
            or not intent.shadow_enabled
            or not self._actual_is_fresh()
        ):
            return
        shadow = Agents()
        shadow.header.frame_id = "map"
        shadow.header.stamp = _stamp_message(
            self._actual["source_stamp_sec"]
        )
        shadow.agents = [self._build_agent(intent)]
        request = ResetAgents.Request()
        request.current_agents = shadow
        robot = self._robot_snapshot()
        request.robot = self._build_robot(robot)
        self._reset_robot = robot
        self._reset_generation = intent.generation
        self._reset_future = self.reset_client.call_async(request)
        self._reset_future.add_done_callback(self._reset_done)
        self._shadow = shadow

    def _reset_done(self, future) -> None:
        generation = self._reset_generation
        reset_robot = self._reset_robot
        self._reset_future = None
        self._reset_generation = None
        self._reset_robot = None
        try:
            accepted = bool(future.result().ok)
        except Exception as exc:
            self._compute_failures += 1
            self.get_logger().error(f"HuNav reset failed: {exc}")
            return
        if not accepted:
            self._compute_failures += 1
            self.get_logger().error("HuNav reset rejected the mirror state.")
            return
        if (
            self._pending_intent is None
            or self._pending_intent.generation != generation
        ):
            self._shadow = None
            return
        self._active_intent = self._pending_intent
        self._pending_intent = None
        self._segment_index += 1
        self._segment_start_stamp = float(self._actual["source_stamp_sec"])
        self._last_shadow_stamp_sec = self._segment_start_stamp
        self._safety_robot_previous = deepcopy(reset_robot)
        self._last_compute_actual_sequence = -1
        self.get_logger().info(
            f"Mirror segment {self._segment_index} started for intent "
            f"generation {generation}."
        )

    def _restore_shadow_fields(self, agent: Agent) -> None:
        intent = self._active_intent
        agent.desired_velocity = float(intent.velocity)
        agent.radius = self.agent_radius
        agent.goal_radius = self.goal_radius
        agent.cyclic_goals = False
        agent.closest_obs = []
        agent.behavior = self._build_behavior(intent.velocity)

    def _dispatch_compute(self) -> None:
        self._shadow.header.frame_id = "map"
        self._shadow.header.stamp = _stamp_message(
            self._actual["source_stamp_sec"]
        )
        for agent in self._shadow.agents:
            self._restore_shadow_fields(agent)
        request = ComputeAgents.Request()
        request.current_agents = self._shadow
        robot = self._robot_snapshot()
        request.robot = self._build_robot(robot)
        stamp_sec = float(self._actual["source_stamp_sec"])
        if self._last_shadow_stamp_sec is None:
            compute_dt = 1.0 / self.compute_hz
        else:
            compute_dt = max(
                1e-3,
                stamp_sec - float(self._last_shadow_stamp_sec),
            )
        self._last_compute_actual_sequence = self._actual_sequence
        self._compute_actual = deepcopy(self._actual)
        self._compute_robot = robot
        self._compute_previous_shadow = deepcopy(self._shadow.agents[0])
        self._compute_dt = compute_dt
        self._compute_intent_generation = self._active_intent.generation
        self._compute_future = self.compute_client.call_async(request)
        self._compute_future.add_done_callback(self._compute_done)

    def _compute_done(self, future) -> None:
        self._compute_future = None
        actual = self._compute_actual
        robot = self._compute_robot
        previous_shadow = self._compute_previous_shadow
        compute_dt = self._compute_dt
        intent_generation = self._compute_intent_generation
        self._compute_actual = None
        self._compute_robot = None
        self._compute_previous_shadow = None
        self._compute_intent_generation = None
        try:
            updated = future.result().updated_agents
        except Exception as exc:
            self._compute_failures += 1
            self.get_logger().error(f"HuNav compute failed: {exc}")
            return
        if (
            self._active_intent is None
            or self._active_intent.generation != intent_generation
            or self._pending_intent is not None
            or actual is None
            or not updated.agents
        ):
            return
        shadow_agent = updated.agents[0]
        if not shadow_agent.goals:
            _set_agent_pose(
                shadow_agent,
                shadow_agent.position.position.x,
                shadow_agent.position.position.y,
                self._active_intent.final_yaw,
            )
            shadow_agent.velocity.linear.x = 0.0
            shadow_agent.velocity.linear.y = 0.0
            shadow_agent.linear_vel = 0.0
        raw_shadow_agent = deepcopy(shadow_agent)
        safety_diagnostics = self._apply_hard_safety(
            shadow_agent,
            previous_shadow=previous_shadow,
            robot=robot,
            dt=compute_dt,
        )
        self._shadow = updated
        self._last_shadow_stamp_sec = float(actual["source_stamp_sec"])
        self._safety_robot_previous = deepcopy(robot)
        self._record_sample(
            shadow_agent,
            raw_shadow_agent,
            actual,
            robot,
            safety_diagnostics,
        )

    def _record_sample(
        self,
        shadow: Agent,
        raw_shadow: Agent,
        actual: Mapping[str, Any],
        robot: Mapping[str, Any] | None,
        safety_diagnostics: Mapping[str, Any],
    ) -> None:
        if self._active_intent is None:
            return
        dx = float(shadow.position.position.x) - float(actual["x"])
        dy = float(shadow.position.position.y) - float(actual["y"])
        actual_yaw = actual.get("yaw")
        yaw_error = (
            None
            if actual_yaw is None
            else normalize_angle(float(shadow.yaw) - float(actual_yaw))
        )
        actual_heading = velocity_heading(
            actual["vx"],
            actual["vy"],
            min_speed=self.motion_heading_min_speed,
        )
        shadow_heading = velocity_heading(
            shadow.velocity.linear.x,
            shadow.velocity.linear.y,
            min_speed=self.motion_heading_min_speed,
        )
        motion_heading_error = (
            None
            if actual_heading is None or shadow_heading is None
            else normalize_angle(shadow_heading - actual_heading)
        )
        actual_root_to_motion_offset = (
            None
            if actual_yaw is None or actual_heading is None
            else normalize_angle(float(actual_yaw) - actual_heading)
        )
        shadow_root_to_motion_offset = (
            None
            if shadow_heading is None
            else normalize_angle(float(shadow.yaw) - shadow_heading)
        )
        route = self._active_intent.goals()
        target_x = self._active_intent.goal_pose[0]
        target_y = self._active_intent.goal_pose[1]
        actual_target_distance = math.hypot(
            float(actual["x"]) - target_x,
            float(actual["y"]) - target_y,
        )
        hunav_target_distance = math.hypot(
            float(shadow.position.position.x) - target_x,
            float(shadow.position.position.y) - target_y,
        )
        raw_target_distance = math.hypot(
            float(raw_shadow.position.position.x) - target_x,
            float(raw_shadow.position.position.y) - target_y,
        )
        interaction = self._interaction_metrics(
            shadow, raw_shadow, actual, robot
        )
        self._sample_index += 1
        sample = {
            "sample_index": self._sample_index,
            "segment_index": self._segment_index,
            "intent_generation": self._active_intent.generation,
            "phase": self._active_intent.phase,
            "source_stamp_sec": actual["source_stamp_sec"],
            "segment_elapsed_sec": max(
                0.0,
                float(actual["source_stamp_sec"]) - self._segment_start_stamp,
            ),
            "position_error_m": math.hypot(dx, dy),
            "yaw_error_rad": yaw_error,
            "root_yaw_error_rad": yaw_error,
            "motion_heading_error_rad": motion_heading_error,
            "actual_motion_heading_rad": actual_heading,
            "hunav_motion_heading_rad": shadow_heading,
            "actual_root_to_motion_offset_rad": actual_root_to_motion_offset,
            "hunav_root_to_motion_offset_rad": shadow_root_to_motion_offset,
            "actual_target_yaw_error_rad": (
                None
                if actual_yaw is None
                else normalize_angle(
                    float(actual_yaw) - self._active_intent.final_yaw
                )
            ),
            "hunav_target_yaw_error_rad": normalize_angle(
                float(shadow.yaw) - self._active_intent.final_yaw
            ),
            "actual_cross_track_error_m": point_to_polyline_distance(
                actual["x"], actual["y"], route
            ),
            "hunav_cross_track_error_m": point_to_polyline_distance(
                shadow.position.position.x,
                shadow.position.position.y,
                route,
            ),
            "actual_target_distance_m": actual_target_distance,
            "hunav_target_distance_m": hunav_target_distance,
            "hunav_raw_target_distance_m": raw_target_distance,
            "actual_near_target": (
                actual_target_distance <= self.near_target_radius
            ),
            "hunav_near_target": (
                hunav_target_distance <= self.near_target_radius
            ),
            "hunav_raw_near_target": (
                raw_target_distance <= self.near_target_radius
            ),
            "actual": {
                key: actual[key]
                for key in (
                    "x",
                    "y",
                    "z",
                    "vx",
                    "vy",
                    "vz",
                    "yaw",
                    "motion_state",
                    "command_generation",
                    "guard_blocked",
                    "guard_block_reason",
                )
            },
            "hunav": {
                "x": float(shadow.position.position.x),
                "y": float(shadow.position.position.y),
                "yaw": float(shadow.yaw),
                "vx": float(shadow.velocity.linear.x),
                "vy": float(shadow.velocity.linear.y),
                "remaining_goals": len(shadow.goals),
            },
            "hunav_raw": {
                "x": float(raw_shadow.position.position.x),
                "y": float(raw_shadow.position.position.y),
                "yaw": float(raw_shadow.yaw),
                "vx": float(raw_shadow.velocity.linear.x),
                "vy": float(raw_shadow.velocity.linear.y),
                "remaining_goals": len(raw_shadow.goals),
            },
            "hunav_raw_cross_track_error_m": point_to_polyline_distance(
                raw_shadow.position.position.x,
                raw_shadow.position.position.y,
                route,
            ),
            "target": {
                "x": target_x,
                "y": target_y,
                "yaw": self._active_intent.final_yaw,
            },
            "robot": dict(robot or {"available": False}),
            "interaction": interaction,
            "safety": dict(safety_diagnostics),
        }
        self._samples.append(sample)
        self._trace_stream.write(json.dumps(sample, sort_keys=True) + "\n")
        self._trace_stream.flush()

    def _interaction_metrics(
        self,
        shadow: Agent,
        raw_shadow: Agent,
        actual: Mapping[str, Any],
        robot: Mapping[str, Any] | None,
    ) -> dict[str, float | None]:
        if not robot or not robot.get("available"):
            return {
                "actual_robot_center_distance_m": None,
                "actual_robot_clearance_m": None,
                "hunav_robot_center_distance_m": None,
                "hunav_robot_clearance_m": None,
                "hunav_raw_robot_center_distance_m": None,
                "hunav_raw_robot_clearance_m": None,
            }
        combined_radius = self.agent_radius + self.robot_radius
        actual_distance = math.hypot(
            float(actual["x"]) - float(robot["x"]),
            float(actual["y"]) - float(robot["y"]),
        )
        hunav_distance = math.hypot(
            float(shadow.position.position.x) - float(robot["x"]),
            float(shadow.position.position.y) - float(robot["y"]),
        )
        hunav_raw_distance = math.hypot(
            float(raw_shadow.position.position.x) - float(robot["x"]),
            float(raw_shadow.position.position.y) - float(robot["y"]),
        )
        return {
            "actual_robot_center_distance_m": actual_distance,
            "actual_robot_clearance_m": actual_distance - combined_radius,
            "hunav_robot_center_distance_m": hunav_distance,
            "hunav_robot_clearance_m": hunav_distance - combined_radius,
            "hunav_raw_robot_center_distance_m": hunav_raw_distance,
            "hunav_raw_robot_clearance_m": (
                hunav_raw_distance - combined_radius
            ),
        }

    def _apply_hard_safety(
        self,
        shadow: Agent,
        *,
        previous_shadow: Agent | None,
        robot: Mapping[str, Any] | None,
        dt: float,
    ) -> dict[str, Any]:
        disabled = {
            "enabled": self.safety_enabled,
            "intervened": False,
            "correction_count": 0,
            "pedestrian_pair_contacts": 0,
            "robot_contacts": 0,
            "static_contacts": 0,
            "max_correction_m": 0.0,
            "constrained_agents": [],
            "residual_violation_count": 0,
        }
        if not self.safety_enabled or previous_shadow is None:
            return disabled

        agent_id = int(shadow.id)
        previous = {
            agent_id: (
                float(previous_shadow.position.position.x),
                float(previous_shadow.position.position.y),
            )
        }
        proposed = {
            agent_id: (
                float(shadow.position.position.x),
                float(shadow.position.position.y),
            )
        }
        current_robot = robot or {}
        previous_robot = self._safety_robot_previous or current_robot
        robot_available = bool(current_robot.get("available"))
        previous_robot_available = bool(previous_robot.get("available"))
        if robot_available:
            robot_proposed = (
                float(current_robot["x"]),
                float(current_robot["y"]),
            )
            if previous_robot_available:
                robot_previous = (
                    float(previous_robot["x"]),
                    float(previous_robot["y"]),
                )
            else:
                robot_previous = robot_proposed
            robot_radius = self.robot_radius
        else:
            robot_previous = (100.0, 100.0)
            robot_proposed = robot_previous
            robot_radius = 0.0

        result = project_safe_step(
            previous=previous,
            proposed=proposed,
            radii={agent_id: self.agent_radius},
            robot_previous=robot_previous,
            robot_proposed=robot_proposed,
            robot_radius=robot_radius,
            static_obstacles={agent_id: ()},
            config=self.safety_config,
        )
        safe_x, safe_y = result.positions[agent_id]
        safe_dt = max(1e-3, float(dt))
        safe_vx = (safe_x - previous[agent_id][0]) / safe_dt
        safe_vy = (safe_y - previous[agent_id][1]) / safe_dt
        _set_agent_pose(shadow, safe_x, safe_y, shadow.yaw)
        shadow.velocity.linear.x = safe_vx
        shadow.velocity.linear.y = safe_vy
        shadow.linear_vel = math.hypot(safe_vx, safe_vy)
        return {
            "enabled": True,
            **result.diagnostics.to_dict(),
        }

    def _tick(self) -> None:
        if self._reset_future is not None or self._compute_future is not None:
            return
        if self._pending_intent is not None:
            self._begin_pending_intent()
            return
        if (
            self._active_intent is None
            or self._shadow is None
            or not self._actual_is_fresh()
            or self._actual_sequence == self._last_compute_actual_sequence
        ):
            return
        self._dispatch_compute()

    def finalize(self) -> dict[str, Any]:
        self._trace_stream.close()
        self._event_stream.close()
        result = summarize_samples(
            self._samples,
            intent_count=self._intent_count,
            compute_failures=self._compute_failures,
            near_target_radius_m=self.near_target_radius,
            settled_speed_mps=self.settled_speed,
            settled_hold_sec=self.settled_hold_sec,
        )
        result.update(
            {
                "agent_name": self.agent_name,
                "mode": "shadow",
                "isaac_motion_commands_sent": 0,
                "trace_path": str(self.log_dir / "trace.jsonl"),
                "events_path": str(self.log_dir / "events.jsonl"),
            }
        )
        (self.log_dir / "result.json").write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        return result


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=_default_config_path())
    parser.add_argument("--agent-name", default="")
    parser.add_argument("--log-dir", type=Path, default=None)
    parser.add_argument("--namespace", default="")
    parser.add_argument("--no-start-manager", action="store_true")
    return parser.parse_args(argv)


def main(args: Sequence[str] | None = None) -> int:
    options = _parse_args(
        list(args) if args is not None else remove_ros_args(sys.argv)[1:]
    )
    config = _load_config(options.config)
    if options.agent_name:
        config["agent_name"] = options.agent_name
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    log_dir = options.log_dir or Path("/tmp/toilet_hunav_mirror") / run_id
    log_dir.mkdir(parents=True, exist_ok=False)
    ros_log_dir = log_dir / "ros_logs"
    ros_log_dir.mkdir()
    os.environ.setdefault("ROS_LOG_DIR", str(ros_log_dir))
    namespace = (
        options.namespace.strip() or f"/toilet_hunav_mirror_{os.getpid()}"
    )
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    manifest = {
        "run_id": run_id,
        "config_path": str(options.config.resolve()),
        "namespace": namespace,
        "manager_started_by_mirror": not options.no_start_manager,
        "config": config,
    }
    (log_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    manager = ManagedHuNavProcess(namespace, log_dir / "hunav.log")
    node = None
    exit_code = 1
    try:
        if not options.no_start_manager:
            manager.start()
        rclpy.init(args=None)
        node = HuNavIsaacMirror(
            config=config,
            namespace=namespace,
            log_dir=log_dir,
        )
        startup_timeout = float(
            (config.get("mirror", {}) or {}).get(
                "startup_timeout_sec", 15.0
            )
        )
        if not node.wait_ready(startup_timeout):
            manager.assert_running()
            raise TimeoutError(
                f"HuNav services did not become ready in namespace {namespace}"
            )
        node.get_logger().info(f"Mirror output directory: {log_dir}")
        while rclpy.ok() and not node.should_stop:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = 0
    except KeyboardInterrupt:
        exit_code = 130
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"HuNav-Isaac mirror failed: {exc}")
        else:
            print(f"HuNav-Isaac mirror failed: {exc}", file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            result = node.finalize()
            node.get_logger().info(
                f"Mirror complete: samples={result['sample_count']}, "
                f"segments={result['segment_count']}, "
                f"result={log_dir / 'result.json'}"
            )
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not options.no_start_manager:
            manager.stop()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
