from __future__ import annotations

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from isaacsim_msgs.srv import DeletePrim, ResetRobot
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .collection_scenarios import ScenarioSelection, ScenarioSelector, load_manual_collection_config
from .domain.events import decode_json_payload
from .episode_recorder import EpisodeRecorder


_AUTHORED_SCENARIO_NODE_NAME = "toilet_authored_scenario"
_AUTHORED_CANCEL_SERVICE = "/toilet_authored_scenario/cancel"


def _default_config_path() -> str:
    return os.path.join(get_package_share_directory("toilet_benchmark"), "config", "manual_collection.yaml")


def _build_authored_scenario_command(
    *,
    episode_path: str,
    status_topic: str = "/toilet_benchmark/pedestrian_runtime_status",
    agent_id_suffix: str = "",
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "toilet_benchmark.tracks.authored_scenario",
        "--episode",
        str(episode_path),
        "--skip-robot-reset",
        "--status-topic",
        status_topic,
        "--pedestrian-robot-policy",
        "detect_and_fail",
    ]
    if agent_id_suffix:
        command.extend(["--agent-id-suffix", str(agent_id_suffix)])
    return command


def _yaw_from_quaternion(quaternion) -> float:
    w = float(quaternion.w)
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _angle_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(float(a) - float(b)), math.cos(float(a) - float(b))))


def _planar_distance(a, b) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _authored_runtime_nodes(nodes: list[tuple[str, str]]) -> tuple[str, ...]:
    return tuple(
        sorted(
            f"{namespace.rstrip('/')}/{name}" if namespace != "/" else f"/{name}"
            for name, namespace in nodes
            if name == _AUTHORED_SCENARIO_NODE_NAME
        )
    )


class _ShutdownSignalLatch:
    """Record a process signal without running ROS cleanup in its handler."""

    def __init__(self) -> None:
        self.reason: str | None = None

    def handle(self, signum: int, _frame) -> None:
        if self.reason is not None:
            return
        self.reason = "operator_interrupt" if signum == signal.SIGINT else "termination_signal"


class ManualCollectionNode(Node):
    """Continuous manual collection coordinator for one or more pedestrians."""

    def __init__(self, config_path: str, *, record_bag: bool | None = None):
        super().__init__("toilet_manual_collection")
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.raw_config = yaml.safe_load(Path(self.config_path).read_text(encoding="utf-8")) or {}
        self.config = load_manual_collection_config(self.raw_config)
        unsupported = [
            scenario.id
            for scenario in self.config.scenarios
            if scenario.source_mode != "authored_route"
        ]
        if unsupported:
            raise ValueError(
                "manual collection only supports authored_route scenarios; "
                f"unsupported scenarios: {unsupported}"
            )
        self.selector = ScenarioSelector(
            self.config.scenarios,
            selection_mode=self.config.session.selection_mode,
            seed=self.config.session.seed,
            fixed_scenario_id=self.config.session.fixed_scenario_id,
        )
        session_id = (
            datetime.now().strftime("session_%Y%m%d_%H%M%S_%f")
            + f"_seed{self.config.session.seed}"
        )
        self._incarnation_session_token = session_id
        recording_cfg = self.raw_config.get("recording", {}) or {}
        recording_enabled = bool(recording_cfg.get("enabled", True)) if record_bag is None else bool(record_bag)
        topics = [str(topic) for topic in recording_cfg.get("topics", []) if str(topic).strip()]
        command_prefix = [str(value) for value in recording_cfg.get("command", ["ros2", "bag", "record"])]

        def _bag_command(context):
            if not recording_enabled:
                return None
            return [*command_prefix, "-o", "rosbag2", *topics]

        self.recorder = EpisodeRecorder.from_config(
            self.config,
            session_id=session_id,
            rosbag_command_factory=_bag_command,
        )
        self.recorder.prepare_session()

        robot_cfg = self.raw_config.get("robot", {}) or {}
        pedestrian_cfg = self.raw_config.get("pedestrian", {}) or {}
        episode_cfg = self.raw_config.get("episode", {}) or {}
        session_cfg = self.raw_config.get("session", {}) or {}
        events_cfg = self.raw_config.get("events", {}) or {}
        collision_cfg = self.raw_config.get("collision_policy", {}) or {}
        self.robot_name = str(robot_cfg.get("name", "xms_mecanum"))
        self.reset_service_name = str(robot_cfg.get("reset_service", "/isaac/reset_mecanum_episode"))
        self.control_hold_service_name = str(
            robot_cfg.get("control_hold_service", "/isaac/set_mecanum_control_hold")
        )
        self.hard_guard_service_name = str(
            collision_cfg.get("hard_guard_service", "/isaac/set_pedestrian_hard_guard")
        )
        self.pedestrian_robot_collision_mode = str(
            collision_cfg.get("pedestrian_robot", "detect_and_fail")
        ).strip().lower()
        if self.pedestrian_robot_collision_mode not in {"detect_and_fail", "hard_guard"}:
            raise ValueError(
                "collision_policy.pedestrian_robot must be detect_and_fail or hard_guard"
            )
        self.odom_topic = str(robot_cfg.get("odom_topic", "/odom"))
        self.recording_enabled = recording_enabled
        self.bag_ready_timeout_sec = max(2.0, float(recording_cfg.get("ready_timeout_sec", 10.0)))
        self.bag_ready_topics = tuple(
            str(topic)
            for topic in recording_cfg.get(
                "ready_topics",
                [self.odom_topic, "/cmd_vel_applied", "/front_scan", "/rear_scan"],
            )
            if str(topic).strip()
        )
        self.parking_service_name = str(pedestrian_cfg.get("parking_service", "/isaac/delete_prim"))
        self.inter_episode_delay_sec = max(0.0, float(session_cfg.get("inter_episode_delay_sec", 1.0)))
        self.parking_timeout_sec = max(1.0, float(session_cfg.get("parking_timeout_sec", 5.0)))
        self.authored_runtime_exit_timeout_sec = max(
            5.0,
            float(session_cfg.get("authored_runtime_exit_timeout_sec", 60.0)),
        )
        self.settle_min_stable_sec = max(0.0, float(episode_cfg.get("settle_min_stable_sec", 0.6)))
        self.settle_position_tolerance_m = max(
            0.01, float(episode_cfg.get("settle_position_tolerance_m", 0.08))
        )
        self.settle_yaw_tolerance_rad = max(
            0.01, float(episode_cfg.get("settle_yaw_tolerance_rad", 0.12))
        )
        self.pedestrian_start_timeout_sec = max(
            2.0, float(episode_cfg.get("pedestrian_start_timeout_sec", 20.0))
        )
        self.pedestrian_status_topic = str(
            events_cfg.get(
                "pedestrian_status_topic",
                "/toilet_benchmark/pedestrian_runtime_status",
            )
        )

        self._reset_client = self.create_client(ResetRobot, self.reset_service_name)
        self._control_hold_client = self.create_client(SetBool, self.control_hold_service_name)
        self._hard_guard_control_client = self.create_client(SetBool, self.hard_guard_service_name)
        self._parking_client = self.create_client(DeletePrim, self.parking_service_name)
        self._authored_cancel_client = self.create_client(Trigger, _AUTHORED_CANCEL_SERVICE)
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self._hard_guard_sub = self.create_subscription(
            String,
            str(events_cfg.get("hard_guard_topic", "/isaac/pedestrian_hard_guard_events")),
            self._hard_guard_cb,
            20,
        )
        self._pedestrian_contact_sub = self.create_subscription(
            String,
            str(
                events_cfg.get(
                    "pedestrian_contact_topic",
                    "/isaac/pedestrian_contact_events",
                )
            ),
            self._pedestrian_contact_cb,
            20,
        )
        self._scene_collision_sub = self.create_subscription(
            String,
            str(events_cfg.get("scene_collision_topic", "/isaac/scene_collision_events")),
            self._scene_collision_cb,
            20,
        )
        self._reset_status_sub = self.create_subscription(
            String,
            "/isaac/mecanum_reset_status",
            self._reset_status_cb,
            20,
        )
        self._pedestrian_status_sub = self.create_subscription(
            String,
            self.pedestrian_status_topic,
            self._pedestrian_status_cb,
            20,
        )
        self._status_pub = self.create_publisher(String, "/toilet_benchmark/collection_status", 10)
        self._timer = self.create_timer(0.1, self._tick)

        self._state = "WAIT_SERVICES"
        self._state_started_at = time.monotonic()
        self._next_episode_at = 0.0
        self._selection: ScenarioSelection | None = None
        self._latest_odom = None
        self._latest_odom_at = 0.0
        self._settle_stable_since = None
        self._expected_reset_generation = 0
        self._reset_status_by_generation: dict[int, dict] = {}
        self._episode_started_at = 0.0
        self._pedestrian_process: subprocess.Popen | None = None
        self._active_pedestrian_generations: dict[str, int] = {}
        self._runtime_agent_ids: tuple[str, ...] = ()
        self._incarnation_counter = 0
        self._episodes_finished = 0
        self._finishing = False
        self._pending_parking: set[str] = set()
        self._parking_inflight: set[str] = set()
        self._parking_failures: list[str] = []
        self._next_parking_retry_at = 0.0
        self._control_hold_enabled = False
        self._shutdown_requested = False
        self._last_status = None
        self.get_logger().info(
            f"Manual collection ready: config={self.config_path}, session={session_id}, "
            f"mode={self.selector.mode}, seed={self.selector.seed}, recording={recording_enabled}, "
            f"scenario_source=authored_route, "
            f"pedestrian_robot_collision={self.pedestrian_robot_collision_mode}"
        )

    def _active_agent_ids(self) -> tuple[str, ...]:
        runtime_ids = tuple(getattr(self, "_runtime_agent_ids", ()))
        if runtime_ids:
            return runtime_ids
        if self._selection is None:
            return ()
        return self._selection.scenario.pedestrian_agent_ids

    def _active_pedestrian_count(self) -> int:
        return len(self._active_agent_ids())

    def _set_state(self, state: str) -> None:
        self._state = str(state)
        self._state_started_at = time.monotonic()
        self._publish_status(force=True)

    def _publish_status(self, *, force: bool = False) -> None:
        scenario_id = self._selection.scenario.id if self._selection is not None else None
        payload = {
            "state": self._state,
            "scenario_id": scenario_id,
            "episodes_finished": self._episodes_finished,
            "coverage": self.selector.coverage_counts,
            "pedestrian_count": self._active_pedestrian_count(),
        }
        encoded = json.dumps(payload, sort_keys=True)
        if force or encoded != self._last_status:
            self._last_status = encoded
            self._status_pub.publish(String(data=encoded))

    def _odom_cb(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        self._latest_odom = (
            float(pose.position.x),
            float(pose.position.y),
            float(pose.position.z),
            _yaw_from_quaternion(pose.orientation),
            math.hypot(float(twist.linear.x), float(twist.linear.y)),
            abs(float(twist.angular.z)),
        )
        self._latest_odom_at = time.monotonic()

    def _event_payload(self, message: String) -> dict:
        return decode_json_payload(message.data)

    def _hard_guard_cb(self, message: String) -> None:
        if self._state != "RUNNING":
            return
        self.recorder.record_event("hard_guard_intervention", self._event_payload(message))

    def _pedestrian_contact_cb(self, message: String) -> None:
        if self._state != "RUNNING" or self._finishing:
            return
        payload = self._event_payload(message)
        if str(payload.get("robot", "")) != self.robot_name:
            return
        self.get_logger().error(
            "Robot-human contact detected; failing and holding this episode: "
            f"pedestrian={payload.get('pedestrian')}, "
            f"penetration_m={float(payload.get('penetration_m', 0.0)):.4f}."
        )
        self.recorder.record_event("robot_human_collision", payload)
        self._finish_episode(
            "failed",
            "robot_human_collision",
            extra={"collision": payload},
            abort_pedestrian_runtime=True,
        )

    def _scene_collision_cb(self, message: String) -> None:
        if self._state != "RUNNING" or self._finishing:
            return
        payload = self._event_payload(message)
        self.recorder.record_event("scene_collision", payload)
        self._finish_episode(
            "failed",
            "scene_collision",
            extra={"collision": payload},
            abort_pedestrian_runtime=True,
        )

    def _reset_status_cb(self, message: String) -> None:
        payload = self._event_payload(message)
        if str(payload.get("robot", "")) != self.robot_name:
            return
        generation = int(payload.get("generation", 0) or 0)
        if generation <= 0:
            return
        self._reset_status_by_generation[generation] = payload
        if self._state == "WAIT_RESET_APPLIED" and generation == self._expected_reset_generation:
            self._handle_reset_status(payload)

    def _handle_reset_status(self, payload: dict) -> None:
        status = str(payload.get("status", ""))
        if status == "applied":
            self.get_logger().info(
                f"Robot reset generation {self._expected_reset_generation} applied; waiting for stable odometry."
            )
            self._set_state("SETTLING")
        elif status == "failed":
            self._finish_episode(
                "failed",
                "reset_apply_failed",
                episode_started=False,
                extra={"reset_status": payload},
            )

    def _pedestrian_status_cb(self, message: String) -> None:
        if self._state != "WAIT_PEDESTRIAN":
            return
        payload = self._event_payload(message)
        if (
            payload.get("event") != "pedestrian_active"
            or payload.get("agent_id") not in self._active_agent_ids()
        ):
            return
        if self._selection is None:
            return
        scenario = self._selection.scenario
        active_agent_id = str(payload.get("agent_id"))
        generation = int(payload.get("generation", 0) or 0)
        if generation <= 0:
            return
        self._active_pedestrian_generations[active_agent_id] = generation
        expected_agents = set(self._active_agent_ids())
        if set(self._active_pedestrian_generations) != expected_agents:
            self.get_logger().info(
                f"Pedestrian {active_agent_id} generation {generation} is ready; "
                f"waiting for {sorted(expected_agents - set(self._active_pedestrian_generations))}."
            )
            return
        self.get_logger().info(
            f"All pedestrians are active from {scenario.source_mode}: "
            f"generations={self._active_pedestrian_generations}; releasing operator control."
        )
        self._request_control_release()

    def _tick(self) -> None:
        if self._shutdown_requested:
            return
        now = time.monotonic()
        self._publish_status()
        if self._state == "WAIT_SERVICES":
            if (
                self._reset_client.wait_for_service(timeout_sec=0.0)
                and self._control_hold_client.wait_for_service(timeout_sec=0.0)
                and self._hard_guard_control_client.wait_for_service(timeout_sec=0.0)
                and self._parking_client.wait_for_service(timeout_sec=0.0)
            ):
                request = SetBool.Request()
                request.data = self.pedestrian_robot_collision_mode == "hard_guard"
                future = self._hard_guard_control_client.call_async(request)
                future.add_done_callback(self._hard_guard_policy_done)
                self._set_state("WAIT_GUARD_POLICY")
            return
        if self._state == "WAIT_GUARD_POLICY":
            if now - self._state_started_at > self.bag_ready_timeout_sec:
                self.request_shutdown("hard_guard_policy_timeout")
            return
        if self._state == "BETWEEN_EPISODES":
            if now >= self._next_episode_at:
                self._begin_episode()
            return
        if self._state == "WAIT_PEDESTRIAN_PARK":
            if now - self._state_started_at > self.parking_timeout_sec:
                self.get_logger().error(
                    f"Pedestrian parking timed out: pending={sorted(self._pending_parking)}"
                )
                self.request_shutdown("pedestrian_parking_timeout")
            elif now >= self._next_parking_retry_at:
                self._request_pending_pedestrian_parks()
            return
        if self._state == "WAIT_AUTHORED_RUNTIME_EXIT":
            self._advance_authored_runtime_exit(now)
            return
        if self._state in {"WAIT_CONTROL_HOLD", "WAIT_CONTROL_RELEASE"}:
            if now - self._state_started_at > self.bag_ready_timeout_sec:
                self._finish_episode("failed", "control_hold_service_timeout", episode_started=False)
            return
        if self._state in {"RESETTING", "WAIT_RESET_APPLIED"}:
            if now - self._state_started_at > self.config.episode.settle_timeout_sec:
                reason = "reset_service_timeout" if self._state == "RESETTING" else "reset_apply_timeout"
                self._finish_episode("failed", reason, episode_started=False)
            return
        if self._state == "SETTLING":
            self._advance_settling(now)
            return
        if self._state == "WAIT_BAG_READY":
            self._advance_bag_readiness(now)
            return
        if self._state == "RUNNING":
            self._advance_running(now)
            return
        if self._state == "WAIT_PEDESTRIAN":
            if self._pedestrian_process is not None and self._pedestrian_process.poll() not in (None, 0):
                self._finish_episode("failed", "pedestrian_runtime_failed", episode_started=False)
            elif now - self._state_started_at > self.pedestrian_start_timeout_sec:
                self._finish_episode("failed", "pedestrian_start_timeout", episode_started=False)

    def _begin_episode(self) -> None:
        max_episodes = self.config.session.max_episodes
        if max_episodes > 0 and self._episodes_finished >= max_episodes:
            self.request_shutdown("max_episodes_reached")
            return
        self._selection = self.selector.select()
        request = SetBool.Request()
        request.data = True
        future = self._control_hold_client.call_async(request)
        future.add_done_callback(self._control_hold_done)
        self._set_state("WAIT_CONTROL_HOLD")

    def _hard_guard_policy_done(self, future) -> None:
        if self._state != "WAIT_GUARD_POLICY":
            return
        try:
            response = future.result()
            accepted = bool(response.success)
            message = str(response.message)
        except Exception as exc:
            accepted = False
            message = str(exc)
        if not accepted:
            self.get_logger().error(f"Could not configure pedestrian hard guard: {message}")
            self.request_shutdown("hard_guard_policy_rejected")
            return
        self.get_logger().info(
            f"Manual collection pedestrian/robot policy applied: {message}"
        )
        self._next_episode_at = time.monotonic()
        self._set_state("BETWEEN_EPISODES")

    def _control_hold_done(self, future) -> None:
        if self._state != "WAIT_CONTROL_HOLD":
            return
        try:
            response = future.result()
            accepted = bool(response.success)
            message = str(response.message)
        except Exception as exc:
            accepted = False
            message = str(exc)
        if not accepted:
            self._finish_episode(
                "failed",
                "control_hold_rejected",
                episode_started=False,
                extra={"message": message},
            )
            return
        self._control_hold_enabled = True
        self.get_logger().info(f"Operator control held before reset: {message}")
        self._start_reset()

    def _start_reset(self) -> None:
        if self._selection is None:
            return
        scenario = self._selection.scenario
        request = ResetRobot.Request()
        request.name = self.robot_name
        x, y, z, yaw = scenario.robot_start
        request.pose.position.x = x
        request.pose.position.y = y
        request.pose.position.z = z
        request.pose.orientation.z = math.sin(0.5 * yaw)
        request.pose.orientation.w = math.cos(0.5 * yaw)
        self._settle_stable_since = None
        self._expected_reset_generation = 0
        future = self._reset_client.call_async(request)
        future.add_done_callback(self._reset_done)
        self._set_state("RESETTING")
        self.get_logger().info(
            f"Resetting robot for selection {self._selection.selection_index}: "
            f"scenario={scenario.id}, start={list(scenario.robot_start)}"
        )

    def _reset_done(self, future) -> None:
        if self._state != "RESETTING":
            return
        try:
            response = future.result()
            accepted = bool(response.accepted)
            message = str(response.message)
        except Exception as exc:
            accepted = False
            message = str(exc)
        if not accepted:
            self.get_logger().error(f"Robot episode reset rejected: {message}")
            self._finish_episode("failed", "reset_rejected", episode_started=False, extra={"message": message})
            return
        self._expected_reset_generation = int(getattr(response, "generation", 0) or 0)
        if self._expected_reset_generation <= 0:
            self._finish_episode(
                "failed",
                "reset_generation_missing",
                episode_started=False,
                extra={"message": message},
            )
            return
        self.get_logger().info(
            f"Robot reset accepted: {message}; waiting for simulation-thread apply status."
        )
        self._set_state("WAIT_RESET_APPLIED")
        cached_status = self._reset_status_by_generation.get(self._expected_reset_generation)
        if cached_status is not None:
            self._handle_reset_status(cached_status)

    def _advance_settling(self, now: float) -> None:
        if self._selection is None:
            return
        if now - self._state_started_at > self.config.episode.settle_timeout_sec:
            self._finish_episode("failed", "settle_timeout", episode_started=False)
            return
        if self._latest_odom is None or now - self._latest_odom_at > 1.0:
            self._settle_stable_since = None
            return
        x, y, _z, yaw, linear_speed, angular_speed = self._latest_odom
        target = self._selection.scenario.robot_start
        stable = (
            _planar_distance((x, y), target) <= self.settle_position_tolerance_m
            and _angle_distance(yaw, target[3]) <= self.settle_yaw_tolerance_rad
            and linear_speed <= self.config.episode.settle_linear_speed_mps
            and angular_speed <= self.config.episode.settle_angular_speed_rps
        )
        if not stable:
            self._settle_stable_since = None
            return
        if self._settle_stable_since is None:
            self._settle_stable_since = now
            return
        if now - self._settle_stable_since >= self.settle_min_stable_sec:
            self._prepare_recording()

    def _prepare_recording(self) -> None:
        if self._selection is None:
            return
        try:
            context = self.recorder.start_episode(self._selection)
        except Exception as exc:
            self.get_logger().error(f"Failed to start episode recorder: {exc}")
            self._finish_episode(
                "failed",
                "recorder_start_failed",
                episode_started=False,
                extra={"message": str(exc)},
            )
            return
        self.recorder.record_event(
            "recording_prepared",
            {
                "scenario_id": self._selection.scenario.id,
                "ready_topics": list(self.bag_ready_topics),
            },
        )
        self._set_state("WAIT_BAG_READY")
        self.get_logger().info(
            f"Prepared {context.episode_id}; waiting for rosbag subscriptions before pedestrian activation."
        )
        if not self.recording_enabled:
            self._start_pedestrian()

    def _bag_subscribed_topics(self) -> set[str]:
        ready = set()
        for topic in self.bag_ready_topics:
            try:
                subscriptions = self.get_subscriptions_info_by_topic(topic)
            except Exception:
                continue
            if any("rosbag2_recorder" in str(endpoint.node_name) for endpoint in subscriptions):
                ready.add(topic)
        return ready

    def _advance_bag_readiness(self, now: float) -> None:
        if not self.recording_enabled:
            return
        if not self.recorder.bag_process_running:
            self._finish_episode("failed", "rosbag_process_exited", episode_started=False)
            return
        ready = self._bag_subscribed_topics()
        if len(ready) == len(self.bag_ready_topics):
            self.recorder.record_event("rosbag_ready", {"topics": sorted(ready)})
            self.get_logger().info(f"Rosbag subscriptions ready: {sorted(ready)}")
            self._start_pedestrian()
            return
        if now - self._state_started_at > self.bag_ready_timeout_sec:
            missing = sorted(set(self.bag_ready_topics) - ready)
            self._finish_episode(
                "failed",
                "rosbag_ready_timeout",
                episode_started=False,
                extra={"missing_topics": missing},
            )

    def _start_pedestrian(self) -> None:
        if self._selection is None:
            return
        scenario = self._selection.scenario
        existing_runtimes = _authored_runtime_nodes(
            self.get_node_names_and_namespaces()
        )
        if existing_runtimes:
            message = (
                "An authored pedestrian runtime is already running: "
                f"{list(existing_runtimes)}. Stop the standalone "
                "toilet_authored_scenario process; manual_collection_node "
                "starts the same runner itself and must be the only route owner."
            )
            self.get_logger().error(message)
            self._finish_episode(
                "failed",
                "pedestrian_runtime_already_running",
                episode_started=False,
                extra={"nodes": list(existing_runtimes), "message": message},
            )
            return
        self._active_pedestrian_generations = {}
        self._incarnation_counter += 1
        incarnation_suffix = (
            f"__{self._incarnation_session_token}_e{self._incarnation_counter:03d}"
        )
        self._runtime_agent_ids = tuple(
            f"{agent_id}{incarnation_suffix}"
            for agent_id in scenario.pedestrian_agent_ids
        )
        command = _build_authored_scenario_command(
            episode_path=str(scenario.episode_path),
            status_topic=self.pedestrian_status_topic,
            agent_id_suffix=incarnation_suffix,
        )
        try:
            self._pedestrian_process = subprocess.Popen(command, start_new_session=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to start pedestrian runtime: {exc}")
            self._finish_episode(
                "failed",
                "pedestrian_runtime_start_failed",
                episode_started=False,
                extra={"message": str(exc)},
            )
            return
        self._set_state("WAIT_PEDESTRIAN")
        self.get_logger().info(
            f"Started pedestrian runtime={scenario.source_mode} for scenario={scenario.id}, "
            f"agents={list(self._active_agent_ids())}; "
            "waiting for the first pedestrian_active before releasing operator control."
        )

    def _request_control_release(self) -> None:
        request = SetBool.Request()
        request.data = False
        future = self._control_hold_client.call_async(request)
        future.add_done_callback(self._control_release_done)
        self._set_state("WAIT_CONTROL_RELEASE")

    def _control_release_done(self, future) -> None:
        if self._state != "WAIT_CONTROL_RELEASE":
            return
        try:
            response = future.result()
            accepted = bool(response.success)
            message = str(response.message)
        except Exception as exc:
            accepted = False
            message = str(exc)
        if not accepted:
            self._finish_episode(
                "failed",
                "control_release_rejected",
                episode_started=False,
                extra={"message": message},
            )
            return
        self._control_hold_enabled = False
        self._activate_episode()

    def _activate_episode(self) -> None:
        if self._selection is None:
            return
        self.recorder.record_event(
            "episode_started",
            {
                "scenario_id": self._selection.scenario.id,
                "scenario_source": self._selection.scenario.source_mode,
                "episode_path": self._selection.scenario.episode_path,
                "episode_sha256": self._selection.scenario.episode_sha256,
                "pedestrian_count": self._active_pedestrian_count(),
                "pedestrian_agent_ids": list(self._active_agent_ids()),
                "pedestrian_source_agent_ids": list(
                    self._selection.scenario.pedestrian_agent_ids
                ),
            },
        )
        self._episode_started_at = time.monotonic()
        self._finishing = False
        self._set_state("RUNNING")
        self.get_logger().info(
            f"Started active collection: scenario={self._selection.scenario.id}; gamepad control is now active."
        )

    def _advance_running(self, now: float) -> None:
        if now - self._episode_started_at > self.config.episode.timeout_sec:
            self._finish_episode("failed", "episode_timeout")
            return
        if self._pedestrian_process is not None:
            return_code = self._pedestrian_process.poll()
            if return_code not in (None, 0):
                self._finish_episode("failed", "pedestrian_runtime_failed", extra={"return_code": return_code})
                return
        if self._selection is None or self._latest_odom is None:
            return
        x, y, _z, _yaw, linear_speed, _angular_speed = self._latest_odom
        if (
            _planar_distance((x, y), self._selection.scenario.robot_goal)
            <= self.config.episode.goal_tolerance_m
            and linear_speed <= self.config.episode.goal_stop_speed_mps
        ):
            self._finish_episode("succeeded", "robot_reached_goal")

    def _stop_pedestrian_runtime(self) -> None:
        process = self._pedestrian_process
        self._pedestrian_process = None
        if process is None or process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5.0)
        except Exception:
            try:
                process.terminate()
                process.wait(timeout=2.0)
            except Exception:
                try:
                    process.kill()
                except Exception:
                    pass

    def _advance_authored_runtime_exit(self, now: float) -> None:
        process = self._pedestrian_process
        return_code = None if process is None else process.poll()
        if process is not None and return_code is None:
            if now - self._state_started_at <= self.authored_runtime_exit_timeout_sec:
                return
            self.get_logger().error(
                "Authored runtime did not retire its pedestrians within "
                f"{self.authored_runtime_exit_timeout_sec:.1f}s; using forced parking fallback."
            )
            agent_ids = self._active_agent_ids()
            self._stop_pedestrian_runtime()
            self._start_pedestrian_parking(agent_ids)
            return

        self._pedestrian_process = None
        if return_code in (None, 0):
            self.get_logger().info(
                "Authored runtime completed its own pedestrian retirement; "
                "advancing to the next collection episode."
            )
            self._complete_episode_cleanup()
            return

        self.get_logger().error(
            f"Authored runtime exited with code {return_code}; using forced parking fallback."
        )
        self._start_pedestrian_parking(self._active_agent_ids())

    def _park_pedestrians_best_effort(self) -> None:
        if not self.context.ok():
            return
        if not self._parking_client.service_is_ready():
            return
        for agent_id in self._active_agent_ids():
            try:
                request = DeletePrim.Request()
                request.name = f"/World/Characters/{agent_id}"
                self._parking_client.call_async(request)
            except Exception as exc:
                self.get_logger().warning(f"Could not park pedestrian {agent_id} during cleanup: {exc}")

    def _start_pedestrian_parking(self, agent_ids: tuple[str, ...]) -> None:
        self._pending_parking = set(agent_ids)
        self._parking_inflight = set()
        self._parking_failures = []
        if not self._pending_parking or not self._parking_client.service_is_ready():
            self._complete_episode_cleanup()
            return
        self._set_state("WAIT_PEDESTRIAN_PARK")
        self._request_pending_pedestrian_parks()

    def _request_pending_pedestrian_parks(self) -> None:
        self._next_parking_retry_at = time.monotonic() + 0.2
        for agent_id in sorted(self._pending_parking - self._parking_inflight):
            request = DeletePrim.Request()
            request.name = f"/World/Characters/{agent_id}"
            future = self._parking_client.call_async(request)
            self._parking_inflight.add(agent_id)
            future.add_done_callback(
                lambda result, parked_agent=agent_id: self._pedestrian_park_done(
                    parked_agent, result
                )
            )

    def _pedestrian_park_done(self, agent_id: str, future) -> None:
        self._parking_inflight.discard(agent_id)
        if agent_id not in self._pending_parking:
            return
        try:
            response = future.result()
            success = bool(response is not None and response.ret)
        except Exception as exc:
            success = False
            self.get_logger().error(f"Failed to park {agent_id}: {exc}")
        if success:
            self._pending_parking.discard(agent_id)
        if not self._pending_parking:
            self._complete_episode_cleanup()

    def _complete_episode_cleanup(self) -> None:
        if self._parking_failures:
            self.get_logger().error(
                f"Pedestrian parking failed: {sorted(self._parking_failures)}"
            )
            self._pending_parking.clear()
            self.request_shutdown("pedestrian_parking_failed")
            return
        self._pending_parking.clear()
        self._parking_inflight.clear()
        self._runtime_agent_ids = ()
        self._selection = None
        self._next_episode_at = time.monotonic() + self.inter_episode_delay_sec
        self._finishing = False
        self._set_state("BETWEEN_EPISODES")

    def _engage_control_hold_best_effort(self) -> None:
        if not self.context.ok():
            return
        try:
            if not self._control_hold_client.service_is_ready():
                return
            request = SetBool.Request()
            request.data = True
            self._control_hold_client.call_async(request)
            self._control_hold_enabled = True
        except Exception as exc:
            self.get_logger().warning(f"Could not engage control hold during cleanup: {exc}")

    def _finish_episode(
        self,
        status: str,
        reason: str,
        *,
        episode_started: bool = True,
        extra: dict | None = None,
        abort_pedestrian_runtime: bool = False,
    ) -> None:
        if self._finishing:
            return
        self._finishing = True
        self._engage_control_hold_best_effort()
        recording_prepared = self.recorder.has_active_episode
        if recording_prepared:
            try:
                self.recorder.record_event("episode_finished", {"status": status, "reason": reason})
            except RuntimeError:
                recording_prepared = False
        if recording_prepared:
            self.recorder.finalize_episode(status=status, termination_reason=reason, extra=extra or {})
            self._episodes_finished += 1
        else:
            self.get_logger().error(f"Episode preparation failed before recording: {reason}")
        self.get_logger().info(f"Episode finished: status={status}, reason={reason}")
        if abort_pedestrian_runtime:
            process = self._pedestrian_process
            if (
                process is not None
                and process.poll() is None
                and self._authored_cancel_client.service_is_ready()
            ):
                future = self._authored_cancel_client.call_async(Trigger.Request())
                future.add_done_callback(self._authored_cancel_done)
                self._set_state("WAIT_AUTHORED_RUNTIME_EXIT")
                self.get_logger().info(
                    "Collision recorded as an immediate episode failure; waiting for "
                    "the authored runtime to stop and park pedestrians gracefully."
                )
                return
            agent_ids = self._active_agent_ids()
            self.get_logger().warning(
                "Authored cancel service is unavailable; using forced process stop and parking fallback."
            )
            self._stop_pedestrian_runtime()
            self._start_pedestrian_parking(agent_ids)
            return
        process = self._pedestrian_process
        if process is not None and process.poll() is None:
            self._set_state("WAIT_AUTHORED_RUNTIME_EXIT")
            self.get_logger().info(
                "Waiting for authored runtime to finish its own People stop and pedestrian retirement."
            )
            return
        self._advance_authored_runtime_exit(time.monotonic())

    def _authored_cancel_done(self, future) -> None:
        try:
            response = future.result()
            accepted = bool(response is not None and response.success)
            message = "" if response is None else str(response.message)
        except Exception as exc:
            accepted = False
            message = str(exc)
        if accepted:
            self.get_logger().info(f"Authored graceful cancellation accepted: {message}")
            return
        self.get_logger().error(
            f"Authored graceful cancellation was rejected: {message}; using forced fallback."
        )
        agent_ids = self._active_agent_ids()
        self._stop_pedestrian_runtime()
        self._start_pedestrian_parking(agent_ids)

    def request_shutdown(self, reason: str = "operator_request") -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        if self._hard_guard_control_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            self._hard_guard_control_client.call_async(request)
        self._engage_control_hold_best_effort()

        active_episode = self.recorder.has_active_episode
        if active_episode and not self._finishing:
            try:
                self.recorder.record_event("episode_finished", {"status": "aborted", "reason": reason})
            except Exception as exc:
                self.get_logger().error(f"Failed to record shutdown event: {exc}")

        # Local subprocess cleanup must not depend on a live ROS context.
        self._stop_pedestrian_runtime()
        self._park_pedestrians_best_effort()

        if active_episode:
            try:
                self.recorder.finalize_episode(status="aborted", termination_reason=reason)
            except Exception as exc:
                self.get_logger().error(f"Failed to finalize active episode during shutdown: {exc}")
        try:
            self.recorder.finalize_session(
                status="completed" if reason == "max_episodes_reached" else "interrupted"
            )
        except Exception as exc:
            self.get_logger().error(f"Failed to finalize session during shutdown: {exc}")
        try:
            self._set_state("FINISHED")
        except Exception as exc:
            self.get_logger().warning(f"Could not publish final collection state: {exc}")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default=_default_config_path())
    parser.add_argument("--no-record", action="store_true", help="Run episodes without starting rosbag2.")
    parsed, ros_args = parser.parse_known_args(args=args)
    shutdown_signal = _ShutdownSignalLatch()
    previous_sigint = signal.getsignal(signal.SIGINT)
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, shutdown_signal.handle)
    signal.signal(signal.SIGTERM, shutdown_signal.handle)
    node = None
    try:
        node = ManualCollectionNode(parsed.config, record_bag=False if parsed.no_record else None)
        while rclpy.ok() and shutdown_signal.reason is None and not node._shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        if node is not None:
            if not node._shutdown_requested:
                node.request_shutdown(shutdown_signal.reason or "process_exit")
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
