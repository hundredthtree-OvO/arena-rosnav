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
from std_srvs.srv import SetBool

from .collection_scenarios import ScenarioSelection, ScenarioSelector, load_manual_collection_config
from .episode_recorder import EpisodeRecorder


def _default_config_path() -> str:
    return os.path.join(get_package_share_directory("toilet_benchmark"), "config", "manual_collection.yaml")


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


class _ShutdownSignalLatch:
    """Record a process signal without running ROS cleanup in its handler."""

    def __init__(self) -> None:
        self.reason: str | None = None

    def handle(self, signum: int, _frame) -> None:
        if self.reason is not None:
            return
        self.reason = "operator_interrupt" if signum == signal.SIGINT else "termination_signal"


class ManualCollectionNode(Node):
    """Continuous single-pedestrian manual collection coordinator."""

    def __init__(self, config_path: str, *, record_bag: bool | None = None):
        super().__init__("toilet_manual_collection")
        self.config_path = str(Path(config_path).expanduser().resolve())
        self.raw_config = yaml.safe_load(Path(self.config_path).read_text(encoding="utf-8")) or {}
        self.config = load_manual_collection_config(self.raw_config)
        self.selector = ScenarioSelector(
            self.config.scenarios,
            selection_mode=self.config.session.selection_mode,
            seed=self.config.session.seed,
            fixed_scenario_id=self.config.session.fixed_scenario_id,
        )
        session_id = datetime.now().strftime("session_%Y%m%d_%H%M%S") + f"_seed{self.config.session.seed}"
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
        self.robot_name = str(robot_cfg.get("name", "xms_mecanum"))
        self.reset_service_name = str(robot_cfg.get("reset_service", "/isaac/reset_mecanum_episode"))
        self.control_hold_service_name = str(
            robot_cfg.get("control_hold_service", "/isaac/set_mecanum_control_hold")
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
        self.agent_id = self.config.pedestrian.agent_id
        self.character_name = str(pedestrian_cfg.get("character_name", "original_female_adult_business_02"))
        self.parking_service_name = str(pedestrian_cfg.get("parking_service", "/isaac/delete_prim"))
        self.semantics_path = str(pedestrian_cfg.get("semantics_path", ""))
        self.benchmark_path = str(pedestrian_cfg.get("benchmark_path", ""))
        self.inter_episode_delay_sec = max(0.0, float(session_cfg.get("inter_episode_delay_sec", 1.0)))
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

        self._reset_client = self.create_client(ResetRobot, self.reset_service_name)
        self._control_hold_client = self.create_client(SetBool, self.control_hold_service_name)
        self._parking_client = self.create_client(DeletePrim, self.parking_service_name)
        self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 20)
        self._hard_guard_sub = self.create_subscription(
            String,
            str(events_cfg.get("hard_guard_topic", "/isaac/pedestrian_hard_guard_events")),
            self._hard_guard_cb,
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
        self._director_status_sub = self.create_subscription(
            String,
            "/toilet_benchmark/director_status",
            self._director_status_cb,
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
        self._director_process: subprocess.Popen | None = None
        self._episodes_finished = 0
        self._finishing = False
        self._control_hold_enabled = False
        self._shutdown_requested = False
        self._last_status = None
        self.get_logger().info(
            f"Manual collection ready: config={self.config_path}, session={session_id}, "
            f"mode={self.selector.mode}, seed={self.selector.seed}, recording={recording_enabled}"
        )

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
        try:
            value = json.loads(message.data)
            return value if isinstance(value, dict) else {"value": value}
        except Exception:
            return {"raw": str(message.data)}

    def _hard_guard_cb(self, message: String) -> None:
        if self._state != "RUNNING":
            return
        self.recorder.record_event("hard_guard_intervention", self._event_payload(message))

    def _scene_collision_cb(self, message: String) -> None:
        if self._state != "RUNNING" or self._finishing:
            return
        payload = self._event_payload(message)
        self.recorder.record_event("scene_collision", payload)
        self._finish_episode("failed", "scene_collision", extra={"collision": payload})

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

    def _director_status_cb(self, message: String) -> None:
        if self._state != "WAIT_PEDESTRIAN":
            return
        payload = self._event_payload(message)
        if payload.get("event") != "pedestrian_active" or payload.get("agent_id") != self.agent_id:
            return
        if self._selection is None:
            return
        if payload.get("resource_id") != self._selection.scenario.pedestrian_target_urinal_id:
            return
        self.get_logger().info(
            f"Pedestrian {self.agent_id} is active at resource target {payload.get('resource_id')}; "
            "releasing operator control."
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
                and self._parking_client.wait_for_service(timeout_sec=0.0)
            ):
                self._next_episode_at = now
                self._set_state("BETWEEN_EPISODES")
            return
        if self._state == "BETWEEN_EPISODES":
            if now >= self._next_episode_at:
                self._begin_episode()
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
            if self._director_process is not None and self._director_process.poll() not in (None, 0):
                self._finish_episode("failed", "pedestrian_director_failed", episode_started=False)
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
            f"scenario={scenario.id}, start={list(scenario.robot_start)}, "
            f"pedestrian_target={scenario.pedestrian_target_urinal_id}"
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
        command = [
            sys.executable,
            "-m",
            "toilet_benchmark.toilet_director_node",
            "--semantics",
            self.semantics_path,
            "--benchmark",
            self.benchmark_path,
            "--initial-agents",
            "1",
            "--character-name",
            self.character_name,
            "--target-resource",
            self._selection.scenario.pedestrian_target_urinal_id,
            "--status-topic",
            "/toilet_benchmark/director_status",
        ]
        try:
            self._director_process = subprocess.Popen(command, start_new_session=True)
        except Exception as exc:
            self.get_logger().error(f"Failed to start pedestrian director: {exc}")
            self._finish_episode(
                "failed",
                "pedestrian_director_start_failed",
                episode_started=False,
                extra={"message": str(exc)},
            )
            return
        self._set_state("WAIT_PEDESTRIAN")
        self.get_logger().info(
            f"Started pedestrian director for scenario={self._selection.scenario.id}; "
            "waiting for pedestrian_active before releasing operator control."
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
                "pedestrian_target": self._selection.scenario.pedestrian_target_urinal_id,
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
        if self._director_process is not None:
            return_code = self._director_process.poll()
            if return_code not in (None, 0):
                self._finish_episode("failed", "pedestrian_director_failed", extra={"return_code": return_code})
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

    def _stop_director(self) -> None:
        process = self._director_process
        self._director_process = None
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

    def _park_pedestrian(self) -> None:
        if not self.context.ok():
            return
        try:
            if not self._parking_client.service_is_ready():
                return
            request = DeletePrim.Request()
            request.name = f"/World/Characters/{self.agent_id}"
            self._parking_client.call_async(request)
        except Exception as exc:
            self.get_logger().warning(f"Could not park pedestrian during cleanup: {exc}")

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
        self._stop_director()
        self._park_pedestrian()
        if recording_prepared:
            self.recorder.finalize_episode(status=status, termination_reason=reason, extra=extra or {})
            self._episodes_finished += 1
        else:
            self.get_logger().error(f"Episode preparation failed before recording: {reason}")
        self.get_logger().info(f"Episode finished: status={status}, reason={reason}")
        self._selection = None
        self._next_episode_at = time.monotonic() + self.inter_episode_delay_sec
        self._finishing = False
        self._set_state("BETWEEN_EPISODES")

    def request_shutdown(self, reason: str = "operator_request") -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        self._engage_control_hold_best_effort()

        active_episode = self.recorder.has_active_episode
        if active_episode and not self._finishing:
            try:
                self.recorder.record_event("episode_finished", {"status": "aborted", "reason": reason})
            except Exception as exc:
                self.get_logger().error(f"Failed to record shutdown event: {exc}")

        # Local subprocess cleanup must not depend on a live ROS context.
        self._stop_director()
        self._park_pedestrian()

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
