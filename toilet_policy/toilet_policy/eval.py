"""Run one authored social-navigation episode with a learned robot policy."""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import time
from pathlib import Path
from typing import Sequence

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from toilet_benchmark.episodes.schema import EpisodeSpec

from .inference import ToiletPolicyInference


class PolicyEvalCoordinator(Node):
    def __init__(
        self,
        episode: EpisodeSpec,
        episode_path: Path,
        policy: ToiletPolicyInference,
        *,
        planner_radius_m: float,
        service_timeout_sec: float,
        incarnation_suffix: str,
    ) -> None:
        super().__init__("toilet_policy_eval")
        self.episode = episode
        self.episode_path = episode_path
        self.policy = policy
        self.planner_radius_m = float(planner_radius_m)
        self.service_timeout_sec = float(service_timeout_sec)
        self.incarnation_suffix = str(incarnation_suffix)
        self.authored_agents = {item.agent_id for item in episode.pedestrians}
        self.expected_agents = {
            f"{agent_id}{self.incarnation_suffix}"
            for agent_id in self.authored_agents
        }
        self.active_agents: set[str] = set()
        self.latest_odom = None
        self.process: subprocess.Popen | None = None
        self.state = "WAIT_HARD_GUARD_SERVICE"
        self.started_at = time.monotonic()
        self.state_started_at = self.started_at
        self.finished = False
        self.exit_code = 1
        self.result: dict = {}
        self._guard_request_inflight = False
        self._cancel_requested = False
        self._finish_started_at = 0.0

        self._hard_guard = self.create_client(
            SetBool, "/isaac/set_pedestrian_hard_guard"
        )
        self._cancel = self.create_client(Trigger, "/toilet_authored_scenario/cancel")
        self.create_subscription(
            String,
            "/toilet_benchmark/pedestrian_runtime_status",
            self._pedestrian_status,
            20,
        )
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(
            String,
            "/isaac/pedestrian_contact_events",
            lambda message: self._collision("robot_human_collision", message),
            20,
        )
        self.create_subscription(
            String,
            "/isaac/scene_collision_events",
            lambda message: self._collision("scene_collision", message),
            20,
        )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"Preparing policy eval: episode={episode.episode_id}, "
            f"robot_start={list(episode.robot.start_pose)}, "
            f"robot_goal={list(episode.robot.goal_pose)}, "
            f"authored_pedestrians={sorted(self.authored_agents)}, "
            f"runtime_pedestrians={sorted(self.expected_agents)}"
        )

    def _set_state(self, state: str) -> None:
        self.state = state
        self.state_started_at = time.monotonic()
        self.get_logger().info(f"Eval state: {state}")

    def _odom(self, message: Odometry) -> None:
        self.latest_odom = message

    def _pedestrian_status(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if payload.get("episode_id") != self.episode.episode_id:
            return
        if payload.get("event") == "pedestrian_active":
            agent_id = str(payload.get("agent_id", ""))
            if agent_id in self.expected_agents:
                self.active_agents.add(agent_id)
                self.get_logger().info(
                    f"Pedestrian ready: {agent_id} "
                    f"({len(self.active_agents)}/{len(self.expected_agents)})"
                )

    def _collision(self, reason: str, message: String) -> None:
        if self.state != "ACTIVE":
            return
        try:
            details = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            details = {"raw": message.data}
        self._begin_finish(False, reason, details=details)

    def _launch_scenario(self) -> None:
        command = [
            "ros2",
            "run",
            "toilet_benchmark",
            "toilet_authored_scenario",
            "--episode",
            str(self.episode_path),
            "--planner-radius-m",
            str(self.planner_radius_m),
            "--pedestrian-robot-policy",
            "detect_and_fail",
            "--agent-id-suffix",
            self.incarnation_suffix,
        ]
        self.process = subprocess.Popen(command)
        self._set_state("WAIT_PEDESTRIANS")
        self.get_logger().info(
            "Started authored scenario; it owns robot reset and pedestrian runtime."
        )

    def _hard_guard_done(self, future) -> None:
        self._guard_request_inflight = False
        try:
            response = future.result()
        except Exception as exc:
            self._begin_finish(False, "hard_guard_disable_failed", details={"error": str(exc)})
            return
        if response is None or not response.success:
            self._begin_finish(
                False,
                "hard_guard_disable_rejected",
                details={"message": "" if response is None else response.message},
            )
            return
        self.get_logger().info("Predictive pedestrian hard guard disabled for eval.")
        self._launch_scenario()

    def _goal_reached(self) -> bool:
        if self.latest_odom is None:
            return False
        position = self.latest_odom.pose.pose.position
        goal = self.episode.robot.goal_pose
        return (
            math.hypot(position.x - goal[0], position.y - goal[1])
            <= self.episode.termination.goal_tolerance_m
        )

    def _tick(self) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if self.state == "WAIT_HARD_GUARD_SERVICE":
            if self._hard_guard.service_is_ready() and not self._guard_request_inflight:
                request = SetBool.Request()
                request.data = False
                self._guard_request_inflight = True
                self._hard_guard.call_async(request).add_done_callback(self._hard_guard_done)
            elif now - self.state_started_at > self.service_timeout_sec:
                self._begin_finish(False, "hard_guard_service_timeout")
            return
        if self.state == "WAIT_PEDESTRIANS":
            if self.process is not None and self.process.poll() is not None:
                self._begin_finish(
                    False,
                    "authored_scenario_exited_before_activation",
                    details={"return_code": self.process.returncode},
                )
                return
            if self.active_agents == self.expected_agents:
                self.policy.set_closed_loop(True)
                self.started_at = now
                self._set_state("ACTIVE")
                self.get_logger().info("Policy now owns /cmd_vel_gamepad_diff.")
            elif now - self.state_started_at > self.service_timeout_sec:
                self._begin_finish(
                    False,
                    "pedestrian_activation_timeout",
                    details={"active_agents": sorted(self.active_agents)},
                )
            return
        if self.state == "ACTIVE":
            if self._goal_reached():
                self._begin_finish(True, "robot_reached_goal")
            elif now - self.started_at > self.episode.termination.timeout_sec:
                self._begin_finish(False, "episode_timeout")
            elif self.process is not None and self.process.poll() not in (None, 0):
                self._begin_finish(
                    False,
                    "authored_scenario_failed",
                    details={"return_code": self.process.returncode},
                )
            return
        if self.state == "FINISHING":
            if self.process is None or self.process.poll() is not None:
                self._complete_finish()
            elif now - self._finish_started_at > 10.0:
                self.process.terminate()
                self._complete_finish()

    def _begin_finish(self, succeeded: bool, reason: str, *, details=None) -> None:
        if self.state == "FINISHING" or self.finished:
            return
        self.policy.set_closed_loop(False)
        self.exit_code = 0 if succeeded else 1
        self.result = {
            "episode_id": self.episode.episode_id,
            "succeeded": bool(succeeded),
            "reason": reason,
            "elapsed_sec": round(time.monotonic() - self.started_at, 4),
            "active_agents": sorted(self.active_agents),
            "incarnation_suffix": self.incarnation_suffix,
            "details": details or {},
        }
        self._finish_started_at = time.monotonic()
        self._set_state("FINISHING")
        if self.process is not None and self.process.poll() is None:
            if self._cancel.service_is_ready() and not self._cancel_requested:
                self._cancel_requested = True
                self._cancel.call_async(Trigger.Request())
            else:
                self.process.terminate()
        else:
            self._complete_finish()

    def _complete_finish(self) -> None:
        self.finished = True
        print(json.dumps(self.result, indent=2, sort_keys=True))

    def shutdown(self) -> None:
        self.policy.set_closed_loop(False)
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()


def load_episode(path: Path) -> EpisodeSpec:
    return EpisodeSpec.from_mapping(json.loads(path.read_text(encoding="utf-8")))


def fresh_incarnation_suffix() -> str:
    return f"__eval_{os.getpid()}_{time.time_ns()}"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--planner-radius-m", type=float, default=0.30)
    parser.add_argument("--service-timeout-sec", type=float, default=30.0)
    parser.add_argument("--device", default="auto")
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _parser().parse_args(args)
    episode_path = namespace.episode.expanduser().resolve()
    episode = load_episode(episode_path)
    rclpy.init(args=None)
    policy = ToiletPolicyInference(
        parameter_overrides=[
            Parameter("checkpoint", value=str(namespace.checkpoint.expanduser().resolve())),
            Parameter("goal_pose", value=list(episode.robot.goal_pose)),
            Parameter("goal_tolerance_m", value=episode.termination.goal_tolerance_m),
            Parameter("closed_loop", value=False),
            Parameter("device", value=str(namespace.device)),
        ]
    )
    coordinator = PolicyEvalCoordinator(
        episode,
        episode_path,
        policy,
        planner_radius_m=namespace.planner_radius_m,
        service_timeout_sec=namespace.service_timeout_sec,
        incarnation_suffix=fresh_incarnation_suffix(),
    )
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(policy)
    executor.add_node(coordinator)
    try:
        while rclpy.ok() and not coordinator.finished:
            executor.spin_once(timeout_sec=0.1)
        return coordinator.exit_code
    except KeyboardInterrupt:
        return 130
    finally:
        coordinator.shutdown()
        executor.remove_node(coordinator)
        executor.remove_node(policy)
        coordinator.destroy_node()
        policy.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
