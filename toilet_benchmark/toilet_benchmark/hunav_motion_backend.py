"""HuNav motion authority with an Isaac external-motion output adapter."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path as FilesystemPath
from types import SimpleNamespace
import time
from typing import Any, Mapping

from geometry_msgs.msg import Point, Pose, PoseStamped
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import ComputeAgents, ResetAgents
from nav_msgs.msg import Odometry, Path
from people_msgs.msg import People
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from visualization_msgs.msg import Marker, MarkerArray

from .hunav_phase0_safety import SafetyConfig, project_safe_step
from .motion_backend import (
    EXTERNAL_MOTION_FREEZE,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
    IsaacPeopleBackend,
    MotionCommand,
)
from .polyline_lookahead import LookaheadTarget, PolylineLookaheadTracker
from .robot_reaction import ReactionDecision, RobotProximityReactionController


@dataclass(frozen=True)
class AvoidanceCandidate:
    side: int
    target: tuple[float, float]
    waypoints: tuple[tuple[float, float], ...]
    accepted: bool
    reason: str
    static_clearance_m: float
    dynamic_clearance_m: float
    path_length_m: float
    forward_progress_m: float
    turn_angle_rad: float

    @property
    def safety_margin_m(self) -> float:
        return min(self.static_clearance_m, self.dynamic_clearance_m)


def _set_agent_pose(agent: Agent, x: float, y: float, yaw: float) -> None:
    agent.position.position.x = float(x)
    agent.position.position.y = float(y)
    agent.position.position.z = 0.0
    agent.position.orientation.z = math.sin(float(yaw) * 0.5)
    agent.position.orientation.w = math.cos(float(yaw) * 0.5)
    agent.yaw = float(yaw)


def _yaw_from_quaternion(orientation) -> float:
    return math.atan2(
        2.0
        * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0
        - 2.0
        * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        ),
    )


class HuNavMotionBackend:
    """Own one pedestrian's continuous motion and stream it into Isaac."""

    def __init__(
        self,
        node,
        *,
        move_service_name: str,
        namespace: str,
        config: Mapping[str, Any] | None = None,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._isaac = IsaacPeopleBackend(node, move_service_name)
        self._namespace = "/" + str(namespace).strip().strip("/")
        self._config = dict(config or {})
        self._compute_hz = max(1.0, float(self._config.get("compute_hz", 10.0)))
        self._state_timeout_sec = max(
            0.05, float(self._config.get("state_timeout_sec", 0.5))
        )
        self._external_timeout_sec = max(
            2.0 / self._compute_hz,
            float(self._config.get("external_timeout_sec", 0.35)),
        )
        self._max_hunav_step_speed_factor = max(
            1.0,
            float(self._config.get("max_hunav_step_speed_factor", 2.0)),
        )
        self._max_hunav_step_min_m = max(
            0.005,
            float(self._config.get("max_hunav_step_min_m", 0.03)),
        )
        self._agent_radius = max(
            0.01, float(self._config.get("agent_radius_m", 0.30))
        )
        self._goal_radius = max(
            0.01, float(self._config.get("goal_radius_m", 0.20))
        )
        self._lookahead_distance_m = max(
            self._goal_radius + 0.10,
            float(self._config.get("lookahead_distance_m", 0.80)),
        )
        self._lookahead_projection_window_m = max(
            self._lookahead_distance_m,
            float(self._config.get("lookahead_projection_window_m", 1.50)),
        )
        self._lookahead_progress_slack_m = max(
            0.0,
            float(self._config.get("lookahead_progress_slack_m", 0.15)),
        )
        self._lookahead_max_cross_track_m = max(
            0.10,
            float(self._config.get("lookahead_max_cross_track_m", 0.80)),
        )
        self._settled_arrival_tolerance_m = max(
            self._goal_radius,
            float(self._config.get("settled_arrival_tolerance_m", 0.35)),
        )
        self._terminal_align_yaw_tolerance_rad = max(
            0.01,
            float(self._config.get("terminal_align_yaw_tolerance_rad", 0.15)),
        )
        self._terminal_align_max_speed_mps = max(
            0.0,
            float(self._config.get("terminal_align_max_speed_mps", 0.05)),
        )
        self._terminal_align_stable_sec = max(
            0.0,
            float(self._config.get("terminal_align_stable_sec", 0.40)),
        )
        self._robot_radius = max(
            0.01, float(self._config.get("robot_radius_m", 0.45))
        )
        self._stationary_robot_radius = max(
            0.01,
            min(
                self._robot_radius,
                float(
                    self._config.get(
                        "stationary_robot_radius_m",
                        self._robot_radius,
                    )
                ),
            ),
        )
        self._stationary_robot_linear_speed_threshold_mps = max(
            0.0,
            float(
                self._config.get(
                    "stationary_robot_linear_speed_threshold_mps",
                    0.05,
                )
            ),
        )
        self._stationary_robot_angular_speed_threshold_rps = max(
            0.0,
            float(
                self._config.get(
                    "stationary_robot_angular_speed_threshold_rps",
                    0.10,
                )
            ),
        )
        self._robot_footprint_half_length_m = max(
            0.01,
            float(
                self._config.get(
                    "robot_footprint_half_length_m",
                    self._stationary_robot_radius,
                )
            ),
        )
        self._robot_footprint_half_width_m = max(
            0.01,
            float(
                self._config.get(
                    "robot_footprint_half_width_m",
                    self._stationary_robot_radius,
                )
            ),
        )
        self._behavior = dict(self._config.get("behavior", {}) or {})
        self._reaction_controller = RobotProximityReactionController(
            self._config.get("robot_proximity_reaction", {}) or {},
            seed=int(self._config.get("reaction_seed", 12345)),
        )
        self._reaction_decision = ReactionDecision(
            state="WALKING",
            reaction=None,
            speed_scale=1.0,
            distance_m=None,
            ttc_sec=None,
        )
        visualization = dict(self._config.get("visualization", {}) or {})
        self._portal_visualization = visualization.get("portal")
        safety = dict(self._config.get("safety", {}) or {})
        self._safety_enabled = bool(safety.get("enabled", True))
        self._safety_config = SafetyConfig(
            clearance_m=float(safety.get("clearance_m", 0.01)),
            contact_epsilon_m=float(safety.get("contact_epsilon_m", 0.0001)),
            max_iterations=int(safety.get("max_iterations", 8)),
        )
        self._constrained_yaw_speed_threshold_mps = max(
            0.03,
            float(safety.get("constrained_yaw_speed_threshold_mps", 0.12)),
        )
        avoidance = dict(self._config.get("regular_avoidance", {}) or {})
        self._regular_avoidance_enabled = bool(avoidance.get("enabled", True))
        self._avoidance_trigger_distance_m = max(
            0.1, float(avoidance.get("trigger_distance_m", 1.40))
        )
        self._avoidance_side_clearance_m = max(
            0.05, float(avoidance.get("side_clearance_m", 0.18))
        )
        self._avoidance_forward_offset_m = max(
            0.0, float(avoidance.get("forward_offset_m", 0.30))
        )
        self._avoidance_release_distance_m = max(
            self._avoidance_trigger_distance_m,
            float(avoidance.get("release_distance_m", 1.60)),
        )
        self._avoidance_prediction_horizon_sec = max(
            0.0,
            float(avoidance.get("prediction_horizon_sec", 1.50)),
        )
        self._avoidance_sample_spacing_m = max(
            0.02,
            float(avoidance.get("sample_spacing_m", 0.10)),
        )
        self._avoidance_min_dynamic_clearance_m = max(
            0.0,
            float(avoidance.get("min_dynamic_clearance_m", 0.01)),
        )
        self._avoidance_suppress_backward_motion = bool(
            avoidance.get("suppress_backward_motion", True)
        )
        self._avoidance_geometry_override_enabled = bool(
            avoidance.get("geometry_override_enabled", True)
        )
        self._avoidance_active_route_infeasible_confirm_ticks = max(
            1,
            int(avoidance.get("active_route_infeasible_confirm_ticks", 4)),
        )
        self._avoidance_active_route_infeasible_ticks = 0
        self._avoidance_stall_timeout_sec = max(
            0.5,
            float(avoidance.get("stall_timeout_sec", 2.0)),
        )
        self._avoidance_stall_progress_epsilon_m = max(
            0.005,
            float(avoidance.get("stall_progress_epsilon_m", 0.03)),
        )
        self._avoidance_narrow_space_fallback = str(
            avoidance.get("narrow_space_fallback", "yielding")
        ).strip().lower()
        self._avoidance_max_recovery_attempts = max(
            0,
            int(avoidance.get("max_recovery_attempts", 1)),
        )
        self._global_replan_min_interval_sec = max(
            0.25,
            float(self._config.get("global_replan_min_interval_sec", 1.5)),
        )
        self._route_visibility_grace_sec = max(
            0.0,
            float(self._config.get("route_visibility_grace_sec", 0.5)),
        )
        self._route_splice_max_cross_track_m = max(
            0.0,
            float(
                self._config.get(
                    "route_splice_max_cross_track_m",
                    0.35,
                )
            ),
        )
        self._static_projection_enabled = bool(
            safety.get("static_projection_enabled", True)
        )
        self._static_projection_max_deflection_deg = max(
            0.0,
            min(
                89.0,
                float(safety.get("static_projection_max_deflection_deg", 80.0)),
            ),
        )
        self._static_projection_angle_step_deg = max(
            5.0,
            float(safety.get("static_projection_angle_step_deg", 20.0)),
        )
        self._visualization_enabled = bool(visualization.get("enabled", True))
        self._visualization_frame = str(visualization.get("frame_id", "map"))
        self._diagnostic_log_period_sec = max(
            0.1, float(visualization.get("diagnostic_log_period_sec", 1.0))
        )
        self._route_publisher = None
        self._marker_publisher = None
        if self._visualization_enabled:
            self._route_publisher = node.create_publisher(
                Path,
                str(
                    visualization.get(
                        "route_topic", "/toilet_benchmark/hunav/route"
                    )
                ),
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
            self._marker_publisher = node.create_publisher(
                MarkerArray,
                str(visualization.get("marker_topic", "/toilet_benchmark/hunav/markers")),
                1,
            )

        self._compute_client = node.create_client(
            ComputeAgents, f"{self._namespace}/compute_agents"
        )
        self._reset_client = node.create_client(
            ResetAgents, f"{self._namespace}/reset_agents"
        )
        node.create_subscription(
            People,
            str(self._config.get("state_topic", "/isaac/pedestrian_states")),
            self._people_cb,
            20,
        )
        node.create_subscription(
            Odometry,
            str(self._config.get("robot_odom_topic", "/odom")),
            self._robot_cb,
            20,
        )
        self._timer = node.create_timer(1.0 / self._compute_hz, self._tick)

        self._actual: dict[str, dict[str, float]] = {}
        self._robot: dict[str, float] | None = None
        self._command: MotionCommand | None = None
        self._shadow: Agents | None = None
        self._generation = 0
        self._active_generation = 0
        self._reset_future = None
        self._reset_generation = 0
        self._compute_future = None
        self._compute_generation = 0
        self._compute_previous = None
        self._external_future = None
        self._safety_robot_previous = None
        self._last_state_wait_log = 0.0
        self._last_diagnostic_log = 0.0
        self._last_compute_at = 0.0
        self._settled_generation = 0
        self._settled_agent_id = ""
        self._terminal_align_active = False
        self._terminal_align_stable_since = 0.0
        self._lookahead_tracker: PolylineLookaheadTracker | None = None
        self._lookahead_target: LookaheadTarget | None = None
        self._last_visible_route_target: LookaheadTarget | None = None
        self._route_visibility_lost_since = 0.0
        self._walkable_planner = None
        self._avoidance_side = 0
        self._avoidance_target: tuple[float, float] | None = None
        self._avoidance_encounter_active = False
        self._last_avoidance_candidates: tuple[AvoidanceCandidate, ...] = ()
        self._local_route_mode: str | None = None
        self._local_route_points: list[tuple[float, float]] = []
        self._local_route_index = 0
        self._local_route_best_distance: float | None = None
        self._local_route_last_progress_at = 0.0
        self._avoidance_failed_side = 0
        self._avoidance_recovery_attempts = 0
        self._local_waypoint_tolerance_m = max(
            0.10,
            float(self._config.get("local_waypoint_tolerance_m", 0.22)),
        )
        self._static_hold_yaw: float | None = None
        self._static_clear_ticks = 0
        self._last_global_replan_at = 0.0
        self._pending_rebase_replan = False
        self._yield_hold_yaw: float | None = None
        self._route_hold_active = False
        self._route_hold_yaw: float | None = None
        self._last_external_motion_mode = 0
        diagnostic_dir = FilesystemPath(
            str(
                self._config.get(
                    "diagnostic_log_dir",
                    "/tmp/toilet_hunav_takeover",
                )
            )
        ).expanduser()
        diagnostic_dir.mkdir(parents=True, exist_ok=True)
        self._hold_diagnostic_path = (
            diagnostic_dir / f"director_hold_{os.getpid()}.jsonl"
        )
        self._logger.info(
            f"HuNav motion backend configured: namespace={self._namespace}, "
            f"compute_hz={self._compute_hz:.1f}, external_timeout_sec="
            f"{self._external_timeout_sec:.2f}, goal_radius_m={self._goal_radius:.2f}, "
            f"settled_arrival_tolerance_m={self._settled_arrival_tolerance_m:.2f}"
            f", lookahead_distance_m={self._lookahead_distance_m:.2f}"
            f", robot_reaction_mode={self._reaction_controller.mode}"
            f", robot_hard_safety={self._safety_enabled}, "
            f"rviz_visualization={self._visualization_enabled}, "
            f"hold_diagnostics={self._hold_diagnostic_path}"
        )

    def set_walkable_planner(self, planner) -> None:
        """Share the director's immutable static map with local HuNav safety."""
        self._walkable_planner = planner
        self._logger.info(
            "HuNav local static safety attached to walkable map: "
            f"resolution={planner.resolution:.3f}, "
            f"obstacles={planner.occupied_count}"
        )

    def wait_for_service(self, timeout_sec: float) -> bool:
        timeout = float(timeout_sec)
        return (
            self._isaac.wait_for_service(timeout)
            and self._compute_client.wait_for_service(timeout_sec=timeout)
            and self._reset_client.wait_for_service(timeout_sec=timeout)
        )

    def allows_director_stall_recovery(self, agent_id: str) -> bool:
        """HuNav owns local waiting and recovery for an active route."""
        return False

    def send(self, command: MotionCommand, done_callback=None):
        if command.stop or command.use_direct_pose:
            if self._command is not None and self._command.agent_id == command.agent_id:
                self._command = None
                self._shadow = None
                self._generation += 1
                self._settled_generation = 0
                self._settled_agent_id = ""
                self._reset_terminal_alignment()
                self._lookahead_tracker = None
                self._lookahead_target = None
                self._last_visible_route_target = None
                self._route_visibility_lost_since = 0.0
                self._pending_rebase_replan = False
                self._reset_local_route()
                self._reset_robot_reaction()
            return self._isaac.send(command, done_callback)

        if self._command is not None and self._command.agent_id != command.agent_id:
            self._logger.error(
                f"HuNav Phase 1 backend rejected {command.agent_id}; "
                f"{self._command.agent_id} already owns the motion authority."
            )
            return self._completed_future(
                accepted=False,
                done_callback=done_callback,
            )

        self._generation += 1
        self._command = command
        self._shadow = None
        self._active_generation = 0
        self._settled_generation = 0
        self._settled_agent_id = ""
        self._reset_terminal_alignment()
        self._lookahead_tracker = None
        self._lookahead_target = None
        self._last_visible_route_target = None
        self._route_visibility_lost_since = 0.0
        self._pending_rebase_replan = False
        self._reset_local_route()
        self._reset_robot_reaction()
        self._safety_robot_previous = None
        self._logger.info(
            f"HuNav motion queued for {command.agent_id}: "
            f"generation={self._generation}, phase={command.phase or 'UNSPECIFIED'}, "
            f"goals={len(command.path_points)}, velocity={command.velocity:.2f}, "
            f"final_yaw={command.orientation:.3f}"
        )
        self._publish_route(command)
        return self._completed_future(
            accepted=True,
            done_callback=done_callback,
        )

    def is_goal_settled(
        self,
        agent_id: str,
        current_pose,
        target_pose,
    ) -> bool:
        """Expose HuNav goal consumption without weakening geometric sanity checks."""
        if (
            self._command is None
            or str(agent_id) != self._settled_agent_id
            or str(agent_id) != self._command.agent_id
            or self._settled_generation != self._generation
        ):
            return False
        try:
            distance = math.hypot(
                float(current_pose[0]) - float(target_pose[0]),
                float(current_pose[1]) - float(target_pose[1]),
            )
        except (IndexError, TypeError, ValueError):
            return False
        return distance <= self._settled_arrival_tolerance_m

    @staticmethod
    def _completed_future(*, accepted: bool, done_callback=None):
        future = Future()
        if done_callback is not None:
            future.add_done_callback(done_callback)
        future.set_result(SimpleNamespace(ret=bool(accepted)))
        return future

    def _people_cb(self, message: People) -> None:
        now = time.monotonic()
        for person in message.people:
            reliability = float(getattr(person, "reliability", 0.0))
            if reliability <= 0.0:
                continue
            identifier = str(getattr(person, "name", "") or "").strip()
            if not identifier:
                continue
            tags = {
                str(name): str(value)
                for name, value in zip(
                    list(getattr(person, "tagnames", []) or []),
                    list(getattr(person, "tags", []) or []),
                )
            }
            yaw = None
            if tags.get("yaw_valid", "false").lower() == "true":
                try:
                    yaw = float(tags["yaw_rad"])
                except (KeyError, TypeError, ValueError):
                    yaw = None
            self._actual[identifier] = {
                "x": float(person.position.x),
                "y": float(person.position.y),
                "z": float(person.position.z),
                "vx": float(person.velocity.x),
                "vy": float(person.velocity.y),
                "yaw": yaw,
                "received_at": now,
            }

    def _robot_cb(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        yaw = _yaw_from_quaternion(pose.orientation)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        self._robot = {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": yaw,
            "vx": cos_yaw * float(twist.linear.x) - sin_yaw * float(twist.linear.y),
            "vy": sin_yaw * float(twist.linear.x) + cos_yaw * float(twist.linear.y),
            "wz": float(twist.angular.z),
            "received_at": time.monotonic(),
        }

    def _actual_for_command(self) -> dict[str, float] | None:
        if self._command is None:
            return None
        actual = self._actual.get(self._command.agent_id)
        if actual is None:
            return None
        if time.monotonic() - actual["received_at"] > self._state_timeout_sec:
            return None
        return actual

    def _tick(self) -> None:
        if self._command is None:
            return
        if (
            self._settled_generation == self._generation
            and self._settled_agent_id == self._command.agent_id
        ):
            return
        actual = self._actual_for_command()
        self._update_robot_reaction(actual)
        if self._terminal_align_active:
            if actual is not None:
                self._tick_terminal_alignment(actual)
            return
        if self._reaction_decision.reaction == "yielding":
            if actual is not None:
                self._send_yield_hold(actual)
            return
        if self._reset_future is not None or self._compute_future is not None:
            return
        if self._active_generation != self._generation or self._shadow is None:
            self._dispatch_reset()
            return
        self._dispatch_compute()

    def _dispatch_reset(self) -> None:
        actual = self._actual_for_command()
        if actual is None:
            now = time.monotonic()
            if now - self._last_state_wait_log >= 2.0:
                self._last_state_wait_log = now
                self._logger.warning(
                    f"HuNav motion is waiting for a fresh Isaac state for "
                    f"{self._command.agent_id}."
                )
            return
        agents = Agents()
        agents.header.frame_id = "map"
        agents.header.stamp = self._node.get_clock().now().to_msg()
        if self._pending_rebase_replan:
            self._replan_to_final_goal(
                (float(actual["x"]), float(actual["y"])),
                reason="freeze release",
                force=True,
            )
            self._pending_rebase_replan = False
        if self._lookahead_tracker is None:
            self._initialize_lookahead(actual)
        agents.agents = [self._build_agent(actual)]
        request = ResetAgents.Request()
        request.current_agents = agents
        request.robot = self._build_robot()
        self._reset_generation = self._generation
        self._shadow = agents
        self._reset_future = self._reset_client.call_async(request)
        self._reset_future.add_done_callback(self._reset_done)

    def _reset_done(self, future) -> None:
        generation = self._reset_generation
        self._reset_future = None
        try:
            accepted = bool(future.result().ok)
        except Exception as exc:
            self._logger.error(f"HuNav reset failed: {exc}")
            self._shadow = None
            return
        if not accepted or generation != self._generation or self._command is None:
            self._shadow = None
            return
        self._active_generation = generation
        self._last_compute_at = time.monotonic()
        self._logger.info(
            f"HuNav motion authority active for {self._command.agent_id}: "
            f"generation={generation}"
        )

    def _dispatch_compute(self) -> None:
        self._synchronize_shadow_to_actual()
        for agent in self._shadow.agents:
            self._restore_agent_fields(agent)
        self._shadow.header.stamp = self._node.get_clock().now().to_msg()
        request = ComputeAgents.Request()
        request.current_agents = self._shadow
        request.robot = self._build_robot()
        self._compute_generation = self._generation
        self._compute_previous = deepcopy(self._shadow.agents[0])
        self._compute_future = self._compute_client.call_async(request)
        self._compute_future.add_done_callback(self._compute_done)

    def _synchronize_shadow_to_actual(self) -> None:
        """Close HuNav's planning loop around the animated Isaac root pose.

        HuNav returns a reference at its own integration rate while Isaac's
        AnimationGraph advances the visible root independently.  Keeping the
        returned reference as the next input makes that drift accumulate until
        the bridge has to teleport the character.  Goals and behavior stay in
        the shadow message; only the measured kinematic state is refreshed.
        """
        actual = self._actual_for_command()
        if actual is None or not self._shadow.agents:
            return
        agent = self._shadow.agents[0]
        yaw = actual.get("yaw")
        if yaw is None or not math.isfinite(float(yaw)):
            yaw = float(agent.yaw)
        _set_agent_pose(agent, actual["x"], actual["y"], float(yaw))
        agent.velocity.linear.x = float(actual["vx"])
        agent.velocity.linear.y = float(actual["vy"])
        agent.linear_vel = math.hypot(float(actual["vx"]), float(actual["vy"]))
        self._update_lookahead_goal(agent, actual["x"], actual["y"])

    def _compute_done(self, future) -> None:
        generation = self._compute_generation
        previous = self._compute_previous
        self._compute_future = None
        self._compute_previous = None
        try:
            updated = future.result().updated_agents
        except Exception as exc:
            self._logger.error(f"HuNav compute failed: {exc}")
            return
        if (
            self._command is None
            or generation != self._generation
            or not updated.agents
            or previous is None
        ):
            return
        now = time.monotonic()
        dt = max(1e-3, now - self._last_compute_at)
        self._last_compute_at = now
        agent = updated.agents[0]
        terminal_align = False
        if self._reaction_decision.reaction == "yielding":
            actual = self._actual_for_command()
            if actual is not None:
                self._send_yield_hold(actual)
            return
        if self._route_hold_active:
            actual = self._actual_for_command()
            if actual is not None:
                self._send_route_hold(actual)
            return
        self._clip_hunav_step(previous, agent, dt)
        safety = self._apply_hard_safety(previous, agent, dt)
        if agent.goals:
            speed = math.hypot(
                float(agent.velocity.linear.x),
                float(agent.velocity.linear.y),
            )
            self._apply_motion_yaw(previous, agent, safety, speed)
        else:
            if (
                self._lookahead_target is not None
                and not self._lookahead_target.is_final
            ):
                self._update_lookahead_goal(
                    agent,
                    float(agent.position.position.x),
                    float(agent.position.position.y),
                )
                self._shadow = updated
                self._publish_live_diagnostics(previous, agent, safety)
                self._send_external_motion(agent)
                return
            final_distance = math.hypot(
                float(agent.position.position.x)
                - float(self._command.goal_pose[0]),
                float(agent.position.position.y)
                - float(self._command.goal_pose[1]),
            )
            if final_distance > self._settled_arrival_tolerance_m:
                position = (
                    float(agent.position.position.x),
                    float(agent.position.position.y),
                )
                if self._replan_to_final_goal(
                    position,
                    reason="HuNav consumed goal before geometric arrival",
                ):
                    self._update_lookahead_goal(agent, *position)
                    self._shadow = updated
                    self._publish_live_diagnostics(previous, agent, safety)
                    self._send_external_motion(agent)
                    return
                self._logger.warning(
                    f"HuNav consumed the goal for {self._command.agent_id} "
                    f"{final_distance:.3f}m before geometric arrival; "
                    "holding the current pose without reporting settlement."
                )
                agent.velocity.linear.x = 0.0
                agent.velocity.linear.y = 0.0
                agent.linear_vel = 0.0
                self._shadow = updated
                self._publish_live_diagnostics(previous, agent, safety)
                self._send_external_motion(agent)
                return
            agent.velocity.linear.x = 0.0
            agent.velocity.linear.y = 0.0
            agent.linear_vel = 0.0
            agent.yaw = float(self._command.orientation)
            terminal_align = True
            self._begin_terminal_alignment()
        self._shadow = updated
        self._publish_live_diagnostics(previous, agent, safety)
        self._send_external_motion(agent, terminal_align=terminal_align)

    def _begin_terminal_alignment(self) -> None:
        if self._terminal_align_active:
            return
        self._terminal_align_active = True
        self._terminal_align_stable_since = 0.0
        self._logger.info(
            f"HuNav reached terminal position for {self._command.agent_id}; "
            f"holding position until yaw={float(self._command.orientation):.3f} "
            "is confirmed from live Isaac state."
        )

    def _reset_terminal_alignment(self) -> None:
        self._terminal_align_active = False
        self._terminal_align_stable_since = 0.0

    def _tick_terminal_alignment(self, actual: Mapping[str, float]) -> None:
        target_yaw = float(self._command.orientation)
        actual_yaw = float(actual.get("yaw", target_yaw))
        yaw_error = math.atan2(
            math.sin(target_yaw - actual_yaw),
            math.cos(target_yaw - actual_yaw),
        )
        speed = math.hypot(
            float(actual.get("vx", 0.0)),
            float(actual.get("vy", 0.0)),
        )
        now = time.monotonic()
        aligned = (
            abs(yaw_error) <= self._terminal_align_yaw_tolerance_rad
            and speed <= self._terminal_align_max_speed_mps
        )
        if aligned:
            if self._terminal_align_stable_since <= 0.0:
                self._terminal_align_stable_since = now
            elif (
                now - self._terminal_align_stable_since
                >= self._terminal_align_stable_sec
            ):
                self._reset_terminal_alignment()
                self._mark_goal_settled()
                return
        else:
            self._terminal_align_stable_since = 0.0

        agent = Agent()
        agent.position.position.x = float(actual["x"])
        agent.position.position.y = float(actual["y"])
        agent.yaw = target_yaw
        agent.velocity.linear.x = 0.0
        agent.velocity.linear.y = 0.0
        agent.linear_vel = 0.0
        self._send_external_motion(agent, terminal_align=True)

    def _mark_goal_settled(self) -> None:
        if (
            self._settled_generation == self._generation
            and self._settled_agent_id == self._command.agent_id
        ):
            return
        self._settled_generation = self._generation
        self._settled_agent_id = self._command.agent_id
        self._logger.info(
            f"HuNav terminal position and yaw confirmed for "
            f"{self._command.agent_id}; reporting settled."
        )

    def _build_agent(self, actual: Mapping[str, float]) -> Agent:
        command = self._command
        target = self._lookahead_target
        if target is None:
            self._initialize_lookahead(actual)
            target = self._lookahead_target
        first_goal = (target.x, target.y)
        yaw = actual.get("yaw")
        if yaw is None or not math.isfinite(float(yaw)):
            yaw = math.atan2(
                float(first_goal[1]) - actual["y"],
                float(first_goal[0]) - actual["x"],
            )
        agent = Agent()
        agent.id = 1
        agent.type = Agent.PERSON
        agent.name = command.agent_id
        agent.group_id = -1
        _set_agent_pose(agent, actual["x"], actual["y"], yaw)
        agent.velocity.linear.x = actual["vx"]
        agent.velocity.linear.y = actual["vy"]
        agent.linear_vel = math.hypot(actual["vx"], actual["vy"])
        agent.goals = [self._goal_pose(target)]
        self._restore_agent_fields(agent)
        return agent

    def _initialize_lookahead(self, actual: Mapping[str, float]) -> None:
        command = self._command
        points = [
            (float(actual["x"]), float(actual["y"])),
            *command.path_points,
            command.goal_pose,
        ]
        self._lookahead_tracker = PolylineLookaheadTracker(
            points,
            lookahead_m=self._lookahead_distance_m,
            projection_window_m=self._lookahead_projection_window_m,
            progress_slack_m=self._lookahead_progress_slack_m,
            max_cross_track_m=self._lookahead_max_cross_track_m,
        )
        self._lookahead_target = self._lookahead_tracker.update(
            float(actual["x"]),
            float(actual["y"]),
        )

    def _reset_lookahead_route(
        self,
        position: tuple[float, float],
        points,
        *,
        reason: str,
    ) -> bool:
        parsed = [
            [float(point[0]), float(point[1]), 0.0]
            for point in points
        ]
        if not parsed:
            return False
        splice = self._route_splice(position, parsed)
        if splice is not None:
            splice_point, tail_index = splice
            parsed = [
                [splice_point[0], splice_point[1], 0.0],
                *parsed[tail_index:],
            ]
        self._route_hold_active = False
        self._route_hold_yaw = None
        self._route_visibility_lost_since = 0.0
        self._lookahead_tracker = PolylineLookaheadTracker(
            [[position[0], position[1], 0.0], *parsed],
            lookahead_m=self._lookahead_distance_m,
            projection_window_m=self._lookahead_projection_window_m,
            progress_slack_m=self._lookahead_progress_slack_m,
            max_cross_track_m=self._lookahead_max_cross_track_m,
        )
        self._lookahead_target = self._lookahead_tracker.update(*position)
        self._last_global_replan_at = time.monotonic()
        self._logger.info(
            f"HuNav global route rebuilt: agent={self._command.agent_id}, "
            f"reason={reason}, points={len(parsed)}, "
            f"start=({position[0]:.3f},{position[1]:.3f}), "
            f"spliced={splice is not None}"
        )
        return True

    def _route_splice(
        self,
        position: tuple[float, float],
        points,
    ) -> tuple[tuple[float, float], int] | None:
        previous = self._last_visible_route_target
        if previous is None or self._walkable_planner is None:
            return None
        splice = (float(previous.x), float(previous.y))
        distance = math.dist(position, splice)
        if distance < 0.10 or distance > self._lookahead_distance_m * 1.25:
            return None
        if not self._segment_is_walkable(position, splice):
            return None
        route = [
            (float(position[0]), float(position[1])),
            *((float(point[0]), float(point[1])) for point in points),
        ]
        best_distance = math.inf
        best_segment = 0
        for index, (start, end) in enumerate(zip(route, route[1:])):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            ratio = max(
                0.0,
                min(
                    1.0,
                    (
                        (splice[0] - start[0]) * dx
                        + (splice[1] - start[1]) * dy
                    )
                    / length_sq,
                ),
            )
            projection = (
                start[0] + ratio * dx,
                start[1] + ratio * dy,
            )
            distance_to_segment = math.dist(splice, projection)
            if distance_to_segment < best_distance:
                best_distance = distance_to_segment
                best_segment = index
        if best_distance > self._route_splice_max_cross_track_m:
            return None
        candidate = PolylineLookaheadTracker(
            route,
            lookahead_m=self._lookahead_distance_m,
            projection_window_m=self._lookahead_projection_window_m,
            progress_slack_m=self._lookahead_progress_slack_m,
            max_cross_track_m=self._lookahead_max_cross_track_m,
        )
        candidate_target = candidate.update(*position)
        old_direction = (
            splice[0] - position[0],
            splice[1] - position[1],
        )
        new_direction = (
            candidate_target.x - position[0],
            candidate_target.y - position[1],
        )
        if (
            old_direction[0] * new_direction[0]
            + old_direction[1] * new_direction[1]
            <= 0.0
        ):
            return None
        return splice, best_segment

    def _replan_to_final_goal(
        self,
        position: tuple[float, float],
        *,
        reason: str,
        force: bool = False,
    ) -> bool:
        if self._walkable_planner is None or self._command is None:
            return False
        now = time.monotonic()
        if (
            not force
            and now - self._last_global_replan_at
            < self._global_replan_min_interval_sec
        ):
            return False
        dynamic_obstacles = self._dynamic_replan_obstacles()
        try:
            points = self._walkable_planner.plan(
                [position[0], position[1], 0.0],
                list(self._command.goal_pose),
                z=0.0,
                dynamic_obstacles=dynamic_obstacles,
            )
        except Exception as exc:
            self._logger.warning(
                f"HuNav global replan failed for {self._command.agent_id}: {exc}"
            )
            return False
        if dynamic_obstacles and not self._route_starts_forward(position, points):
            self._logger.warning(
                f"HuNav rejected backward dynamic replan for "
                f"{self._command.agent_id}: reason={reason}"
            )
            return False
        return self._reset_lookahead_route(position, points, reason=reason)

    def _route_starts_forward(
        self,
        position: tuple[float, float],
        points,
    ) -> bool:
        goal = (
            float(self._command.goal_pose[0]),
            float(self._command.goal_pose[1]),
        )
        goal_dx = goal[0] - position[0]
        goal_dy = goal[1] - position[1]
        goal_length = math.hypot(goal_dx, goal_dy)
        if goal_length <= 1e-6:
            return True
        for point in points:
            step_x = float(point[0]) - position[0]
            step_y = float(point[1]) - position[1]
            if math.hypot(step_x, step_y) < 0.10:
                continue
            forward_progress = (
                step_x * goal_dx + step_y * goal_dy
            ) / goal_length
            return forward_progress >= -0.05
        return True

    def _dynamic_replan_obstacles(self) -> list[tuple[float, float, float]]:
        robot = self._robot
        if (
            robot is None
            or time.monotonic() - float(robot["received_at"]) > 1.0
        ):
            return []
        radius = self._effective_robot_radius(robot)
        obstacles = [(float(robot["x"]), float(robot["y"]), radius)]
        speed = math.hypot(
            float(robot.get("vx", 0.0)),
            float(robot.get("vy", 0.0)),
        )
        if speed >= 0.05 and self._avoidance_prediction_horizon_sec > 0.0:
            obstacles.append(
                (
                    float(robot["x"])
                    + float(robot.get("vx", 0.0))
                    * self._avoidance_prediction_horizon_sec,
                    float(robot["y"])
                    + float(robot.get("vy", 0.0))
                    * self._avoidance_prediction_horizon_sec,
                    radius,
                )
            )
        return obstacles

    @staticmethod
    def _goal_pose(target: LookaheadTarget) -> Pose:
        goal = Pose()
        goal.position.x = float(target.x)
        goal.position.y = float(target.y)
        goal.orientation.w = 1.0
        return goal

    def _update_lookahead_goal(
        self,
        agent: Agent,
        x: float,
        y: float,
    ) -> None:
        if self._lookahead_tracker is None:
            return
        position = (float(x), float(y))
        self._lookahead_target = self._lookahead_tracker.update(*position)
        if self._release_regular_avoidance_if_ready(
            position,
            self._lookahead_target,
        ):
            agent.goals = [self._goal_pose(self._lookahead_target)]
            return
        local_goal = self._active_local_route_goal(position)
        if local_goal is not None:
            agent.goals = [self._point_goal_pose(local_goal)]
            return

        avoidance = self._start_regular_avoidance(
            position,
            self._lookahead_target,
        )
        if avoidance is not None:
            agent.goals = [self._point_goal_pose(avoidance)]
            return

        self._lookahead_target = self._visible_route_target(
            position,
            self._lookahead_target,
        )
        agent.goals = [self._goal_pose(self._lookahead_target)]

    @staticmethod
    def _point_goal_pose(point: tuple[float, float]) -> Pose:
        goal = Pose()
        goal.position.x = float(point[0])
        goal.position.y = float(point[1])
        goal.orientation.w = 1.0
        return goal

    def _restore_agent_fields(self, agent: Agent) -> None:
        command = self._command
        speed_scale = float(self._reaction_decision.speed_scale)
        agent.desired_velocity = float(command.velocity) * speed_scale
        agent.radius = self._agent_radius
        agent.goal_radius = self._goal_radius
        agent.cyclic_goals = False
        # Dense occupied-cell centers must not be passed to HuNav here: its
        # social-force model sums them and can push an agent out of a doorway.
        # Static legality is enforced after compute by the walkable-map sweep.
        agent.closest_obs = []
        behavior = AgentBehavior()
        behavior.type = AgentBehavior.BEH_REGULAR
        behavior.state = AgentBehavior.BEH_NO_ACTIVE
        behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
        behavior.duration = 0.0
        behavior.once = False
        behavior.vel = float(command.velocity) * speed_scale
        behavior.dist = 1.0
        behavior.social_force_factor = float(
            self._behavior.get("social_force_factor", 5.0)
        )
        behavior.goal_force_factor = float(
            self._behavior.get("goal_force_factor", 2.0)
        )
        behavior.obstacle_force_factor = float(
            self._behavior.get("obstacle_force_factor", 10.0)
        )
        behavior.other_force_factor = float(
            self._behavior.get("other_force_factor", 20.0)
        )
        agent.behavior = behavior

    def _segment_is_walkable(
        self,
        start: tuple[float, float],
        goal: tuple[float, float],
    ) -> bool:
        if self._walkable_planner is None:
            return True
        points = (
            [start[0], start[1], 0.0],
            [goal[0], goal[1], 0.0],
        )
        if self._walkable_planner.polyline_is_free(points):
            return True
        _, fraction, clipped = self._walkable_planner.clip_step(
            start,
            goal,
        )
        return not clipped and fraction >= 1.0 - 1e-6

    def _reset_local_route(self) -> None:
        self._local_route_mode = None
        self._local_route_points = []
        self._local_route_index = 0
        self._local_route_best_distance = None
        self._local_route_last_progress_at = 0.0
        self._static_hold_yaw = None
        self._static_clear_ticks = 0

    def _set_local_route(
        self,
        mode: str,
        points,
    ) -> tuple[float, float] | None:
        parsed = [
            (float(point[0]), float(point[1]))
            for point in points
        ]
        if not parsed:
            return None
        self._local_route_mode = str(mode)
        self._local_route_points = parsed
        self._local_route_index = 0
        self._local_route_best_distance = None
        self._local_route_last_progress_at = time.monotonic()
        return parsed[0]

    def _active_local_route_goal(
        self,
        position: tuple[float, float],
    ) -> tuple[float, float] | None:
        if self._local_route_mode is None:
            return None
        while self._local_route_index < len(self._local_route_points):
            target = self._local_route_points[self._local_route_index]
            distance = math.dist(position, target)
            now = time.monotonic()
            if (
                self._local_route_best_distance is None
                or distance
                <= self._local_route_best_distance
                - self._avoidance_stall_progress_epsilon_m
            ):
                self._local_route_best_distance = distance
                self._local_route_last_progress_at = now
            if (
                self._local_route_mode == "AVOIDANCE"
                and now - self._local_route_last_progress_at
                >= self._avoidance_stall_timeout_sec
            ):
                recovered = self._recover_stalled_avoidance(position)
                if recovered is not None:
                    return recovered
            if distance > self._local_waypoint_tolerance_m:
                return target
            self._local_route_index += 1
            self._local_route_best_distance = None
            self._local_route_last_progress_at = now
        completed_mode = self._local_route_mode
        self._local_route_mode = None
        self._local_route_points = []
        self._local_route_index = 0
        self._logger.info(
            f"HuNav local route completed: agent={self._command.agent_id}, "
            f"mode={completed_mode}"
        )
        if completed_mode == "AVOIDANCE":
            self._avoidance_side = 0
            self._avoidance_target = None
            self._avoidance_active_route_infeasible_ticks = 0
            self._avoidance_recovery_attempts = 0
            self._replan_to_final_goal(
                position,
                reason="regular avoidance completed",
            )
        return None

    def _recover_stalled_avoidance(
        self,
        position: tuple[float, float],
    ) -> tuple[float, float] | None:
        failed_side = self._avoidance_side
        robot = self._robot
        nominal = self._lookahead_target
        if robot is None or nominal is None:
            return None
        if (
            self._avoidance_recovery_attempts
            >= self._avoidance_max_recovery_attempts
        ):
            return self._replan_or_yield_after_avoidance(position)
        blocking, candidates, _ = self._avoidance_candidates(
            position,
            nominal,
            robot,
        )
        alternatives = [
            candidate for candidate in candidates if candidate.side != failed_side
        ]
        pool = alternatives or candidates
        if blocking and pool:
            selected = min(
                pool,
                key=lambda item: self._avoidance_candidate_sort_key(
                    item,
                    preferred_side=-failed_side if failed_side else 1,
                ),
            )
            self._avoidance_failed_side = failed_side
            self._avoidance_recovery_attempts += 1
            self._avoidance_side = selected.side
            self._avoidance_target = selected.target
            self._avoidance_active_route_infeasible_ticks = 0
            goal = self._set_local_route("AVOIDANCE", selected.waypoints)
            self._logger.warning(
                f"HuNav stalled avoidance rebuilt for "
                f"{self._command.agent_id}: side={failed_side:+d}->"
                f"{selected.side:+d}, recovery_attempt="
                f"{self._avoidance_recovery_attempts}/"
                f"{self._avoidance_max_recovery_attempts}, "
                f"target={selected.target}"
            )
            return goal

        return self._replan_or_yield_after_avoidance(position)

    def _replan_or_yield_after_avoidance(
        self,
        position: tuple[float, float],
    ) -> tuple[float, float]:
        if self._replan_to_final_goal(
            position,
            reason="local avoidance exhausted",
        ):
            self._reset_local_route()
            self._avoidance_side = 0
            self._avoidance_target = None
            self._avoidance_recovery_attempts = 0
            return (
                float(self._lookahead_target.x),
                float(self._lookahead_target.y),
            )
        self._reaction_controller.force_reaction("yielding")
        self._reset_local_route()
        self._avoidance_side = 0
        self._avoidance_target = None
        self._avoidance_encounter_active = False
        self._avoidance_recovery_attempts = 0
        self._logger.warning(
            f"HuNav stalled avoidance has no dynamic global alternative for "
            f"{self._command.agent_id}; yielding on the next control tick."
        )
        return position

    def _release_regular_avoidance_if_ready(
        self,
        position: tuple[float, float],
        nominal: LookaheadTarget,
    ) -> bool:
        if not self._avoidance_encounter_active:
            return False
        reason = None
        robot = self._robot
        if self._reaction_decision.reaction != "regular":
            reason = "robot reaction released"
        elif (
            robot is None
            or time.monotonic() - float(robot["received_at"]) > 1.0
        ):
            reason = "robot state unavailable"
        else:
            robot_xy = (float(robot["x"]), float(robot["y"]))
            tangent = (
                float(nominal.x) - position[0],
                float(nominal.y) - position[1],
            )
            tangent_length = math.hypot(*tangent)
            if tangent_length > 1e-6:
                tangent = (
                    tangent[0] / tangent_length,
                    tangent[1] / tangent_length,
                )
                relative = (
                    robot_xy[0] - position[0],
                    robot_xy[1] - position[1],
                )
                longitudinal = (
                    relative[0] * tangent[0] + relative[1] * tangent[1]
                )
                if longitudinal <= -0.10:
                    reason = "blocking robot passed"
            if (
                reason is None
                and math.dist(position, robot_xy)
                > self._avoidance_release_distance_m
            ):
                reason = "robot cleared avoidance envelope"
        if reason is None:
            return False

        old_target = self._avoidance_target
        if self._local_route_mode == "AVOIDANCE":
            self._reset_local_route()
        self._avoidance_side = 0
        self._avoidance_target = None
        self._avoidance_encounter_active = False
        self._avoidance_active_route_infeasible_ticks = 0
        self._avoidance_recovery_attempts = 0
        replanned = self._replan_to_final_goal(
            position,
            reason=reason,
        )
        if not replanned and self._lookahead_tracker is not None:
            self._lookahead_target = self._lookahead_tracker.update(*position)
        self._logger.info(
            f"HuNav regular avoidance released early: "
            f"agent={self._command.agent_id}, reason={reason}, "
            f"old_target={old_target}, replanned={replanned}"
        )
        return True

    def _visible_route_target(
        self,
        position: tuple[float, float],
        nominal: LookaheadTarget,
    ) -> LookaheadTarget:
        tracker = self._lookahead_tracker
        if (
            self._walkable_planner is None
            or tracker is None
            or self._segment_is_walkable(position, (nominal.x, nominal.y))
        ):
            self._route_hold_active = False
            self._route_hold_yaw = None
            self._route_visibility_lost_since = 0.0
            self._last_visible_route_target = nominal
            return nominal
        minimum_forward = min(
            0.20,
            max(0.01, self._goal_radius + 0.01),
        )
        minimum_progress = min(
            tracker.total_length_m,
            nominal.progress_m + minimum_forward,
        )
        progress = nominal.target_progress_m
        step = max(0.05, float(self._walkable_planner.resolution))
        while progress > minimum_progress + 1e-6:
            candidate = tracker.target_at_progress(
                progress,
                cross_track_error_m=nominal.cross_track_error_m,
                progress_limited=nominal.progress_limited,
            )
            if self._segment_is_walkable(position, (candidate.x, candidate.y)):
                self._route_hold_active = False
                self._route_hold_yaw = None
                self._route_visibility_lost_since = 0.0
                self._last_visible_route_target = candidate
                return candidate
            progress -= step
        candidate = tracker.target_at_progress(
            minimum_progress,
            cross_track_error_m=nominal.cross_track_error_m,
            progress_limited=nominal.progress_limited,
        )
        if self._segment_is_walkable(position, (candidate.x, candidate.y)):
            self._route_hold_active = False
            self._route_hold_yaw = None
            self._route_visibility_lost_since = 0.0
            self._last_visible_route_target = candidate
            return candidate
        if (
            time.monotonic() - self._last_global_replan_at
            >= self._global_replan_min_interval_sec
            and self._replan_to_final_goal(
                position,
                reason="no visible forward route target",
            )
        ):
            replanned = self._lookahead_target
            if self._segment_is_walkable(
                position,
                (replanned.x, replanned.y),
            ):
                self._route_hold_active = False
                self._route_hold_yaw = None
                self._route_visibility_lost_since = 0.0
                self._last_visible_route_target = replanned
                return replanned
        now = time.monotonic()
        if self._route_visibility_lost_since <= 0.0:
            self._route_visibility_lost_since = now
        if now - self._route_visibility_lost_since < self._route_visibility_grace_sec:
            self._route_hold_active = False
            self._route_hold_yaw = None
            return self._last_visible_route_target or nominal
        self._route_hold_active = True
        self._logger.warning(
            f"HuNav has no visible forward route target for "
            f"{self._command.agent_id} at ({position[0]:.3f},{position[1]:.3f}); "
            "holding route progress instead of cutting a static corner."
        )
        return LookaheadTarget(
            x=position[0],
            y=position[1],
            progress_m=nominal.progress_m,
            remaining_m=nominal.remaining_m,
            target_progress_m=nominal.progress_m,
            is_final=False,
            cross_track_error_m=nominal.cross_track_error_m,
            progress_limited=True,
        )

    def _start_regular_avoidance(
        self,
        position: tuple[float, float],
        nominal: LookaheadTarget,
    ) -> tuple[float, float] | None:
        robot = self._robot
        if (
            not self._regular_avoidance_enabled
            or self._reaction_decision.reaction != "regular"
            or robot is None
            or time.monotonic() - robot["received_at"] > 1.0
        ):
            self._avoidance_side = 0
            self._avoidance_target = None
            self._avoidance_encounter_active = False
            self._avoidance_active_route_infeasible_ticks = 0
            self._avoidance_recovery_attempts = 0
            return None

        robot_xy = (float(robot["x"]), float(robot["y"]))
        distance = math.dist(position, robot_xy)
        blocking, candidates, longitudinal = self._avoidance_candidates(
            position,
            nominal,
            robot,
        )
        robot_passed = longitudinal <= -0.10
        if self._avoidance_encounter_active:
            if not robot_passed and distance <= self._avoidance_release_distance_m:
                return None
            self._avoidance_side = 0
            self._avoidance_target = None
            self._avoidance_encounter_active = False
            self._avoidance_active_route_infeasible_ticks = 0
            self._avoidance_recovery_attempts = 0
        if distance > self._avoidance_trigger_distance_m or not blocking:
            return None
        if not candidates:
            return None
        selected = min(
            candidates,
            key=lambda item: self._avoidance_candidate_sort_key(
                item,
                preferred_side=1 if longitudinal >= 0.0 else -1,
            ),
        )
        self._avoidance_side = selected.side
        self._avoidance_target = selected.target
        self._avoidance_encounter_active = True
        self._avoidance_active_route_infeasible_ticks = 0
        self._avoidance_recovery_attempts = 0
        goal = self._set_local_route(
            "AVOIDANCE",
            selected.waypoints,
        )
        self._logger.info(
            f"HuNav regular avoidance latched: agent={self._command.agent_id}, "
            f"side={self._avoidance_side:+d}, target="
            f"({self._avoidance_target[0]:.3f},{self._avoidance_target[1]:.3f}), "
            f"static_clearance_m={selected.static_clearance_m:.3f}, "
            f"dynamic_clearance_m={selected.dynamic_clearance_m:.3f}, "
            f"path_length_m={selected.path_length_m:.3f}, "
            f"forward_progress_m={selected.forward_progress_m:.3f}, "
            f"turn_angle_deg={math.degrees(selected.turn_angle_rad):.1f}, "
            f"candidates={self._format_avoidance_candidates()}"
        )
        return goal

    def _avoidance_candidates(
        self,
        position: tuple[float, float],
        nominal,
        robot: Mapping[str, float],
    ) -> tuple[bool, list[AvoidanceCandidate], float]:
        robot_xy = (float(robot["x"]), float(robot["y"]))
        tangent = (float(nominal.x) - position[0], float(nominal.y) - position[1])
        tangent_length = math.hypot(*tangent)
        if tangent_length <= 1e-6:
            self._last_avoidance_candidates = ()
            return False, [], 0.0
        tangent = (tangent[0] / tangent_length, tangent[1] / tangent_length)
        normal_left = (-tangent[1], tangent[0])
        relative = (robot_xy[0] - position[0], robot_xy[1] - position[1])
        longitudinal = relative[0] * tangent[0] + relative[1] * tangent[1]
        lateral = relative[0] * normal_left[0] + relative[1] * normal_left[1]
        robot_forward_support = self._robot_support_radius(robot, tangent)
        robot_lateral_support = self._robot_support_radius(robot, normal_left)
        blocking_radius = (
            self._agent_radius
            + robot_lateral_support
            + self._avoidance_side_clearance_m
        )
        blocking = (
            math.dist(position, robot_xy) <= self._avoidance_trigger_distance_m
            and longitudinal > 0.0
            and abs(lateral) < blocking_radius
        )
        if not blocking:
            self._last_avoidance_candidates = ()
            return False, [], longitudinal

        evaluations = []
        for side in (-1, 1):
            normal = (normal_left[0] * side, normal_left[1] * side)
            entry_forward = max(
                0.0,
                min(
                    self._avoidance_forward_offset_m,
                    longitudinal - self._agent_radius - robot_forward_support,
                ),
            )
            entry_lateral = lateral + side * blocking_radius
            entry = (
                position[0]
                + tangent[0] * entry_forward
                + normal_left[0] * entry_lateral,
                position[1]
                + tangent[1] * entry_forward
                + normal_left[1] * entry_lateral,
            )
            candidate = (
                robot_xy[0]
                + tangent[0] * self._avoidance_forward_offset_m
                + normal[0] * blocking_radius,
                robot_xy[1]
                + tangent[1] * self._avoidance_forward_offset_m
                + normal[1] * blocking_radius,
            )
            route = (
                position,
                entry,
                candidate,
            )
            evaluation = self._evaluate_avoidance_candidate(
                side=side,
                route=route,
                tangent=tangent,
                robot=robot,
            )
            evaluations.append(evaluation)
        self._last_avoidance_candidates = tuple(evaluations)
        return (
            True,
            [candidate for candidate in evaluations if candidate.accepted],
            longitudinal,
        )

    def _evaluate_avoidance_candidate(
        self,
        *,
        side: int,
        route: tuple[tuple[float, float], ...],
        tangent: tuple[float, float],
        robot: Mapping[str, float],
    ) -> AvoidanceCandidate:
        target = route[-1]
        path_length = sum(
            math.dist(route[index], route[index + 1])
            for index in range(len(route) - 1)
        )
        first_leg = (target[0] - route[0][0], target[1] - route[0][1])
        first_length = math.hypot(*first_leg)
        turn_angle = (
            math.pi
            if first_length <= 1e-9
            else math.acos(
                max(
                    -1.0,
                    min(
                        1.0,
                        (
                            first_leg[0] * tangent[0]
                            + first_leg[1] * tangent[1]
                        )
                        / first_length,
                    ),
                )
            )
        )
        forward_progress = (
            (target[0] - route[0][0]) * tangent[0]
            + (target[1] - route[0][1]) * tangent[1]
        )
        samples = self._sample_avoidance_route(route)
        static_clearance = math.inf
        if self._walkable_planner is not None:
            route_points = tuple([point[0], point[1], 0.0] for point in route)
            if not self._walkable_planner.polyline_is_free(route_points):
                return AvoidanceCandidate(
                    side=side,
                    target=target,
                    waypoints=route[1:],
                    accepted=False,
                    reason="static corridor blocked",
                    static_clearance_m=0.0,
                    dynamic_clearance_m=math.inf,
                    path_length_m=path_length,
                    forward_progress_m=forward_progress,
                    turn_angle_rad=turn_angle,
                )
            clearance_samples = samples[1:] if len(samples) > 1 else samples
            static_clearance = min(
                self._walkable_planner.clearance_at(point)
                for _, point in clearance_samples
            )

        pedestrian_speed = max(
            0.10,
            float(self._command.velocity)
            if self._command is not None
            else 0.60,
        )
        robot_x = float(robot["x"])
        robot_y = float(robot["y"])
        robot_vx = float(robot.get("vx", 0.0))
        robot_vy = float(robot.get("vy", 0.0))
        dynamic_clearance = math.inf
        for distance_along, point in samples:
            time_at_point = distance_along / pedestrian_speed
            prediction_time = min(
                time_at_point,
                self._avoidance_prediction_horizon_sec,
            )
            predicted_robot = {
                "x": robot_x + robot_vx * prediction_time,
                "y": robot_y + robot_vy * prediction_time,
                "yaw": float(robot.get("yaw", 0.0))
                + float(robot.get("wz", 0.0)) * prediction_time,
            }
            dynamic_clearance = min(
                dynamic_clearance,
                self._robot_footprint_clearance(point, predicted_robot)
                - self._agent_radius,
            )
        accepted = (
            dynamic_clearance + 1e-9
            >= self._avoidance_min_dynamic_clearance_m
        )
        return AvoidanceCandidate(
            side=side,
            target=target,
            waypoints=route[1:],
            accepted=accepted,
            reason="accepted" if accepted else "predicted robot conflict",
            static_clearance_m=static_clearance,
            dynamic_clearance_m=dynamic_clearance,
            path_length_m=path_length,
            forward_progress_m=forward_progress,
            turn_angle_rad=turn_angle,
        )

    def _sample_avoidance_route(
        self,
        route: tuple[tuple[float, float], ...],
    ) -> list[tuple[float, tuple[float, float]]]:
        samples = [(0.0, route[0])]
        distance_along = 0.0
        for start, end in zip(route, route[1:]):
            segment_length = math.dist(start, end)
            count = max(
                1,
                int(math.ceil(segment_length / self._avoidance_sample_spacing_m)),
            )
            for index in range(1, count + 1):
                fraction = index / count
                point = (
                    start[0] + fraction * (end[0] - start[0]),
                    start[1] + fraction * (end[1] - start[1]),
                )
                samples.append(
                    (distance_along + fraction * segment_length, point)
                )
            distance_along += segment_length
        return samples

    def _effective_robot_radius(self, robot: Mapping[str, float]) -> float:
        linear_speed = math.hypot(
            float(robot.get("vx", 0.0)),
            float(robot.get("vy", 0.0)),
        )
        angular_speed = abs(float(robot.get("wz", 0.0)))
        if (
            linear_speed <= self._stationary_robot_linear_speed_threshold_mps
            and angular_speed
            <= self._stationary_robot_angular_speed_threshold_rps
        ):
            return self._stationary_robot_radius
        return self._robot_radius

    def _robot_support_radius(
        self,
        robot: Mapping[str, float],
        direction: tuple[float, float],
    ) -> float:
        length = math.hypot(*direction)
        if length <= 1e-9:
            return self._effective_robot_radius(robot)
        direction = (direction[0] / length, direction[1] / length)
        yaw = float(robot.get("yaw", 0.0))
        forward = (math.cos(yaw), math.sin(yaw))
        left = (-forward[1], forward[0])
        return (
            abs(direction[0] * forward[0] + direction[1] * forward[1])
            * self._robot_footprint_half_length_m
            + abs(direction[0] * left[0] + direction[1] * left[1])
            * self._robot_footprint_half_width_m
        )

    def _robot_footprint_clearance(
        self,
        point: tuple[float, float],
        robot: Mapping[str, float],
    ) -> float:
        yaw = float(robot.get("yaw", 0.0))
        dx = float(point[0]) - float(robot["x"])
        dy = float(point[1]) - float(robot["y"])
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        qx = abs(local_x) - self._robot_footprint_half_length_m
        qy = abs(local_y) - self._robot_footprint_half_width_m
        outside = math.hypot(max(qx, 0.0), max(qy, 0.0))
        inside = min(max(qx, qy), 0.0)
        return outside + inside

    def _active_avoidance_route_is_feasible(
        self,
        position: tuple[float, float],
        robot: Mapping[str, float],
    ) -> bool:
        if (
            not self._avoidance_encounter_active
            or self._local_route_mode != "AVOIDANCE"
        ):
            return True
        remaining = tuple(self._local_route_points[self._local_route_index :])
        if not remaining:
            return True
        route = (position, *remaining)
        if self._walkable_planner is not None:
            route_points = tuple((point[0], point[1], 0.0) for point in route)
            if not self._walkable_planner.polyline_is_free(route_points):
                return False

        samples = self._sample_avoidance_route(route)
        future_samples = samples[1:] if len(samples) > 1 else ()
        if not future_samples:
            return True
        pedestrian_speed = max(
            0.10,
            float(self._command.velocity)
            if self._command is not None
            else 0.60,
        )
        for distance_along, point in future_samples:
            prediction_time = min(
                distance_along / pedestrian_speed,
                self._avoidance_prediction_horizon_sec,
            )
            predicted_robot = {
                "x": float(robot["x"])
                + float(robot.get("vx", 0.0)) * prediction_time,
                "y": float(robot["y"])
                + float(robot.get("vy", 0.0)) * prediction_time,
                "yaw": float(robot.get("yaw", 0.0))
                + float(robot.get("wz", 0.0)) * prediction_time,
            }
            clearance = (
                self._robot_footprint_clearance(point, predicted_robot)
                - self._agent_radius
            )
            if (
                clearance + 1e-9
                < self._avoidance_min_dynamic_clearance_m
            ):
                return False
        return True

    @staticmethod
    def _avoidance_candidate_sort_key(
        candidate: AvoidanceCandidate,
        *,
        preferred_side: int,
    ) -> tuple[float, ...]:
        safety_band = math.floor(
            max(0.0, candidate.safety_margin_m) / 0.05 + 1e-9
        )
        return (
            -float(safety_band),
            candidate.path_length_m,
            candidate.turn_angle_rad,
            -candidate.static_clearance_m,
            -candidate.dynamic_clearance_m,
            -candidate.forward_progress_m,
            0.0 if candidate.side == preferred_side else 1.0,
        )

    def _format_avoidance_candidates(self) -> str:
        return "[" + ", ".join(
            (
                f"side={candidate.side:+d}:"
                f"{'ok' if candidate.accepted else 'reject'}"
                f"({candidate.reason},static={candidate.static_clearance_m:.3f},"
                f"dynamic={candidate.dynamic_clearance_m:.3f},"
                f"length={candidate.path_length_m:.3f},"
                f"progress={candidate.forward_progress_m:.3f},"
                f"turn={math.degrees(candidate.turn_angle_rad):.1f}deg)"
            )
            for candidate in self._last_avoidance_candidates
        ) + "]"

    def _apply_motion_yaw(
        self,
        previous: Agent,
        agent: Agent,
        safety: Mapping[str, object],
        speed: float,
    ) -> None:
        static_clipped = bool(safety.get("static_clip", False))
        constrained = bool(
            safety.get("robot_contacts", 0)
            or safety.get("static_contacts", 0)
            or static_clipped
        )
        if static_clipped:
            self._static_clear_ticks = 0
            if self._static_hold_yaw is None:
                self._static_hold_yaw = float(previous.yaw)
        else:
            self._static_clear_ticks += 1
            if self._static_clear_ticks >= 3:
                self._static_hold_yaw = None
        if (
            constrained
            and speed < self._constrained_yaw_speed_threshold_mps
        ):
            agent.yaw = float(
                previous.yaw
                if self._static_hold_yaw is None
                else self._static_hold_yaw
            )
        elif speed > 0.03:
            agent.yaw = math.atan2(
                float(agent.velocity.linear.y),
                float(agent.velocity.linear.x),
            )

    def _update_robot_reaction(
        self,
        actual: Mapping[str, float] | None,
    ) -> None:
        if actual is None:
            return
        robot = self._robot
        if robot is not None and time.monotonic() - robot["received_at"] > 1.0:
            robot = None
        previous_state = self._reaction_decision.state
        self._reaction_decision = self._reaction_controller.update(
            agent=actual,
            robot=robot,
            now=time.monotonic(),
        )
        geometry_override = False
        if (
            self._avoidance_geometry_override_enabled
            and self._avoidance_narrow_space_fallback == "yielding"
            and self._reaction_decision.reaction in {"regular", "impatient"}
            and robot is not None
            and self._lookahead_target is not None
        ):
            position = (float(actual["x"]), float(actual["y"]))
            if (
                self._avoidance_encounter_active
                and self._local_route_mode == "AVOIDANCE"
            ):
                route_feasible = self._active_avoidance_route_is_feasible(
                    position,
                    robot,
                )
                if route_feasible:
                    self._avoidance_active_route_infeasible_ticks = 0
                else:
                    self._avoidance_active_route_infeasible_ticks += 1
                # Once a passing side is latched, temporary loss of predicted
                # clearance is expected while the pedestrian moves laterally.
                # Hard safety remains authoritative; waypoint progress recovery
                # decides whether the local route itself must be rebuilt.
                should_yield = False
            else:
                self._avoidance_active_route_infeasible_ticks = 0
                blocking, candidates, _ = self._avoidance_candidates(
                    position,
                    self._lookahead_target,
                    robot,
                )
                should_yield = blocking and not candidates
            if should_yield:
                self._reaction_controller.force_reaction("yielding")
                self._reaction_decision = ReactionDecision(
                    state="YIELDING_TO_ROBOT",
                    reaction="yielding",
                    speed_scale=0.0,
                    distance_m=self._reaction_decision.distance_m,
                    ttc_sec=self._reaction_decision.ttc_sec,
                    transition=f"{previous_state}->YIELDING_TO_ROBOT",
                )
                geometry_override = True
        if self._reaction_decision.transition is not None:
            if (
                self._reaction_decision.state == "YIELDING_TO_ROBOT"
                and previous_state != "YIELDING_TO_ROBOT"
            ):
                self._yield_hold_yaw = self._resolve_hold_yaw(actual)
            self._write_hold_diagnostic(
                "reaction_transition",
                actual,
                previous_state=previous_state,
                transition=self._reaction_decision.transition,
            )
            self._logger.info(
                f"HuNav robot reaction: agent={self._command.agent_id}, "
                f"transition={self._reaction_decision.transition}, "
                f"distance_m={self._reaction_decision.distance_m}, "
                f"ttc_sec={self._reaction_decision.ttc_sec}, "
                f"hold_yaw={self._yield_hold_yaw}, "
                f"geometry_override={geometry_override}, "
                f"avoidance_candidates={self._format_avoidance_candidates()}"
            )
            if previous_state == "YIELDING_TO_ROBOT":
                self._yield_hold_yaw = None
                self._last_compute_at = time.monotonic()
                self._safety_robot_previous = None
                self._active_generation = 0
                self._shadow = None
                self._lookahead_tracker = None
                self._lookahead_target = None
                self._reset_local_route()
                self._avoidance_side = 0
                self._avoidance_target = None
                self._avoidance_encounter_active = False
                self._avoidance_recovery_attempts = 0
                self._pending_rebase_replan = True
                self._logger.info(
                    f"HuNav state rebase scheduled for "
                    f"{self._command.agent_id} after freeze release."
                )

    def _reset_robot_reaction(self) -> None:
        self._reaction_controller.reset()
        self._yield_hold_yaw = None
        self._route_hold_active = False
        self._route_hold_yaw = None
        self._avoidance_side = 0
        self._avoidance_target = None
        self._avoidance_encounter_active = False
        self._avoidance_active_route_infeasible_ticks = 0
        self._avoidance_recovery_attempts = 0
        self._reaction_decision = ReactionDecision(
            state="WALKING",
            reaction=None,
            speed_scale=1.0,
            distance_m=None,
            ttc_sec=None,
        )

    def _resolve_hold_yaw(self, actual: Mapping[str, float]) -> float:
        yaw = actual.get("yaw")
        if yaw is not None and math.isfinite(float(yaw)):
            return float(yaw)
        velocity_x = float(actual.get("vx", 0.0))
        velocity_y = float(actual.get("vy", 0.0))
        if math.hypot(velocity_x, velocity_y) > 0.03:
            return math.atan2(velocity_y, velocity_x)
        if self._shadow is not None and self._shadow.agents:
            shadow_yaw = float(self._shadow.agents[0].yaw)
            if math.isfinite(shadow_yaw):
                return shadow_yaw
        return 0.0

    def _write_hold_diagnostic(
        self,
        event: str,
        actual: Mapping[str, float],
        **extra: object,
    ) -> None:
        path = getattr(self, "_hold_diagnostic_path", None)
        if path is None:
            return
        actual_yaw = actual.get("yaw")
        hold_yaw = self._yield_hold_yaw
        yaw_error = None
        if (
            actual_yaw is not None
            and hold_yaw is not None
            and math.isfinite(float(actual_yaw))
        ):
            yaw_error = math.atan2(
                math.sin(float(hold_yaw) - float(actual_yaw)),
                math.cos(float(hold_yaw) - float(actual_yaw)),
            )
        payload = {
            "event": str(event),
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "agent_id": None if self._command is None else self._command.agent_id,
            "phase": None if self._command is None else self._command.phase,
            "reaction_state": self._reaction_decision.state,
            "reaction": self._reaction_decision.reaction,
            "hold_yaw": hold_yaw,
            "actual_yaw": actual_yaw,
            "yaw_error": yaw_error,
            "pose": [
                float(actual["x"]),
                float(actual["y"]),
                float(actual.get("z", 0.0)),
            ],
            "velocity": [
                float(actual.get("vx", 0.0)),
                float(actual.get("vy", 0.0)),
            ],
            **extra,
        }
        try:
            with path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(payload, sort_keys=True) + "\n")
        except OSError as exc:
            self._logger.warning(
                f"Failed to write HuNav hold diagnostic {path}: {exc}"
            )

    def _send_yield_hold(self, actual: Mapping[str, float]) -> None:
        if self._external_future is not None and not self._external_future.done():
            return
        if self._yield_hold_yaw is None:
            self._yield_hold_yaw = self._resolve_hold_yaw(actual)
        self._record_motion_mode(
            EXTERNAL_MOTION_FREEZE,
            actual,
            target_yaw=self._yield_hold_yaw,
        )
        self._write_hold_diagnostic("hold_dispatch", actual)
        hold = MotionCommand(
            agent_id=self._command.agent_id,
            goal_pose=self._command.goal_pose,
            path_points=(),
            velocity=0.0,
            orientation=self._yield_hold_yaw,
            direct_pose=(
                float(actual["x"]),
                float(actual["y"]),
                float(actual.get("z", 0.0)),
            ),
            use_external_motion=True,
            external_velocity=(0.0, 0.0, 0.0),
            external_timeout_sec=self._external_timeout_sec,
            external_freeze_pose=True,
            external_motion_mode=EXTERNAL_MOTION_FREEZE,
            phase=self._command.phase,
        )
        self._external_future = self._isaac.send(hold, self._external_done)

    def _send_route_hold(self, actual: Mapping[str, float]) -> None:
        if self._external_future is not None and not self._external_future.done():
            return
        if self._route_hold_yaw is None:
            self._route_hold_yaw = self._resolve_hold_yaw(actual)
        self._record_motion_mode(
            EXTERNAL_MOTION_FREEZE,
            actual,
            target_yaw=self._route_hold_yaw,
        )
        self._write_hold_diagnostic(
            "route_visibility_hold_dispatch",
            actual,
        )
        hold = MotionCommand(
            agent_id=self._command.agent_id,
            goal_pose=self._command.goal_pose,
            path_points=(),
            velocity=0.0,
            orientation=self._route_hold_yaw,
            direct_pose=(
                float(actual["x"]),
                float(actual["y"]),
                float(actual.get("z", 0.0)),
            ),
            use_external_motion=True,
            external_velocity=(0.0, 0.0, 0.0),
            external_timeout_sec=self._external_timeout_sec,
            external_freeze_pose=True,
            external_motion_mode=EXTERNAL_MOTION_FREEZE,
            phase=self._command.phase,
        )
        self._external_future = self._isaac.send(hold, self._external_done)

    def _record_motion_mode(
        self,
        mode: int,
        actual: Mapping[str, float],
        *,
        target_yaw: float,
    ) -> None:
        previous = int(getattr(self, "_last_external_motion_mode", 0))
        current = int(mode)
        if previous == current:
            return
        self._last_external_motion_mode = current
        names = {
            0: "LOCOMOTION",
            EXTERNAL_MOTION_FREEZE: "FREEZE",
            EXTERNAL_MOTION_TERMINAL_ALIGN: "TERMINAL_ALIGN",
        }
        self._write_hold_diagnostic(
            "motion_mode_transition",
            actual,
            previous_motion_mode=names.get(previous, str(previous)),
            motion_mode=names.get(current, str(current)),
            target_yaw=float(target_yaw),
        )
        self._logger.info(
            f"HuNav external mode: agent={self._command.agent_id}, "
            f"{names.get(previous, previous)}->{names.get(current, current)}, "
            f"target_yaw={float(target_yaw):.3f}, "
            f"actual_yaw={float(actual.get('yaw', 0.0)):.3f}"
        )

    def _build_robot(self) -> Agent:
        robot_state = self._robot
        if (
            robot_state is None
            or time.monotonic() - robot_state["received_at"] > 1.0
        ):
            robot_state = {
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
        robot.name = "toilet_robot"
        robot.group_id = -1
        _set_agent_pose(
            robot,
            robot_state["x"],
            robot_state["y"],
            robot_state["yaw"],
        )
        robot.velocity.linear.x = robot_state["vx"]
        robot.velocity.linear.y = robot_state["vy"]
        robot.velocity.angular.z = robot_state["wz"]
        robot.desired_velocity = math.hypot(
            robot_state["vx"], robot_state["vy"]
        )
        robot.radius = self._effective_robot_radius(robot_state)
        return robot

    def _clip_hunav_step(self, previous: Agent, proposed: Agent, dt: float) -> bool:
        start = (
            float(previous.position.position.x),
            float(previous.position.position.y),
        )
        target = (
            float(proposed.position.position.x),
            float(proposed.position.position.y),
        )
        dx = target[0] - start[0]
        dy = target[1] - start[1]
        distance = math.hypot(dx, dy)
        commanded_speed = (
            float(self._command.velocity)
            if self._command is not None
            else 0.60
        )
        maximum_step = max(
            self._max_hunav_step_min_m,
            commanded_speed * self._max_hunav_step_speed_factor * dt,
        )
        if distance <= maximum_step + 1e-9:
            return False
        scale = maximum_step / distance
        safe_x = start[0] + dx * scale
        safe_y = start[1] + dy * scale
        proposed.position.position.x = safe_x
        proposed.position.position.y = safe_y
        proposed.velocity.linear.x = (safe_x - start[0]) / dt
        proposed.velocity.linear.y = (safe_y - start[1]) / dt
        proposed.linear_vel = math.hypot(
            float(proposed.velocity.linear.x),
            float(proposed.velocity.linear.y),
        )
        self._logger.warning(
            f"HuNav proposal step clipped for {self._command.agent_id}: "
            f"distance_m={distance:.3f}, maximum_m={maximum_step:.3f}"
        )
        return True

    def _apply_hard_safety(
        self,
        previous: Agent,
        proposed: Agent,
        dt: float,
    ) -> dict[str, object]:
        if not self._safety_enabled:
            return {"enabled": False}
        robot = self._build_robot()
        agent_id = int(proposed.id)
        robot_current = (
            float(robot.position.position.x),
            float(robot.position.position.y),
        )
        robot_available = abs(robot_current[0]) < 50.0
        robot_previous = (
            self._safety_robot_previous
            if self._safety_robot_previous is not None
            else robot_current
        )
        result = project_safe_step(
            previous={
                agent_id: (
                    float(previous.position.position.x),
                    float(previous.position.position.y),
                )
            },
            proposed={
                agent_id: (
                    float(proposed.position.position.x),
                    float(proposed.position.position.y),
                )
            },
            radii={agent_id: self._agent_radius},
            robot_previous=robot_previous,
            robot_proposed=robot_current,
            robot_radius=float(robot.radius) if robot_available else 0.0,
            static_obstacles={agent_id: ()},
            config=self._safety_config,
        )
        self._safety_robot_previous = robot_current if robot_available else None
        safe_x, safe_y = result.positions[agent_id]
        safe_x, safe_y = self._suppress_backward_avoidance_step(
            (
                float(previous.position.position.x),
                float(previous.position.position.y),
            ),
            (safe_x, safe_y),
        )
        static_fraction = 1.0
        static_clipped = False
        static_projection = "disabled"
        if self._walkable_planner is not None:
            start_xy = (
                float(previous.position.position.x),
                float(previous.position.position.y),
            )
            if self._static_projection_enabled:
                target = (
                    None
                    if self._lookahead_target is None
                    else (
                        float(self._lookahead_target.x),
                        float(self._lookahead_target.y),
                    )
                )
                (
                    (safe_x, safe_y),
                    static_fraction,
                    static_clipped,
                    static_projection,
                ) = self._walkable_planner.project_step(
                    start_xy,
                    (safe_x, safe_y),
                    target=target,
                    max_deflection_deg=self._static_projection_max_deflection_deg,
                    angle_step_deg=self._static_projection_angle_step_deg,
                )
            else:
                (safe_x, safe_y), static_fraction, static_clipped = (
                    self._walkable_planner.clip_step(start_xy, (safe_x, safe_y))
                )
                static_projection = "clipped" if static_clipped else "direct"
        proposed.position.position.x = safe_x
        proposed.position.position.y = safe_y
        proposed.velocity.linear.x = (
            safe_x - float(previous.position.position.x)
        ) / dt
        proposed.velocity.linear.y = (
            safe_y - float(previous.position.position.y)
        ) / dt
        proposed.linear_vel = math.hypot(
            float(proposed.velocity.linear.x),
            float(proposed.velocity.linear.y),
        )
        diagnostics = result.diagnostics.to_dict()
        diagnostics["static_clip"] = static_clipped
        diagnostics["static_fraction"] = static_fraction
        diagnostics["static_projection"] = static_projection
        diagnostics["intervened"] = bool(
            diagnostics["intervened"] or static_clipped
        )
        if diagnostics["intervened"]:
            self._logger.info(
                f"HuNav hard safety: agent={self._command.agent_id}, "
                f"generation={self._generation}, phase={self._command.phase or 'UNSPECIFIED'}, "
                f"robot_contacts={diagnostics['robot_contacts']}, "
                f"static_contacts={diagnostics['static_contacts']}, "
                f"static_clip={static_clipped}, "
                f"static_fraction={static_fraction:.3f}, "
                f"static_projection={static_projection}"
            )
        return diagnostics

    def _suppress_backward_avoidance_step(
        self,
        start: tuple[float, float],
        proposed: tuple[float, float],
    ) -> tuple[float, float]:
        """Keep HuNav's lateral response without allowing avoidance retreat."""
        if (
            not self._avoidance_suppress_backward_motion
            or not self._avoidance_encounter_active
            or self._avoidance_target is None
        ):
            return proposed
        forward = (
            float(self._avoidance_target[0]) - float(start[0]),
            float(self._avoidance_target[1]) - float(start[1]),
        )
        forward_length = math.hypot(*forward)
        if forward_length <= 1e-6:
            return proposed
        forward = (
            forward[0] / forward_length,
            forward[1] / forward_length,
        )
        step = (
            float(proposed[0]) - float(start[0]),
            float(proposed[1]) - float(start[1]),
        )
        longitudinal = step[0] * forward[0] + step[1] * forward[1]
        if longitudinal >= 0.0:
            return proposed
        return (
            float(proposed[0]) - longitudinal * forward[0],
            float(proposed[1]) - longitudinal * forward[1],
        )

    def _publish_route(self, command: MotionCommand) -> None:
        if self._route_publisher is None:
            return
        message = Path()
        message.header.frame_id = self._visualization_frame
        message.header.stamp = self._node.get_clock().now().to_msg()
        for point in [*command.path_points, command.goal_pose]:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1])
            pose.pose.position.z = float(point[2]) if len(point) > 2 else 0.0
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        self._route_publisher.publish(message)

    def _publish_live_diagnostics(self, previous: Agent, agent: Agent, safety: Mapping[str, object]) -> None:
        if self._marker_publisher is None:
            return
        stamp = self._node.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.append(self._portal_marker(stamp))
        current = Marker()
        current.header.frame_id = self._visualization_frame
        current.header.stamp = stamp
        current.ns = "hunav_reference"
        current.id = 1
        current.type = Marker.ARROW
        current.action = Marker.ADD
        current.points = [
            Point(x=float(previous.position.position.x), y=float(previous.position.position.y), z=0.08),
            Point(x=float(agent.position.position.x), y=float(agent.position.position.y), z=0.08),
        ]
        current.scale.x = 0.03
        current.scale.y = 0.06
        current.color.r = 0.1
        current.color.g = 0.8
        current.color.b = 1.0
        current.color.a = 0.95
        markers.markers.append(current)
        body = Marker()
        body.header.frame_id = self._visualization_frame
        body.header.stamp = stamp
        body.ns = "hunav_agent"
        body.id = 2
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = float(agent.position.position.x)
        body.pose.position.y = float(agent.position.position.y)
        body.pose.position.z = 0.85
        body.pose.orientation.w = 1.0
        body.scale.x = 2.0 * self._agent_radius
        body.scale.y = 2.0 * self._agent_radius
        body.scale.z = 1.7
        body.color.r = 0.1
        body.color.g = 0.85
        body.color.b = 0.35
        body.color.a = 0.55
        markers.markers.append(body)
        heading = Marker()
        heading.header = body.header
        heading.ns = "hunav_agent"
        heading.id = 3
        heading.type = Marker.ARROW
        heading.action = Marker.ADD
        heading.points = [
            Point(
                x=float(agent.position.position.x),
                y=float(agent.position.position.y),
                z=1.0,
            ),
            Point(
                x=float(agent.position.position.x) + 0.55 * math.cos(float(agent.yaw)),
                y=float(agent.position.position.y) + 0.55 * math.sin(float(agent.yaw)),
                z=1.0,
            ),
        ]
        heading.scale.x = 0.05
        heading.scale.y = 0.10
        heading.color.r = 0.1
        heading.color.g = 1.0
        heading.color.b = 0.2
        heading.color.a = 0.95
        markers.markers.append(heading)
        self._marker_publisher.publish(markers)
        message = (
            f"HuNav diagnostics: agent={self._command.agent_id}, generation={self._generation}, "
            f"phase={self._command.phase or 'UNSPECIFIED'}, pose=({agent.position.position.x:.3f},"
            f"{agent.position.position.y:.3f}), yaw={agent.yaw:.3f}, speed={agent.linear_vel:.3f}, "
            f"route_progress_m="
            f"{self._lookahead_target.progress_m if self._lookahead_target else 0.0:.2f}, "
            f"route_remaining_m="
            f"{self._lookahead_target.remaining_m if self._lookahead_target else 0.0:.2f}, "
            f"lookahead=("
            f"{self._lookahead_target.x if self._lookahead_target else 0.0:.2f},"
            f"{self._lookahead_target.y if self._lookahead_target else 0.0:.2f}), "
            f"lookahead_final="
            f"{self._lookahead_target.is_final if self._lookahead_target else False}, "
            f"cross_track_m="
            f"{self._lookahead_target.cross_track_error_m if self._lookahead_target else 0.0:.2f}, "
            f"progress_limited="
            f"{self._lookahead_target.progress_limited if self._lookahead_target else False}, "
            f"avoidance_side={self._avoidance_side:+d}, "
            f"avoidance_target={self._avoidance_target}, "
            f"local_route_mode={self._local_route_mode}, "
            f"local_route_waypoint={self._local_route_index}/"
            f"{len(self._local_route_points)}, "
            f"static_hold_yaw={self._static_hold_yaw}, "
            f"reaction_state={self._reaction_decision.state}, "
            f"robot_contacts={safety.get('robot_contacts', 0)}, "
            f"static_clip={safety.get('static_clip', False)}"
        )
        now = time.monotonic()
        if now - self._last_diagnostic_log >= self._diagnostic_log_period_sec:
            self._last_diagnostic_log = now
            self._logger.info(message)
        else:
            self._logger.debug(message)

    def _portal_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._visualization_frame
        marker.header.stamp = stamp
        marker.ns = "hunav_portal"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.color.r = 1.0
        marker.color.g = 0.65
        marker.color.b = 0.0
        marker.color.a = 0.95
        portal = self._portal_visualization
        if not isinstance(portal, Mapping):
            marker.action = Marker.DELETE
            return marker
        outside = portal.get("outside")
        inside = portal.get("inside")
        if not isinstance(outside, (list, tuple)) or not isinstance(inside, (list, tuple)):
            marker.action = Marker.DELETE
            return marker
        dx, dy = float(inside[0]) - float(outside[0]), float(inside[1]) - float(outside[1])
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            marker.action = Marker.DELETE
            return marker
        nx, ny = -dy / length, dx / length
        half = float(portal.get("half_width_m", 0.45))
        corners = [
            (float(outside[0]) + nx * half, float(outside[1]) + ny * half),
            (float(inside[0]) + nx * half, float(inside[1]) + ny * half),
            (float(inside[0]) - nx * half, float(inside[1]) - ny * half),
            (float(outside[0]) - nx * half, float(outside[1]) - ny * half),
            (float(outside[0]) + nx * half, float(outside[1]) + ny * half),
        ]
        marker.points = [Point(x=x, y=y, z=0.03) for x, y in corners]
        return marker

    def _send_external_motion(
        self,
        agent: Agent,
        *,
        terminal_align: bool = False,
    ) -> None:
        if (
            self._external_future is not None
            and not self._external_future.done()
        ):
            return
        actual = self._actual.get(self._command.agent_id) or {}
        mode = (
            EXTERNAL_MOTION_TERMINAL_ALIGN
            if terminal_align
            else 0
        )
        self._record_motion_mode(
            mode,
            actual,
            target_yaw=float(agent.yaw),
        )
        command = MotionCommand(
            agent_id=self._command.agent_id,
            goal_pose=self._command.goal_pose,
            path_points=(),
            velocity=float(agent.linear_vel),
            orientation=float(agent.yaw),
            direct_pose=(
                float(agent.position.position.x),
                float(agent.position.position.y),
                float(actual.get("z", 0.0)),
            ),
            use_external_motion=True,
            external_velocity=(
                float(agent.velocity.linear.x),
                float(agent.velocity.linear.y),
                0.0,
            ),
            external_timeout_sec=self._external_timeout_sec,
            external_motion_mode=mode,
        )
        self._external_future = self._isaac.send(
            command,
            self._external_done,
        )

    def _external_done(self, future) -> None:
        try:
            accepted = bool(future.result().ret)
        except Exception as exc:
            self._logger.error(f"Isaac external-motion update failed: {exc}")
            self._external_future = None
            return
        if not accepted:
            self._logger.error("Isaac rejected a HuNav external-motion update.")
        self._external_future = None
