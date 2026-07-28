"""HuNav motion authority with an Isaac external-motion output adapter."""

from __future__ import annotations

from copy import deepcopy
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
from .motion_backend import IsaacPeopleBackend, MotionCommand
from .polyline_lookahead import LookaheadTarget, PolylineLookaheadTracker
from .robot_reaction import ReactionDecision, RobotProximityReactionController


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
        self._settled_arrival_tolerance_m = max(
            self._goal_radius,
            float(self._config.get("settled_arrival_tolerance_m", 0.35)),
        )
        self._robot_radius = max(
            0.01, float(self._config.get("robot_radius_m", 0.45))
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
        self._lookahead_tracker: PolylineLookaheadTracker | None = None
        self._lookahead_target: LookaheadTarget | None = None
        self._yield_hold_yaw: float | None = None
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
                self._lookahead_tracker = None
                self._lookahead_target = None
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
        self._lookahead_tracker = None
        self._lookahead_target = None
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
        if self._reaction_decision.reaction == "yielding":
            actual = self._actual_for_command()
            if actual is not None:
                self._send_yield_hold(actual)
            return
        safety = self._apply_hard_safety(previous, agent, dt)
        if agent.goals:
            speed = math.hypot(
                float(agent.velocity.linear.x),
                float(agent.velocity.linear.y),
            )
            if speed > 0.03:
                agent.yaw = math.atan2(
                    float(agent.velocity.linear.y),
                    float(agent.velocity.linear.x),
                )
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
            agent.velocity.linear.x = 0.0
            agent.velocity.linear.y = 0.0
            agent.linear_vel = 0.0
            agent.yaw = float(self._command.orientation)
            self._mark_goal_settled()
        self._shadow = updated
        self._publish_live_diagnostics(previous, agent, safety)
        self._send_external_motion(agent)

    def _mark_goal_settled(self) -> None:
        if (
            self._settled_generation == self._generation
            and self._settled_agent_id == self._command.agent_id
        ):
            return
        self._settled_generation = self._generation
        self._settled_agent_id = self._command.agent_id
        self._logger.info(
            f"HuNav consumed terminal goal for {self._command.agent_id}; "
            "streaming a final zero-velocity external update."
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
        )
        self._lookahead_target = self._lookahead_tracker.update(
            float(actual["x"]),
            float(actual["y"]),
        )

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
        self._lookahead_target = self._lookahead_tracker.update(x, y)
        agent.goals = [self._goal_pose(self._lookahead_target)]

    def _restore_agent_fields(self, agent: Agent) -> None:
        command = self._command
        speed_scale = float(self._reaction_decision.speed_scale)
        agent.desired_velocity = float(command.velocity) * speed_scale
        agent.radius = self._agent_radius
        agent.goal_radius = self._goal_radius
        agent.cyclic_goals = False
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
                f"hold_yaw={self._yield_hold_yaw}"
            )
            if previous_state == "YIELDING_TO_ROBOT":
                self._yield_hold_yaw = None
                self._last_compute_at = time.monotonic()
                self._safety_robot_previous = None

    def _reset_robot_reaction(self) -> None:
        self._reaction_controller.reset()
        self._yield_hold_yaw = None
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
            phase=self._command.phase,
        )
        self._external_future = self._isaac.send(hold, self._external_done)

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
        robot.radius = self._robot_radius
        return robot

    def _apply_hard_safety(self, previous: Agent, proposed: Agent, dt: float) -> dict[str, object]:
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
            robot_radius=self._robot_radius if robot_available else 0.0,
            static_obstacles={agent_id: ()},
            config=self._safety_config,
        )
        self._safety_robot_previous = robot_current if robot_available else None
        safe_x, safe_y = result.positions[agent_id]
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
        if diagnostics["intervened"]:
            self._logger.info(
                f"HuNav hard safety: agent={self._command.agent_id}, "
                f"generation={self._generation}, phase={self._command.phase or 'UNSPECIFIED'}, "
                f"robot_contacts={diagnostics['robot_contacts']}"
            )
        return diagnostics

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
            f"reaction_state={self._reaction_decision.state}, "
            f"robot_contacts={safety.get('robot_contacts', 0)}"
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

    def _send_external_motion(self, agent: Agent) -> None:
        if (
            self._external_future is not None
            and not self._external_future.done()
        ):
            return
        actual = self._actual.get(self._command.agent_id) or {}
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
