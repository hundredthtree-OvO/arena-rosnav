"""Independent E2 local-motion backend driving Isaac external motion."""

from __future__ import annotations

from concurrent.futures import Future
from dataclasses import replace
import math
import time
from typing import Mapping, Sequence

from .domain.agent import AgentSnapshot
from .domain.task import (
    EXTERNAL_MOTION_LOCOMOTION,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
    MotionCommand,
)
from .motion import (
    BehaviorPolicyRequest,
    ContextualBehaviorConfig,
    ContextualBehaviorPolicy,
    LocalMotionPipeline,
    RoutePlan,
    SampledRvoConfig,
    SampledRvoLocalMotion,
    SweptEnvelopeGeometrySafety,
)
from .motion.swept_envelope import SweptEnvelope, SweptEnvelopeConfig
from .external_motion_stream import ExternalMotionStreamPublisher
from .motion_backend import IsaacPeopleBackend
from .polyline_lookahead import PolylineLookaheadTracker


def _completed_future(*, accepted: bool, done_callback=None):
    future = Future()
    if done_callback is not None:
        future.add_done_callback(done_callback)
    future.set_result(type("MoveResult", (), {"ret": bool(accepted)})())
    return future


def _yaw_error(target: float, actual: float) -> float:
    return math.atan2(math.sin(target - actual), math.cos(target - actual))


class LocalMotionBackend:
    """Run GlobalRoute -> LocalMotionPort -> GeometrySafety without HuNav."""

    def __init__(
        self,
        node,
        *,
        move_service_name: str,
        config: Mapping[str, object] | None = None,
    ):
        self._node = node
        self._logger = node.get_logger()
        self._config = dict(config or {})
        self._isaac = IsaacPeopleBackend(node, move_service_name)
        self._external_stream = ExternalMotionStreamPublisher(
            node,
            str(
                self._config.get(
                    "external_motion_topic",
                    "/isaac/pedestrian_external_motion",
                )
            ),
        )
        self._compute_hz = max(1.0, float(self._config.get("compute_hz", 10.0)))
        self._state_timeout_sec = max(
            0.1, float(self._config.get("state_timeout_sec", 0.5))
        )
        self._external_timeout_sec = max(
            1.0,
            4.0 / self._compute_hz,
            float(self._config.get("external_timeout_sec", 0.35)),
        )
        self._lookahead_m = max(
            0.10, float(self._config.get("lookahead_distance_m", 0.70))
        )
        self._arrival_tolerance_m = max(
            0.05, float(self._config.get("settled_arrival_tolerance_m", 0.25))
        )
        self._yaw_tolerance_rad = max(
            0.01,
            float(self._config.get("terminal_align_yaw_tolerance_rad", 0.15)),
        )
        self._terminal_speed_mps = max(
            0.0, float(self._config.get("terminal_align_max_speed_mps", 0.05))
        )
        # Keep this run inside the currently validated root-motion envelope.
        # This is a configurable safety baseline, not a permanent character
        # speed limit; faster gait bands need their own tracking calibration.
        self._embodiment_max_speed_mps = max(
            0.05, float(self._config.get("embodiment_max_speed_mps", 0.31))
        )
        self._agent_radius_m = max(
            0.01, float(self._config.get("agent_radius_m", 0.26))
        )
        behavior_cfg = dict(self._config.get("behavior", {}) or {})
        sampled_cfg = dict(self._config.get("sampled_rvo", {}) or {})
        safety_cfg = dict(self._config.get("safety", {}) or {})
        self._agent_half_length_m = max(
            0.0,
            float(
                safety_cfg.get("swept_envelope", {}).get(
                    "half_length_m",
                    0.16,
                )
            ),
        )
        envelope_cfg = SweptEnvelopeConfig.from_mapping(
            safety_cfg.get("swept_envelope", {}) or {}
        )
        if envelope_cfg.disc_radius_m < self._agent_radius_m:
            envelope_cfg = replace(
                envelope_cfg,
                disc_radius_m=self._agent_radius_m,
            )
        envelope = SweptEnvelope(envelope_cfg)
        self._geometry = SweptEnvelopeGeometrySafety(envelope)
        self._pipeline = LocalMotionPipeline(
            ContextualBehaviorPolicy(
                ContextualBehaviorConfig(
                    conflict_horizon_sec=float(
                        behavior_cfg.get("conflict_horizon_sec", 2.0)
                    ),
                    minimum_passing_clearance_m=float(
                        behavior_cfg.get("minimum_passing_clearance_m", 0.45)
                    ),
                    following_speed_scale=float(
                        behavior_cfg.get("following_speed_scale", 0.8)
                    ),
                )
            ),
            SampledRvoLocalMotion(
                SampledRvoConfig(
                    time_horizon_sec=float(
                        sampled_cfg.get("time_horizon_sec", 2.0)
                    ),
                    neighbor_distance_m=float(
                        sampled_cfg.get("neighbor_distance_m", 3.0)
                    ),
                    clearance_m=float(sampled_cfg.get("clearance_m", 0.05)),
                    angular_samples=int(sampled_cfg.get("angular_samples", 36)),
                    speed_samples=tuple(
                        float(value)
                        for value in sampled_cfg.get(
                            "speed_samples",
                            (1.0, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25),
                        )
                    ),
                    boundary_samples=int(sampled_cfg.get("boundary_samples", 3)),
                    max_acceleration_mps2=float(
                        sampled_cfg.get("max_acceleration_mps2", 2.0)
                    ),
                    max_heading_rate_rps=float(
                        sampled_cfg.get("max_heading_rate_rps", math.radians(240.0))
                    ),
                    dynamic_clearance_target_m=float(
                        sampled_cfg.get("dynamic_clearance_target_m", 0.15)
                    ),
                    dynamic_clearance_weight=float(
                        sampled_cfg.get("dynamic_clearance_weight", 8.0)
                    ),
                    overlap_recovery_horizon_sec=float(
                        sampled_cfg.get("overlap_recovery_horizon_sec", 0.50)
                    ),
                    overlap_recovery_min_gain_m=float(
                        sampled_cfg.get("overlap_recovery_min_gain_m", 0.005)
                    ),
                ),
                geometry_safety=self._geometry,
            ),
        )
        self._commands: dict[str, MotionCommand] = {}
        self._snapshots: dict[str, AgentSnapshot] = {}
        self._robot: AgentSnapshot | None = None
        self._trackers: dict[str, PolylineLookaheadTracker] = {}
        self._settled: set[str] = set()
        self._stream_batch: dict[str, MotionCommand] = {}
        self._commanded_velocity: dict[str, tuple[float, float]] = {}
        self._last_tick_at = time.monotonic()
        self._last_diagnostic_at: dict[str, float] = {}
        self._timer = node.create_timer(1.0 / self._compute_hz, self._tick)
        self._logger.info(
            "Independent local-motion backend configured: "
            f"compute_hz={self._compute_hz:.1f}, lookahead_m={self._lookahead_m:.2f}, "
            f"external_timeout_sec={self._external_timeout_sec:.2f}, "
            f"embodiment_max_speed_mps={self._embodiment_max_speed_mps:.2f}, "
            f"envelope_radius_m={envelope_cfg.disc_radius_m:.2f}"
        )

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._isaac.wait_for_service(timeout_sec)

    @property
    def agent_radius_m(self) -> float:
        return self._agent_radius_m

    @property
    def agent_half_length_m(self) -> float:
        return self._agent_half_length_m

    def set_walkable_planner(self, planner) -> None:
        self._geometry.bind(planner)
        self._logger.info(
            "Independent local-motion geometry attached to walkable map: "
            f"resolution={planner.resolution:.3f}"
        )

    def register_agents(self, commands: Sequence[MotionCommand]) -> None:
        # Pre-spawn roster commands are parking holds, not active motion goals.
        del commands

    def update_agent_snapshot(self, snapshot: AgentSnapshot) -> None:
        self._snapshots[str(snapshot.agent_id)] = snapshot

    def update_robot_snapshot(self, snapshot: AgentSnapshot | None) -> None:
        self._robot = snapshot

    def send(self, command: MotionCommand, done_callback=None):
        agent_id = str(command.agent_id)
        if command.use_direct_pose or command.stop:
            self._external_stream.cancel((agent_id,))
            self._stream_batch.pop(agent_id, None)
            self._commanded_velocity.pop(agent_id, None)
            if command.stop:
                self._commands.pop(agent_id, None)
                self._trackers.pop(agent_id, None)
                self._settled.discard(agent_id)
            return self._isaac.send(command, done_callback)
        self._commands[agent_id] = command
        self._trackers.pop(agent_id, None)
        self._settled.discard(agent_id)
        return _completed_future(accepted=True, done_callback=done_callback)

    def remove_agent(self, agent_id: str) -> None:
        agent_id = str(agent_id)
        self._external_stream.cancel((agent_id,))
        self._commands.pop(agent_id, None)
        self._snapshots.pop(agent_id, None)
        self._trackers.pop(agent_id, None)
        self._settled.discard(agent_id)
        self._stream_batch.pop(agent_id, None)
        self._commanded_velocity.pop(agent_id, None)

    def allows_director_stall_recovery(self, agent_id: str) -> bool:
        return False

    def is_goal_settled(
        self,
        agent_id: str,
        current_pose: Sequence[float],
        target_pose: Sequence[float],
    ) -> bool:
        del current_pose, target_pose
        return str(agent_id) in self._settled

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(1.0 / self._compute_hz, min(0.25, now - self._last_tick_at))
        self._last_tick_at = now
        for agent_id, command in tuple(self._commands.items()):
            snapshot = self._snapshots.get(agent_id)
            if snapshot is None or now - snapshot.timestamp_sec > self._state_timeout_sec:
                continue
            self._step_agent(command, snapshot, now, dt)
        self._external_stream.publish(tuple(self._stream_batch.values()))
        self._stream_batch.clear()

    def _step_agent(
        self,
        command: MotionCommand,
        snapshot: AgentSnapshot,
        now: float,
        dt: float,
    ) -> None:
        distance = math.hypot(
            snapshot.x - float(command.goal_pose[0]),
            snapshot.y - float(command.goal_pose[1]),
        )
        if distance <= self._arrival_tolerance_m:
            self._send_terminal_alignment(command, snapshot)
            return
        tracker = self._trackers.get(command.agent_id)
        if tracker is None:
            points = [
                (snapshot.x, snapshot.y, snapshot.z),
                *command.path_points,
                tuple(float(value) for value in command.goal_pose[:3]),
            ]
            tracker = PolylineLookaheadTracker(
                points,
                lookahead_m=self._lookahead_m,
            )
            self._trackers[command.agent_id] = tracker
        target = tracker.update(snapshot.x, snapshot.y)
        route = RoutePlan.from_points(
            agent_id=command.agent_id,
            points=(
                (snapshot.x, snapshot.y, snapshot.z),
                *command.path_points,
                tuple(float(value) for value in command.goal_pose[:3]),
            ),
            planner_id="director_global_route",
        )
        peers = tuple(
            replace(
                item,
                vx=float(self._commanded_velocity.get(other_id, (item.vx, item.vy))[0]),
                vy=float(self._commanded_velocity.get(other_id, (item.vx, item.vy))[1]),
            )
            for other_id, item in self._snapshots.items()
            if other_id != command.agent_id
            and now - item.timestamp_sec <= self._state_timeout_sec
        )
        commanded_velocity = self._commanded_velocity.get(
            command.agent_id,
            (snapshot.vx, snapshot.vy),
        )
        planning_snapshot = replace(
            snapshot,
            vx=float(commanded_velocity[0]),
            vy=float(commanded_velocity[1]),
        )
        robot = self._fresh_robot(now)
        result = self._pipeline.step(
            BehaviorPolicyRequest(
                timestamp_sec=now,
                agent=planning_snapshot,
                route=route,
                preferred_speed_mps=min(
                    float(command.velocity), self._embodiment_max_speed_mps
                ),
                robot=robot,
                peers=peers,
                task_phase=str(command.phase),
            ),
            dt_sec=dt,
            route_target_xy=(target.x, target.y),
        )
        velocity = result.motion.velocity_xy
        self._commanded_velocity[command.agent_id] = velocity
        predicted = (
            snapshot.x + velocity[0] * dt,
            snapshot.y + velocity[1] * dt,
            snapshot.z,
        )
        external = MotionCommand(
            agent_id=command.agent_id,
            goal_pose=command.goal_pose,
            path_points=(),
            velocity=math.hypot(*velocity),
            orientation=result.motion.heading_rad,
            direct_pose=predicted,
            use_external_motion=True,
            external_velocity=(velocity[0], velocity[1], 0.0),
            external_timeout_sec=self._external_timeout_sec,
            external_motion_mode=EXTERNAL_MOTION_LOCOMOTION,
            phase=command.phase,
            behavior=command.behavior,
        )
        self._queue_external(command.agent_id, external)
        if now - self._last_diagnostic_at.get(command.agent_id, 0.0) >= 1.0:
            self._last_diagnostic_at[command.agent_id] = now
            self._logger.info(
                "Local motion diagnostics: "
                f"agent={command.agent_id}, phase={command.phase or 'UNSPECIFIED'}, "
                f"feasible={result.motion.feasible}, "
                f"command_speed={math.hypot(*velocity):.3f}, "
                f"actual_speed={math.hypot(snapshot.vx, snapshot.vy):.3f}, "
                f"remaining_m={target.remaining_m:.3f}, "
                f"static_clearance_m="
                f"{float(result.motion.diagnostics.get('minimum_static_clearance_m', math.nan)):.3f}, "
                "service_inflight=False, "
                f"static_rejected={result.motion.diagnostics.get('static_rejected_count', 0)}, "
                f"dynamic_rejected={result.motion.diagnostics.get('dynamic_rejected_count', 0)}, "
                f"dynamic_clearance_m="
                f"{float(result.motion.diagnostics.get('minimum_dynamic_clearance_m', math.inf)):.3f}, "
                f"peer_clearance_m="
                f"{float(result.motion.diagnostics.get('minimum_peer_dynamic_clearance_m', math.inf)):.3f}, "
                f"robot_clearance_m="
                f"{float(result.motion.diagnostics.get('minimum_robot_dynamic_clearance_m', math.inf)):.3f}, "
                f"soft_clearance_target_m="
                f"{float(result.motion.diagnostics.get('effective_dynamic_clearance_target_m', 0.0)):.3f}, "
                f"overlap_recovery_neighbors="
                f"{int(result.motion.diagnostics.get('overlap_recovery_neighbor_count', 0))}, "
                f"pose=({snapshot.x:.3f},{snapshot.y:.3f}), "
                f"robot_fresh={robot is not None}, "
                f"robot_distance_m="
                f"{math.hypot(robot.x - snapshot.x, robot.y - snapshot.y) if robot is not None else math.inf:.3f}, "
                "transport=stream_batch"
            )

    def _send_terminal_alignment(
        self,
        command: MotionCommand,
        snapshot: AgentSnapshot,
    ) -> None:
        yaw_aligned = (
            abs(_yaw_error(float(command.orientation), snapshot.yaw))
            <= self._yaw_tolerance_rad
        )
        speed = math.hypot(snapshot.vx, snapshot.vy)
        if yaw_aligned and speed <= self._terminal_speed_mps:
            self._settled.add(command.agent_id)
        external = MotionCommand(
            agent_id=command.agent_id,
            goal_pose=command.goal_pose,
            path_points=(),
            velocity=0.0,
            orientation=float(command.orientation),
            direct_pose=(snapshot.x, snapshot.y, snapshot.z),
            use_external_motion=True,
            external_velocity=(0.0, 0.0, 0.0),
            external_timeout_sec=self._external_timeout_sec,
            external_motion_mode=EXTERNAL_MOTION_TERMINAL_ALIGN,
            phase=command.phase,
            behavior=command.behavior,
        )
        self._commanded_velocity[command.agent_id] = (0.0, 0.0)
        self._queue_external(command.agent_id, external)

    def _queue_external(self, agent_id: str, command: MotionCommand) -> None:
        """Coalesce each agent to one command in the current world tick."""
        self._stream_batch[str(agent_id)] = command

    def _fresh_robot(self, now: float) -> AgentSnapshot | None:
        if self._robot is None:
            return None
        if now - self._robot.timestamp_sec > self._state_timeout_sec:
            return None
        return self._robot
