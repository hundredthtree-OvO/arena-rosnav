"""Execute an authored-route EpisodeSpec against the Isaac People services."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from typing import Sequence

import rclpy
from isaacsim_msgs.msg import Person
from isaacsim_msgs.srv import DeletePrim, Pedestrian, ResetRobot
from people_msgs.msg import People
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ..domain.task import MotionCommand
from ..episodes.schema import EpisodeSpec
from ..motion_backend import IsaacPeopleBackend
from ..pedestrian_state_stream import observations_from_message
from ..walkable_map_planner import WalkableMapPlanner, WalkableMapPlannerConfig
from .authored_scenario_core import (
    RouteRuntime,
    expand_authored_route,
    with_agent_id_suffix,
)


class AuthoredScenarioNode(Node):
    def __init__(
        self,
        episode: EpisodeSpec,
        *,
        spawn_service: str = "/isaac/spawn_pedestrian",
        move_service: str = "/isaac/move_pedestrians",
        reset_service: str = "/isaac/reset_mecanum_episode",
        retire_service: str = "/isaac/delete_prim",
        people_topic: str = "/isaac/pedestrian_states",
        skip_robot_reset: bool = False,
        planner_radius_m: float = 0.30,
        status_topic: str = "/toilet_benchmark/pedestrian_runtime_status",
        cancel_service: str = "/toilet_authored_scenario/cancel",
    ) -> None:
        super().__init__("toilet_authored_scenario")
        if episode.task_type != "authored_route":
            raise ValueError("authored scenario runner requires task_type=authored_route")
        if not episode.pedestrians:
            raise ValueError("authored scenario needs at least one pedestrian")
        self.episode = episode
        self.skip_robot_reset = bool(skip_robot_reset)
        planned_specs = plan_episode_routes(episode, planner_radius_m=planner_radius_m)
        for spec, planned in zip(episode.pedestrians, planned_specs):
            self.get_logger().info(
                f"{spec.agent_id} route planned: authored_targets={len(spec.route_waypoints)}, "
                f"planned_points={len(planned.route_waypoints)}"
            )
        self._runtimes = {
            spec.agent_id: RouteRuntime.create(spec) for spec in planned_specs
        }
        self._poses: dict[str, tuple[float, float, float]] = {}
        self._pose_times: dict[str, float] = {}
        self._pose_metadata: dict[str, dict[str, str]] = {}
        self._backend = IsaacPeopleBackend(self, move_service)
        self._spawn_client = self.create_client(Pedestrian, spawn_service)
        self._reset_client = self.create_client(ResetRobot, reset_service)
        self._retire_client = self.create_client(DeletePrim, retire_service)
        self._status_pub = self.create_publisher(String, status_topic, 10) if status_topic else None
        self._cancel_service = self.create_service(Trigger, cancel_service, self._cancel_cb)
        self._active_announced: set[str] = set()
        self._retire_retry_at: dict[str, float] = {}
        self._retire_inflight: set[str] = set()
        self._cancel_requested = False
        self._cancel_stop_sent: set[str] = set()
        self._cancel_stable_samples: dict[str, int] = {}
        self._cancel_last_pose: dict[str, tuple[float, float, float]] = {}
        self._departure_started_at: dict[str, float] = {}
        self._departure_max_lateral_m: dict[str, float] = {}
        self._departure_reported: set[str] = set()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(People, people_topic, self._people_cb, qos)
        self._timer = self.create_timer(0.1, self._tick)
        self._started_at = time.monotonic()
        self._spawned = False
        self._finished = False
        self._activation_tolerance_m = 0.15
        self._activation_stable_samples = 4
        self._activation_position_stability_m = 0.02
        self._activation_yaw_tolerance_rad = 0.20
        self._activation_yaw_stability_rad = 0.08
        self._boundary_ack_tolerance_m = min(
            0.15,
            float(self.episode.termination.goal_tolerance_m),
        )
        self.exit_code = 1

    @property
    def finished(self) -> bool:
        return self._finished

    def start(self, service_timeout_sec: float = 20.0) -> None:
        if not self._spawn_client.wait_for_service(timeout_sec=service_timeout_sec):
            raise RuntimeError("spawn pedestrian service is unavailable")
        if not self._backend.wait_for_service(service_timeout_sec):
            raise RuntimeError("move pedestrian service is unavailable")
        if not self._retire_client.wait_for_service(timeout_sec=service_timeout_sec):
            raise RuntimeError("pedestrian retirement service is unavailable")
        if not self.skip_robot_reset:
            if not self._reset_client.wait_for_service(timeout_sec=service_timeout_sec):
                raise RuntimeError("robot reset service is unavailable")
            self._reset_robot(service_timeout_sec)
        self._spawn_pedestrians(service_timeout_sec)
        self._spawned = True
        self._started_at = time.monotonic()
        self.get_logger().info(
            f"Authored scenario started: episode={self.episode.episode_id}, "
            f"pedestrians={len(self._runtimes)}, seed={self.episode.seed}"
        )

    def _reset_robot(self, timeout_sec: float) -> None:
        request = ResetRobot.Request()
        request.name = self.episode.robot.model
        x, y, z, yaw = self.episode.robot.start_pose
        request.pose.position.x = float(x)
        request.pose.position.y = float(y)
        request.pose.position.z = float(z)
        request.pose.orientation.z = math.sin(float(yaw) * 0.5)
        request.pose.orientation.w = math.cos(float(yaw) * 0.5)
        future = self._reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        if response is None or not bool(response.accepted):
            raise RuntimeError("Isaac rejected robot reset")

    def _spawn_pedestrians(self, timeout_sec: float) -> None:
        request = Pedestrian.Request()
        for runtime in self._runtimes.values():
            spec = runtime.spec
            if spec.start_pose is None:
                raise ValueError(f"{spec.agent_id}: start_pose is required")
            person = Person()
            person.stage_prefix = spec.agent_id
            person.character_name = str(spec.character or "original_female_adult_business_02")
            person.initial_pose = [float(value) for value in spec.start_pose]
            person.goal_pose = [float(value) for value in spec.start_pose]
            person.path_points_flat = []
            person.loop_path = False
            person.orientation = float(spec.start_yaw or 0.0)
            person.controller_stats = False
            person.velocity = 0.0
            request.people.append(person)
        future = self._spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result() if future.done() else None
        if response is None or not bool(response.ret):
            raise RuntimeError("Isaac rejected pedestrian spawn/reactivation")

    def _people_cb(self, message: People) -> None:
        observed_at = time.monotonic()
        for observation in observations_from_message(message, observed_at=observed_at):
            if observation.identifier in self._runtimes:
                self._poses[observation.identifier] = observation.position
                self._pose_times[observation.identifier] = observed_at
                self._pose_metadata[observation.identifier] = dict(observation.metadata)

    def _tick(self) -> None:
        if not self._spawned or self._finished:
            return
        now = time.monotonic()
        if self._cancel_requested:
            self._advance_cancel()
            if all(runtime.complete for runtime in self._runtimes.values()):
                self.get_logger().info("Authored scenario cancelled after stable pedestrian retirement")
                self._finish(0)
            return
        if now - self._started_at > self.episode.termination.timeout_sec:
            self.get_logger().error("Authored scenario timed out")
            self._finish(1)
            return
        for agent_id, runtime in self._runtimes.items():
            if runtime.complete:
                continue
            if runtime.state == "RETIRING":
                if (
                    agent_id not in self._retire_inflight
                    and now >= self._retire_retry_at.get(agent_id, 0.0)
                ):
                    self._request_retire(runtime)
                continue
            if runtime.state == "HOLDING":
                action = runtime.release_hold(now)
                if action == "dispatch":
                    self._dispatch(runtime)
                elif action == "complete":
                    self._retire(runtime, reason="completed after terminal hold")
                continue
            pose = self._poses.get(agent_id)
            if pose is None or now - self._pose_times.get(agent_id, 0.0) > 2.0:
                continue
            if runtime.state == "WAITING_FOR_POSE":
                metadata = self._pose_metadata.get(agent_id, {})
                if not runtime.observe_activation(
                    pose,
                    metadata,
                    position_tolerance_m=self._activation_tolerance_m,
                    position_stability_m=self._activation_position_stability_m,
                    yaw_tolerance_rad=self._activation_yaw_tolerance_rad,
                    yaw_stability_rad=self._activation_yaw_stability_rad,
                    required_samples=self._activation_stable_samples,
                ):
                    continue
                self.get_logger().info(
                    f"{agent_id} activation stabilized: "
                    f"generation={runtime.generation}, "
                    f"samples={runtime.activation_stable_samples}, "
                    f"pose=({pose[0]:.3f},{pose[1]:.3f}), "
                    f"yaw={runtime.activation_yaw:.3f}"
                )
                self._dispatch(runtime)
                continue
            if runtime.state == "WAITING_FOR_EXECUTION":
                metadata = self._pose_metadata.get(agent_id, {})
                try:
                    generation = int(metadata.get("embodiment_generation", "0"))
                    command_generation = int(metadata.get("command_generation", "0"))
                except ValueError:
                    continue
                if (
                    generation != runtime.generation
                    or command_generation < runtime.command_generation
                    or metadata.get("motion_state", "").lower() != "executing"
                ):
                    continue
                runtime.state = "MOVING"
                self._announce_active(runtime)
                continue
            self._observe_departure(runtime, pose, now)
            target = runtime.spec.route_waypoints[runtime.current_boundary]
            if math.hypot(pose[0] - target[0], pose[1] - target[1]) > self._boundary_ack_tolerance_m:
                continue
            action = runtime.arrive(now)
            self._send_stop(runtime)
            if action == "hold":
                self.get_logger().info(
                    f"{agent_id} holding at waypoint {runtime.current_boundary} "
                    f"for {runtime.hold_duration():.2f}s"
                )
            elif action == "complete":
                self._retire(runtime, reason="reached route terminal")
            elif action == "dispatch":
                self._dispatch(runtime)
        if all(runtime.complete for runtime in self._runtimes.values()):
            self.get_logger().info("Authored scenario completed")
            self._finish(0)

    def _cancel_cb(self, _request, response):
        if self._finished:
            response.success = True
            response.message = "authored scenario is already finished"
            return response
        self._cancel_requested = True
        self._publish_status("pedestrian_cancel_requested", episode_id=self.episode.episode_id)
        response.success = True
        response.message = "graceful pedestrian cancellation accepted"
        return response

    def _advance_cancel(self) -> None:
        for agent_id, runtime in self._runtimes.items():
            if runtime.complete:
                continue
            if runtime.state == "RETIRING":
                if agent_id not in self._retire_inflight:
                    self._request_retire(runtime)
                continue
            pose = self._poses.get(agent_id)
            metadata = self._pose_metadata.get(agent_id, {})
            if pose is None:
                continue
            if agent_id not in self._cancel_stop_sent:
                self._send_stop_at_pose(runtime, pose, metadata)
                self._cancel_stop_sent.add(agent_id)
                self._cancel_last_pose[agent_id] = pose
                self._cancel_stable_samples[agent_id] = 0
                runtime.state = "CANCELLING"
                continue
            previous = self._cancel_last_pose.get(agent_id, pose)
            stable = (
                metadata.get("motion_state", "").lower() == "idle"
                and math.dist(previous[:2], pose[:2]) <= 0.01
            )
            self._cancel_last_pose[agent_id] = pose
            self._cancel_stable_samples[agent_id] = (
                self._cancel_stable_samples.get(agent_id, 0) + 1 if stable else 0
            )
            if self._cancel_stable_samples[agent_id] >= 4:
                self._retire(runtime, reason="graceful cancellation reached stable idle")

    def _send_stop_at_pose(
        self,
        runtime: RouteRuntime,
        pose: Sequence[float],
        metadata: dict[str, str],
    ) -> None:
        try:
            yaw = float(metadata.get("yaw_rad", runtime.spec.start_yaw or 0.0))
        except (TypeError, ValueError):
            yaw = float(runtime.spec.start_yaw or 0.0)
        self._backend.send(
            MotionCommand(
                agent_id=runtime.spec.agent_id,
                goal_pose=tuple(float(value) for value in pose[:3]),
                path_points=(),
                velocity=0.0,
                orientation=yaw,
                stop=True,
                constrain_to_path=runtime.spec.constrain_to_path,
                phase="AUTHORED_CANCEL",
            )
        )

    def _dispatch(self, runtime: RouteRuntime) -> None:
        segment = runtime.execution_segment()
        if not segment:
            raise RuntimeError(
                f"{runtime.spec.agent_id}: authored motion segment has no forward targets"
            )
        target = segment[-1]
        speed = runtime.spec.behavior.walking_speed_mps or 0.8
        heading_start = runtime.spec.route_waypoints[runtime.previous_boundary]
        heading_end = segment[-1] if len(segment) == 1 else segment[-2]
        yaw = math.atan2(
            segment[-1][1] - heading_end[1],
            segment[-1][0] - heading_end[0],
        )
        if len(segment) == 1:
            yaw = math.atan2(
                segment[-1][1] - heading_start[1],
                segment[-1][0] - heading_start[0],
            )
        metadata = self._pose_metadata.get(runtime.spec.agent_id, {})
        try:
            baseline_generation = int(metadata.get("command_generation", "0"))
        except ValueError:
            baseline_generation = 0
        runtime.command_generation = baseline_generation + 1
        if runtime.spec.agent_id not in self._departure_started_at:
            self._departure_started_at[runtime.spec.agent_id] = time.monotonic()
            self._departure_max_lateral_m[runtime.spec.agent_id] = 0.0
        self._backend.send(
            MotionCommand(
                agent_id=runtime.spec.agent_id,
                goal_pose=target,
                path_points=segment,
                velocity=float(speed),
                orientation=yaw,
                constrain_to_path=runtime.spec.constrain_to_path,
                phase="AUTHORED_ROUTE",
                behavior=runtime.spec.behavior.behavior_type,
            )
        )
        runtime.state = (
            "MOVING"
            if runtime.spec.agent_id in self._active_announced
            else "WAITING_FOR_EXECUTION"
        )
        self.get_logger().info(
            f"{runtime.spec.agent_id} dispatched segment "
            f"{runtime.previous_boundary}->{runtime.current_boundary}, "
            f"generation={runtime.generation}, "
            f"command_generation={runtime.command_generation}, "
            "transport=people_pathpoints"
        )

    def _observe_departure(
        self,
        runtime: RouteRuntime,
        pose: Sequence[float],
        now: float,
    ) -> None:
        agent_id = runtime.spec.agent_id
        started_at = self._departure_started_at.get(agent_id)
        if started_at is None or agent_id in self._departure_reported:
            return
        start, forward_target = runtime.spec.route_waypoints[:2]
        dx, dy = forward_target[0] - start[0], forward_target[1] - start[1]
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            return
        ux, uy = dx / length, dy / length
        offset_x, offset_y = float(pose[0]) - start[0], float(pose[1]) - start[1]
        progress = offset_x * ux + offset_y * uy
        lateral = abs(-offset_x * uy + offset_y * ux)
        self._departure_max_lateral_m[agent_id] = max(
            self._departure_max_lateral_m.get(agent_id, 0.0), lateral
        )
        elapsed = now - started_at
        if elapsed < 2.0:
            return
        self._departure_reported.add(agent_id)
        metadata = self._pose_metadata.get(agent_id, {})
        try:
            observed_yaw = float(metadata.get("yaw_rad", "nan"))
        except (TypeError, ValueError):
            observed_yaw = math.nan
        fields = {
            "agent_id": agent_id,
            "episode_id": self.episode.episode_id,
            "generation": runtime.generation,
            "elapsed_sec": round(elapsed, 4),
            "forward_progress_m": round(progress, 4),
            "lateral_offset_m": round(lateral, 4),
            "max_lateral_offset_m": round(self._departure_max_lateral_m[agent_id], 4),
            "expected_root_yaw": round(float(runtime.spec.start_yaw or 0.0), 5),
            "observed_root_yaw": None if not math.isfinite(observed_yaw) else round(observed_yaw, 5),
        }
        self.get_logger().info(
            "Authored departure diagnostic: " + json.dumps(fields, sort_keys=True)
        )
        self._publish_status("pedestrian_departure_diagnostic", **fields)

    def _announce_active(self, runtime: RouteRuntime) -> None:
        if runtime.spec.agent_id in self._active_announced:
            return
        self._active_announced.add(runtime.spec.agent_id)
        self._publish_status(
            "pedestrian_active",
            agent_id=runtime.spec.agent_id,
            phase="AUTHORED_ROUTE",
            episode_id=self.episode.episode_id,
            generation=runtime.generation,
            command_generation=runtime.command_generation,
        )

    def _publish_status(self, event: str, **fields) -> None:
        if self._status_pub is None:
            return
        self._status_pub.publish(
            String(data=json.dumps({"event": str(event), "source": "authored_route", **fields}, sort_keys=True))
        )

    def _send_stop(self, runtime: RouteRuntime) -> None:
        target = runtime.spec.route_waypoints[runtime.current_boundary]
        self._backend.send(
            MotionCommand(
                agent_id=runtime.spec.agent_id,
                goal_pose=target,
                path_points=(),
                velocity=0.0,
                orientation=float(runtime.spec.start_yaw or 0.0),
                stop=True,
                constrain_to_path=runtime.spec.constrain_to_path,
                phase="AUTHORED_HOLD",
            )
        )

    def _retire(self, runtime: RouteRuntime, *, reason: str) -> None:
        runtime.state = "RETIRING"
        self._retire_retry_at[runtime.spec.agent_id] = 0.0
        self._request_retire(runtime)
        self.get_logger().info(
            f"Parking {runtime.spec.agent_id} after authored route: {reason}."
        )

    def _request_retire(self, runtime: RouteRuntime) -> None:
        agent_id = runtime.spec.agent_id
        if agent_id in self._retire_inflight:
            return
        request = DeletePrim.Request()
        request.name = f"/World/Characters/{agent_id}"
        future = self._retire_client.call_async(request)
        self._retire_inflight.add(agent_id)
        future.add_done_callback(
            lambda result, retired_agent=agent_id: self._retire_done(
                retired_agent,
                result,
            )
        )

    def _retire_done(self, agent_id: str, future) -> None:
        self._retire_inflight.discard(agent_id)
        runtime = self._runtimes.get(agent_id)
        if runtime is None:
            return
        try:
            response = future.result()
            success = bool(response is not None and response.ret)
        except Exception as exc:
            success = False
            self.get_logger().error(f"Failed to park {agent_id}: {exc}")
        if not success:
            self._retire_retry_at[agent_id] = time.monotonic() + 0.2
            return
        runtime.state = "COMPLETE"
        self._retire_retry_at.pop(agent_id, None)
        self._poses.pop(agent_id, None)
        self._pose_times.pop(agent_id, None)
        self._pose_metadata.pop(agent_id, None)
        self.get_logger().info(f"{agent_id} parked and removed from the visible scene.")
        self._publish_status(
            "pedestrian_retired",
            agent_id=agent_id,
            episode_id=self.episode.episode_id,
            generation=runtime.generation,
        )

    def _finish(self, exit_code: int) -> None:
        self.exit_code = int(exit_code)
        self._finished = True


def load_authored_episode(path: str | Path) -> EpisodeSpec:
    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    episode = EpisodeSpec.from_mapping(payload)
    if episode.task_type != "authored_route":
        raise ValueError("episode task_type must be authored_route")
    return episode


def plan_episode_routes(
    episode: EpisodeSpec,
    *,
    planner_radius_m: float = 0.30,
) -> tuple:
    map_path = str(episode.assets.get("walkable_map", "")).strip()
    if not map_path:
        raise ValueError("authored scenario assets.walkable_map is required")
    planner = WalkableMapPlanner.from_file(
        WalkableMapPlannerConfig(
            map_path=map_path,
            agent_radius_m=float(planner_radius_m),
            constrained_segment_length_m=0.35,
            preferred_clearance_m=max(0.35, float(planner_radius_m) + 0.08),
        )
    )
    return tuple(
        expand_authored_route(
            spec,
            lambda start, goal, route_planner=planner: route_planner.plan(
                list(start),
                list(goal),
                z=float(goal[2]),
            ),
        )
        for spec in episode.pedestrians
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Execute a route-editor authored episode in Isaac")
    parser.add_argument("--episode", required=True)
    parser.add_argument("--spawn-service", default="/isaac/spawn_pedestrian")
    parser.add_argument("--move-service", default="/isaac/move_pedestrians")
    parser.add_argument("--reset-service", default="/isaac/reset_mecanum_episode")
    parser.add_argument("--retire-service", default="/isaac/delete_prim")
    parser.add_argument("--people-topic", default="/isaac/pedestrian_states")
    parser.add_argument("--status-topic", default="/toilet_benchmark/pedestrian_runtime_status")
    parser.add_argument("--agent-id-suffix", default="")
    parser.add_argument("--service-timeout-sec", type=float, default=20.0)
    parser.add_argument("--planner-radius-m", type=float, default=0.30)
    parser.add_argument("--skip-robot-reset", action="store_true")
    parser.add_argument("--validate-only", action="store_true")
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _parser().parse_args(args)
    try:
        episode = with_agent_id_suffix(
            load_authored_episode(namespace.episode),
            namespace.agent_id_suffix,
        )
        planned_specs = plan_episode_routes(
            episode,
            planner_radius_m=namespace.planner_radius_m,
        )
        runtimes = [RouteRuntime.create(spec) for spec in planned_specs]
        if namespace.validate_only:
            print(json.dumps({
                "agent_ids": [runtime.spec.agent_id for runtime in runtimes],
                "episode_id": episode.episode_id,
                "pedestrian_count": len(runtimes),
                "valid": True,
            }, sort_keys=True))
            return 0
        rclpy.init(args=None)
        node = AuthoredScenarioNode(
            episode,
            spawn_service=namespace.spawn_service,
            move_service=namespace.move_service,
            reset_service=namespace.reset_service,
            retire_service=namespace.retire_service,
            people_topic=namespace.people_topic,
            skip_robot_reset=namespace.skip_robot_reset,
            planner_radius_m=namespace.planner_radius_m,
            status_topic=namespace.status_topic,
        )
        node.start(namespace.service_timeout_sec)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_code = node.exit_code
        node.destroy_node()
        rclpy.shutdown()
        return exit_code
    except (OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"failed to execute authored scenario: {exc}")
        if rclpy.ok():
            rclpy.shutdown()
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
