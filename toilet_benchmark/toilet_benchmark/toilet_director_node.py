from __future__ import annotations

import argparse
import math
import os
import random
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from isaacsim_msgs.msg import NavPed, Person
from isaacsim_msgs.srv import DeletePrim, GetPrimAttributes, MovePed, Pedestrian
from rclpy.node import Node

try:
    from people_msgs.msg import People
except Exception:  # pragma: no cover - optional runtime dependency
    People = None

try:
    from arena_people_msgs.msg import Pedestrians as ArenaPedestrians
except Exception:  # pragma: no cover - optional runtime dependency
    ArenaPedestrians = None

from .hunav_adapter import DirectedAgentState, HunavAdapter
from .pose_utils import SemanticPose, parse_semantic_pose
from .resource_manager import QueueSlot, Resource, ResourceManager
from .semantic_rules import (
    PlacementRule,
    QueueRule,
    build_queue_poses,
    resolve_local_placement,
    yaw_from_quaternion_xyzw,
)
from .voxel_path_planner import VoxelPathPlanner, VoxelPathPlannerConfig


def _load_yaml(path: str | Path) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _default_config_path(filename: str) -> str:
    try:
        return str(Path(get_package_share_directory("toilet_benchmark")) / "config" / filename)
    except PackageNotFoundError:
        return str(Path(__file__).resolve().parents[1] / "config" / filename)


def _planar_distance(a, b) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _parse_placement_rule(item: dict) -> PlacementRule | None:
    data = item.get("placement")
    if not data:
        return None
    return PlacementRule(
        offset_local=tuple(float(v) for v in data.get("offset_local", [0.0, 0.0, 0.0])),
        yaw_mode=str(data.get("yaw_mode", "prim")),
        yaw_local=float(data.get("yaw_local", 0.0)),
    )


def _parse_queue_rule(item: dict) -> QueueRule | None:
    data = item.get("queue_rule")
    if not data:
        return None
    return QueueRule(
        slots=int(data.get("slots", 0)),
        first_gap=float(data.get("first_gap", 0.4)),
        spacing=float(data.get("spacing", 0.4)),
        axis_local=tuple(float(v) for v in data.get("axis_local", [0.0, -1.0, 0.0])),
        yaw_mode=str(data.get("yaw_mode", "same_as_resource")),
        yaw_local=float(data.get("yaw_local", 0.0)),
    )


def _location_dict(entry_id: str, scene_prim: str, pose: SemanticPose) -> dict:
    return {
        "id": str(entry_id),
        "scene_prim": str(scene_prim),
        "position": pose.as_list(),
        "yaw": float(pose.yaw),
    }


def _pose_response_is_finite(response) -> bool:
    values = (
        float(response.pose.position.x),
        float(response.pose.position.y),
        float(response.pose.position.z),
        float(response.pose.orientation.w),
    )
    return all(math.isfinite(v) for v in values)


def _flatten_path_points(points: list[list[float]]) -> list[float]:
    return [float(value) for point in points for value in point[:3]]


@dataclass
class RuntimeAgent:
    agent_id: str
    character_name: str
    profile_name: str
    velocity: float
    resource_id: str
    current_phase: str
    current_pose: list[float]
    initial_yaw: float
    target_pose: list[float]
    target_yaw: float
    queue_slot_id: str | None = None
    reached_deadline: float = 0.0
    service_deadline: float = 0.0
    exit_pose: list[float] | None = None
    exit_yaw: float = 0.0
    last_pose_update: float = 0.0
    aligned_at_target: bool = False
    last_arrival_wait_log: float = 0.0
    last_pose_reject_log: float = 0.0
    target_reached: bool = False
    despawn_requested: bool = False
    spawned: bool = False
    spawn_requested: bool = False


class ToiletDirectorNode(Node):
    """Minimal director for ENTERING -> URINAL -> EXITING toilet benchmark flow."""

    def __init__(
        self,
        *,
        semantics_path: str,
        benchmark_path: str,
        spawn_service: str,
        move_service: str,
        initial_agents: int,
        character_name: str,
        character_pool: list[str] | None,
        profile_name: str,
    ):
        super().__init__("toilet_director_node")
        self.semantics_path = str(semantics_path)
        self.benchmark_path = str(benchmark_path)
        self.spawn_service_name = str(spawn_service)
        self.move_service_name = str(move_service)
        self.initial_agents = max(0, int(initial_agents))
        self.character_name = str(character_name)
        self.profile_name = str(profile_name)
        self.semantics = _load_yaml(self.semantics_path)
        self.benchmark = _load_yaml(self.benchmark_path)
        self.scene_root_path = str(
            self.semantics.get("scene_root_path")
            or self.semantics.get("scene_root")
            or "/World"
        ).rstrip("/")
        director_cfg = self.benchmark.get("director", {}) or {}
        self.arrival_tolerance_m = float(director_cfg.get("arrival_tolerance_m", 0.55))
        self.actor_stop_radius_m = float(director_cfg.get("actor_stop_radius_m", 0.5))
        self.exit_arrival_tolerance_m = float(director_cfg.get("exit_arrival_tolerance_m", 0.35))
        self.walk_plane_z = float(director_cfg.get("walk_plane_z", 0.0))
        self.pose_stale_sec = float(director_cfg.get("pose_stale_sec", 5.0))
        self.max_live_pose_jump_m = float(director_cfg.get("max_live_pose_jump_m", 3.0))
        self.allow_eta_fallback = bool(director_cfg.get("allow_eta_fallback", False))
        self.exit_when_complete = bool(director_cfg.get("exit_when_complete", True))
        self.initial_spawn_interval_sec = float(director_cfg.get("initial_spawn_interval_sec", 1.0))
        self.live_pose_topic = str(director_cfg.get("live_pose_topic", "/isaac/pedestrian_states"))
        self.path_planner: VoxelPathPlanner | None = None
        self.character_pool = [
            str(name).strip()
            for name in (character_pool or self.benchmark.get("character_pool", []) or [])
            if str(name).strip()
        ]
        self.entrances: list[dict] = []
        self.exits: list[dict] = []
        self.resources = ResourceManager({})
        self.adapter = HunavAdapter()
        self._rng = random.Random(int(self.benchmark.get("arrival", {}).get("seed", 12345)))
        self._spawn_client = self.create_client(Pedestrian, self.spawn_service_name)
        self._move_client = self.create_client(MovePed, self.move_service_name)
        self._get_prim_client = self.create_client(GetPrimAttributes, "/isaac/get_prim_attributes")
        self._delete_client = self.create_client(DeletePrim, "/isaac/delete_prim")
        self._spawned_agents: list[DirectedAgentState] = []
        self._runtime_agents: dict[str, RuntimeAgent] = {}
        self._pending_despawns: set[str] = set()
        self._initial_spawn_queue: list[str] = []
        self._initial_spawn_in_flight = False
        self._next_initial_spawn_time = 0.0
        self._pose_subscriptions = []
        self._bootstrap_phase = "await_services"
        self._bootstrap_index = 0
        self._bootstrap_pending_future = None
        self._bootstrap_pending_payload: dict | None = None
        self._bootstrap_resources: dict[str, Resource] = {}
        self._boot_timer = self.create_timer(0.5, self._bootstrap_once)
        self._fsm_timer = self.create_timer(0.5, self._advance_state_machine)
        self._bootstrapped = False
        self._setup_pose_subscriptions()
        self._setup_path_planner()

        self.get_logger().info(
            "Loaded toilet benchmark configs: "
            f"semantics={self.semantics_path}, benchmark={self.benchmark_path}, "
            f"scene_root_path={self.scene_root_path}, "
            f"spawn_service={self.spawn_service_name}, move_service={self.move_service_name}, "
            f"default_character={self.character_name}, character_pool={len(self.character_pool)}"
        )
        self.get_logger().info(
            f"Director tolerances: arrival_tolerance_m={self.arrival_tolerance_m:.2f}, "
            f"actor_stop_radius_m={self.actor_stop_radius_m:.2f}, "
            f"exit_arrival_tolerance_m={self.exit_arrival_tolerance_m:.2f}, "
            f"walk_plane_z={self.walk_plane_z:.3f}, "
            f"pose_stale_sec={self.pose_stale_sec:.1f}, allow_eta_fallback={self.allow_eta_fallback}, "
            f"max_live_pose_jump_m={self.max_live_pose_jump_m:.2f}, "
            f"exit_when_complete={self.exit_when_complete}, "
            f"initial_spawn_interval_sec={self.initial_spawn_interval_sec:.2f}"
        )

    def _setup_path_planner(self):
        planner_cfg = self.benchmark.get("path_planner", {}) or {}
        if not bool(planner_cfg.get("enabled", False)):
            self.get_logger().info("Toilet path planner disabled; Isaac navmesh fallback remains active.")
            return
        backend = str(planner_cfg.get("backend", "voxel")).strip().lower()
        if backend != "voxel":
            self.get_logger().warning(f"Unsupported toilet path planner backend={backend!r}; using navmesh fallback.")
            return
        map_path = str(
            planner_cfg.get("voxel_map_path")
            or os.environ.get("ARENA_ISAAC_VOXEL_MAP_PATH", "")
            or "/home/stardust/resources/arena_ws/arena_assets/collision_configs/shenxinfu_841837.voxel.json.gz"
        )
        try:
            self.path_planner = VoxelPathPlanner.from_file(
                VoxelPathPlannerConfig(
                    map_path=map_path,
                    z_min=float(planner_cfg.get("z_min", 0.05)),
                    z_max=float(planner_cfg.get("z_max", 1.2)),
                    agent_radius_m=float(planner_cfg.get("agent_radius_m", 0.25)),
                    bounds_padding_m=float(planner_cfg.get("bounds_padding_m", 1.0)),
                    max_expansions=int(planner_cfg.get("max_expansions", 20000)),
                    nearest_free_radius_m=float(planner_cfg.get("nearest_free_radius_m", 0.8)),
                    simplify=bool(planner_cfg.get("simplify", True)),
                )
            )
            self.get_logger().info(
                f"Toilet voxel path planner loaded: map={map_path}, "
                f"resolution={self.path_planner.resolution:.3f}, "
                f"occupied={len(self.path_planner.occupied)}, inflated={len(self.path_planner.inflated_occupied)}"
            )
        except Exception as exc:
            self.path_planner = None
            self.get_logger().warning(f"Failed to load toilet voxel path planner; using navmesh fallback: {exc}")

    def _setup_pose_subscriptions(self):
        if People is not None:
            self._pose_subscriptions.append(
                self.create_subscription(People, self.live_pose_topic, self._people_cb, 10)
            )
            self.get_logger().info(f"Subscribed to {self.live_pose_topic} for live pedestrian poses.")
            self._pose_subscriptions.append(
                self.create_subscription(People, "/task_generator_node/people", self._people_cb, 10)
            )
            self.get_logger().info("Subscribed to /task_generator_node/people as a fallback pedestrian pose source.")
        if ArenaPedestrians is not None and self.live_pose_topic != "/task_generator_node/arena_peds":
            self._pose_subscriptions.append(
                self.create_subscription(ArenaPedestrians, "/task_generator_node/arena_peds", self._people_cb, 10)
            )
            self.get_logger().info("Subscribed to /task_generator_node/arena_peds for live pedestrian poses.")
        if not self._pose_subscriptions:
            self.get_logger().warning(
                "No pedestrian pose topic types available; arrival progression will stall unless "
                "director.allow_eta_fallback is enabled."
            )

    def _bootstrap_once(self):
        if self._bootstrapped:
            return
        if not self._services_ready():
            return

        if self._bootstrap_phase == "await_services":
            self._bootstrap_phase = "resolve_entrances"
            self._bootstrap_index = 0
            self._bootstrap_resources = {}
            self.get_logger().info("Resolving semantic layout from scene_prim rules...")

        if self._bootstrap_pending_future is not None:
            if not self._bootstrap_pending_future.done():
                return
            payload = self._bootstrap_pending_payload or {}
            future = self._bootstrap_pending_future
            self._bootstrap_pending_future = None
            self._bootstrap_pending_payload = None
            try:
                response = future.result()
            except Exception as exc:
                self.get_logger().warning(f"Bootstrap request failed during {payload.get('action', 'unknown')}: {exc}")
                return
            try:
                self._handle_bootstrap_response(payload, response)
            except Exception as exc:
                self.get_logger().warning(
                    f"Bootstrap response handling failed during {payload.get('action', 'unknown')}: {exc}"
                )
                return
            if self._bootstrap_pending_future is not None:
                return

        try:
            self._dispatch_bootstrap_step()
        except Exception as exc:
            self.get_logger().warning(f"Bootstrap dispatch failed during phase {self._bootstrap_phase}: {exc}")
            return

        if self._bootstrap_phase != "ready":
            return

        self._bootstrapped = True
        self._boot_timer.cancel()
        if self.initial_agents <= 0:
            self.get_logger().info("Bootstrap complete; initial_agents=0 so no pedestrians were spawned.")
            return
        directed_agents = self._build_initial_agents(self.initial_agents)
        self._queue_initial_spawns(directed_agents)

    def _services_ready(self) -> bool:
        if not self._spawn_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info(f"Waiting for service: {self.spawn_service_name}")
            return False
        if not self._move_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info(f"Waiting for service: {self.move_service_name}")
            return False
        if not self._get_prim_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for service: /isaac/get_prim_attributes")
            return False
        if not self._delete_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info("Waiting for service: /isaac/delete_prim")
            return False
        return True

    def _dispatch_bootstrap_step(self):
        if self._bootstrap_phase == "resolve_entrances":
            entries = list(self.semantics.get("entrances", []) or [])
            if self._bootstrap_index >= len(entries):
                self._bootstrap_phase = "resolve_exits"
                self._bootstrap_index = 0
                self.get_logger().info(f"Resolved {len(self.entrances)} entrances.")
                return
            self._dispatch_location_resolution(
                phase="resolve_entrances",
                item=entries[self._bootstrap_index],
                index=self._bootstrap_index,
            )
            return

        if self._bootstrap_phase == "resolve_exits":
            entries = list(self.semantics.get("exits", []) or [])
            if self._bootstrap_index >= len(entries):
                self._bootstrap_phase = "resolve_resources"
                self._bootstrap_index = 0
                self.get_logger().info(f"Resolved {len(self.exits)} exits.")
                return
            self._dispatch_location_resolution(
                phase="resolve_exits",
                item=entries[self._bootstrap_index],
                index=self._bootstrap_index,
            )
            return

        if self._bootstrap_phase == "resolve_resources":
            entries = [
                (category, item)
                for category in ("urinals", "stalls", "sinks")
                for item in (self.semantics.get(category, []) or [])
            ]
            if self._bootstrap_index >= len(entries):
                self.resources = ResourceManager(self._bootstrap_resources)
                self._bootstrap_phase = "ready"
                self.get_logger().info(
                    f"Resolved semantic layout: entrances={len(self.entrances)}, exits={len(self.exits)}, "
                    f"resources={len(self.resources.resources)}"
                )
                self.get_logger().info(f"Resource snapshot: {self.resources.snapshot()}")
                return
            category, item = entries[self._bootstrap_index]
            self._dispatch_resource_resolution(
                phase="resolve_resources",
                category=category,
                item=item,
                index=self._bootstrap_index,
            )

    def _dispatch_location_resolution(self, *, phase: str, item: dict, index: int):
        placement_rule = _parse_placement_rule(item)
        if placement_rule is None:
            pose = self._coerce_walk_plane(parse_semantic_pose(item.get("pose", [0.0, 0.0, 0.0])))
            self._complete_location_resolution(phase=phase, item=item, index=index, pose=pose)
            return
        self._dispatch_anchor_query(
            action="location_anchor",
            payload={
                "phase": phase,
                "item": item,
                "index": index,
                "placement_rule": placement_rule,
            },
        )

    def _dispatch_resource_resolution(self, *, phase: str, category: str, item: dict, index: int):
        entry_id = str(item["id"])
        placement_rule = _parse_placement_rule(item)
        queue_rule = _parse_queue_rule(item)
        if placement_rule is None:
            pose = self._coerce_walk_plane(parse_semantic_pose(item.get("pose", [0.0, 0.0, 0.0])))
            queue_slots = [
                QueueSlot(slot_id=f"{entry_id}_queue_{slot_idx}", position=slot_pose.as_list(), yaw=float(slot_pose.yaw))
                for slot_idx, slot in enumerate(item.get("queue_slots", []) or [])
                for slot_pose in [self._coerce_walk_plane(parse_semantic_pose(slot))]
            ]
            self._complete_resource_resolution(
                category=category,
                item=item,
                index=index,
                pose=pose,
                queue_slots=queue_slots,
            )
            return
        self._dispatch_anchor_query(
            action="resource_anchor",
            payload={
                "phase": phase,
                "category": category,
                "item": item,
                "index": index,
                "placement_rule": placement_rule,
                "queue_rule": queue_rule,
            },
        )

    def _dispatch_anchor_query(self, *, action: str, payload: dict):
        scene_prim = str(payload["item"].get("scene_prim", "")).strip()
        prim_hint = self._scene_prim_hint(scene_prim)
        request = GetPrimAttributes.Request(prim_path=prim_hint)
        self._bootstrap_pending_future = self._get_prim_client.call_async(request)
        self._bootstrap_pending_payload = {
            **payload,
            "action": action,
            "scene_prim": scene_prim,
            "prim_hint": prim_hint,
        }
        self.get_logger().info(
            f"Resolving {payload['item'].get('id', '<unknown>')} via prim hint {prim_hint}"
        )

    def _handle_bootstrap_response(self, payload: dict, response):
        action = payload.get("action")
        if action == "location_anchor":
            self._handle_location_anchor_response(payload, response)
            return
        if action == "resource_anchor":
            self._handle_resource_anchor_response(payload, response)
            return

    def _handle_location_anchor_response(self, payload: dict, response):
        if not _pose_response_is_finite(response):
            raise RuntimeError(
                f"Invalid prim pose for {payload['scene_prim']} (hint={payload['prim_hint']})"
            )
        anchor_position = (
            float(response.pose.position.x),
            float(response.pose.position.y),
            float(response.pose.position.z),
        )
        anchor_yaw = yaw_from_quaternion_xyzw(
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w,
        )
        theoretical_pose = resolve_local_placement(
            anchor_position=anchor_position,
            anchor_yaw=anchor_yaw,
            placement_rule=payload["placement_rule"],
        )
        self._complete_location_resolution(
            phase=payload["phase"],
            item=payload["item"],
            index=payload["index"],
            pose=theoretical_pose,
        )

    def _handle_resource_anchor_response(self, payload: dict, response):
        if not _pose_response_is_finite(response):
            raise RuntimeError(
                f"Invalid prim pose for {payload['scene_prim']} (hint={payload['prim_hint']})"
            )
        anchor_position = (
            float(response.pose.position.x),
            float(response.pose.position.y),
            float(response.pose.position.z),
        )
        anchor_yaw = yaw_from_quaternion_xyzw(
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w,
        )
        theoretical_pose = resolve_local_placement(
            anchor_position=anchor_position,
            anchor_yaw=anchor_yaw,
            placement_rule=payload["placement_rule"],
        )
        self._continue_resource_queue_resolution(
            {
                **payload,
                "anchor_position": anchor_position,
                "anchor_yaw": anchor_yaw,
            },
            theoretical_pose,
        )

    def _continue_resource_queue_resolution(self, payload: dict, resolved_resource_pose: SemanticPose):
        queue_rule: QueueRule | None = payload.get("queue_rule")
        if queue_rule is None:
            self._complete_resource_resolution(
                category=payload["category"],
                item=payload["item"],
                index=payload["index"],
                pose=resolved_resource_pose,
                queue_slots=[],
            )
            return
        theoretical_queue = build_queue_poses(
            anchor_position=payload["anchor_position"],
            anchor_yaw=payload["anchor_yaw"],
            resource_pose=resolved_resource_pose,
            queue_rule=queue_rule,
        )
        queue_slots = []
        for next_index, queue_pose in enumerate(theoretical_queue):
            queue_pose = self._coerce_walk_plane(queue_pose)
            queue_slots.append(
                QueueSlot(
                    slot_id=f"{payload['item']['id']}_queue_{next_index}",
                    position=queue_pose.as_list(),
                    yaw=float(queue_pose.yaw),
                )
            )
        self._complete_resource_resolution(
            category=payload["category"],
            item=payload["item"],
            index=payload["index"],
            pose=resolved_resource_pose,
            queue_slots=queue_slots,
        )

    def _complete_location_resolution(self, *, phase: str, item: dict, index: int, pose: SemanticPose):
        resolved = _location_dict(
            entry_id=str(item.get("id", f"location_{index}")),
            scene_prim=str(item.get("scene_prim", "")),
            pose=pose,
        )
        if phase == "resolve_entrances":
            self.entrances.append(resolved)
        elif phase == "resolve_exits":
            self.exits.append(resolved)
        else:
            raise ValueError(f"Unknown location resolution phase: {phase}")
        self._bootstrap_index += 1
        self.get_logger().info(
            f"Resolved {resolved['id']}: position={resolved['position']}, yaw={resolved['yaw']:.3f}"
        )

    def _complete_resource_resolution(
        self,
        *,
        category: str,
        item: dict,
        index: int,
        pose: SemanticPose,
        queue_slots: list[QueueSlot],
    ):
        entry_id = str(item["id"])
        self._bootstrap_resources[entry_id] = Resource(
            resource_id=entry_id,
            category=category,
            position=pose.as_list(),
            yaw=float(pose.yaw),
            scene_prim=str(item.get("scene_prim", "")),
            queue_slots=queue_slots,
        )
        self._bootstrap_index += 1
        self.get_logger().info(
            f"Resolved {entry_id}: position={pose.as_list()}, yaw={pose.yaw:.3f}, queue_slots={len(queue_slots)}"
        )

    def _query_anchor_pose(self, item: dict) -> tuple[tuple[float, float, float], float]:
        scene_prim = str(item.get("scene_prim", "")).strip()
        if not scene_prim:
            raise ValueError(f"placement rule requires scene_prim for item {item.get('id', '<unknown>')}")
        prim_hint = self._scene_prim_hint(scene_prim)
        response = self._get_prim_client.call(GetPrimAttributes.Request(prim_path=prim_hint))
        if response is None:
            raise RuntimeError(f"Failed to query prim attributes for {scene_prim} (hint={prim_hint})")
        values = (
            float(response.pose.position.x),
            float(response.pose.position.y),
            float(response.pose.position.z),
            float(response.pose.orientation.w),
        )
        if not all(math.isfinite(v) for v in values):
            raise RuntimeError(f"Invalid prim pose for {scene_prim} (hint={prim_hint})")
        position = (
            float(response.pose.position.x),
            float(response.pose.position.y),
            float(response.pose.position.z),
        )
        yaw = yaw_from_quaternion_xyzw(
            response.pose.orientation.x,
            response.pose.orientation.y,
            response.pose.orientation.z,
            response.pose.orientation.w,
        )
        return position, yaw

    def _scene_prim_hint(self, scene_prim: str) -> str:
        text = str(scene_prim or "").strip()
        if not text:
            return text
        if text.startswith("/"):
            return text
        if "/" in text:
            return os.path.join(self.scene_root_path, text).replace("\\", "/")
        return f"{self.scene_root_path}/{text}"

    def _coerce_walk_plane(self, pose: SemanticPose) -> SemanticPose:
        return pose.with_z(self.walk_plane_z)

    @staticmethod
    def _character_root_path(agent_id: str) -> str:
        return f"/World/Characters/{str(agent_id).strip()}"

    def _request_despawn(self, state: RuntimeAgent):
        if state.despawn_requested:
            return
        state.despawn_requested = True
        self._pending_despawns.add(state.agent_id)
        request = DeletePrim.Request(name=self._character_root_path(state.agent_id))
        future = self._delete_client.call_async(request)
        future.add_done_callback(lambda fut, agent_id=state.agent_id: self._despawn_done_cb(agent_id, fut))
        self.get_logger().info(f"Despawning {state.agent_id} after EXITING.")

    def _despawn_done_cb(self, agent_id: str, future):
        self._pending_despawns.discard(agent_id)
        success = True
        try:
            response = future.result()
            success = bool(getattr(response, "ret", False))
        except Exception as exc:
            success = False
            self.get_logger().warning(f"Despawn request failed for {agent_id}: {exc}")
        self._runtime_agents.pop(agent_id, None)
        if success:
            self.get_logger().info(f"{agent_id} despawned and removed from runtime state.")
        else:
            self.get_logger().warning(f"{agent_id} removed from runtime state after despawn failure.")
        self._maybe_shutdown_when_complete()

    def _maybe_shutdown_when_complete(self):
        if not self.exit_when_complete:
            return
        if self._runtime_agents:
            return
        if self._pending_despawns:
            return
        if self._initial_spawn_queue or self._initial_spawn_in_flight:
            return
        if not self._bootstrapped:
            return
        self.get_logger().info("All toilet benchmark agents completed; shutting down director.")
        try:
            self._fsm_timer.cancel()
        except Exception:
            pass
        rclpy.shutdown()

    def _build_initial_agents(self, count: int) -> list[DirectedAgentState]:
        if not self.entrances:
            self.get_logger().warning("No entrances defined in semantics config; nothing to spawn.")
            return []
        resources = [res for res in self.resources.resources.values() if res.category == "urinals"]
        if not resources:
            self.get_logger().warning("No urinal resources defined in semantics config; nothing to target.")
            return []

        velocity_range = self.benchmark.get("hunav_profile", {}).get(self.profile_name, {}).get(
            "desired_velocity", [0.6, 0.8]
        )
        try:
            min_v, max_v = float(velocity_range[0]), float(velocity_range[1])
        except Exception:
            min_v, max_v = 0.6, 0.8

        directed_agents: list[DirectedAgentState] = []
        for idx in range(count):
            entrance = self.entrances[idx % len(self.entrances)]
            agent_id = f"toilet_agent_{idx + 1:02d}"
            candidate_ids = [res.resource_id for res in resources]
            self._rng.shuffle(candidate_ids)
            resource_id, reservation = self.resources.acquire_or_queue(
                agent_id,
                "urinals",
                candidate_ids=candidate_ids,
            )
            if resource_id is None:
                self.get_logger().warning(f"No available urinal or queue slot for {agent_id}")
                continue

            goal_pose = self.resources.resource_pose(resource_id)
            goal_yaw = self.resources.resource_yaw(resource_id)
            queue_path: list[list[float]] = []
            target_pose = goal_pose
            target_yaw = goal_yaw
            phase = "WALK_TO_URINAL"
            queue_slot_id = None
            if reservation != "resource":
                queue_slot_id = reservation
                slot_pose = self.resources.queue_slot_pose(resource_id, queue_slot_id)
                slot_yaw = self.resources.queue_slot_yaw(resource_id, queue_slot_id)
                if slot_pose is not None:
                    target_pose = slot_pose
                    target_yaw = float(slot_yaw or 0.0)
                    queue_path = [slot_pose]
                    phase = "QUEUEING"

            path_points = [*queue_path, goal_pose] if queue_path else [goal_pose]
            velocity = min_v if min_v >= max_v else self._rng.uniform(min_v, max_v)
            directed_agents.append(
                DirectedAgentState(
                    agent_id=agent_id,
                    status=phase.lower(),
                    goal_pose=list(path_points[-1]),
                    velocity=float(velocity),
                )
            )

            exit_target = self.exits[idx % len(self.exits)] if self.exits else entrance
            travel_target = list(target_pose)
            now = time.monotonic()
            eta = self._estimate_arrival_deadline(entrance["position"], travel_target, velocity, now=now)
            self._runtime_agents[agent_id] = RuntimeAgent(
                agent_id=agent_id,
                character_name=self._select_character_name(idx),
                profile_name=self.profile_name,
                velocity=float(velocity),
                resource_id=resource_id,
                current_phase=phase,
                current_pose=list(entrance["position"]),
                initial_yaw=float(entrance["yaw"]),
                target_pose=list(travel_target),
                target_yaw=float(target_yaw),
                queue_slot_id=queue_slot_id,
                reached_deadline=eta,
                exit_pose=list(exit_target["position"]),
                exit_yaw=float(exit_target["yaw"]),
            )
            self.get_logger().info(
                f"Prepared {agent_id}: entrance={entrance['id']} -> resource={resource_id}, "
                f"path_points={path_points}, velocity={velocity:.2f}, phase={phase}, reservation={reservation}, "
                f"candidate_order={candidate_ids}"
            )
        self._spawned_agents = directed_agents
        return directed_agents

    def _select_character_name(self, idx: int) -> str:
        if self.character_pool:
            return self.character_pool[idx % len(self.character_pool)]
        return self.character_name

    def _queue_initial_spawns(self, directed_agents: list[DirectedAgentState]):
        self._initial_spawn_queue = [state.agent_id for state in directed_agents]
        self._initial_spawn_in_flight = False
        self._next_initial_spawn_time = time.monotonic()
        if self._initial_spawn_queue:
            self.get_logger().info(
                f"Queued {len(self._initial_spawn_queue)} initial pedestrians with "
                f"spawn interval {self.initial_spawn_interval_sec:.2f}s."
            )

    def _advance_initial_spawn_queue(self, now: float):
        if self._initial_spawn_in_flight:
            return
        if not self._initial_spawn_queue:
            return
        if now < self._next_initial_spawn_time:
            return
        agent_id = self._initial_spawn_queue.pop(0)
        state = self._runtime_agents.get(agent_id)
        if state is None:
            return
        self._spawn_initial_agent(state)

    def _spawn_initial_agent(self, state: RuntimeAgent):
        request = Pedestrian.Request()
        msg = Person()
        msg.stage_prefix = state.agent_id
        msg.character_name = state.character_name
        msg.initial_pose = [float(x) for x in state.current_pose]
        msg.goal_pose = [float(x) for x in state.target_pose]
        msg.orientation = float(state.initial_yaw)
        msg.controller_stats = False
        msg.velocity = float(state.velocity)
        request.people.append(msg)
        state.spawn_requested = True
        self._initial_spawn_in_flight = True
        self.get_logger().info(
            f"Spawn request {state.agent_id}: character={state.character_name}, initial_pose={msg.initial_pose}, "
            f"target={msg.goal_pose}"
        )
        future = self._spawn_client.call_async(request)
        future.add_done_callback(lambda fut, agent_id=state.agent_id: self._spawn_done_cb(fut, agent_id=agent_id))

    def _spawn_done_cb(self, future, *, agent_id: str | None = None):
        success = False
        try:
            response = future.result()
            success = bool(getattr(response, "ret", False))
        except Exception as exc:
            self.get_logger().error(f"Spawn pedestrian service call failed: {exc}")
        self.get_logger().info(f"Spawn pedestrian response: ret={success}, agent_id={agent_id}")
        if agent_id is not None:
            state = self._runtime_agents.get(agent_id)
            if state is not None and success:
                state.spawned = True
                self._dispatch_initial_moves([
                    DirectedAgentState(
                        agent_id=state.agent_id,
                        status=state.current_phase.lower(),
                        goal_pose=list(state.target_pose),
                        velocity=float(state.velocity),
                    )
                ])
            elif state is not None and not success:
                try:
                    self.resources.release(state.agent_id, state.resource_id)
                except Exception:
                    pass
                self._runtime_agents.pop(agent_id, None)
            self._initial_spawn_in_flight = False
            self._next_initial_spawn_time = time.monotonic() + max(0.0, self.initial_spawn_interval_sec)

    def _dispatch_initial_moves(self, directed_agents: list[DirectedAgentState]):
        request = MovePed.Request()
        for state in directed_agents:
            runtime_state = self._runtime_agents[state.agent_id]
            path_points = self._path_points_for_move(
                agent_id=state.agent_id,
                start_pose=runtime_state.current_pose,
                goal_pose=runtime_state.target_pose,
            )
            nav = NavPed()
            nav.path = state.agent_id
            nav.goal_pose = [float(x) for x in runtime_state.target_pose]
            nav.path_points_flat = _flatten_path_points(path_points)
            nav.loop_path = False
            nav.velocity = float(state.velocity)
            nav.orientation = float(runtime_state.target_yaw)
            request.nav_list.append(nav)
            self.get_logger().info(f"Dispatch move: {self.adapter.to_debug_dict(state)}")
        future = self._move_client.call_async(request)
        future.add_done_callback(self._move_done_cb)

    def _move_done_cb(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Move pedestrian service call failed: {exc}")
            return
        self.get_logger().info(f"Move pedestrian response: ret={getattr(response, 'ret', None)}")

    def _send_move(
        self,
        *,
        agent_id: str,
        goal_pose: list[float],
        velocity: float,
        orientation: float = 0.0,
        stop: bool = False,
        use_direct_pose: bool = False,
        direct_pose: list[float] | None = None,
    ):
        request = MovePed.Request()
        nav = NavPed()
        path_points = []
        if not stop and not use_direct_pose:
            runtime_state = self._runtime_agents.get(agent_id)
            start_pose = runtime_state.current_pose if runtime_state is not None else goal_pose
            path_points = self._path_points_for_move(
                agent_id=agent_id,
                start_pose=start_pose,
                goal_pose=goal_pose,
            )
        nav.path = agent_id
        nav.goal_pose = [float(x) for x in goal_pose]
        nav.path_points_flat = _flatten_path_points(path_points)
        nav.loop_path = False
        nav.velocity = float(velocity)
        nav.orientation = float(orientation)
        nav.stop = bool(stop)
        nav.use_direct_pose = bool(use_direct_pose)
        if direct_pose is not None:
            nav.direct_pose = [float(x) for x in direct_pose]
        request.nav_list = [nav]
        future = self._move_client.call_async(request)
        future.add_done_callback(self._move_done_cb)

    def _path_points_for_move(
        self,
        *,
        agent_id: str,
        start_pose: list[float],
        goal_pose: list[float],
    ) -> list[list[float]]:
        if self.path_planner is None:
            return []
        try:
            points = self.path_planner.plan(start_pose, goal_pose, z=self.walk_plane_z)
        except Exception as exc:
            self.get_logger().warning(f"{agent_id} voxel path planning failed; using navmesh fallback: {exc}")
            return []
        if not points:
            return []
        self.get_logger().info(
            f"{agent_id} voxel path: start={start_pose[:3]}, goal={goal_pose[:3]}, points={len(points)}"
        )
        return points

    def _advance_state_machine(self):
        if not self._bootstrapped:
            return
        now = time.monotonic()
        self._advance_initial_spawn_queue(now)
        urinal_range = self.benchmark.get("service_time_sec", {}).get("urinal", [20.0, 45.0])
        try:
            min_service, max_service = float(urinal_range[0]), float(urinal_range[1])
        except Exception:
            min_service, max_service = 20.0, 45.0

        for state in list(self._runtime_agents.values()):
            if not state.spawned:
                continue
            if state.current_phase == "DONE":
                continue
            if state.current_phase == "DESPAWNING":
                continue
            if state.current_phase == "USING_URINAL" and now < state.service_deadline:
                continue
            if state.current_phase in {"WALK_TO_URINAL", "QUEUEING", "EXITING"}:
                if not self._align_if_arrived(state, now):
                    continue

            if state.current_phase == "QUEUEING":
                resource = self.resources.resources[state.resource_id]
                if resource.occupied_by != state.agent_id:
                    continue
                state.current_phase = "WALK_TO_URINAL"
                state.queue_slot_id = None
                state.target_pose = self.resources.resource_pose(state.resource_id)
                state.target_yaw = self.resources.resource_yaw(state.resource_id)
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=state.target_pose,
                    velocity=state.velocity,
                    orientation=state.target_yaw,
                )
                self.get_logger().info(f"{state.agent_id} promoted from queue to urinal {state.resource_id}")
                continue

            if state.current_phase == "WALK_TO_URINAL":
                state.current_phase = "USING_URINAL"
                service_duration = (
                    min_service if min_service >= max_service else self._rng.uniform(min_service, max_service)
                )
                state.service_deadline = now + service_duration
                self.get_logger().info(
                    f"{state.agent_id} started using {state.resource_id} for ~{service_duration:.1f}s"
                )
                continue

            if state.current_phase == "USING_URINAL":
                promoted = self.resources.release(state.agent_id, state.resource_id)
                state.current_phase = "EXITING"
                state.target_pose = list(state.exit_pose or state.current_pose)
                state.target_yaw = float(state.exit_yaw)
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=state.target_pose,
                    velocity=state.velocity,
                    orientation=state.target_yaw,
                )
                self.get_logger().info(f"{state.agent_id} leaving {state.resource_id}; next queued={promoted}")
                continue

            if state.current_phase == "EXITING":
                state.current_phase = "DESPAWNING"
                self._request_despawn(state)
                continue

        self._maybe_shutdown_when_complete()

    def _estimate_arrival_deadline(
        self,
        start_pose: list[float],
        target_pose: list[float],
        velocity: float,
        *,
        now: float | None = None,
    ) -> float:
        anchor = time.monotonic() if now is None else float(now)
        return anchor + max(1.0, _planar_distance(start_pose, target_pose) / max(float(velocity), 1e-3) + 1.0)

    def _has_live_pose(self, state: RuntimeAgent, now: float) -> bool:
        return state.last_pose_update > 0.0 and (now - state.last_pose_update) <= self.pose_stale_sec

    def _align_if_arrived(self, state: RuntimeAgent, now: float) -> bool:
        if state.aligned_at_target:
            return True

        threshold = (
            max(float(self.exit_arrival_tolerance_m), float(self.actor_stop_radius_m) + 0.05)
            if state.current_phase == "EXITING"
            else max(float(self.arrival_tolerance_m), float(self.actor_stop_radius_m) + 0.05)
        )
        if self._has_live_pose(state, now):
            distance = _planar_distance(state.current_pose, state.target_pose)
            if distance > threshold:
                return False
            state.target_reached = True
        else:
            if not self.allow_eta_fallback:
                if (now - state.last_arrival_wait_log) >= 2.0:
                    state.last_arrival_wait_log = now
                    self.get_logger().warning(
                        f"{state.agent_id} has no fresh live pose while approaching {state.current_phase}; "
                        "waiting instead of forcing arrival."
                    )
                return False
            if now < state.reached_deadline:
                return False
            state.target_reached = True

        state.aligned_at_target = True
        align_pose = list(state.current_pose)
        if state.current_phase != "EXITING":
            align_pose = list(state.target_pose)
            state.current_pose = list(align_pose)
        if state.current_phase != "EXITING" and self._has_live_pose(state, now):
            self._send_move(
                agent_id=state.agent_id,
                goal_pose=align_pose,
                velocity=0.0,
                orientation=state.target_yaw,
                stop=True,
                use_direct_pose=True,
                direct_pose=align_pose,
            )
        self.get_logger().info(
            f"{state.agent_id} aligned at {state.current_phase}: pose={align_pose}, target={state.target_pose}, "
            f"yaw={state.target_yaw:.3f}"
        )
        return True

    def _people_cb(self, msg):
        try:
            people = getattr(msg, "people", None) or getattr(msg, "pedestrians", None)
            if not people:
                return
            observed_at = time.monotonic()
            for person in people:
                identifier = getattr(person, "stage_prefix", None) or getattr(person, "name", None) or getattr(
                    person, "id", None
                )
                state = self._lookup_runtime_agent(identifier)
                if state is None:
                    continue
                pose_field = getattr(person, "pose", None) or getattr(person, "position", None)
                xyz = self._extract_xyz_from_field(pose_field)
                if xyz is None:
                    continue
                if not self._accept_live_pose_update(state, xyz, observed_at):
                    continue
                state.current_pose = xyz
                state.last_pose_update = observed_at
        except Exception as exc:
            self.get_logger().debug(f"Failed to process pedestrian pose update: {exc}")

    def _accept_live_pose_update(self, state: RuntimeAgent, xyz: list[float], observed_at: float) -> bool:
        if not all(math.isfinite(float(value)) for value in xyz[:3]):
            return False
        if state.last_pose_update <= 0.0:
            return True
        jump = _planar_distance(state.current_pose, xyz)
        if jump <= float(self.max_live_pose_jump_m):
            return True
        if (observed_at - state.last_pose_reject_log) >= 2.0:
            state.last_pose_reject_log = observed_at
            self.get_logger().warning(
                f"Ignoring implausible live pose jump for {state.agent_id}: "
                f"from={state.current_pose} to={xyz}, jump={jump:.2f}m"
            )
        return False

    def _lookup_runtime_agent(self, identifier) -> RuntimeAgent | None:
        if identifier is None:
            return None
        key = str(identifier).strip()
        state = self._runtime_agents.get(key)
        if state is not None:
            return state
        stripped = key.strip("/")
        for agent_id, candidate in self._runtime_agents.items():
            if stripped.endswith(agent_id):
                return candidate
        return None

    @staticmethod
    def _extract_xyz_from_field(field) -> list[float] | None:
        if field is None:
            return None
        if hasattr(field, "x") and hasattr(field, "y") and hasattr(field, "z"):
            return [float(field.x), float(field.y), float(field.z)]
        inner = getattr(field, "position", None)
        if inner is not None and hasattr(inner, "x") and hasattr(inner, "y") and hasattr(inner, "z"):
            return [float(inner.x), float(inner.y), float(inner.z)]
        pose = getattr(field, "pose", None)
        if pose is not None:
            inner_pose = getattr(pose, "position", None)
            if inner_pose is not None and hasattr(inner_pose, "x") and hasattr(inner_pose, "y") and hasattr(
                inner_pose, "z"
            ):
                return [float(inner_pose.x), float(inner_pose.y), float(inner_pose.z)]
        return None


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--semantics", default=_default_config_path("toilet_semantics.yaml"))
    parser.add_argument("--benchmark", default=_default_config_path("toilet_benchmark.yaml"))
    parser.add_argument("--spawn-service", default="/isaac/spawn_pedestrian")
    parser.add_argument("--move-service", default="/isaac/move_pedestrians")
    parser.add_argument("--initial-agents", type=int, default=1)
    parser.add_argument("--character-name", default="original_female_adult_business_02")
    parser.add_argument("--character-pool", default="")
    parser.add_argument("--profile", default="regular")
    parsed, ros_args = parser.parse_known_args(args=args)

    character_pool = [
        name.strip()
        for name in str(parsed.character_pool).split(",")
        if name.strip()
    ]

    rclpy.init(args=ros_args)
    node = ToiletDirectorNode(
        semantics_path=parsed.semantics,
        benchmark_path=parsed.benchmark,
        spawn_service=parsed.spawn_service,
        move_service=parsed.move_service,
        initial_agents=parsed.initial_agents,
        character_name=parsed.character_name,
        character_pool=character_pool or None,
        profile_name=parsed.profile,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
