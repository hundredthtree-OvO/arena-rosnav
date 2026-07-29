from __future__ import annotations

import argparse
import json
import math
import os
import random
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from isaacsim_msgs.msg import Person
from isaacsim_msgs.srv import DeletePrim, GetPrimAttributes, Pedestrian
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

try:
    from people_msgs.msg import People
except Exception:  # pragma: no cover - optional runtime dependency
    People = None

try:
    from arena_people_msgs.msg import Pedestrians as ArenaPedestrians
except Exception:  # pragma: no cover - optional runtime dependency
    ArenaPedestrians = None

from .hunav_adapter import DirectedAgentState, HunavAdapter
from .hunav_isaac_mirror_core import build_motion_intent_payload
from .interaction_policy import (
    dynamic_robot_obstacles,
    guard_block_requires_wait,
    robot_blocks_pedestrian,
)
from .motion_backend import IsaacPeopleBackend, MotionCommand
from .pedestrian_state_stream import observations_from_message
from .pose_utils import SemanticPose, parse_semantic_pose
from .resource_manager import QueueSlot, Resource, ResourceManager
from .route_provider import (
    RouteProvider,
    RouteRequest,
    VoxelRouteProvider,
    WalkableMapRouteProvider,
)
from .semantic_rules import (
    PlacementRule,
    PortalCorridor,
    QueueRule,
    build_queue_poses,
    classify_motion_observation,
    portal_inside_plane_reached,
    remaining_polyline_waypoints,
    resolve_local_placement,
    yaw_from_quaternion_xyzw,
)
from .voxel_path_planner import VoxelPathPlanner, VoxelPathPlannerConfig
from .walkable_map_planner import WalkableMapPlanner, WalkableMapPlannerConfig


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


@dataclass
class RuntimeAgent:
    agent_id: str
    character_name: str
    profile_name: str
    velocity: float
    resource_id: str
    current_phase: str
    activity_phase: str
    current_pose: list[float]
    entrance_pose: list[float]
    initial_yaw: float
    target_pose: list[float]
    target_yaw: float
    activity_pose: list[float]
    activity_yaw: float
    queue_slot_id: str | None = None
    reached_deadline: float = 0.0
    service_deadline: float = 0.0
    exit_pose: list[float] | None = None
    exit_yaw: float = 0.0
    last_pose_update: float = 0.0
    current_yaw: float | None = None
    current_speed_mps: float = 0.0
    final_alignment_started_at: float = 0.0
    aligned_at_target: bool = False
    last_arrival_wait_log: float = 0.0
    last_pose_reject_log: float = 0.0
    target_reached: bool = False
    despawn_requested: bool = False
    spawned: bool = False
    spawn_requested: bool = False
    activated: bool = False
    activation_requested: bool = False
    activation_pose_applied: bool = False
    activation_pose_confirmed: bool = False
    activation_ready_at: float = 0.0
    activation_confirm_deadline: float = 0.0
    activation_attempts: int = 0
    last_phase_wait_log: float = 0.0
    portal_direction: str | None = None
    portal_acquired_at: float = 0.0
    portal_last_motion_at: float = 0.0
    portal_last_pose: list[float] | None = None
    portal_best_distance: float = math.inf
    portal_recovery_attempts: int = 0
    motion_watch_phase: str | None = None
    motion_last_motion_at: float = 0.0
    motion_last_pose: list[float] | None = None
    motion_best_distance: float = math.inf
    motion_recovery_attempts: int = 0
    guard_block_generation: int = -1
    guard_block_count: int = 0
    guard_feedback_at: float = 0.0
    guard_blocked: bool = False
    guard_block_reason: str = ""
    guard_clear_grace_until: float = 0.0
    nominal_path: list[list[float]] | None = None
    nominal_goal: list[float] | None = None


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
        target_resource_ids: list[str] | None = None,
        status_topic: str = "/toilet_benchmark/director_status",
        motion_backend_name: str = "",
        hunav_namespace: str = "",
    ):
        super().__init__("toilet_director_node")
        self.semantics_path = str(semantics_path)
        self.benchmark_path = str(benchmark_path)
        self.spawn_service_name = str(spawn_service)
        self.move_service_name = str(move_service)
        self.initial_agents = max(0, int(initial_agents))
        self.character_name = str(character_name)
        self.profile_name = str(profile_name)
        self.target_resource_ids = [
            str(resource_id).strip()
            for resource_id in (target_resource_ids or [])
            if str(resource_id).strip()
        ]
        self.status_topic = str(status_topic).strip()
        self._status_publisher = (
            self.create_publisher(String, self.status_topic, 10)
            if self.status_topic
            else None
        )
        self._motion_intent_generations: dict[str, int] = {}
        self.semantics = _load_yaml(self.semantics_path)
        self.benchmark = _load_yaml(self.benchmark_path)
        motion_cfg = dict(self.benchmark.get("motion_backend", {}) or {})
        self.motion_backend_name = str(
            motion_backend_name or motion_cfg.get("type", "isaac_people")
        ).strip().lower()
        self.hunav_namespace = str(
            hunav_namespace
            or motion_cfg.get("hunav_namespace", "/toilet_hunav_takeover")
        ).strip()
        hunav_motion_cfg = dict(motion_cfg.get("hunav", {}) or {})
        self.hunav_use_portal_controller = bool(
            hunav_motion_cfg.get(
                "use_portal_controller",
                hunav_motion_cfg.get("use_exit_portal_controller", False),
            )
        )
        self.portal = self._load_portal_config(self.semantics)
        self.scene_root_path = str(
            self.semantics.get("scene_root_path")
            or self.semantics.get("scene_root")
            or "/World"
        ).rstrip("/")
        director_cfg = self.benchmark.get("director", {}) or {}
        self.arrival_tolerance_m = float(director_cfg.get("arrival_tolerance_m", 0.55))
        self.actor_stop_radius_m = float(director_cfg.get("actor_stop_radius_m", 0.5))
        self.exit_arrival_tolerance_m = float(director_cfg.get("exit_arrival_tolerance_m", 0.35))
        self.urinal_final_yaw_tolerance_rad = max(
            0.01,
            float(director_cfg.get("urinal_final_yaw_tolerance_rad", 0.20)),
        )
        self.urinal_final_alignment_stable_sec = max(
            0.0,
            float(director_cfg.get("urinal_final_alignment_stable_sec", 0.50)),
        )
        self.urinal_final_alignment_max_speed_mps = max(
            0.0,
            float(director_cfg.get("urinal_final_alignment_max_speed_mps", 0.05)),
        )
        self.walk_plane_z = float(director_cfg.get("walk_plane_z", 0.0))
        self.pose_stale_sec = float(director_cfg.get("pose_stale_sec", 5.0))
        self.max_live_pose_jump_m = float(director_cfg.get("max_live_pose_jump_m", 3.0))
        self.allow_eta_fallback = bool(director_cfg.get("allow_eta_fallback", False))
        self.exit_when_complete = bool(director_cfg.get("exit_when_complete", True))
        self.initial_spawn_interval_sec = float(director_cfg.get("initial_spawn_interval_sec", 1.0))
        self.serialize_exit_corridor = bool(director_cfg.get("serialize_exit_corridor", True))
        self.pre_spawn_holding_origin = self._optional_vector3(
            director_cfg.get("pre_spawn_holding_origin")
        )
        self.pre_spawn_holding_offset = self._vector3(
            director_cfg.get("pre_spawn_holding_offset", [-0.8, 0.0, 0.0]),
            default=[-0.8, 0.0, 0.0],
        )
        self.pre_spawn_holding_axis = self._vector3(
            director_cfg.get("pre_spawn_holding_axis", [0.0, -1.0, 0.0]),
            default=[0.0, -1.0, 0.0],
        )
        self.pre_spawn_holding_spacing_m = float(director_cfg.get("pre_spawn_holding_spacing_m", 0.5))
        self.portal_stall_recovery_sec = float(director_cfg.get("portal_stall_recovery_sec", 3.0))
        self.portal_max_recovery_attempts = max(
            0,
            int(director_cfg.get("portal_max_recovery_attempts", 3)),
        )
        self.portal_staging_arrival_tolerance_m = max(
            0.05,
            float(director_cfg.get("portal_staging_arrival_tolerance_m", 0.20)),
        )
        self.exit_staging_arrival_tolerance_m = max(
            0.05,
            float(director_cfg.get("exit_staging_arrival_tolerance_m", 0.30)),
        )
        self.portal_crossing_velocity_mps = max(
            0.1,
            float(director_cfg.get("portal_crossing_velocity_mps", 0.45)),
        )
        self.activation_settle_sec = max(
            0.0,
            float(director_cfg.get("activation_settle_sec", 0.20)),
        )
        self.activation_confirm_timeout_sec = max(
            0.5,
            float(director_cfg.get("activation_confirm_timeout_sec", 2.0)),
        )
        self.activation_max_attempts = max(
            1,
            int(director_cfg.get("activation_max_attempts", 3)),
        )
        self.activation_confirmation_tolerance_m = max(
            0.05,
            float(director_cfg.get("activation_confirmation_tolerance_m", 0.35)),
        )
        self.motion_stall_recovery_sec = max(
            1.0,
            float(director_cfg.get("motion_stall_recovery_sec", 4.0)),
        )
        self.motion_max_recovery_attempts = max(
            0,
            int(director_cfg.get("motion_max_recovery_attempts", 3)),
        )
        self.stall_displacement_epsilon_m = max(
            0.005,
            float(director_cfg.get("stall_displacement_epsilon_m", 0.02)),
        )
        self.stall_progress_epsilon_m = max(
            0.005,
            float(director_cfg.get("stall_progress_epsilon_m", 0.03)),
        )
        self.guard_clear_grace_sec = max(
            0.0,
            float(director_cfg.get("guard_clear_grace_sec", 0.75)),
        )
        self.live_pose_topic = str(director_cfg.get("live_pose_topic", "/isaac/pedestrian_states"))
        planner_cfg = self.benchmark.get("path_planner", {}) or {}
        self.robot_obstacle_mode = str(
            planner_cfg.get("robot_obstacle_mode", "static_route")
        ).strip().lower()
        self.robot_odom_topic = str(planner_cfg.get("robot_odom_topic", "/odom"))
        self.robot_obstacle_radius_m = max(
            0.0,
            float(planner_cfg.get("robot_obstacle_radius_m", 0.38)),
        )
        self.robot_obstacle_timeout_sec = max(
            0.1,
            float(planner_cfg.get("robot_obstacle_timeout_sec", 1.0)),
        )
        self.robot_prediction_horizon_sec = max(
            0.0,
            float(planner_cfg.get("robot_prediction_horizon_sec", 0.75)),
        )
        self.robot_yield_pedestrian_radius_m = max(
            0.0,
            float(planner_cfg.get("robot_yield_pedestrian_radius_m", 0.22)),
        )
        self.robot_yield_clearance_m = max(
            0.0,
            float(planner_cfg.get("robot_yield_clearance_m", 0.10)),
        )
        self.path_planner: VoxelPathPlanner | WalkableMapPlanner | None = None
        self.route_provider: RouteProvider | None = None
        self._robot_state: tuple[float, float, float, float, float] | None = None
        self.character_pool = [
            str(name).strip()
            for name in (character_pool or self.benchmark.get("character_pool", []) or [])
            if str(name).strip()
        ]
        self.entrances: list[dict] = []
        self.exits: list[dict] = []
        self.resources = ResourceManager({})
        self.adapter = HunavAdapter()
        self.arrival_seed = int(
            self.benchmark.get("arrival", {}).get("seed", 12345)
        )
        self._rng = random.Random(self.arrival_seed)
        self._spawn_client = self.create_client(Pedestrian, self.spawn_service_name)
        if self.motion_backend_name == "isaac_people":
            self.motion_backend = IsaacPeopleBackend(self, self.move_service_name)
        elif self.motion_backend_name == "hunav":
            if self.initial_agents > 1:
                raise ValueError(
                    "Phase 1 HuNav takeover currently supports exactly one active "
                    "pedestrian; use --initial-agents 1."
                )
            from .hunav_motion_backend import HuNavMotionBackend

            hunav_config = hunav_motion_cfg
            hunav_config.setdefault("reaction_seed", self.arrival_seed)
            hunav_config.setdefault("visualization", {})
            hunav_config["visualization"] = {
                **dict(hunav_config["visualization"] or {}),
                "portal": self._hunav_portal_visualization_config(),
            }
            self.motion_backend = HuNavMotionBackend(
                self,
                move_service_name=self.move_service_name,
                namespace=self.hunav_namespace,
                config=hunav_config,
            )
        else:
            raise ValueError(
                f"Unsupported pedestrian motion backend: {self.motion_backend_name!r}"
            )
        self._get_prim_client = self.create_client(GetPrimAttributes, "/isaac/get_prim_attributes")
        self._delete_client = self.create_client(DeletePrim, "/isaac/delete_prim")
        self._spawned_agents: list[DirectedAgentState] = []
        self._runtime_agents: dict[str, RuntimeAgent] = {}
        self._pending_despawns: set[str] = set()
        self._exit_corridor_agent_id: str | None = None
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
        self._robot_odom_subscription = self.create_subscription(
            Odometry,
            self.robot_odom_topic,
            self._robot_odom_cb,
            10,
        )
        self._setup_path_planner()
        if (
            self.motion_backend_name == "hunav"
            and isinstance(self.path_planner, WalkableMapPlanner)
        ):
            self.motion_backend.set_walkable_planner(self.path_planner)

        self.get_logger().info(
            "Loaded toilet benchmark configs: "
            f"semantics={self.semantics_path}, benchmark={self.benchmark_path}, "
            f"scene_root_path={self.scene_root_path}, "
            f"spawn_service={self.spawn_service_name}, move_service={self.move_service_name}, "
            f"motion_backend={self.motion_backend_name}, "
            f"arrival_seed={self.arrival_seed}, "
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
            f"initial_spawn_interval_sec={self.initial_spawn_interval_sec:.2f}, "
            f"serialize_exit_corridor={self.serialize_exit_corridor}, "
            f"hunav_use_portal_controller={self.hunav_use_portal_controller}, "
            f"portal={self.portal.get('id') if self.portal else None}, "
            f"portal_half_width_m={self.portal['corridor'].half_width_m if self.portal else 0.0:.2f}, "
            f"portal_clearance_m={self.portal['corridor'].clearance_m if self.portal else 0.0:.2f}, "
            f"portal_stall_recovery_sec={self.portal_stall_recovery_sec:.1f}, "
            f"portal_max_recovery_attempts={self.portal_max_recovery_attempts}, "
            f"portal_staging_arrival_tolerance_m={self.portal_staging_arrival_tolerance_m:.2f}, "
            f"exit_staging_arrival_tolerance_m={self.exit_staging_arrival_tolerance_m:.2f}, "
            f"portal_crossing_velocity_mps={self.portal_crossing_velocity_mps:.2f}, "
            f"activation_settle_sec={self.activation_settle_sec:.2f}, "
            f"activation_confirm_timeout_sec={self.activation_confirm_timeout_sec:.2f}, "
            f"activation_max_attempts={self.activation_max_attempts}, "
            f"activation_confirmation_tolerance_m={self.activation_confirmation_tolerance_m:.2f}, "
            f"motion_stall_recovery_sec={self.motion_stall_recovery_sec:.1f}, "
            f"motion_max_recovery_attempts={self.motion_max_recovery_attempts}, "
            f"stall_displacement_epsilon_m={self.stall_displacement_epsilon_m:.3f}, "
            f"stall_progress_epsilon_m={self.stall_progress_epsilon_m:.3f}, "
            f"robot_odom_topic={self.robot_odom_topic}, "
            f"robot_obstacle_mode={self.robot_obstacle_mode}, "
            f"robot_obstacle_radius_m={self.robot_obstacle_radius_m:.2f}, "
            f"robot_prediction_horizon_sec={self.robot_prediction_horizon_sec:.2f}, "
            f"pre_spawn_holding_origin={self.pre_spawn_holding_origin}, "
            f"pre_spawn_holding_offset={self.pre_spawn_holding_offset}, "
            f"pre_spawn_holding_axis={self.pre_spawn_holding_axis}"
        )

    def _setup_path_planner(self):
        planner_cfg = self.benchmark.get("path_planner", {}) or {}
        if not bool(planner_cfg.get("enabled", False)):
            self.get_logger().warning("Toilet path planner disabled; pedestrian motion will remain fail-closed.")
            return
        backend = str(
            planner_cfg.get("hunav_backend", "walkable_map")
            if self.motion_backend_name == "hunav"
            else planner_cfg.get("backend", "voxel")
        ).strip().lower()
        if backend == "walkable_map":
            self._setup_walkable_map_planner(planner_cfg)
            return
        if backend != "voxel":
            self.get_logger().error(
                f"Unsupported toilet path planner backend={backend!r}; pedestrian motion will remain fail-closed."
            )
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
                    agent_radius_m=float(planner_cfg.get("agent_radius_m", 0.30)),
                    bounds_padding_m=float(planner_cfg.get("bounds_padding_m", 1.0)),
                    max_expansions=int(planner_cfg.get("max_expansions", 20000)),
                    nearest_free_radius_m=float(planner_cfg.get("nearest_free_radius_m", 0.8)),
                    simplify=bool(planner_cfg.get("simplify", True)),
                    max_segment_length_m=float(planner_cfg.get("max_segment_length_m", 2.0)),
                    constrained_segment_length_m=float(
                        planner_cfg.get("constrained_segment_length_m", 0.9)
                    ),
                    preferred_clearance_m=float(planner_cfg.get("preferred_clearance_m", 0.50)),
                    clearance_cost_weight=float(planner_cfg.get("clearance_cost_weight", 4.0)),
                    turn_cost_weight=float(planner_cfg.get("turn_cost_weight", 0.35)),
                )
            )
            self.route_provider = VoxelRouteProvider(
                planner=self.path_planner,
                walk_plane_z=self.walk_plane_z,
                dynamic_obstacles=self._dynamic_obstacles,
                logger=self.get_logger(),
            )
            if self.portal is not None:
                portal_path = self._portal_entry_waypoints()
                if not self.path_planner.polyline_is_free(
                    portal_path,
                    allow_out_of_bounds=True,
                ):
                    raise ValueError(
                        f"portal {self.portal['id']} intersects the inflated voxel map: {portal_path}"
                    )
                self.get_logger().info(
                    f"Validated constrained entry portal path: id={self.portal['id']}, "
                    f"points={portal_path}"
                )
            self.get_logger().info(
                f"Toilet voxel path planner loaded: map={map_path}, "
                f"resolution={self.path_planner.resolution:.3f}, "
                f"occupied={len(self.path_planner.occupied)}, inflated={len(self.path_planner.inflated_occupied)}"
            )
        except Exception as exc:
            self.path_planner = None
            self.route_provider = None
            self.get_logger().error(f"Failed to load toilet voxel path planner; pedestrian motion is disabled: {exc}")

    def _setup_walkable_map_planner(self, planner_cfg: dict) -> None:
        map_path = str(
            planner_cfg.get("walkable_map_path")
            or "/home/stardust/resources/arena_ws/arena_assets/navigation/"
            "shenxinfu_841837.walkable.json"
        )
        try:
            planner = WalkableMapPlanner.from_file(
                WalkableMapPlannerConfig(
                    map_path=map_path,
                    agent_radius_m=float(
                        planner_cfg.get("walkable_agent_radius_m", 0.24)
                    ),
                    nearest_free_radius_m=float(
                        planner_cfg.get("walkable_nearest_free_radius_m", 0.60)
                    ),
                    max_expansions=int(
                        planner_cfg.get("walkable_max_expansions", 30000)
                    ),
                    simplify=bool(planner_cfg.get("walkable_simplify", True)),
                    max_segment_length_m=float(
                        planner_cfg.get("walkable_max_segment_length_m", 1.50)
                    ),
                    constrained_segment_length_m=float(
                        planner_cfg.get(
                            "walkable_constrained_segment_length_m",
                            0.40,
                        )
                    ),
                    preferred_clearance_m=float(
                        planner_cfg.get("walkable_preferred_clearance_m", 0.40)
                    ),
                    clearance_cost_weight=float(
                        planner_cfg.get("walkable_clearance_cost_weight", 3.0)
                    ),
                    turn_cost_weight=float(
                        planner_cfg.get("walkable_turn_cost_weight", 0.25)
                    ),
                )
            )
            self.path_planner = planner
            self.route_provider = WalkableMapRouteProvider(
                planner=planner,
                walk_plane_z=self.walk_plane_z,
                logger=self.get_logger(),
                dynamic_obstacles=self._dynamic_obstacles,
            )
            if self.portal is not None:
                portal_path = self._portal_entry_waypoints()
                if not planner.polyline_is_free(
                    portal_path,
                    allow_out_of_bounds=False,
                ):
                    raise ValueError(
                        f"portal {self.portal['id']} intersects the walkable-map "
                        f"configuration space: {portal_path}"
                    )
            self.get_logger().info(
                f"Toilet walkable-map planner loaded: map={map_path}, "
                f"fingerprint={planner.scene_fingerprint[:12]}, "
                f"resolution={planner.resolution:.3f}, "
                f"occupied={planner.occupied_count}, "
                f"inflated={planner.inflated_occupied_count}"
            )
        except Exception as exc:
            self.path_planner = None
            self.route_provider = None
            self.get_logger().error(
                "Failed to load toilet walkable-map planner; pedestrian motion "
                f"is disabled: {exc}"
            )

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

    def _robot_odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose.position
        velocity = msg.twist.twist.linear
        values = (
            float(pose.x),
            float(pose.y),
            float(velocity.x),
            float(velocity.y),
        )
        if not all(math.isfinite(value) for value in values):
            return
        self._robot_state = (*values, time.monotonic())

    def _dynamic_obstacles(self, now: float) -> list[tuple[float, float, float]]:
        return dynamic_robot_obstacles(
            mode=self.robot_obstacle_mode,
            robot_state=self._robot_state,
            now=now,
            timeout_sec=self.robot_obstacle_timeout_sec,
            radius_m=self.robot_obstacle_radius_m,
            prediction_horizon_sec=self.robot_prediction_horizon_sec,
        )

    def _is_yielding_to_robot(self, state: RuntimeAgent, now: float) -> bool:
        return robot_blocks_pedestrian(
            pedestrian_pose=state.current_pose,
            robot_state=self._robot_state,
            now=now,
            timeout_sec=self.robot_obstacle_timeout_sec,
            robot_radius_m=self.robot_obstacle_radius_m,
            pedestrian_radius_m=self.robot_yield_pedestrian_radius_m,
            clearance_m=self.robot_yield_clearance_m,
            prediction_horizon_sec=self.robot_prediction_horizon_sec,
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
        self._spawn_initial_agents_idle(directed_agents)

    def _services_ready(self) -> bool:
        if not self._spawn_client.wait_for_service(timeout_sec=0.0):
            self.get_logger().info(f"Waiting for service: {self.spawn_service_name}")
            return False
        if not self.motion_backend.wait_for_service(timeout_sec=0.0):
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
        pose = self._coerce_walk_plane(pose)
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
    def _optional_vector3(value) -> list[float] | None:
        if value is None:
            return None
        try:
            return [float(value[0]), float(value[1]), float(value[2])]
        except Exception:
            return None

    @staticmethod
    def _load_portal_config(semantics: dict) -> dict | None:
        entries = list(semantics.get("portals", []) or [])
        if not entries:
            return None
        item = entries[0]
        try:
            outside = [float(value) for value in item["outside_pose"][:3]]
            inside = [float(value) for value in item["inside_pose"][:3]]
            corridor = PortalCorridor(
                outside=tuple(outside),
                inside=tuple(inside),
                half_width_m=float(item.get("half_width_m", 0.45)),
                clearance_m=float(item.get("clearance_m", 0.40)),
                capacity=int(item.get("capacity", 1)),
            )
            return {
                "id": str(item.get("id", "door_portal")),
                "outside": outside,
                "center": [float(value) for value in item["center_pose"][:3]],
                "inside": inside,
                "inside_staging": [
                    float(value)
                    for value in item.get("inside_staging_pose", item["inside_pose"])[:3]
                ],
                "corridor": corridor,
                "release_tolerance_m": float(item.get("release_tolerance_m", 0.30)),
            }
        except Exception as exc:
            raise ValueError(f"Invalid portal semantics: {exc}") from exc

    def _hunav_portal_visualization_config(self) -> dict | None:
        """Return plain geometry only; the backend never owns portal state."""
        if self.portal is None:
            return None
        corridor = self.portal["corridor"]
        return {
            "id": self.portal["id"],
            "outside": list(self.portal["outside"]),
            "inside": list(self.portal["inside"]),
            "half_width_m": float(corridor.half_width_m),
            "clearance_m": float(corridor.clearance_m),
        }

    @staticmethod
    def _vector3(value, *, default: list[float]) -> list[float]:
        parsed = ToiletDirectorNode._optional_vector3(value)
        return list(default) if parsed is None else parsed

    @staticmethod
    def _character_root_path(agent_id: str) -> str:
        return f"/World/Characters/{str(agent_id).strip()}"

    def _request_despawn(self, state: RuntimeAgent, *, reason: str = "completed EXITING"):
        if state.despawn_requested:
            return
        state.despawn_requested = True
        self._pending_despawns.add(state.agent_id)
        request = DeletePrim.Request(name=self._character_root_path(state.agent_id))
        future = self._delete_client.call_async(request)
        future.add_done_callback(lambda fut, agent_id=state.agent_id: self._despawn_done_cb(agent_id, fut))
        self.get_logger().info(f"Releasing {state.agent_id} to the bridge pedestrian pool: {reason}.")
        self._publish_status_event("pedestrian_retiring", state, reason=reason)

    def _publish_status_event(self, event: str, state: RuntimeAgent, **extra) -> None:
        if self._status_publisher is None:
            return
        payload = {
            "event": str(event),
            "agent_id": state.agent_id,
            "phase": state.current_phase,
            "resource_id": state.resource_id,
            **extra,
        }
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _portal_motion_metadata(
        self,
        state: RuntimeAgent | None,
        phase: str,
    ) -> dict | None:
        if self.portal is None or state is None:
            return None
        direction = state.portal_direction
        if direction is None:
            if phase == "WALK_TO_ENTRY_CLEARANCE":
                direction = "entering"
            elif phase == "EXITING":
                direction = "exiting"
        if direction not in {"entering", "exiting"}:
            return None
        corridor: PortalCorridor = self.portal["corridor"]
        return {
            "portal_id": self.portal["id"],
            "direction": direction,
            "outside": corridor.outside,
            "inside": corridor.inside,
            "half_width_m": corridor.half_width_m,
            "clearance_m": corridor.clearance_m,
            "capacity": corridor.capacity,
        }

    def _build_motion_intent(
        self,
        *,
        agent_id: str,
        goal_pose: list[float],
        path_points: list[list[float]],
        velocity: float,
        orientation: float,
        stop: bool,
        use_direct_pose: bool,
        constrain_to_path: bool,
        intent_phase: str | None,
    ) -> dict | None:
        if self._status_publisher is None:
            return None
        generation = self._motion_intent_generations.get(agent_id, 0) + 1
        self._motion_intent_generations[agent_id] = generation
        state = self._runtime_agents.get(agent_id)
        phase = (
            str(intent_phase)
            if intent_phase is not None
            else ("" if state is None else state.current_phase)
        )
        return build_motion_intent_payload(
            agent_id=agent_id,
            generation=generation,
            phase=phase,
            resource_id=None if state is None else state.resource_id,
            goal_pose=goal_pose,
            path_points=path_points,
            velocity=velocity,
            final_yaw=orientation,
            stop=stop,
            use_direct_pose=use_direct_pose,
            constrain_to_path=constrain_to_path,
            portal=self._portal_motion_metadata(state, phase),
        )

    def _publish_motion_intent_if_accepted(self, payload: dict | None, future) -> None:
        if payload is None or self._status_publisher is None:
            return
        try:
            accepted = bool(getattr(future.result(), "ret", False))
        except Exception:
            return
        if not accepted:
            return
        self._status_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _despawn_done_cb(self, agent_id: str, future):
        self._pending_despawns.discard(agent_id)
        success = True
        try:
            response = future.result()
            success = bool(getattr(response, "ret", False))
        except Exception as exc:
            success = False
            self.get_logger().warning(f"Despawn request failed for {agent_id}: {exc}")
        if success:
            state = self._runtime_agents.get(agent_id)
            if state is not None:
                self._release_portal(state, "pedestrian retired")
            self._runtime_agents.pop(agent_id, None)
            self.get_logger().info(f"{agent_id} parked in the bridge pool and removed from runtime state.")
        else:
            state = self._runtime_agents.get(agent_id)
            if state is not None:
                state.current_phase = "DESPAWNING"
                state.despawn_requested = False
            self.get_logger().warning(f"{agent_id} pool release failed; keeping runtime state for retry.")
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
        if self.target_resource_ids:
            requested = set(self.target_resource_ids)
            resources = [res for res in resources if res.resource_id in requested]
            missing = requested - {res.resource_id for res in resources}
            if missing:
                self.get_logger().warning(
                    f"Requested target resources are unavailable: {sorted(missing)}"
                )
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
            if self.target_resource_ids:
                candidate_ids = [
                    resource_id
                    for resource_id in self.target_resource_ids
                    if any(res.resource_id == resource_id for res in resources)
                ]
                if candidate_ids:
                    candidate_ids = candidate_ids[idx % len(candidate_ids):] + candidate_ids[:idx % len(candidate_ids)]
            else:
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
                activity_phase=phase,
                current_pose=list(entrance["position"]),
                entrance_pose=list(entrance["position"]),
                initial_yaw=float(entrance["yaw"]),
                target_pose=list(travel_target),
                target_yaw=float(target_yaw),
                activity_pose=list(travel_target),
                activity_yaw=float(target_yaw),
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

    def _pre_spawn_holding_pose(self, index: int, entrance_pose: list[float]) -> list[float]:
        if self.pre_spawn_holding_origin is None:
            pose = [
                float(entrance_pose[0]) + float(self.pre_spawn_holding_offset[0]),
                float(entrance_pose[1]) + float(self.pre_spawn_holding_offset[1]),
                float(entrance_pose[2]) + float(self.pre_spawn_holding_offset[2]),
            ]
        else:
            pose = list(self.pre_spawn_holding_origin)
        spacing = max(float(self.pre_spawn_holding_spacing_m), 0.1)
        pose[0] += float(index) * spacing * float(self.pre_spawn_holding_axis[0])
        pose[1] += float(index) * spacing * float(self.pre_spawn_holding_axis[1])
        pose[2] = self.walk_plane_z
        return pose

    def _queue_initial_activations(self, agent_ids: list[str]):
        self._initial_spawn_queue = list(agent_ids)
        self._initial_spawn_in_flight = False
        self._next_initial_spawn_time = time.monotonic()
        if self._initial_spawn_queue:
            self.get_logger().info(
                f"Queued {len(self._initial_spawn_queue)} pre-spawned pedestrians for activation with "
                f"interval {self.initial_spawn_interval_sec:.2f}s."
            )

    def _advance_initial_spawn_queue(self, now: float):
        if self._initial_spawn_in_flight:
            return
        if any(
            state.activation_requested and not state.activated
            for state in self._runtime_agents.values()
        ):
            return
        if not self._initial_spawn_queue:
            return
        if now < self._next_initial_spawn_time:
            return
        agent_id = self._initial_spawn_queue.pop(0)
        state = self._runtime_agents.get(agent_id)
        if state is None:
            return
        if not state.spawned:
            self.get_logger().warning(f"Skipping activation for {agent_id}; pedestrian is not spawned.")
            return
        self._activate_initial_agent(state)

    def _spawn_initial_agents_idle(self, directed_agents: list[DirectedAgentState]):
        if not directed_agents:
            self.get_logger().info("No initial pedestrians were prepared for spawning.")
            self._maybe_shutdown_when_complete()
            return
        request = Pedestrian.Request()
        agent_ids = []
        for spawn_index, directed in enumerate(directed_agents):
            state = self._runtime_agents.get(directed.agent_id)
            if state is None:
                continue
            holding_pose = self._pre_spawn_holding_pose(spawn_index, state.current_pose)
            state.current_pose = list(holding_pose)
            msg = Person()
            msg.stage_prefix = state.agent_id
            msg.character_name = state.character_name
            msg.initial_pose = [float(x) for x in holding_pose]
            msg.goal_pose = [float(x) for x in holding_pose]
            msg.orientation = float(state.initial_yaw)
            msg.controller_stats = False
            msg.velocity = 0.0
            request.people.append(msg)
            state.spawn_requested = True
            agent_ids.append(state.agent_id)
            self.get_logger().info(
                f"Spawn idle request {state.agent_id}: character={state.character_name}, "
                f"holding_pose={msg.initial_pose}, target={state.target_pose}"
            )
        if not request.people:
            self.get_logger().warning("No valid initial pedestrian spawn requests were created.")
            self._maybe_shutdown_when_complete()
            return
        self._initial_spawn_in_flight = True
        future = self._spawn_client.call_async(request)
        future.add_done_callback(lambda fut, ids=agent_ids: self._spawn_all_done_cb(fut, agent_ids=ids))

    def _spawn_all_done_cb(self, future, *, agent_ids: list[str]):
        success = False
        try:
            response = future.result()
            success = bool(getattr(response, "ret", False))
        except Exception as exc:
            self.get_logger().error(f"Spawn pedestrian service call failed: {exc}")
        self.get_logger().info(f"Spawn pedestrian response: ret={success}, agents={agent_ids}")
        if success:
            active_ids = []
            for agent_id in agent_ids:
                state = self._runtime_agents.get(agent_id)
                if state is None:
                    continue
                state.spawned = True
                active_ids.append(agent_id)
            self._queue_initial_activations(active_ids)
        else:
            for agent_id in agent_ids:
                state = self._runtime_agents.get(agent_id)
                if state is None:
                    continue
                try:
                    self.resources.release(state.agent_id, state.resource_id)
                except Exception:
                    pass
                self._runtime_agents.pop(agent_id, None)
            self._initial_spawn_in_flight = False
            self._maybe_shutdown_when_complete()

    def _activate_initial_agent(self, state: RuntimeAgent):
        if state.activated or state.activation_requested:
            return
        now = time.monotonic()
        state.activation_requested = True
        state.activation_attempts += 1
        if self._uses_portal_controller():
            if not self._acquire_portal(state, "entering", now):
                state.activation_requested = False
                self._initial_spawn_queue.append(state.agent_id)
                self._next_initial_spawn_time = now + max(0.5, self.initial_spawn_interval_sec)
                return
        entrance_pose = list(self.portal["outside"] if self.portal is not None else state.entrance_pose)
        dispatched = self._send_move(
            agent_id=state.agent_id,
            goal_pose=entrance_pose,
            velocity=0.0,
            orientation=state.initial_yaw,
            stop=True,
            use_direct_pose=True,
            direct_pose=entrance_pose,
            done_callback=lambda fut, agent_id=state.agent_id: self._activation_pose_done_cb(
                agent_id,
                fut,
            ),
        )
        if not dispatched:
            self._release_portal(state, "activation pose rejected")
            state.activation_requested = False
            self._initial_spawn_queue.append(state.agent_id)
            self._next_initial_spawn_time = time.monotonic() + max(1.0, self.initial_spawn_interval_sec)
            self.get_logger().error(f"Activation deferred for {state.agent_id}; entrance pose was rejected.")
            return
        state.current_phase = "ACTIVATING"
        state.target_pose = entrance_pose
        state.activation_pose_applied = False
        state.activation_pose_confirmed = False
        state.activation_ready_at = 0.0
        state.activation_confirm_deadline = 0.0
        self.get_logger().info(
            f"Activating {state.agent_id} at shared entrance pose {entrance_pose}; "
            f"{'portal motion' if self._uses_portal_controller() else 'direct activity motion'} "
            "will start after pose settlement."
        )

    def _activation_pose_done_cb(self, agent_id: str, future):
        state = self._runtime_agents.get(agent_id)
        if state is None:
            return
        accepted = False
        try:
            response = future.result()
            accepted = bool(getattr(response, "ret", False))
        except Exception as exc:
            self.get_logger().error(f"Entrance pose service call failed for {agent_id}: {exc}")
        if not accepted:
            self._release_portal(state, "activation pose command failed")
            state.activation_requested = False
            state.activation_pose_applied = False
            state.activation_pose_confirmed = False
            self._initial_spawn_queue.append(agent_id)
            self._next_initial_spawn_time = time.monotonic() + max(1.0, self.initial_spawn_interval_sec)
            self.get_logger().error(f"Activation deferred for {agent_id}; bridge rejected entrance pose.")
            return
        observed_at = time.monotonic()
        state.last_pose_update = 0.0
        state.activation_pose_applied = True
        state.activation_pose_confirmed = False
        state.activation_ready_at = 0.0
        state.activation_confirm_deadline = observed_at + self.activation_confirm_timeout_sec
        self.get_logger().info(
            f"{agent_id} entrance pose command accepted; waiting for live-pose confirmation."
        )

    def _advance_pending_activation(self, now: float):
        for state in self._runtime_agents.values():
            if state.activated or not state.activation_requested or not state.activation_pose_applied:
                continue
            if not state.activation_pose_confirmed:
                if now >= state.activation_confirm_deadline:
                    self._retry_unconfirmed_activation(state, now)
                continue
            if now < state.activation_ready_at:
                continue
            self._dispatch_initial_motion(state, now)
            return

    def _retry_unconfirmed_activation(self, state: RuntimeAgent, now: float) -> None:
        self._release_portal(state, "entrance pose was not confirmed")
        state.activation_requested = False
        state.activation_pose_applied = False
        state.activation_pose_confirmed = False
        state.activation_ready_at = 0.0
        state.activation_confirm_deadline = 0.0
        if state.activation_attempts >= self.activation_max_attempts:
            state.current_phase = "DESPAWNING"
            self.get_logger().error(
                f"{state.agent_id} entrance pose was not confirmed after "
                f"{state.activation_attempts} attempts; returning it to the bridge pool."
            )
            self._request_despawn(state, reason="entrance activation confirmation exhausted")
            return
        self._initial_spawn_queue.append(state.agent_id)
        self._next_initial_spawn_time = now + max(0.5, self.initial_spawn_interval_sec)
        self.get_logger().warning(
            f"{state.agent_id} entrance pose was not observed within "
            f"{self.activation_confirm_timeout_sec:.1f}s; retrying activation "
            f"({state.activation_attempts}/{self.activation_max_attempts})."
        )

    def _dispatch_initial_motion(self, state: RuntimeAgent, now: float):
        use_portal = self._uses_portal_controller()
        if use_portal:
            target_pose = self._portal_staging_pose()
            target_yaw = self._portal_entry_clearance_yaw()
            dispatched = self._send_move(
                agent_id=state.agent_id,
                goal_pose=target_pose,
                velocity=self._portal_motion_velocity(state),
                orientation=target_yaw,
                path_points_override=self._portal_entry_waypoints(),
                constrain_to_path=True,
                intent_phase="WALK_TO_ENTRY_CLEARANCE",
            )
            next_phase = "WALK_TO_ENTRY_CLEARANCE"
        else:
            target_pose = list(state.activity_pose)
            target_yaw = float(state.activity_yaw)
            activity_path = self._path_points_for_move(
                agent_id=state.agent_id,
                start_pose=state.current_pose,
                goal_pose=target_pose,
            )
            if activity_path is None:
                dispatched = False
            else:
                dispatched = self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=target_pose,
                    velocity=state.velocity,
                    orientation=target_yaw,
                    path_points_override=activity_path,
                    intent_phase=state.activity_phase,
                )
            next_phase = state.activity_phase
        if not dispatched:
            self._release_portal(state, "activation path rejected")
            state.activation_requested = False
            state.activation_pose_applied = False
            state.activation_pose_confirmed = False
            self._initial_spawn_queue.append(state.agent_id)
            self._next_initial_spawn_time = now + max(1.0, self.initial_spawn_interval_sec)
            self.get_logger().error(
                f"Activation deferred for {state.agent_id}; no safe entry path is available."
            )
            return
        state.current_phase = next_phase
        state.target_pose = target_pose
        state.target_yaw = target_yaw
        state.reached_deadline = self._estimate_arrival_deadline(
            state.current_pose,
            target_pose,
            self._portal_motion_velocity(state) if use_portal else state.velocity,
            now=now,
        )
        state.aligned_at_target = False
        state.target_reached = False
        if state.portal_direction is not None:
            self._reset_portal_motion_watch(state, now)
        state.activation_requested = False
        state.activation_pose_applied = False
        state.activation_pose_confirmed = False
        state.activated = True
        self._next_initial_spawn_time = now + max(0.0, self.initial_spawn_interval_sec)
        self.get_logger().info(
            f"Activated {state.agent_id}: start={state.current_pose}, phase={state.current_phase}, "
            f"target={state.target_pose}"
        )
        self._publish_status_event("pedestrian_active", state)

    def _move_done_cb(self, agent_id: str, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Move pedestrian service call failed for {agent_id}: {exc}")
            return
        accepted = bool(getattr(response, "ret", False))
        if accepted:
            self.get_logger().info(f"Move pedestrian response for {agent_id}: accepted=True")
        else:
            self.get_logger().error(f"Move pedestrian command rejected for {agent_id}")

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
        path_points_override: list[list[float]] | None = None,
        constrain_to_path: bool = False,
        preserve_nominal_path: bool = False,
        intent_phase: str | None = None,
        done_callback=None,
    ) -> bool:
        path_points = []
        if path_points_override is not None:
            path_points = self._dedupe_waypoints(path_points_override)
        elif not stop and not use_direct_pose:
            runtime_state = self._runtime_agents.get(agent_id)
            start_pose = runtime_state.current_pose if runtime_state is not None else goal_pose
            path_points = self._path_points_for_move(
                agent_id=agent_id,
                start_pose=start_pose,
                goal_pose=goal_pose,
            )
            if path_points is None:
                return False
        runtime_state = self._runtime_agents.get(agent_id)
        command = MotionCommand(
            agent_id=agent_id,
            goal_pose=tuple(float(value) for value in goal_pose),
            path_points=tuple(
                tuple(float(value) for value in point[:3])
                for point in path_points
            ),
            velocity=float(velocity),
            orientation=float(orientation),
            stop=bool(stop),
            constrain_to_path=bool(constrain_to_path),
            use_direct_pose=bool(use_direct_pose),
            direct_pose=(
                None
                if direct_pose is None
                else tuple(float(value) for value in direct_pose)
            ),
            phase=str(
                intent_phase
                or (runtime_state.current_phase if runtime_state is not None else "")
            ),
        )
        if (
            runtime_state is not None
            and path_points
            and not stop
            and not use_direct_pose
            and not preserve_nominal_path
        ):
            runtime_state.nominal_path = self._dedupe_waypoints(
                [list(runtime_state.current_pose), *path_points]
            )
            runtime_state.nominal_goal = [float(x) for x in goal_pose]
        callback = (
            done_callback
            if done_callback is not None
            else lambda fut, current_agent=agent_id: self._move_done_cb(
                current_agent,
                fut,
            )
        )
        future = self.motion_backend.send(command, callback)
        motion_intent = self._build_motion_intent(
            agent_id=agent_id,
            goal_pose=list(command.goal_pose),
            path_points=path_points,
            velocity=command.velocity,
            orientation=command.orientation,
            stop=command.stop,
            use_direct_pose=command.use_direct_pose,
            constrain_to_path=command.constrain_to_path,
            intent_phase=intent_phase,
        )
        future.add_done_callback(
            lambda fut, payload=motion_intent: self._publish_motion_intent_if_accepted(
                payload,
                fut,
            )
        )
        return True

    @staticmethod
    def _dedupe_waypoints(points: list[list[float]]) -> list[list[float]]:
        result: list[list[float]] = []
        for point in points:
            parsed = [float(point[0]), float(point[1]), float(point[2])]
            if result and _planar_distance(result[-1], parsed) <= 1e-4:
                result[-1] = parsed
            else:
                result.append(parsed)
        return result

    def _portal_entry_waypoints(self) -> list[list[float]]:
        if self.portal is None:
            return []
        if self.motion_backend_name == "hunav":
            return self._dedupe_waypoints(
                [
                    self.portal["outside"],
                    self.portal["inside"],
                    self._hunav_portal_clearance_pose(),
                ]
            )
        return self._dedupe_waypoints(
            [
                self.portal["outside"],
                self.portal["center"],
                self.portal["inside"],
                self.portal["inside_staging"],
            ]
        )

    def _hunav_portal_clearance_pose(self) -> list[float]:
        """Extend the portal axis indoors instead of forcing a lateral staging turn."""
        if self.portal is None:
            return [0.0, 0.0, self.walk_plane_z]
        pose = self.portal["corridor"].clear_pose("entering")
        pose[2] = float(self.walk_plane_z)
        return pose

    def _portal_staging_pose(self) -> list[float]:
        if self.portal is None:
            return [0.0, 0.0, self.walk_plane_z]
        if self.motion_backend_name == "hunav":
            return self._hunav_portal_clearance_pose()
        return list(self.portal["inside_staging"])

    def _portal_motion_velocity(self, state: RuntimeAgent) -> float:
        if self.motion_backend_name == "hunav":
            return float(state.velocity)
        return min(float(state.velocity), self.portal_crossing_velocity_mps)

    def _entry_portal_recovery_path(self, state: RuntimeAgent) -> list[list[float]]:
        if self.portal is None:
            return []
        return remaining_polyline_waypoints(
            self._portal_entry_waypoints(),
            state.current_pose,
        )

    def _exit_staging_path(self, state: RuntimeAgent) -> list[list[float]] | None:
        if self.portal is None:
            return self._path_points_for_move(
                agent_id=state.agent_id,
                start_pose=state.current_pose,
                goal_pose=list(state.exit_pose or state.current_pose),
            )
        return self._path_points_for_move(
            agent_id=state.agent_id,
            start_pose=state.current_pose,
            goal_pose=self._portal_staging_pose(),
        )

    def _uses_portal_controller(self) -> bool:
        return (
            self.portal is not None
            and (
                self.motion_backend_name != "hunav"
                or self.hunav_use_portal_controller
            )
        )

    def _exiting_portal_path(self, state: RuntimeAgent) -> list[list[float]]:
        if self.portal is None:
            return [list(state.exit_pose or state.current_pose)]
        if self.motion_backend_name == "hunav":
            return remaining_polyline_waypoints(
                [
                    self._hunav_portal_clearance_pose(),
                    self.portal["inside"],
                    self.portal["outside"],
                    list(state.exit_pose or self.portal["outside"]),
                ],
                state.current_pose,
            )
        return remaining_polyline_waypoints(
            [
                self.portal["inside"],
                self.portal["center"],
                self.portal["outside"],
                list(state.exit_pose or self.portal["outside"]),
            ],
            state.current_pose,
        )

    def _portal_exit_yaw(self) -> float:
        if self.portal is None:
            return 0.0
        inside = self.portal["inside"]
        outside = self.portal["outside"]
        return math.atan2(float(outside[1]) - float(inside[1]), float(outside[0]) - float(inside[0]))

    def _portal_entry_clearance_yaw(self) -> float:
        if self.portal is None:
            return 0.0
        inside = self.portal["inside"]
        staging = self._portal_staging_pose()
        return math.atan2(float(staging[1]) - float(inside[1]), float(staging[0]) - float(inside[0]))

    def _acquire_portal(self, state: RuntimeAgent, direction: str, now: float) -> bool:
        if not self.serialize_exit_corridor:
            return True
        if self._exit_corridor_agent_id not in (None, state.agent_id):
            return False
        self._exit_corridor_agent_id = state.agent_id
        state.portal_direction = str(direction)
        state.portal_acquired_at = float(now)
        state.portal_last_motion_at = float(now)
        state.portal_last_pose = list(state.current_pose)
        state.portal_best_distance = math.inf
        state.portal_recovery_attempts = 0
        return True

    def _reset_portal_motion_watch(self, state: RuntimeAgent, now: float) -> None:
        state.portal_last_motion_at = float(now)
        state.portal_last_pose = list(state.current_pose)
        state.portal_best_distance = _planar_distance(state.current_pose, state.target_pose)
        state.portal_recovery_attempts = 0

    def _release_portal(self, state: RuntimeAgent, reason: str) -> None:
        if self._exit_corridor_agent_id == state.agent_id:
            self._exit_corridor_agent_id = None
            self.get_logger().info(f"{state.agent_id} released portal {self.portal.get('id') if self.portal else ''}: {reason}")
        state.portal_direction = None
        state.portal_acquired_at = 0.0
        state.portal_last_motion_at = 0.0
        state.portal_last_pose = None
        state.portal_best_distance = math.inf
        state.portal_recovery_attempts = 0

    def _log_phase_wait(self, state: RuntimeAgent, reason: str, now: float) -> None:
        if now - state.last_phase_wait_log < 2.0:
            return
        state.last_phase_wait_log = float(now)
        self.get_logger().info(
            f"{state.agent_id} paused at {state.current_phase}: {reason}; pose={state.current_pose}, "
            f"target={state.target_pose}"
        )

    def _abort_stalled_portal_agent(self, state: RuntimeAgent) -> None:
        direction = str(state.portal_direction or "unknown")
        promoted = self.resources.release(state.agent_id, state.resource_id)
        state.current_phase = "DESPAWNING"
        self._release_portal(state, "stalled agent aborted")
        self.get_logger().error(
            f"{state.agent_id} exceeded {self.portal_max_recovery_attempts} portal recovery attempts "
            f"while {direction}; retiring it so other agents can continue, next queued={promoted}."
        )
        self._request_despawn(state, reason=f"portal {direction} recovery exhausted")

    def _abort_stalled_motion_agent(self, state: RuntimeAgent) -> None:
        phase = str(state.current_phase)
        promoted = self.resources.release(state.agent_id, state.resource_id)
        state.current_phase = "DESPAWNING"
        self._release_portal(state, "stalled motion aborted")
        self.get_logger().error(
            f"{state.agent_id} exceeded {self.motion_max_recovery_attempts} motion recovery attempts "
            f"while {phase}; retiring it so the scenario can continue, next queued={promoted}."
        )
        self._request_despawn(state, reason=f"{phase} motion recovery exhausted")

    def _director_stall_recovery_enabled(self, agent_id: str) -> bool:
        capability = getattr(
            self.motion_backend,
            "allows_director_stall_recovery",
            None,
        )
        if capability is None:
            return True
        return bool(capability(agent_id))

    def _recover_motion_if_stalled(self, state: RuntimeAgent, now: float) -> None:
        if not self._director_stall_recovery_enabled(state.agent_id):
            state.motion_last_motion_at = float(now)
            state.motion_last_pose = list(state.current_pose)
            return
        tracked_phases = {"WALK_TO_URINAL", "QUEUEING", "WALK_TO_EXIT_STAGING"}
        if state.portal_direction is not None or state.current_phase not in tracked_phases:
            return
        if state.aligned_at_target:
            return
        if guard_block_requires_wait(state.guard_block_reason) and state.guard_blocked:
            state.motion_last_motion_at = float(now)
            state.motion_last_pose = list(state.current_pose)
            self._log_phase_wait(
                state,
                f"guard wait: {state.guard_block_reason}",
                now,
            )
            return
        if now < state.guard_clear_grace_until:
            state.motion_last_motion_at = float(now)
            state.motion_last_pose = list(state.current_pose)
            return
        if self._is_yielding_to_robot(state, now):
            state.motion_last_motion_at = float(now)
            state.motion_last_pose = list(state.current_pose)
            self._log_phase_wait(state, "yielding to robot", now)
            return
        if state.motion_watch_phase != state.current_phase:
            state.motion_watch_phase = state.current_phase
            state.motion_last_motion_at = float(now)
            state.motion_last_pose = list(state.current_pose)
            state.motion_best_distance = _planar_distance(state.current_pose, state.target_pose)
            state.motion_recovery_attempts = 0
            return
        if now - state.motion_last_motion_at < self.motion_stall_recovery_sec:
            return
        if state.motion_recovery_attempts >= self.motion_max_recovery_attempts:
            self._abort_stalled_motion_agent(state)
            return

        recovery_path = None
        if (
            state.nominal_path
            and state.nominal_goal
            and _planar_distance(state.nominal_goal, state.target_pose) <= 1e-4
        ):
            recovery_path = remaining_polyline_waypoints(
                state.nominal_path,
                state.current_pose,
            )
        if not recovery_path:
            recovery_path = self._path_points_for_move(
                agent_id=state.agent_id,
                start_pose=state.current_pose,
                goal_pose=state.target_pose,
            )
        state.motion_last_motion_at = float(now)
        state.motion_last_pose = list(state.current_pose)
        if recovery_path is None:
            return
        if self._send_move(
            agent_id=state.agent_id,
            goal_pose=state.target_pose,
            velocity=self._portal_motion_velocity(state),
            orientation=state.target_yaw,
            path_points_override=recovery_path,
            constrain_to_path=True,
            preserve_nominal_path=True,
        ):
            state.motion_recovery_attempts += 1
            self.get_logger().warning(
                f"{state.agent_id} stalled during {state.current_phase}; redispatched remaining nominal path "
                f"with constrained recovery (attempt {state.motion_recovery_attempts}/"
                f"{self.motion_max_recovery_attempts})."
            )

    def _recover_portal_if_stalled(self, state: RuntimeAgent, now: float) -> None:
        if not self._director_stall_recovery_enabled(state.agent_id):
            state.portal_last_motion_at = float(now)
            state.portal_last_pose = list(state.current_pose)
            return
        if self.portal is None or state.portal_direction is None:
            return
        if guard_block_requires_wait(state.guard_block_reason) and state.guard_blocked:
            state.portal_last_motion_at = float(now)
            state.portal_last_pose = list(state.current_pose)
            self._log_phase_wait(
                state,
                f"guard wait in portal: {state.guard_block_reason}",
                now,
            )
            return
        if now < state.guard_clear_grace_until:
            state.portal_last_motion_at = float(now)
            state.portal_last_pose = list(state.current_pose)
            return
        if self._is_yielding_to_robot(state, now):
            state.portal_last_motion_at = float(now)
            state.portal_last_pose = list(state.current_pose)
            self._log_phase_wait(state, "yielding to robot in portal", now)
            return
        if state.portal_last_motion_at <= 0.0:
            state.portal_last_motion_at = float(now)
            return
        if now - state.portal_last_motion_at < max(1.0, self.portal_stall_recovery_sec):
            return
        if state.portal_recovery_attempts >= self.portal_max_recovery_attempts:
            self._abort_stalled_portal_agent(state)
            return
        if state.portal_direction == "entering":
            recovery_path = self._entry_portal_recovery_path(state)
            if not recovery_path:
                return
        else:
            recovery_path = self._exiting_portal_path(state)
        if self._send_move(
            agent_id=state.agent_id,
            goal_pose=state.target_pose,
            velocity=state.velocity,
            orientation=state.target_yaw,
            path_points_override=recovery_path,
            constrain_to_path=True,
        ):
            state.portal_recovery_attempts += 1
            state.portal_last_motion_at = float(now)
            state.portal_last_pose = list(state.current_pose)
            self.get_logger().warning(
                f"{state.agent_id} stalled in portal; redispatched {state.portal_direction} portal path "
                f"(attempt {state.portal_recovery_attempts}/{self.portal_max_recovery_attempts})."
            )

    def _path_points_for_move(
        self,
        *,
        agent_id: str,
        start_pose: list[float],
        goal_pose: list[float],
    ) -> list[list[float]] | None:
        if self.route_provider is None:
            self.get_logger().error(
                f"{agent_id} has no global route provider; refusing unsafe direct motion."
            )
            return None
        return self.route_provider.plan(
            RouteRequest(
                agent_id=agent_id,
                start_pose=start_pose,
                goal_pose=goal_pose,
            )
        )

    def _advance_state_machine(self):
        if not self._bootstrapped:
            return
        now = time.monotonic()
        self._advance_pending_activation(now)
        self._advance_initial_spawn_queue(now)
        urinal_range = self.benchmark.get("service_time_sec", {}).get("urinal", [20.0, 45.0])
        try:
            min_service, max_service = float(urinal_range[0]), float(urinal_range[1])
        except Exception:
            min_service, max_service = 20.0, 45.0

        for state in list(self._runtime_agents.values()):
            if not state.spawned:
                continue
            if state.current_phase == "DESPAWNING":
                if not state.despawn_requested:
                    self._request_despawn(state, reason="retrying pedestrian retirement")
                continue
            if not state.activated:
                continue
            if state.current_phase == "DONE":
                continue
            if state.current_phase == "USING_URINAL" and now < state.service_deadline:
                continue
            if state.current_phase in {
                "WALK_TO_ENTRY_CLEARANCE",
                "WALK_TO_URINAL",
                "QUEUEING",
                "WALK_TO_EXIT_STAGING",
                "EXITING",
            }:
                if not self._align_if_arrived(state, now):
                    self._recover_portal_if_stalled(state, now)
                    self._recover_motion_if_stalled(state, now)
                    continue

            if state.current_phase == "WALK_TO_ENTRY_CLEARANCE":
                activity_path = self._path_points_for_move(
                    agent_id=state.agent_id,
                    start_pose=state.current_pose,
                    goal_pose=state.activity_pose,
                )
                if activity_path is None:
                    continue
                if not self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=state.activity_pose,
                    velocity=state.velocity,
                    orientation=state.activity_yaw,
                    path_points_override=activity_path,
                    intent_phase=state.activity_phase,
                ):
                    continue
                self._release_portal(state, "reached indoor entry staging")
                state.current_phase = state.activity_phase
                state.target_pose = list(state.activity_pose)
                state.target_yaw = float(state.activity_yaw)
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self.get_logger().info(
                    f"{state.agent_id} reached indoor staging and started travel to {state.resource_id}"
                )
                continue

            if state.current_phase == "QUEUEING":
                resource = self.resources.resources[state.resource_id]
                if resource.occupied_by != state.agent_id:
                    continue
                target_pose = self.resources.resource_pose(state.resource_id)
                target_yaw = self.resources.resource_yaw(state.resource_id)
                if not self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=target_pose,
                    velocity=state.velocity,
                    orientation=target_yaw,
                    intent_phase="WALK_TO_URINAL",
                ):
                    continue
                state.current_phase = "WALK_TO_URINAL"
                state.queue_slot_id = None
                state.target_pose = target_pose
                state.target_yaw = target_yaw
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self.get_logger().info(f"{state.agent_id} promoted from queue to urinal {state.resource_id}")
                continue

            if state.current_phase == "WALK_TO_URINAL":
                if self.motion_backend_name == "hunav":
                    state.current_phase = "FINAL_ALIGN_URINAL"
                    state.final_alignment_started_at = 0.0
                    self.get_logger().info(
                        f"{state.agent_id} reached {state.resource_id}; waiting for final yaw alignment."
                    )
                    continue
                self._begin_urinal_service(state, now, min_service, max_service)
                continue

            if state.current_phase == "FINAL_ALIGN_URINAL":
                if not self._urinal_final_alignment_complete(state, now):
                    continue
                if not self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=state.current_pose,
                    velocity=0.0,
                    orientation=state.target_yaw,
                    stop=True,
                    intent_phase="USING_URINAL",
                ):
                    continue
                self._begin_urinal_service(state, now, min_service, max_service)
                continue

            if state.current_phase == "USING_URINAL":
                exit_pose = list(state.exit_pose or state.current_pose)
                exit_yaw = float(state.exit_yaw)
                use_exit_portal = self._uses_portal_controller()
                move_target = self._portal_staging_pose() if use_exit_portal else exit_pose
                move_path = (
                    self._exit_staging_path(state)
                    if use_exit_portal
                    else self._path_points_for_move(
                        agent_id=state.agent_id,
                        start_pose=state.current_pose,
                        goal_pose=exit_pose,
                    )
                )
                if move_path is None:
                    continue
                next_phase = "WALK_TO_EXIT_STAGING" if use_exit_portal else "EXITING"
                if not self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=move_target,
                    velocity=state.velocity,
                    orientation=self._portal_exit_yaw() if use_exit_portal else exit_yaw,
                    path_points_override=move_path,
                    intent_phase=next_phase,
                ):
                    continue
                promoted = self.resources.release(state.agent_id, state.resource_id)
                state.current_phase = next_phase
                state.target_pose = move_target
                state.target_yaw = self._portal_exit_yaw() if use_exit_portal else exit_yaw
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self.get_logger().info(
                    f"{state.agent_id} leaving {state.resource_id} for "
                    f"{'exit staging' if use_exit_portal else 'direct walkable-map exit'}; "
                    f"next queued={promoted}"
                )
                continue

            if state.current_phase == "WALK_TO_EXIT_STAGING":
                if not self._acquire_portal(state, "exiting", now):
                    self._log_phase_wait(state, "waiting for serialized exit portal", now)
                    continue
                exit_pose = list(state.exit_pose or state.current_pose)
                exit_yaw = float(state.exit_yaw)
                if not self._send_move(
                    agent_id=state.agent_id,
                    goal_pose=exit_pose,
                    velocity=self._portal_motion_velocity(state),
                    orientation=exit_yaw,
                    path_points_override=self._exiting_portal_path(state),
                    constrain_to_path=True,
                    intent_phase="EXITING",
                ):
                    self._release_portal(state, "exit crossing command rejected")
                    continue
                state.current_phase = "EXITING"
                state.target_pose = exit_pose
                state.target_yaw = exit_yaw
                state.reached_deadline = self._estimate_arrival_deadline(
                    state.current_pose,
                    state.target_pose,
                    state.velocity,
                    now=now,
                )
                state.aligned_at_target = False
                state.target_reached = False
                self._reset_portal_motion_watch(state, now)
                self.get_logger().info(
                    f"{state.agent_id} acquired portal and started continuous exit crossing"
                )
                continue

            if state.current_phase == "EXITING":
                state.current_phase = "DESPAWNING"
                self._request_despawn(state, reason="completed EXITING")
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

    def _begin_urinal_service(
        self,
        state: RuntimeAgent,
        now: float,
        min_service: float,
        max_service: float,
    ) -> None:
        state.current_phase = "USING_URINAL"
        service_duration = (
            min_service if min_service >= max_service else self._rng.uniform(min_service, max_service)
        )
        state.service_deadline = now + service_duration
        self.get_logger().info(
            f"{state.agent_id} started using {state.resource_id} for ~{service_duration:.1f}s"
        )

    def _urinal_final_alignment_complete(self, state: RuntimeAgent, now: float) -> bool:
        if not self._has_live_pose(state, now):
            return False
        yaw = state.current_yaw
        if yaw is None or not math.isfinite(float(yaw)):
            return False
        yaw_error = math.atan2(
            math.sin(float(state.target_yaw) - float(yaw)),
            math.cos(float(state.target_yaw) - float(yaw)),
        )
        stationary = float(state.current_speed_mps) <= self.urinal_final_alignment_max_speed_mps
        if abs(yaw_error) > self.urinal_final_yaw_tolerance_rad or not stationary:
            state.final_alignment_started_at = 0.0
            return False
        if state.final_alignment_started_at <= 0.0:
            state.final_alignment_started_at = now
            return False
        return (now - state.final_alignment_started_at) >= self.urinal_final_alignment_stable_sec

    def _has_live_pose(self, state: RuntimeAgent, now: float) -> bool:
        return state.last_pose_update > 0.0 and (now - state.last_pose_update) <= self.pose_stale_sec

    def _align_if_arrived(self, state: RuntimeAgent, now: float) -> bool:
        if state.aligned_at_target:
            return True

        if state.current_phase == "EXITING":
            threshold = max(float(self.exit_arrival_tolerance_m), float(self.actor_stop_radius_m) + 0.05)
        elif state.current_phase == "WALK_TO_EXIT_STAGING":
            threshold = float(self.exit_staging_arrival_tolerance_m)
        elif state.current_phase == "WALK_TO_ENTRY_CLEARANCE":
            threshold = float(self.portal_staging_arrival_tolerance_m)
        else:
            threshold = float(self.arrival_tolerance_m)
        arrival_source = "geometric tolerance"
        if self._has_live_pose(state, now):
            distance = _planar_distance(state.current_pose, state.target_pose)
            portal_entry_complete = (
                state.current_phase == "WALK_TO_ENTRY_CLEARANCE"
                and self.portal is not None
                and (
                    self.portal["corridor"].cleared(
                        state.current_pose,
                        "entering",
                        lateral_margin_m=self.portal["release_tolerance_m"],
                    )
                    if self.motion_backend_name == "hunav"
                    else portal_inside_plane_reached(
                        current_pose=state.current_pose,
                        inside_pose=self.portal["inside"],
                        staging_pose=self.portal["inside_staging"],
                        lateral_tolerance_m=self.portal["release_tolerance_m"],
                    )
                )
            )
            backend_settled = bool(
                self.motion_backend.is_goal_settled(
                    state.agent_id,
                    state.current_pose,
                    state.target_pose,
                )
            )
            requires_backend_settlement = (
                self.motion_backend_name == "hunav"
                and state.current_phase == "WALK_TO_URINAL"
            )
            if requires_backend_settlement and not backend_settled:
                return False
            if distance > threshold and not portal_entry_complete and not backend_settled:
                return False
            if backend_settled and distance > threshold:
                arrival_source = "motion-backend settled handshake"
            elif portal_entry_complete and distance > threshold:
                arrival_source = "portal inside-plane crossing"
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
        self.get_logger().info(
            f"{state.agent_id} aligned at {state.current_phase}: pose={align_pose}, target={state.target_pose}, "
            f"yaw={state.target_yaw:.3f}, source={arrival_source}"
        )
        return True

    def _people_cb(self, msg):
        try:
            observed_at = time.monotonic()
            for observation in observations_from_message(msg, observed_at=observed_at):
                state = self._lookup_runtime_agent(observation.identifier)
                if state is None:
                    continue
                xyz = list(observation.position)
                if not state.activated:
                    if state.activation_requested and state.activation_pose_applied:
                        self._confirm_activation_pose(state, xyz, observed_at)
                    continue
                if not self._accept_live_pose_update(state, xyz, observed_at):
                    continue
                self._apply_guard_feedback(state, observation.metadata, observed_at)
                state.current_yaw = self._metadata_yaw(observation.metadata)
                if observation.velocity is not None:
                    state.current_speed_mps = math.hypot(
                        float(observation.velocity[0]),
                        float(observation.velocity[1]),
                    )
                if state.portal_direction is not None:
                    observation = classify_motion_observation(
                        previous_pose=state.portal_last_pose,
                        current_pose=xyz,
                        target_pose=state.target_pose,
                        best_distance=state.portal_best_distance,
                        displacement_epsilon_m=self.stall_displacement_epsilon_m,
                        progress_epsilon_m=self.stall_progress_epsilon_m,
                    )
                    if observation.moved:
                        state.portal_last_pose = list(xyz)
                        state.portal_last_motion_at = observed_at
                    if observation.progressed:
                        state.portal_best_distance = observation.distance_to_target
                        state.portal_recovery_attempts = 0
                if state.motion_watch_phase == state.current_phase:
                    observation = classify_motion_observation(
                        previous_pose=state.motion_last_pose,
                        current_pose=xyz,
                        target_pose=state.target_pose,
                        best_distance=state.motion_best_distance,
                        displacement_epsilon_m=self.stall_displacement_epsilon_m,
                        progress_epsilon_m=self.stall_progress_epsilon_m,
                    )
                    if observation.moved:
                        state.motion_last_pose = list(xyz)
                        state.motion_last_motion_at = observed_at
                    if observation.progressed:
                        state.motion_best_distance = observation.distance_to_target
                        state.motion_recovery_attempts = 0
                state.current_pose = xyz
                state.last_pose_update = observed_at
        except Exception as exc:
            self.get_logger().debug(f"Failed to process pedestrian pose update: {exc}")

    @staticmethod
    def _metadata_yaw(metadata) -> float | None:
        if str(metadata.get("yaw_valid", "false")).lower() != "true":
            return None
        try:
            yaw = float(metadata["yaw_rad"])
        except (KeyError, TypeError, ValueError):
            return None
        return yaw if math.isfinite(yaw) else None

    def _confirm_activation_pose(
        self,
        state: RuntimeAgent,
        xyz: list[float],
        observed_at: float,
    ) -> None:
        if not all(math.isfinite(float(value)) for value in xyz[:3]):
            return
        distance = _planar_distance(xyz, state.target_pose)
        if distance > self.activation_confirmation_tolerance_m:
            if observed_at - state.last_phase_wait_log >= 2.0:
                state.last_phase_wait_log = observed_at
                self.get_logger().warning(
                    f"{state.agent_id} activation command accepted but live pose is still "
                    f"{distance:.2f}m from entrance; observed={xyz}, target={state.target_pose}."
                )
            return
        if state.activation_pose_confirmed:
            return
        state.current_pose = list(xyz)
        state.last_pose_update = observed_at
        state.activation_pose_confirmed = True
        state.activation_ready_at = observed_at + self.activation_settle_sec
        if state.portal_direction is not None:
            state.portal_last_pose = list(xyz)
            state.portal_best_distance = _planar_distance(xyz, state.target_pose)
            state.portal_last_motion_at = observed_at
            state.portal_recovery_attempts = 0
        self.get_logger().info(
            f"{state.agent_id} confirmed at shared entrance pose; walking starts after "
            f"{self.activation_settle_sec:.2f}s."
        )

    def _apply_guard_feedback(self, state: RuntimeAgent, metadata, observed_at: float) -> None:
        blocked = metadata.get("guard_blocked", "false").lower() == "true"
        if not blocked:
            if state.guard_blocked:
                state.guard_blocked = False
                state.guard_block_reason = ""
                state.guard_clear_grace_until = observed_at + self.guard_clear_grace_sec
                state.motion_last_motion_at = observed_at
                state.portal_last_motion_at = observed_at
            return
        reason = metadata.get("guard_block_reason", "unknown").strip().lower() or "unknown"
        state.guard_blocked = True
        state.guard_block_reason = reason
        state.guard_clear_grace_until = 0.0
        if guard_block_requires_wait(reason):
            state.motion_last_motion_at = observed_at
            state.portal_last_motion_at = observed_at
            state.motion_last_pose = list(state.current_pose)
            state.portal_last_pose = list(state.current_pose)
            state.motion_recovery_attempts = 0
            state.portal_recovery_attempts = 0
        if metadata.get("guard_block_generation") is None:
            return
        try:
            generation = int(metadata.get("guard_block_generation", "0"))
            count = int(metadata.get("guard_block_count", "0"))
        except ValueError:
            return
        if generation == state.guard_block_generation:
            state.guard_block_count = max(state.guard_block_count, count)
            return
        state.guard_block_generation = generation
        state.guard_block_count = count
        if observed_at - state.guard_feedback_at < 1.0:
            return
        state.guard_feedback_at = observed_at
        if guard_block_requires_wait(reason):
            self.get_logger().info(
                f"{state.agent_id} guard blocked by {reason} at command generation {generation}; "
                "pausing stall recovery until the external actor clears."
            )
        else:
            self.get_logger().warning(
                f"{state.agent_id} guard blocked by {reason} at command generation {generation}; "
                "normal stall timeout remains active."
            )

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

def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--semantics", default=_default_config_path("toilet_semantics.yaml"))
    parser.add_argument("--benchmark", default=_default_config_path("toilet_benchmark.yaml"))
    parser.add_argument("--spawn-service", default="/isaac/spawn_pedestrian")
    parser.add_argument("--move-service", default="/isaac/move_pedestrians")
    parser.add_argument(
        "--motion-backend",
        choices=("isaac_people", "hunav"),
        default="",
        help="Override motion_backend.type from the benchmark YAML.",
    )
    parser.add_argument(
        "--hunav-namespace",
        default="",
        help="Namespace containing HuNav compute_agents/reset_agents services.",
    )
    parser.add_argument(
        "--no-start-hunav-manager",
        action="store_true",
        help="Connect to an already-running HuNav manager instead of starting one.",
    )
    parser.add_argument("--initial-agents", type=int, default=1)
    parser.add_argument("--character-name", default="original_female_adult_business_02")
    parser.add_argument("--character-pool", default="")
    parser.add_argument("--profile", default="regular")
    parser.add_argument(
        "--target-resource",
        action="append",
        default=[],
        help="Restrict initial agents to one or more explicit semantic resource IDs.",
    )
    parser.add_argument(
        "--status-topic",
        default="/toilet_benchmark/director_status",
        help="Publish pedestrian lifecycle JSON events for orchestration.",
    )
    parsed, ros_args = parser.parse_known_args(args=args)

    character_pool = [
        name.strip()
        for name in str(parsed.character_pool).split(",")
        if name.strip()
    ]

    benchmark = _load_yaml(parsed.benchmark)
    motion_cfg = dict(benchmark.get("motion_backend", {}) or {})
    motion_backend_name = str(
        parsed.motion_backend or motion_cfg.get("type", "isaac_people")
    ).strip().lower()
    hunav_namespace = str(
        parsed.hunav_namespace
        or motion_cfg.get("hunav_namespace", "/toilet_hunav_takeover")
    ).strip()
    if hunav_namespace and not hunav_namespace.startswith("/"):
        hunav_namespace = "/" + hunav_namespace
    manager = None
    if motion_backend_name == "hunav" and not parsed.no_start_hunav_manager:
        from .hunav_phase0_smoke import ManagedHuNavProcess

        log_dir = Path("/tmp/toilet_hunav_takeover")
        log_dir.mkdir(parents=True, exist_ok=True)
        manager = ManagedHuNavProcess(
            hunav_namespace,
            log_dir / f"hunav_manager_{os.getpid()}.log",
        )
        manager.start()

    rclpy.init(args=ros_args)
    node = None
    try:
        node = ToiletDirectorNode(
            semantics_path=parsed.semantics,
            benchmark_path=parsed.benchmark,
            spawn_service=parsed.spawn_service,
            move_service=parsed.move_service,
            initial_agents=parsed.initial_agents,
            character_name=parsed.character_name,
            character_pool=character_pool or None,
            profile_name=parsed.profile,
            target_resource_ids=parsed.target_resource,
            status_topic=parsed.status_topic,
            motion_backend_name=motion_backend_name,
            hunav_namespace=hunav_namespace,
        )
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if manager is not None:
            manager.stop()


if __name__ == "__main__":
    main()
