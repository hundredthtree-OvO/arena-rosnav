"""Backend-neutral route requests and the legacy voxel planner adapter."""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from .motion import GlobalRouterPort, RoutePlan
from .voxel_path_planner import PathPlanningError


DynamicObstacle = tuple[float, float, float]


@dataclass(frozen=True)
class RouteRequest:
    agent_id: str
    start_pose: Sequence[float]
    goal_pose: Sequence[float]


class RouteProvider(GlobalRouterPort, Protocol):
    def plan(self, request: RouteRequest) -> RoutePlan | None:
        ...


class VoxelRouteProvider:
    """Preserve the current fail-closed voxel routing policy behind one interface."""

    def __init__(
        self,
        *,
        planner,
        walk_plane_z: float,
        dynamic_obstacles: Callable[[float], list[DynamicObstacle]],
        logger,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._planner = planner
        self._walk_plane_z = float(walk_plane_z)
        self._dynamic_obstacles = dynamic_obstacles
        self._logger = logger
        self._clock = clock
        self._dynamic_wait_log: dict[str, float] = {}

    def plan(self, request: RouteRequest) -> RoutePlan | None:
        now = self._clock()
        obstacles = self._dynamic_obstacles(now)
        start_pose = list(request.start_pose)
        goal_pose = list(request.goal_pose)
        try:
            points = self._planner.plan(
                start_pose,
                goal_pose,
                z=self._walk_plane_z,
                dynamic_obstacles=obstacles,
            )
        except PathPlanningError as exc:
            if obstacles and self._static_route_exists(start_pose, goal_pose):
                if now - self._dynamic_wait_log.get(request.agent_id, 0.0) >= 2.0:
                    self._dynamic_wait_log[request.agent_id] = now
                    self._logger.warning(
                        f"{request.agent_id} voxel route is temporarily blocked by the robot; "
                        "waiting for a safe replan."
                    )
                return None
            self._logger.error(
                f"{request.agent_id} voxel path planning failed; motion rejected: {exc}"
            )
            return None
        except Exception as exc:
            self._logger.error(
                f"{request.agent_id} voxel planner error; motion rejected: {exc}"
            )
            return None
        if not points:
            self._logger.error(
                f"{request.agent_id} voxel planner returned an empty path; motion rejected."
            )
            return None
        self._logger.info(
            f"{request.agent_id} voxel path: start={start_pose[:3]}, goal={goal_pose[:3]}, "
            f"points={len(points)}, dynamic_obstacles={len(obstacles)}"
        )
        return RoutePlan.from_points(
            agent_id=request.agent_id,
            points=points,
            planner_id="voxel",
            dynamic_obstacle_count=len(obstacles),
        )

    def _static_route_exists(
        self,
        start_pose: list[float],
        goal_pose: list[float],
    ) -> bool:
        try:
            self._planner.plan(
                start_pose,
                goal_pose,
                z=self._walk_plane_z,
            )
        except PathPlanningError:
            return False
        return True


class WalkableMapRouteProvider:
    """Plan walkable-map routes with an event-sampled dynamic robot layer."""

    def __init__(
        self,
        *,
        planner,
        walk_plane_z: float,
        logger,
        dynamic_obstacles: Callable[[float], list[DynamicObstacle]] = lambda _now: [],
        clock: Callable[[], float] = time.monotonic,
    ):
        self._planner = planner
        self._walk_plane_z = float(walk_plane_z)
        self._logger = logger
        self._dynamic_obstacles = dynamic_obstacles
        self._clock = clock

    def plan(self, request: RouteRequest) -> RoutePlan | None:
        start_pose = list(request.start_pose)
        goal_pose = list(request.goal_pose)
        obstacles = self._dynamic_obstacles(self._clock())
        try:
            points = self._planner.plan(
                start_pose,
                goal_pose,
                z=self._walk_plane_z,
                dynamic_obstacles=obstacles,
            )
        except PathPlanningError as exc:
            reason = str(exc).replace("voxel", "walkable-map")
            self._logger.error(
                f"{request.agent_id} walkable-map route failed: {reason}"
            )
            return None
        except Exception as exc:
            self._logger.error(
                f"{request.agent_id} walkable-map planner error: {exc}"
            )
            return None
        if not points:
            self._logger.error(
                f"{request.agent_id} walkable-map planner returned an empty path."
            )
            return None
        self._logger.info(
            f"{request.agent_id} walkable-map route: start={start_pose[:3]}, "
            f"goal={goal_pose[:3]}, points={len(points)}, "
            f"dynamic_obstacles={len(obstacles)}, "
            f"fingerprint={self._planner.scene_fingerprint[:12]}, "
            f"route={[[round(float(value), 3) for value in point] for point in points]}"
        )
        return RoutePlan.from_points(
            agent_id=request.agent_id,
            points=points,
            planner_id="walkable_map",
            map_version=str(self._planner.scene_fingerprint),
            dynamic_obstacle_count=len(obstacles),
        )
