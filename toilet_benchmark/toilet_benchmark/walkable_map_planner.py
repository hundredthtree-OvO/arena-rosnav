"""Global routing over an exported static pedestrian walkable map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path

from .voxel_path_planner import VoxelPathPlanner, VoxelPathPlannerConfig
from .walkable_map import load_walkable_map


@dataclass(frozen=True)
class WalkableMapPlannerConfig:
    map_path: str
    agent_radius_m: float = 0.24
    nearest_free_radius_m: float = 0.60
    max_expansions: int = 30000
    simplify: bool = True
    max_segment_length_m: float = 1.50
    constrained_segment_length_m: float = 0.40
    preferred_clearance_m: float = 0.40
    clearance_cost_weight: float = 3.0
    turn_cost_weight: float = 0.25


class WalkableMapPlanner:
    """Adapt a ROS-order occupancy asset to the shared deterministic grid search."""

    def __init__(self, payload: dict, config: WalkableMapPlannerConfig):
        self.config = config
        self.map_path = str(Path(config.map_path).expanduser())
        self.scene_fingerprint = str(payload.get("scene_fingerprint", ""))
        self.resolution = float(payload["resolution"])
        self.origin = (
            float(payload["origin"][0]),
            float(payload["origin"][1]),
            float(payload["origin"][2]),
        )
        self.width = int(payload["width"])
        self.height = int(payload["height"])
        # Unknown is fail-closed. Semantic anchors may enter the footprint
        # margin, but exact targets never enter a raw occupied cell.
        occupied = {
            (index % self.width, index // self.width)
            for index, value in enumerate(payload["data"])
            if int(value) != 0
        }
        self._planner = VoxelPathPlanner(
            resolution=self.resolution,
            origin=self.origin,
            occupied=occupied,
            grid_bounds=(0, self.width - 1, 0, self.height - 1),
            config=VoxelPathPlannerConfig(
                map_path=self.map_path,
                agent_radius_m=float(config.agent_radius_m),
                bounds_padding_m=0.0,
                max_expansions=int(config.max_expansions),
                nearest_free_radius_m=float(config.nearest_free_radius_m),
                simplify=bool(config.simplify),
                max_segment_length_m=float(config.max_segment_length_m),
                constrained_segment_length_m=float(
                    config.constrained_segment_length_m
                ),
                preferred_clearance_m=float(config.preferred_clearance_m),
                clearance_cost_weight=float(config.clearance_cost_weight),
                turn_cost_weight=float(config.turn_cost_weight),
                any_angle=True,
            ),
        )

    @classmethod
    def from_file(cls, config: WalkableMapPlannerConfig) -> "WalkableMapPlanner":
        return cls(load_walkable_map(config.map_path), config)

    @property
    def occupied_count(self) -> int:
        return len(self._planner.occupied)

    @property
    def inflated_occupied_count(self) -> int:
        return len(self._planner.inflated_occupied)

    def plan(
        self,
        start: list[float],
        goal: list[float],
        *,
        z: float,
        dynamic_obstacles=None,
    ) -> list[list[float]]:
        return self._planner.plan(
            start,
            goal,
            z=z,
            dynamic_obstacles=dynamic_obstacles,
        )

    def polyline_is_free(
        self,
        points,
        *,
        allow_out_of_bounds: bool = False,
    ) -> bool:
        return self._planner.polyline_is_free(
            points,
            allow_out_of_bounds=allow_out_of_bounds,
        )

    def polyline_avoids_raw_obstacles(
        self,
        points,
        *,
        allow_out_of_bounds: bool = False,
    ) -> bool:
        return self._planner.polyline_avoids_raw_obstacles(
            points,
            allow_out_of_bounds=allow_out_of_bounds,
        )

    def clearance_at(self, position) -> float:
        cell = self._planner.world_to_cell(
            float(position[0]),
            float(position[1]),
        )
        return self._planner.clearance_m(cell)

    def clip_step(
        self,
        start,
        proposed,
        *,
        iterations: int = 10,
    ) -> tuple[tuple[float, float], float, bool]:
        """Clip a swept step to the inflated walkable configuration space."""
        start_xy = (float(start[0]), float(start[1]))
        proposed_xy = (float(proposed[0]), float(proposed[1]))
        start_point = [start_xy[0], start_xy[1], 0.0]
        proposed_point = [proposed_xy[0], proposed_xy[1], 0.0]
        if self.polyline_is_free((start_point, proposed_point)):
            return proposed_xy, 1.0, False

        # Semantic interaction anchors may intentionally sit in the inflated
        # margin. Permit monotonic escape while still forbidding raw geometry.
        start_cell = self._planner.world_to_cell(*start_xy)
        proposed_cell = self._planner.world_to_cell(*proposed_xy)
        if (
            start_cell in self._planner.inflated_occupied
            and start_cell not in self._planner.occupied
            and proposed_cell not in self._planner.occupied
            and self._planner._line_avoids_raw_obstacles(start_cell, proposed_cell)
            and self._planner.clearance_m(proposed_cell)
            >= self._planner.clearance_m(start_cell) - 1e-9
        ):
            return proposed_xy, 1.0, False

        low = 0.0
        high = 1.0
        for _ in range(max(1, int(iterations))):
            fraction = 0.5 * (low + high)
            point = [
                start_xy[0] + fraction * (proposed_xy[0] - start_xy[0]),
                start_xy[1] + fraction * (proposed_xy[1] - start_xy[1]),
                0.0,
            ]
            if self.polyline_is_free((start_point, point)):
                low = fraction
            else:
                high = fraction
        safe = (
            start_xy[0] + low * (proposed_xy[0] - start_xy[0]),
            start_xy[1] + low * (proposed_xy[1] - start_xy[1]),
        )
        return safe, low, True

    def project_step(
        self,
        start,
        proposed,
        *,
        target=None,
        max_deflection_deg: float = 80.0,
        angle_step_deg: float = 20.0,
    ) -> tuple[tuple[float, float], float, bool, str]:
        """Project an obstructed step onto a nearby walkable direction.

        The direct sweep remains preferred. When it is substantially blocked,
        bounded left/right candidates preserve forward progress toward the
        route target instead of repeatedly returning a zero-length step.
        """
        start_xy = (float(start[0]), float(start[1]))
        proposed_xy = (float(proposed[0]), float(proposed[1]))
        direct, direct_fraction, direct_clipped = self.clip_step(
            start_xy,
            proposed_xy,
        )
        if not direct_clipped:
            return direct, direct_fraction, False, "direct"

        dx = proposed_xy[0] - start_xy[0]
        dy = proposed_xy[1] - start_xy[1]
        step_length = math.hypot(dx, dy)
        if step_length <= 1e-6:
            return direct, direct_fraction, True, "stopped"

        target_xy = (
            None
            if target is None
            else (float(target[0]), float(target[1]))
        )
        target_distance = (
            0.0
            if target_xy is None
            else max(1e-6, math.dist(start_xy, target_xy))
        )
        base_angle = math.atan2(dy, dx)
        max_deflection = max(0.0, min(89.0, float(max_deflection_deg)))
        angle_step = max(5.0, float(angle_step_deg))
        deflections = []
        angle = angle_step
        while angle <= max_deflection + 1e-6:
            deflections.extend((angle, -angle))
            angle += angle_step

        best = (direct, direct_fraction, "clipped")
        direct_progress = self._target_progress(
            start_xy,
            direct,
            target_xy,
            target_distance,
        )
        best_score = self._projection_score(
            direct_fraction,
            1.0,
            direct_progress,
            self.clearance_at(direct),
        )
        for deflection_deg in deflections:
            angle_rad = base_angle + math.radians(deflection_deg)
            candidate = (
                start_xy[0] + step_length * math.cos(angle_rad),
                start_xy[1] + step_length * math.sin(angle_rad),
            )
            safe, fraction, _ = self.clip_step(start_xy, candidate)
            alignment = max(0.0, math.cos(math.radians(deflection_deg)))
            progress = self._target_progress(
                start_xy,
                safe,
                target_xy,
                target_distance,
            )
            # Never trade a blocked forward step for motion away from the route.
            if target_xy is not None and progress < -1e-6:
                continue
            score = self._projection_score(
                fraction,
                alignment,
                progress,
                self.clearance_at(safe),
            )
            if score > best_score + 1e-6:
                best_score = score
                best = (safe, fraction, "tangent")

        safe, fraction, mode = best
        return safe, fraction, True, mode

    @staticmethod
    def _target_progress(start, end, target, initial_distance: float) -> float:
        if target is None:
            return 0.0
        return (initial_distance - math.dist(end, target)) / initial_distance

    @staticmethod
    def _projection_score(
        fraction: float,
        alignment: float,
        target_progress: float,
        clearance: float,
    ) -> float:
        finite_clearance = 1.0 if not math.isfinite(clearance) else min(1.0, clearance)
        return (
            0.60 * float(fraction)
            + 0.15 * float(alignment)
            + 0.20 * float(target_progress)
            + 0.05 * finite_clearance
        )
