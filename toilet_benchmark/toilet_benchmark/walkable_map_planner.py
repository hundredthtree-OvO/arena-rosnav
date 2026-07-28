"""Global routing over an exported static pedestrian walkable map."""

from __future__ import annotations

from dataclasses import dataclass
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
                constrained_segment_length_m=float(config.max_segment_length_m),
                preferred_clearance_m=float(config.preferred_clearance_m),
                clearance_cost_weight=float(config.clearance_cost_weight),
                turn_cost_weight=float(config.turn_cost_weight),
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

    def plan(self, start: list[float], goal: list[float], *, z: float) -> list[list[float]]:
        return self._planner.plan(start, goal, z=z)

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
