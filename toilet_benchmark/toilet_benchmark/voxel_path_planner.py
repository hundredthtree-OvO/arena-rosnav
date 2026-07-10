from __future__ import annotations

import gzip
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


GridCell = tuple[int, int]


@dataclass(frozen=True)
class VoxelPathPlannerConfig:
    map_path: str
    z_min: float = 0.05
    z_max: float = 1.2
    agent_radius_m: float = 0.25
    bounds_padding_m: float = 1.0
    max_expansions: int = 20000
    nearest_free_radius_m: float = 0.8
    simplify: bool = True


def _open_json_maybe_gz(path: str | Path):
    text = str(path)
    if text.endswith(".gz"):
        return gzip.open(text, "rt", encoding="utf-8")
    return open(text, "r", encoding="utf-8")


class VoxelPathPlanner:
    """A small 2D global planner backed by the arena voxel-map format."""

    def __init__(
        self,
        *,
        resolution: float,
        origin: tuple[float, float, float],
        occupied: set[GridCell],
        config: VoxelPathPlannerConfig,
    ):
        self.resolution = float(resolution)
        self.origin = origin
        self.config = config
        self.occupied = set(occupied)
        self.inflated_occupied = self._inflate_occupied(self.occupied, config.agent_radius_m)

    @classmethod
    def from_file(cls, config: VoxelPathPlannerConfig) -> "VoxelPathPlanner":
        with _open_json_maybe_gz(config.map_path) as handle:
            data = json.load(handle)
        resolution = float(data.get("resolution", data.get("voxel_size", 0.05)))
        origin_raw = data.get("origin", [0.0, 0.0, 0.0])
        origin = (float(origin_raw[0]), float(origin_raw[1]), float(origin_raw[2]))
        occupied = _occupied_columns_from_voxel_data(
            data,
            resolution=resolution,
            origin_z=origin[2],
            z_min=float(config.z_min),
            z_max=float(config.z_max),
        )
        return cls(resolution=resolution, origin=origin, occupied=occupied, config=config)

    def plan(self, start: list[float], goal: list[float], *, z: float = 0.0) -> list[list[float]]:
        start_cell = self.nearest_free(self.world_to_cell(float(start[0]), float(start[1])))
        goal_cell = self.nearest_free(self.world_to_cell(float(goal[0]), float(goal[1])))
        if start_cell is None or goal_cell is None:
            return [[float(goal[0]), float(goal[1]), float(z)]]
        if start_cell == goal_cell:
            return [[float(goal[0]), float(goal[1]), float(z)]]

        cells = self._astar(start_cell, goal_cell)
        if not cells:
            return [[float(goal[0]), float(goal[1]), float(z)]]
        if self.config.simplify:
            cells = self._simplify_cells(cells)

        points = [self.cell_to_world(cell, z=z) for cell in cells[1:]]
        exact_goal = [float(goal[0]), float(goal[1]), float(z)]
        if not points or _planar_distance(points[-1], exact_goal) > 1e-6:
            points.append(exact_goal)
        return _dedupe_nearby_points(points)

    def world_to_cell(self, x: float, y: float) -> GridCell:
        return (
            math.floor((float(x) - self.origin[0]) / self.resolution),
            math.floor((float(y) - self.origin[1]) / self.resolution),
        )

    def cell_to_world(self, cell: GridCell, *, z: float) -> list[float]:
        return [
            self.origin[0] + (float(cell[0]) + 0.5) * self.resolution,
            self.origin[1] + (float(cell[1]) + 0.5) * self.resolution,
            float(z),
        ]

    def nearest_free(self, cell: GridCell) -> GridCell | None:
        if self.is_free(cell):
            return cell
        max_radius = max(1, math.ceil(float(self.config.nearest_free_radius_m) / self.resolution))
        for radius in range(1, max_radius + 1):
            candidates: list[GridCell] = []
            for dx in range(-radius, radius + 1):
                candidates.append((cell[0] + dx, cell[1] - radius))
                candidates.append((cell[0] + dx, cell[1] + radius))
            for dy in range(-radius + 1, radius):
                candidates.append((cell[0] - radius, cell[1] + dy))
                candidates.append((cell[0] + radius, cell[1] + dy))
            for candidate in sorted(candidates, key=lambda c: _cell_distance(c, cell)):
                if self.is_free(candidate):
                    return candidate
        return None

    def is_free(self, cell: GridCell) -> bool:
        return cell not in self.inflated_occupied

    def _inflate_occupied(self, occupied: set[GridCell], radius_m: float) -> set[GridCell]:
        radius_cells = max(0, math.ceil(float(radius_m) / self.resolution))
        if radius_cells <= 0:
            return set(occupied)
        inflated = set(occupied)
        radius_sq = radius_cells * radius_cells
        for ix, iy in occupied:
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= radius_sq:
                        inflated.add((ix + dx, iy + dy))
        return inflated

    def _astar(self, start: GridCell, goal: GridCell) -> list[GridCell]:
        min_x, max_x, min_y, max_y = self._search_bounds(start, goal)
        open_heap: list[tuple[float, float, GridCell]] = []
        heapq.heappush(open_heap, (_cell_distance(start, goal), 0.0, start))
        came_from: dict[GridCell, GridCell] = {}
        cost_so_far: dict[GridCell, float] = {start: 0.0}
        expansions = 0

        while open_heap and expansions < int(self.config.max_expansions):
            _, current_cost, current = heapq.heappop(open_heap)
            if current == goal:
                return _reconstruct_path(came_from, current)
            if current_cost > cost_so_far.get(current, math.inf):
                continue
            expansions += 1
            for neighbor, step_cost in self._neighbors(current):
                if neighbor[0] < min_x or neighbor[0] > max_x or neighbor[1] < min_y or neighbor[1] > max_y:
                    continue
                if not self.is_free(neighbor):
                    continue
                new_cost = current_cost + step_cost
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                priority = new_cost + _cell_distance(neighbor, goal)
                heapq.heappush(open_heap, (priority, new_cost, neighbor))
        return []

    def _neighbors(self, cell: GridCell) -> Iterable[tuple[GridCell, float]]:
        for dx, dy, cost in (
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ):
            yield (cell[0] + dx, cell[1] + dy), cost

    def _search_bounds(self, start: GridCell, goal: GridCell) -> tuple[int, int, int, int]:
        padding = max(4, math.ceil(float(self.config.bounds_padding_m) / self.resolution))
        xs = [start[0], goal[0]]
        ys = [start[1], goal[1]]
        if self.inflated_occupied:
            xs.extend(cell[0] for cell in self.inflated_occupied)
            ys.extend(cell[1] for cell in self.inflated_occupied)
        return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding

    def _simplify_cells(self, cells: list[GridCell]) -> list[GridCell]:
        if len(cells) <= 2:
            return cells
        simplified = [cells[0]]
        anchor_index = 0
        probe_index = 2
        while probe_index < len(cells):
            if self._line_is_free(cells[anchor_index], cells[probe_index]):
                probe_index += 1
                continue
            simplified.append(cells[probe_index - 1])
            anchor_index = probe_index - 1
            probe_index = anchor_index + 2
        simplified.append(cells[-1])
        return simplified

    def _line_is_free(self, start: GridCell, goal: GridCell) -> bool:
        for cell in _bresenham_cells(start, goal):
            if not self.is_free(cell):
                return False
        return True


def _occupied_columns_from_voxel_data(
    data: dict,
    *,
    resolution: float,
    origin_z: float,
    z_min: float,
    z_max: float,
) -> set[GridCell]:
    z_min_idx = math.floor((float(z_min) - float(origin_z)) / float(resolution))
    z_max_idx = math.ceil((float(z_max) - float(origin_z)) / float(resolution))
    occupied: set[GridCell] = set()
    if data.get("columns"):
        for item in data.get("columns", []) or []:
            if len(item) < 4:
                continue
            ix, iy, iz0, iz1 = int(item[0]), int(item[1]), int(item[2]), int(item[3])
            if iz1 < z_min_idx or iz0 > z_max_idx:
                continue
            occupied.add((ix, iy))
    else:
        for item in data.get("voxels", []) or []:
            if len(item) < 3:
                continue
            ix, iy, iz = int(item[0]), int(item[1]), int(item[2])
            if iz < z_min_idx or iz > z_max_idx:
                continue
            occupied.add((ix, iy))
    return occupied


def _reconstruct_path(came_from: dict[GridCell, GridCell], current: GridCell) -> list[GridCell]:
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def _bresenham_cells(start: GridCell, goal: GridCell) -> Iterable[GridCell]:
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        err2 = 2 * err
        if err2 > -dy:
            err -= dy
            x0 += sx
        if err2 < dx:
            err += dx
            y0 += sy


def _cell_distance(a: GridCell, b: GridCell) -> float:
    return math.hypot(float(a[0] - b[0]), float(a[1] - b[1]))


def _planar_distance(a: list[float], b: list[float]) -> float:
    return math.hypot(float(a[0]) - float(b[0]), float(a[1]) - float(b[1]))


def _dedupe_nearby_points(points: list[list[float]], *, eps: float = 1e-4) -> list[list[float]]:
    deduped: list[list[float]] = []
    for point in points:
        if deduped and _planar_distance(deduped[-1], point) <= eps:
            deduped[-1] = point
            continue
        deduped.append(point)
    return deduped
