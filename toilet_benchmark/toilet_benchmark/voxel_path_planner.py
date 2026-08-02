from __future__ import annotations

import gzip
import heapq
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


GridCell = tuple[int, int]
DynamicObstacle = tuple[float, ...]


class PathPlanningError(RuntimeError):
    """Raised when the voxel planner cannot produce a collision-free path."""


@dataclass(frozen=True)
class VoxelPathPlannerConfig:
    map_path: str
    z_min: float = 0.05
    z_max: float = 1.2
    agent_radius_m: float = 0.30
    bounds_padding_m: float = 1.0
    max_expansions: int = 20000
    nearest_free_radius_m: float = 0.8
    simplify: bool = True
    max_segment_length_m: float = 2.0
    constrained_segment_length_m: float = 0.9
    preferred_clearance_m: float = 0.50
    clearance_cost_weight: float = 4.0
    turn_cost_weight: float = 0.35
    any_angle: bool = False


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
        grid_bounds: tuple[int, int, int, int] | None,
        config: VoxelPathPlannerConfig,
    ):
        self.resolution = float(resolution)
        self.origin = origin
        self.config = config
        self.occupied = set(occupied)
        self.grid_bounds = grid_bounds
        self._clearance_by_cell = self._build_clearance_field(
            max(float(config.agent_radius_m), float(config.preferred_clearance_m))
        )
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
        bounds_raw = data.get("grid_bounds")
        grid_bounds = None
        if isinstance(bounds_raw, (list, tuple)) and len(bounds_raw) >= 4:
            grid_bounds = tuple(int(value) for value in bounds_raw[:4])
        return cls(
            resolution=resolution,
            origin=origin,
            occupied=occupied,
            grid_bounds=grid_bounds,
            config=config,
        )

    def plan(
        self,
        start: list[float],
        goal: list[float],
        *,
        z: float = 0.0,
        dynamic_obstacles: Iterable[DynamicObstacle] | None = None,
    ) -> list[list[float]]:
        dynamic_blocked, dynamic_cost = self._build_dynamic_layer(
            dynamic_obstacles or ()
        )
        requested_start_cell = self.world_to_cell(float(start[0]), float(start[1]))
        start_cell = self.nearest_free(requested_start_cell, dynamic_blocked=dynamic_blocked)
        goal_cell = self.nearest_free(
            self.world_to_cell(float(goal[0]), float(goal[1])),
            dynamic_blocked=dynamic_blocked,
        )
        if start_cell is None or goal_cell is None:
            raise PathPlanningError("start or goal has no nearby free voxel cell")
        if start_cell == goal_cell:
            points = [self.cell_to_world(goal_cell, z=z)]
            return self._append_safe_exact_goal(points, goal, z=z, dynamic_blocked=dynamic_blocked)

        cells = self._astar(
            start_cell,
            goal_cell,
            dynamic_blocked=dynamic_blocked,
            dynamic_cost=dynamic_cost,
        )
        if not cells:
            raise PathPlanningError(f"no voxel path from {start_cell} to {goal_cell}")
        if self.config.simplify:
            cells = self._simplify_cells(
                cells,
                dynamic_blocked=dynamic_blocked,
                dynamic_cost=dynamic_cost,
            )

        points = [self.cell_to_world(cell, z=z) for cell in cells[1:]]
        if requested_start_cell != start_cell:
            if not self._line_avoids_raw_obstacles(
                requested_start_cell,
                start_cell,
                dynamic_blocked=dynamic_blocked,
            ):
                raise PathPlanningError("start cannot safely enter the bounded voxel map")
            points.insert(0, self.cell_to_world(start_cell, z=z))
        if not points:
            points = [self.cell_to_world(goal_cell, z=z)]
        return self._append_safe_exact_goal(points, goal, z=z, dynamic_blocked=dynamic_blocked)

    def _append_safe_exact_goal(
        self,
        points: list[list[float]],
        goal: list[float],
        *,
        z: float,
        dynamic_blocked: set[GridCell] | None = None,
    ) -> list[list[float]]:
        exact_goal = [float(goal[0]), float(goal[1]), float(z)]
        if _planar_distance(points[-1], exact_goal) <= 1e-6:
            return _dedupe_nearby_points(points)
        goal_cell = self.world_to_cell(exact_goal[0], exact_goal[1])
        start_cell = self.world_to_cell(points[-1][0], points[-1][1])
        # Final semantic placement may enter the inflated safety margin, but it
        # must never enter raw scene geometry or cross a raw occupied cell.
        if (
            self._in_bounds(goal_cell)
            and goal_cell not in self.occupied
            and goal_cell not in (dynamic_blocked or set())
            and self._line_avoids_raw_obstacles(
                start_cell,
                goal_cell,
                dynamic_blocked=dynamic_blocked,
            )
        ):
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

    def nearest_free(self, cell: GridCell, *, dynamic_blocked: set[GridCell] | None = None) -> GridCell | None:
        if self.is_free(cell, dynamic_blocked=dynamic_blocked):
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
                if self.is_free(candidate, dynamic_blocked=dynamic_blocked):
                    return candidate
        return None

    def is_free(self, cell: GridCell, *, dynamic_blocked: set[GridCell] | None = None) -> bool:
        return (
            self._in_bounds(cell)
            and cell not in self.inflated_occupied
            and cell not in (dynamic_blocked or set())
        )

    def clearance_m(self, cell: GridCell) -> float:
        if not self._in_bounds(cell):
            return 0.0
        return float(self._clearance_by_cell.get(cell, math.inf))

    def polyline_is_free(
        self,
        points: Iterable[list[float]],
        *,
        allow_out_of_bounds: bool = False,
    ) -> bool:
        cells = [self.world_to_cell(float(point[0]), float(point[1])) for point in points]
        if not cells:
            return False
        if any(
            cell in self.inflated_occupied
            or (not allow_out_of_bounds and not self._in_bounds(cell))
            for cell in cells
        ):
            return False
        for start, goal in zip(cells, cells[1:]):
            for cell in _supercover_cells(start, goal):
                if cell in self.inflated_occupied:
                    return False
                if not allow_out_of_bounds and not self._in_bounds(cell):
                    return False
        return True

    def polyline_avoids_raw_obstacles(
        self,
        points: Iterable[list[float]],
        *,
        allow_out_of_bounds: bool = False,
    ) -> bool:
        """Check line of sight against scene geometry without agent inflation."""
        cells = [
            self.world_to_cell(float(point[0]), float(point[1]))
            for point in points
        ]
        if not cells:
            return False
        for cell in cells:
            if cell in self.occupied:
                return False
            if not allow_out_of_bounds and not self._in_bounds(cell):
                return False
        for start, goal in zip(cells, cells[1:]):
            for cell in _supercover_cells(start, goal):
                if cell in self.occupied:
                    return False
                if not allow_out_of_bounds and not self._in_bounds(cell):
                    return False
        return True

    def _in_bounds(self, cell: GridCell) -> bool:
        if self.grid_bounds is None:
            return True
        min_x, max_x, min_y, max_y = self.grid_bounds
        return min_x <= cell[0] <= max_x and min_y <= cell[1] <= max_y

    def _inflate_occupied(self, occupied: set[GridCell], radius_m: float) -> set[GridCell]:
        radius = max(0.0, float(radius_m))
        return {
            cell
            for cell, clearance in self._clearance_by_cell.items()
            if clearance <= radius + 1e-9
        }.union(occupied)

    def _build_clearance_field(self, max_distance_m: float) -> dict[GridCell, float]:
        if not self.occupied:
            return {}
        radius_cells = max(1, math.ceil(max(0.0, float(max_distance_m)) / self.resolution) + 1)
        field: dict[GridCell, float] = {}
        half_cell = 0.5 * self.resolution
        for obstacle_x, obstacle_y in self.occupied:
            for dx in range(-radius_cells, radius_cells + 1):
                for dy in range(-radius_cells, radius_cells + 1):
                    cell = (obstacle_x + dx, obstacle_y + dy)
                    if not self._in_bounds(cell):
                        continue
                    gap_x = max(abs(float(dx)) * self.resolution - half_cell, 0.0)
                    gap_y = max(abs(float(dy)) * self.resolution - half_cell, 0.0)
                    clearance = math.hypot(gap_x, gap_y)
                    if clearance > max_distance_m + self.resolution:
                        continue
                    if clearance < field.get(cell, math.inf):
                        field[cell] = clearance
        return field

    def _clearance_cost(
        self,
        cell: GridCell,
        dynamic_cost: dict[GridCell, float] | None = None,
    ) -> float:
        preferred = max(float(self.config.preferred_clearance_m), float(self.config.agent_radius_m))
        hard = max(0.0, float(self.config.agent_radius_m))
        clearance = self.clearance_m(cell)
        if not math.isfinite(clearance) or clearance >= preferred or preferred <= hard + 1e-9:
            static_cost = 0.0
        else:
            normalized = (preferred - max(clearance, hard)) / (preferred - hard)
            static_cost = (
                max(0.0, float(self.config.clearance_cost_weight))
                * normalized
                * normalized
            )
        return max(static_cost, float((dynamic_cost or {}).get(cell, 0.0)))

    def _turn_cost(self, previous: GridCell | None, current: GridCell, neighbor: GridCell) -> float:
        if previous is None:
            return 0.0
        ax, ay = current[0] - previous[0], current[1] - previous[1]
        bx, by = neighbor[0] - current[0], neighbor[1] - current[1]
        denom = math.hypot(ax, ay) * math.hypot(bx, by)
        if denom <= 1e-9:
            return 0.0
        cosine = max(-1.0, min(1.0, float(ax * bx + ay * by) / denom))
        return max(0.0, float(self.config.turn_cost_weight)) * (1.0 - cosine)

    def _build_dynamic_layer(
        self,
        obstacles: Iterable[DynamicObstacle],
    ) -> tuple[set[GridCell], dict[GridCell, float]]:
        hard_radius = max(0.0, float(self.config.agent_radius_m))
        blocked: set[GridCell] = set()
        cost_by_cell: dict[GridCell, float] = {}
        weight = max(0.0, float(self.config.clearance_cost_weight))
        for obstacle in obstacles:
            if len(obstacle) < 3:
                continue
            obstacle_x, obstacle_y, obstacle_radius = obstacle[:3]
            preferred = (
                hard_radius + max(0.0, float(obstacle[3]))
                if len(obstacle) >= 4
                else max(
                    hard_radius,
                    float(self.config.preferred_clearance_m),
                )
            )
            radius = max(0.0, float(obstacle_radius))
            center = self.world_to_cell(float(obstacle_x), float(obstacle_y))
            search_radius = math.ceil((radius + preferred + self.resolution) / self.resolution)
            for dx in range(-search_radius, search_radius + 1):
                for dy in range(-search_radius, search_radius + 1):
                    cell = (center[0] + dx, center[1] + dy)
                    if not self._in_bounds(cell):
                        continue
                    world = self.cell_to_world(cell, z=0.0)
                    clearance = max(
                        0.0,
                        math.hypot(world[0] - float(obstacle_x), world[1] - float(obstacle_y)) - radius,
                    )
                    if clearance > preferred + self.resolution:
                        continue
                    if clearance <= hard_radius + 1e-9:
                        blocked.add(cell)
                        continue
                    if preferred <= hard_radius + 1e-9:
                        continue
                    normalized = (
                        preferred - min(clearance, preferred)
                    ) / (preferred - hard_radius)
                    cost = weight * normalized * normalized
                    if cost > cost_by_cell.get(cell, 0.0):
                        cost_by_cell[cell] = cost
        return blocked, cost_by_cell

    def _astar(
        self,
        start: GridCell,
        goal: GridCell,
        *,
        dynamic_blocked: set[GridCell] | None = None,
        dynamic_cost: dict[GridCell, float] | None = None,
    ) -> list[GridCell]:
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
            for neighbor, step_cost in self._neighbors(current, dynamic_blocked=dynamic_blocked):
                if neighbor[0] < min_x or neighbor[0] > max_x or neighbor[1] < min_y or neighbor[1] > max_y:
                    continue
                if not self.is_free(neighbor, dynamic_blocked=dynamic_blocked):
                    continue
                predecessor = current
                new_cost = (
                    current_cost
                    + step_cost
                    + self._clearance_cost(neighbor, dynamic_cost)
                    + self._turn_cost(came_from.get(current), current, neighbor)
                )
                parent = came_from.get(current)
                if (
                    self.config.any_angle
                    and parent is not None
                    and self._line_is_free(
                        parent,
                        neighbor,
                        dynamic_blocked=dynamic_blocked,
                    )
                ):
                    line_cells = list(_supercover_cells(parent, neighbor))
                    average_clearance_cost = sum(
                        self._clearance_cost(cell, dynamic_cost)
                        for cell in line_cells[1:]
                    ) / max(1, len(line_cells) - 1)
                    distance = _cell_distance(parent, neighbor)
                    parent_cost = (
                        cost_so_far[parent]
                        + distance * (1.0 + average_clearance_cost)
                        + self._turn_cost(came_from.get(parent), parent, neighbor)
                    )
                    if parent_cost < new_cost:
                        predecessor = parent
                        new_cost = parent_cost
                if new_cost >= cost_so_far.get(neighbor, math.inf):
                    continue
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = predecessor
                priority = new_cost + _cell_distance(neighbor, goal)
                heapq.heappush(open_heap, (priority, new_cost, neighbor))
        return []

    def _neighbors(
        self,
        cell: GridCell,
        *,
        dynamic_blocked: set[GridCell] | None = None,
    ) -> Iterable[tuple[GridCell, float]]:
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
            if dx != 0 and dy != 0:
                # Do not let a diagonal step cut between two blocked corners.
                if not self.is_free(
                    (cell[0] + dx, cell[1]), dynamic_blocked=dynamic_blocked
                ) or not self.is_free((cell[0], cell[1] + dy), dynamic_blocked=dynamic_blocked):
                    continue
            yield (cell[0] + dx, cell[1] + dy), cost

    def _search_bounds(self, start: GridCell, goal: GridCell) -> tuple[int, int, int, int]:
        padding = max(4, math.ceil(float(self.config.bounds_padding_m) / self.resolution))
        xs = [start[0], goal[0]]
        ys = [start[1], goal[1]]
        if self.inflated_occupied:
            xs.extend(cell[0] for cell in self.inflated_occupied)
            ys.extend(cell[1] for cell in self.inflated_occupied)
        return min(xs) - padding, max(xs) + padding, min(ys) - padding, max(ys) + padding

    def _simplify_cells(
        self,
        cells: list[GridCell],
        *,
        dynamic_blocked: set[GridCell] | None = None,
        dynamic_cost: dict[GridCell, float] | None = None,
    ) -> list[GridCell]:
        if len(cells) <= 2:
            return cells
        max_length = float(self.config.max_segment_length_m)
        constrained_length = min(max_length, float(self.config.constrained_segment_length_m))
        preferred_clearance = float(self.config.preferred_clearance_m)
        simplified = [cells[0]]
        anchor_index = 0
        while anchor_index < len(cells) - 1:
            best_index = anchor_index + 1
            probe_index = best_index + 1
            while probe_index < len(cells):
                segment_length = _cell_distance(cells[anchor_index], cells[probe_index]) * self.resolution
                line_cells = list(_supercover_cells(cells[anchor_index], cells[probe_index]))
                if not all(self.is_free(cell, dynamic_blocked=dynamic_blocked) for cell in line_cells):
                    break
                segment_limit = max_length
                if preferred_clearance > 0.0 and any(
                    self.clearance_m(cell) < preferred_clearance
                    or float((dynamic_cost or {}).get(cell, 0.0)) > 0.0
                    for cell in line_cells
                ):
                    segment_limit = constrained_length
                if segment_limit > 0.0 and segment_length > segment_limit + 1e-9:
                    break
                best_index = probe_index
                probe_index += 1
            simplified.append(cells[best_index])
            anchor_index = best_index
        return simplified

    def _line_is_free(
        self,
        start: GridCell,
        goal: GridCell,
        *,
        dynamic_blocked: set[GridCell] | None = None,
    ) -> bool:
        for cell in _supercover_cells(start, goal):
            if not self.is_free(cell, dynamic_blocked=dynamic_blocked):
                return False
        return True

    def _line_avoids_raw_obstacles(
        self,
        start: GridCell,
        goal: GridCell,
        *,
        dynamic_blocked: set[GridCell] | None = None,
    ) -> bool:
        blocked = dynamic_blocked or set()
        return all(
            cell not in self.occupied and cell not in blocked
            for cell in _supercover_cells(start, goal)
        )


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


def _supercover_cells(start: GridCell, goal: GridCell) -> Iterable[GridCell]:
    x0, y0 = start
    x1, y1 = goal
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    x, y = x0, y0
    ix = 0
    iy = 0
    yielded: set[GridCell] = set()

    def emit(cell: GridCell):
        if cell not in yielded:
            yielded.add(cell)
            return cell
        return None

    first = emit((x, y))
    if first is not None:
        yield first
    while ix < dx or iy < dy:
        decision = (1 + 2 * ix) * dy - (1 + 2 * iy) * dx
        if decision == 0:
            side_x = emit((x + sx, y))
            if side_x is not None:
                yield side_x
            side_y = emit((x, y + sy))
            if side_y is not None:
                yield side_y
            x0 += sx
            y0 += sy
            x, y = x0, y0
            ix += 1
            iy += 1
        elif decision < 0:
            x0 += sx
            x = x0
            ix += 1
        else:
            y0 += sy
            y = y0
            iy += 1
        current = emit((x, y))
        if current is not None:
            yield current


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
