import json
import math
import tempfile
import unittest
from pathlib import Path

from toilet_benchmark.voxel_path_planner import PathPlanningError, VoxelPathPlanner, VoxelPathPlannerConfig


class TestVoxelPathPlanner(unittest.TestCase):
    def _write_map(self, columns, *, grid_bounds=None):
        tmpdir = tempfile.TemporaryDirectory()
        path = Path(tmpdir.name) / "test.voxel.json"
        path.write_text(
            json.dumps(
                {
                    "format": "arena_voxel_guard_v1",
                    "resolution": 1.0,
                    "origin": [0.0, 0.0, 0.0],
                    "columns": columns,
                    **({"grid_bounds": grid_bounds} if grid_bounds is not None else {}),
                }
            ),
            encoding="utf-8",
        )
        self.addCleanup(tmpdir.cleanup)
        return str(path)

    def test_plan_routes_around_voxel_columns(self):
        path = self._write_map([[2, y, 0, 4] for y in range(-2, 3) if y != 2])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
                bounds_padding_m=3.0,
                simplify=False,
            )
        )

        points = planner.plan([0.5, 0.5, 0.0], [4.5, 0.5, 0.0], z=0.0)
        cells = [planner.world_to_cell(point[0], point[1]) for point in points]

        self.assertGreater(len(points), 2)
        self.assertNotIn((2, 0), cells)
        self.assertEqual(points[-1], [4.5, 0.5, 0.0])

    def test_plan_does_not_append_an_occupied_exact_goal(self):
        path = self._write_map([[1, 0, 0, 4]])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
                nearest_free_radius_m=2.0,
            )
        )

        points = planner.plan([0.5, 0.5, 0.0], [1.5, 0.5, 0.0], z=0.0)

        self.assertTrue(points)
        self.assertNotEqual(points[-1], [1.5, 0.5, 0.0])
        self.assertNotEqual(planner.world_to_cell(points[-1][0], points[-1][1]), (1, 0))

    def test_out_of_bounds_cells_are_not_free(self):
        path = self._write_map([], grid_bounds=[0, 2, 0, 2])
        planner = VoxelPathPlanner.from_file(VoxelPathPlannerConfig(map_path=path, nearest_free_radius_m=2.0))

        self.assertFalse(planner.is_free((-1, 1)))
        points = planner.plan([-0.5, 1.5, 0.0], [2.5, 1.5, 0.0], z=0.0)
        self.assertEqual(planner.world_to_cell(points[0][0], points[0][1]), (0, 1))

    def test_no_path_raises_instead_of_returning_direct_goal(self):
        path = self._write_map([[1, y, 0, 4] for y in range(0, 3)], grid_bounds=[0, 2, 0, 2])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(map_path=path, z_min=0.0, z_max=2.0, nearest_free_radius_m=1.0)
        )

        with self.assertRaises(PathPlanningError):
            planner.plan([0.5, 1.5, 0.0], [2.5, 1.5, 0.0], z=0.0)

    def test_simplified_path_respects_max_segment_length(self):
        path = self._write_map([], grid_bounds=[0, 6, 0, 2])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                agent_radius_m=0.0,
                simplify=True,
                max_segment_length_m=2.0,
            )
        )

        start = [0.5, 0.5, 0.0]
        points = planner.plan(start, [6.5, 0.5, 0.0], z=0.0)
        segments = zip([start, *points[:-1]], points)

        self.assertGreater(len(points), 2)
        self.assertTrue(
            all(math.hypot(end[0] - begin[0], end[1] - begin[1]) <= 2.0 for begin, end in segments)
        )

    def test_clearance_cost_moves_path_away_from_wall_when_space_exists(self):
        columns = [[x, y, 0, 4] for x in range(0, 9) for y in (0, 6)]
        path = self._write_map(columns, grid_bounds=[0, 8, 0, 6])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
                preferred_clearance_m=2.5,
                clearance_cost_weight=8.0,
                simplify=False,
            )
        )

        points = planner.plan([0.5, 1.5, 0.0], [8.5, 1.5, 0.0], z=0.0)
        cells = [planner.world_to_cell(point[0], point[1]) for point in points]

        self.assertTrue(any(cell[1] >= 3 for cell in cells[1:-1]))

    def test_supercover_rejects_diagonal_corner_shortcut(self):
        path = self._write_map([[1, 0, 0, 4]], grid_bounds=[0, 2, 0, 2])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
            )
        )

        self.assertFalse(planner._line_is_free((0, 0), (1, 1)))

    def test_polyline_validation_checks_every_segment(self):
        path = self._write_map([[1, 1, 0, 4]], grid_bounds=[0, 3, 0, 3])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
            )
        )

        self.assertTrue(
            planner.polyline_is_free([[0.5, 0.5, 0.0], [3.5, 0.5, 0.0]])
        )
        self.assertFalse(
            planner.polyline_is_free(
                [[0.5, 0.5, 0.0], [0.5, 1.5, 0.0], [3.5, 1.5, 0.0]]
            )
        )

    def test_raw_visibility_does_not_treat_inflation_as_scene_geometry(self):
        path = self._write_map([[1, 1, 0, 4]], grid_bounds=[0, 3, 0, 3])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=1.1,
            )
        )
        segment = [[0.5, 0.5, 0.0], [2.5, 0.5, 0.0]]

        self.assertFalse(planner.polyline_is_free(segment))
        self.assertTrue(planner.polyline_avoids_raw_obstacles(segment))

    def test_polyline_can_enter_map_from_outside_without_ignoring_obstacles(self):
        path = self._write_map([[1, 1, 0, 4]], grid_bounds=[0, 3, 0, 3])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                z_min=0.0,
                z_max=2.0,
                agent_radius_m=0.0,
            )
        )

        self.assertTrue(
            planner.polyline_is_free(
                [[-1.5, 0.5, 0.0], [2.5, 0.5, 0.0]],
                allow_out_of_bounds=True,
            )
        )
        self.assertFalse(
            planner.polyline_is_free(
                [[-1.5, 1.5, 0.0], [2.5, 1.5, 0.0]],
                allow_out_of_bounds=True,
            )
        )

    def test_dynamic_obstacle_is_included_in_replan(self):
        path = self._write_map([], grid_bounds=[0, 8, 0, 4])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                agent_radius_m=0.0,
                preferred_clearance_m=1.5,
                clearance_cost_weight=6.0,
                simplify=False,
            )
        )

        points = planner.plan(
            [0.5, 2.5, 0.0],
            [8.5, 2.5, 0.0],
            z=0.0,
            dynamic_obstacles=[(4.5, 2.5, 0.75)],
        )
        cells = [planner.world_to_cell(point[0], point[1]) for point in points]

        self.assertNotIn((4, 2), cells)
        self.assertTrue(any(cell[1] != 2 for cell in cells[1:-1]))

    def test_any_angle_search_connects_visible_non_neighbor_cells(self):
        path = self._write_map([], grid_bounds=[0, 8, 0, 5])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                agent_radius_m=0.0,
                simplify=False,
                any_angle=True,
            )
        )

        points = planner.plan(
            [0.5, 0.5, 0.0],
            [8.5, 5.5, 0.0],
            z=0.0,
        )

        self.assertEqual(points, [[8.5, 5.5, 0.0]])

    def test_exact_goal_append_does_not_cross_dynamic_obstacle(self):
        path = self._write_map([], grid_bounds=[0, 8, 0, 4])
        planner = VoxelPathPlanner.from_file(
            VoxelPathPlannerConfig(
                map_path=path,
                agent_radius_m=0.0,
                simplify=True,
            )
        )

        points = planner._append_safe_exact_goal(
            [[2.5, 2.5, 0.0]],
            [6.5, 2.5, 0.0],
            z=0.0,
            dynamic_blocked={(4, 2)},
        )

        self.assertEqual(points, [[2.5, 2.5, 0.0]])


if __name__ == "__main__":
    unittest.main()
