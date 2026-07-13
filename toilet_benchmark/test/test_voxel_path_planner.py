import json
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


if __name__ == "__main__":
    unittest.main()
