import json
import tempfile
import unittest
from pathlib import Path

from toilet_benchmark.voxel_path_planner import VoxelPathPlanner, VoxelPathPlannerConfig


class TestVoxelPathPlanner(unittest.TestCase):
    def _write_map(self, columns):
        tmpdir = tempfile.TemporaryDirectory()
        path = Path(tmpdir.name) / "test.voxel.json"
        path.write_text(
            json.dumps(
                {
                    "format": "arena_voxel_guard_v1",
                    "resolution": 1.0,
                    "origin": [0.0, 0.0, 0.0],
                    "columns": columns,
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

    def test_plan_uses_nearest_free_when_goal_cell_is_occupied(self):
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
        self.assertEqual(points[-1], [1.5, 0.5, 0.0])


if __name__ == "__main__":
    unittest.main()
