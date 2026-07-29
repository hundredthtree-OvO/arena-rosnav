import unittest

from toilet_benchmark.route_provider import (
    RouteRequest,
    VoxelRouteProvider,
    WalkableMapRouteProvider,
)
from toilet_benchmark.voxel_path_planner import PathPlanningError


class _Logger:
    def __init__(self):
        self.info_messages = []
        self.warning_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def warning(self, message):
        self.warning_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


class _Planner:
    def __init__(self, results):
        self.results = list(results)
        self.calls = []

    def plan(self, start, goal, **kwargs):
        self.calls.append((start, goal, kwargs))
        result = self.results.pop(0)
        if isinstance(result, Exception):
            raise result
        return result


class TestVoxelRouteProvider(unittest.TestCase):
    def test_plan_passes_dynamic_obstacles_and_walk_plane(self):
        planner = _Planner([[[1.0, 2.0, 0.0]]])
        logger = _Logger()
        provider = VoxelRouteProvider(
            planner=planner,
            walk_plane_z=0.0,
            dynamic_obstacles=lambda now: [(3.0, 4.0, 0.4)],
            logger=logger,
            clock=lambda: 10.0,
        )

        points = provider.plan(
            RouteRequest("toilet_agent_01", [0.0, 0.0, 1.0], [1.0, 2.0, 2.0])
        )

        self.assertEqual(points, [[1.0, 2.0, 0.0]])
        self.assertEqual(planner.calls[0][2]["z"], 0.0)
        self.assertEqual(
            planner.calls[0][2]["dynamic_obstacles"],
            [(3.0, 4.0, 0.4)],
        )
        self.assertIn("dynamic_obstacles=1", logger.info_messages[0])

    def test_dynamic_block_returns_none_when_static_route_exists(self):
        planner = _Planner(
            [
                PathPlanningError("blocked"),
                [[1.0, 0.0, 0.0]],
            ]
        )
        logger = _Logger()
        provider = VoxelRouteProvider(
            planner=planner,
            walk_plane_z=0.0,
            dynamic_obstacles=lambda now: [(0.5, 0.0, 0.4)],
            logger=logger,
            clock=lambda: 3.0,
        )

        points = provider.plan(
            RouteRequest("toilet_agent_01", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        )

        self.assertIsNone(points)
        self.assertEqual(len(planner.calls), 2)
        self.assertNotIn("dynamic_obstacles", planner.calls[1][2])
        self.assertIn("temporarily blocked by the robot", logger.warning_messages[0])

    def test_static_planning_failure_remains_fail_closed(self):
        planner = _Planner([PathPlanningError("no route")])
        logger = _Logger()
        provider = VoxelRouteProvider(
            planner=planner,
            walk_plane_z=0.0,
            dynamic_obstacles=lambda now: [],
            logger=logger,
            clock=lambda: 1.0,
        )

        points = provider.plan(
            RouteRequest("toilet_agent_01", [0.0, 0.0, 0.0], [1.0, 0.0, 0.0])
        )

        self.assertIsNone(points)
        self.assertIn("no route", logger.error_messages[0])

    def test_walkable_provider_passes_dynamic_robot_obstacles(self):
        planner = _Planner([[[1.0, 2.0, 0.0]]])
        planner.scene_fingerprint = "fingerprint"
        logger = _Logger()
        provider = WalkableMapRouteProvider(
            planner=planner,
            walk_plane_z=0.0,
            dynamic_obstacles=lambda now: [(3.0, 4.0, 0.4)],
            logger=logger,
            clock=lambda: 10.0,
        )

        points = provider.plan(
            RouteRequest("toilet_agent_01", [0.0, 0.0, 0.0], [1.0, 2.0, 0.0])
        )

        self.assertEqual(points, [[1.0, 2.0, 0.0]])
        self.assertEqual(
            planner.calls[0][2]["dynamic_obstacles"],
            [(3.0, 4.0, 0.4)],
        )


if __name__ == "__main__":
    unittest.main()
