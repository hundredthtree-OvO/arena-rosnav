import json
from pathlib import Path
import tempfile
import unittest

from toilet_benchmark.route_provider import RouteRequest, WalkableMapRouteProvider
from toilet_benchmark.walkable_map_planner import (
    WalkableMapPlanner,
    WalkableMapPlannerConfig,
)


class _Logger:
    def __init__(self):
        self.info_messages = []
        self.error_messages = []

    def info(self, message):
        self.info_messages.append(message)

    def error(self, message):
        self.error_messages.append(message)


def _payload(width=8, height=6):
    data = [0] * (width * height)
    for y in range(height):
        if y != 3:
            data[y * width + 3] = 100
    return {
        "schema": "arena.walkable_map.v1",
        "scene_fingerprint": "abc123",
        "resolution": 0.5,
        "origin": [0.0, 0.0, 0.0],
        "width": width,
        "height": height,
        "data": data,
    }


class TestWalkableMapPlanner(unittest.TestCase):
    def _planner(self, payload=None):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "map.json"
        path.write_text(json.dumps(payload or _payload()), encoding="utf-8")
        return WalkableMapPlanner.from_file(
            WalkableMapPlannerConfig(
                map_path=str(path),
                agent_radius_m=0.0,
                preferred_clearance_m=0.0,
            )
        )

    def test_route_passes_through_static_wall_opening(self):
        planner = self._planner()
        points = planner.plan([0.25, 0.25, 0.0], [3.75, 0.25, 0.0], z=0.0)

        self.assertGreaterEqual(len(points), 2)
        self.assertAlmostEqual(points[-1][0], 3.75)
        self.assertAlmostEqual(points[-1][1], 0.25)
        self.assertTrue(any(point[1] >= 1.5 for point in points))

    def test_unknown_cells_are_fail_closed(self):
        payload = _payload()
        payload["data"] = [-1] * len(payload["data"])
        planner = self._planner(payload)

        with self.assertRaisesRegex(RuntimeError, "no nearby free"):
            planner.plan([0.25, 0.25, 0.0], [3.75, 0.25, 0.0], z=0.0)

    def test_provider_logs_scene_fingerprint(self):
        planner = self._planner()
        logger = _Logger()
        provider = WalkableMapRouteProvider(
            planner=planner,
            walk_plane_z=0.0,
            logger=logger,
        )

        points = provider.plan(
            RouteRequest("agent", [0.25, 0.25, 0.0], [3.75, 0.25, 0.0])
        )

        self.assertIsNotNone(points)
        self.assertIn("fingerprint=abc123", logger.info_messages[0])


if __name__ == "__main__":
    unittest.main()
