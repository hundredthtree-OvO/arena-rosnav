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
    def test_raw_visibility_is_exposed_for_local_subgoal_selection(self):
        payload = _payload(width=4, height=4)
        payload["data"] = [0] * 16
        payload["data"][1 * 4 + 1] = 100
        planner = WalkableMapPlanner(
            payload,
            WalkableMapPlannerConfig(
                map_path="unused.json",
                agent_radius_m=1.1,
            ),
        )
        segment = [[0.25, 0.25, 0.0], [1.25, 0.25, 0.0]]

        self.assertFalse(planner.polyline_is_free(segment))
        self.assertTrue(planner.polyline_avoids_raw_obstacles(segment))

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

        plan = provider.plan(
            RouteRequest("agent", [0.25, 0.25, 0.0], [3.75, 0.25, 0.0])
        )

        self.assertIsNotNone(plan)
        self.assertEqual(plan.map_version, "abc123")
        self.assertIn("fingerprint=abc123", logger.info_messages[0])

    def test_clip_step_stops_before_crossing_static_wall(self):
        planner = self._planner()

        safe, fraction, clipped = planner.clip_step(
            [1.0, 0.25],
            [2.5, 0.25],
        )

        self.assertTrue(clipped)
        self.assertLess(fraction, 1.0)
        self.assertLess(safe[0], 1.5)

    def test_project_step_slides_along_wall_toward_route_target(self):
        payload = {
            "schema": "arena.walkable_map.v1",
            "scene_fingerprint": "slide",
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "width": 30,
            "height": 30,
            "data": [
                100 if x == 15 else 0
                for y in range(30)
                for x in range(30)
            ],
        }
        planner = self._planner(payload)

        safe, fraction, clipped, mode = planner.project_step(
            [1.35, 1.0],
            [1.55, 1.2],
            target=[1.35, 2.0],
        )

        self.assertTrue(clipped)
        self.assertEqual(mode, "tangent")
        self.assertLess(safe[0], 1.5)
        self.assertGreater(safe[1], 1.0)
        self.assertGreater(fraction, 0.5)

    def test_clip_step_allows_escape_from_inflated_margin(self):
        payload = {
            "schema": "arena.walkable_map.v1",
            "scene_fingerprint": "escape",
            "resolution": 0.1,
            "origin": [0.0, 0.0, 0.0],
            "width": 20,
            "height": 20,
            "data": [
                100 if x == 5 else 0
                for y in range(20)
                for x in range(20)
            ],
        }
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        path = Path(directory.name) / "map.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        planner = WalkableMapPlanner.from_file(
            WalkableMapPlannerConfig(
                map_path=str(path),
                agent_radius_m=0.2,
                preferred_clearance_m=0.3,
            )
        )

        safe, fraction, clipped = planner.clip_step([0.75, 1.0], [1.1, 1.0])

        self.assertFalse(clipped)
        self.assertEqual(fraction, 1.0)
        self.assertEqual(safe, (1.1, 1.0))

    def test_walkable_config_keeps_distinct_constrained_segment_length(self):
        planner = WalkableMapPlanner(
            _payload(),
            WalkableMapPlannerConfig(
                map_path="/tmp/test-map.json",
                agent_radius_m=0.0,
                max_segment_length_m=1.5,
                constrained_segment_length_m=0.35,
                preferred_clearance_m=0.5,
            ),
        )

        self.assertAlmostEqual(
            planner._planner.config.constrained_segment_length_m,
            0.35,
        )

    def test_walkable_planner_enables_any_angle_search(self):
        planner = self._planner()

        self.assertTrue(planner._planner.config.any_angle)

    def test_walkable_plan_accepts_dynamic_robot_obstacles(self):
        payload = {
            "schema": "arena.walkable_map.v1",
            "scene_fingerprint": "dynamic",
            "resolution": 1.0,
            "origin": [0.0, 0.0, 0.0],
            "width": 9,
            "height": 5,
            "data": [0] * 45,
        }
        planner = self._planner(payload)

        points = planner.plan(
            [0.5, 2.5, 0.0],
            [8.5, 2.5, 0.0],
            z=0.0,
            dynamic_obstacles=[(4.5, 2.5, 0.5)],
        )

        self.assertTrue(any(abs(point[1] - 2.5) > 0.1 for point in points[:-1]))

if __name__ == "__main__":
    unittest.main()
