import math
import unittest

from toilet_benchmark.semantic_rules import (
    PlacementRule,
    QueueRule,
    build_queue_poses,
    remaining_polyline_waypoints,
    resolve_local_placement,
)


class TestSemanticRules(unittest.TestCase):
    def test_resolve_local_placement_rotates_offset_with_anchor_yaw(self):
        pose = resolve_local_placement(
            anchor_position=(1.0, 2.0, 0.0),
            anchor_yaw=math.pi / 2.0,
            placement_rule=PlacementRule(offset_local=(0.0, -0.5, 0.0), yaw_mode="face_anchor"),
        )
        self.assertAlmostEqual(pose.position[0], 1.5, places=6)
        self.assertAlmostEqual(pose.position[1], 2.0, places=6)
        self.assertAlmostEqual(pose.yaw, math.pi, places=6)

    def test_build_queue_poses_extends_along_configured_local_axis(self):
        resource_pose = resolve_local_placement(
            anchor_position=(1.0, 2.0, 0.0),
            anchor_yaw=math.pi / 2.0,
            placement_rule=PlacementRule(offset_local=(0.0, -0.5, 0.0), yaw_mode="face_anchor"),
        )
        queue = build_queue_poses(
            anchor_position=(1.0, 2.0, 0.0),
            anchor_yaw=math.pi / 2.0,
            resource_pose=resource_pose,
            queue_rule=QueueRule(
                slots=2,
                first_gap=0.4,
                spacing=0.4,
                axis_local=(0.0, -1.0, 0.0),
                yaw_mode="same_as_resource",
            ),
        )
        self.assertEqual(len(queue), 2)
        self.assertAlmostEqual(queue[0].position[0], 1.9, places=6)
        self.assertAlmostEqual(queue[1].position[0], 2.3, places=6)
        self.assertAlmostEqual(queue[0].yaw, resource_pose.yaw, places=6)

    def test_remaining_polyline_rejoins_then_moves_forward(self):
        points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]

        remaining = remaining_polyline_waypoints(points, [0.4, 0.2, 0.0])

        self.assertEqual(remaining[0], [0.4, 0.0, 0.0])
        self.assertEqual(remaining[1:], [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]])

    def test_remaining_polyline_does_not_backtrack_after_corner(self):
        points = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]

        remaining = remaining_polyline_waypoints(points, [1.0, 0.6, 0.0])

        self.assertEqual(remaining, [[1.0, 1.0, 0.0]])

    def test_remaining_portal_path_keeps_crossing_after_duplicate_inside_point(self):
        portal_path = [
            [-2.24, -0.9, 0.0],
            [-2.24, -0.9, 0.0],
            [-3.0, -0.9, 0.0],
            [-3.8, -0.91, 0.0],
        ]

        remaining = remaining_polyline_waypoints(portal_path, [-1.8, -0.9, 0.0])

        self.assertEqual(remaining[0], [-2.24, -0.9, 0.0])
        self.assertEqual(remaining[-1], [-3.8, -0.91, 0.0])
        self.assertGreater(len(remaining), 1)


if __name__ == "__main__":
    unittest.main()
