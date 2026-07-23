import math
import unittest

from toilet_benchmark.semantic_rules import (
    PlacementRule,
    QueueRule,
    build_queue_poses,
    classify_motion_observation,
    portal_inside_plane_reached,
    remaining_polyline_waypoints,
    resolve_local_placement,
)


class TestSemanticRules(unittest.TestCase):
    def test_portal_entry_accepts_root_at_indoor_plane_before_short_turn(self):
        self.assertTrue(
            portal_inside_plane_reached(
                current_pose=[-2.24, -0.90, 0.0],
                inside_pose=[-2.24, -0.90, 0.0],
                staging_pose=[-2.24, -0.675, 0.0],
                lateral_tolerance_m=0.10,
            )
        )

    def test_portal_entry_rejects_pose_beside_corridor(self):
        self.assertFalse(
            portal_inside_plane_reached(
                current_pose=[-2.00, -0.90, 0.0],
                inside_pose=[-2.24, -0.90, 0.0],
                staging_pose=[-2.24, -0.675, 0.0],
                lateral_tolerance_m=0.10,
            )
        )

    def test_lateral_root_motion_is_not_misclassified_as_stationary(self):
        observation = classify_motion_observation(
            previous_pose=[-2.24, -0.90, 0.0],
            current_pose=[-2.24, -0.86, 0.0],
            target_pose=[-2.24, -0.675, 0.0],
            best_distance=0.225,
            displacement_epsilon_m=0.02,
            progress_epsilon_m=0.03,
        )

        self.assertTrue(observation.moved)
        self.assertTrue(observation.progressed)

    def test_motion_without_target_progress_keeps_activity_but_not_progress(self):
        observation = classify_motion_observation(
            previous_pose=[0.0, 0.0, 0.0],
            current_pose=[0.0, 0.04, 0.0],
            target_pose=[1.0, 0.0, 0.0],
            best_distance=1.0,
            displacement_epsilon_m=0.02,
            progress_epsilon_m=0.03,
        )

        self.assertTrue(observation.moved)
        self.assertFalse(observation.progressed)

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

    def test_remaining_path_rejoins_near_current_pose_before_suffix(self):
        points = [
            [-1.675, -0.225, 0.0],
            [0.075, 0.675, 0.0],
            [2.075, 0.675, 0.0],
            [2.875, 0.875, 0.0],
            [2.862, 1.032, 0.0],
        ]

        remaining = remaining_polyline_waypoints(points, [1.54, 0.65, 0.0])

        self.assertEqual(remaining[0], [2.075, 0.675, 0.0])
        self.assertNotEqual(remaining[0], [2.875, 0.875, 0.0])


if __name__ == "__main__":
    unittest.main()
