import unittest

from toilet_benchmark.interaction_policy import (
    dynamic_robot_obstacles,
    guard_block_requires_wait,
    robot_blocks_pedestrian,
)


class TestInteractionPolicy(unittest.TestCase):
    def test_external_actor_guard_blocks_pause_recovery(self):
        self.assertTrue(guard_block_requires_wait("robot"))
        self.assertTrue(guard_block_requires_wait("pedestrian"))
        self.assertFalse(guard_block_requires_wait("static_voxel"))
        self.assertFalse(guard_block_requires_wait(""))

    def test_static_route_never_adds_robot_to_global_plan(self):
        state = (1.0, 2.0, 0.5, 0.0, 9.5)

        obstacles = dynamic_robot_obstacles(
            mode="static_route",
            robot_state=state,
            now=10.0,
            timeout_sec=1.0,
            radius_m=0.4,
            prediction_horizon_sec=1.0,
        )

        self.assertEqual(obstacles, [])

    def test_dynamic_replan_keeps_current_and_predicted_robot_obstacles(self):
        obstacles = dynamic_robot_obstacles(
            mode="dynamic_replan",
            robot_state=(1.0, 2.0, 0.5, 0.0, 9.5),
            now=10.0,
            timeout_sec=1.0,
            radius_m=0.4,
            prediction_horizon_sec=1.0,
        )

        self.assertEqual(obstacles, [(1.0, 2.0, 0.4), (1.5, 2.0, 0.4)])

    def test_oriented_stationary_robot_uses_tight_current_footprint(self):
        obstacles = dynamic_robot_obstacles(
            mode="dynamic_replan",
            robot_state=(1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 9.5),
            now=10.0,
            timeout_sec=1.0,
            radius_m=0.4,
            prediction_horizon_sec=1.0,
            footprint_half_length_m=0.36,
            footprint_half_width_m=0.27,
        )

        self.assertEqual(len(obstacles), 3)
        self.assertAlmostEqual(obstacles[0][0], 0.91)
        self.assertAlmostEqual(obstacles[-1][0], 1.09)
        self.assertTrue(all(obstacle[3] == 0.05 for obstacle in obstacles))

    def test_oriented_moving_robot_adds_predicted_footprint(self):
        obstacles = dynamic_robot_obstacles(
            mode="dynamic_replan",
            robot_state=(1.0, 2.0, 0.0, 0.5, 0.0, 0.2, 9.5),
            now=10.0,
            timeout_sec=1.0,
            radius_m=0.4,
            prediction_horizon_sec=1.0,
            footprint_half_length_m=0.36,
            footprint_half_width_m=0.27,
        )

        self.assertEqual(len(obstacles), 6)
        self.assertLess(max(item[0] for item in obstacles[:3]), 1.2)
        self.assertGreater(min(item[0] for item in obstacles[3:]), 1.3)
        self.assertTrue(all(obstacle[3] == 0.12 for obstacle in obstacles))

    def test_nearby_fresh_robot_blocks_pedestrian_recovery(self):
        self.assertTrue(
            robot_blocks_pedestrian(
                pedestrian_pose=[0.0, 0.0, 0.0],
                robot_state=(0.7, 0.0, 0.0, 0.0, 9.5),
                now=10.0,
                timeout_sec=1.0,
                robot_radius_m=0.4,
                pedestrian_radius_m=0.22,
                clearance_m=0.1,
                prediction_horizon_sec=1.0,
            )
        )

    def test_stale_robot_does_not_block_pedestrian_recovery(self):
        self.assertFalse(
            robot_blocks_pedestrian(
                pedestrian_pose=[0.0, 0.0, 0.0],
                robot_state=(0.3, 0.0, 0.0, 0.0, 8.0),
                now=10.0,
                timeout_sec=1.0,
                robot_radius_m=0.4,
                pedestrian_radius_m=0.22,
                clearance_m=0.1,
                prediction_horizon_sec=1.0,
            )
        )


if __name__ == "__main__":
    unittest.main()
