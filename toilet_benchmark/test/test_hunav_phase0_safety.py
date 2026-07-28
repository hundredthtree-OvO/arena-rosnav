import math
import unittest

from toilet_benchmark.hunav_phase0_safety import (
    SafetyConfig,
    project_safe_step,
)


CONFIG = SafetyConfig(
    clearance_m=0.01,
    contact_epsilon_m=1e-4,
    static_obstacle_radius_m=0.03,
    max_iterations=8,
)


class TestHuNavPhase0Safety(unittest.TestCase):
    def test_clear_motion_is_unchanged(self):
        result = project_safe_step(
            previous={1: (-1.0, 0.0), 2: (1.0, 1.0)},
            proposed={1: (-0.9, 0.0), 2: (0.9, 1.0)},
            radii={1: 0.3, 2: 0.3},
            robot_previous=(100.0, 100.0),
            robot_proposed=(100.0, 100.0),
            robot_radius=0.45,
            static_obstacles={},
            config=CONFIG,
        )

        self.assertEqual(result.positions[1], (-0.9, 0.0))
        self.assertEqual(result.positions[2], (0.9, 1.0))
        self.assertFalse(result.diagnostics.intervened)
        self.assertEqual(result.diagnostics.residual_violation_count, 0)

    def test_swept_pair_collision_cannot_tunnel(self):
        result = project_safe_step(
            previous={1: (-1.0, 0.0), 2: (1.0, 0.0)},
            proposed={1: (1.0, 0.0), 2: (-1.0, 0.0)},
            radii={1: 0.3, 2: 0.3},
            robot_previous=(100.0, 100.0),
            robot_proposed=(100.0, 100.0),
            robot_radius=0.45,
            static_obstacles={},
            config=CONFIG,
        )

        distance = math.dist(result.positions[1], result.positions[2])
        self.assertGreaterEqual(distance, 0.61009)
        self.assertEqual(result.diagnostics.pedestrian_pair_contacts, 1)
        self.assertEqual(result.diagnostics.residual_violation_count, 0)

    def test_robot_is_immovable_and_tangent_motion_is_retained(self):
        result = project_safe_step(
            previous={1: (-1.0, 0.4)},
            proposed={1: (1.0, 0.4)},
            radii={1: 0.3},
            robot_previous=(0.0, 0.0),
            robot_proposed=(0.0, 0.0),
            robot_radius=0.45,
            static_obstacles={},
            config=CONFIG,
        )

        self.assertGreaterEqual(math.dist(result.positions[1], (0.0, 0.0)), 0.76009)
        self.assertNotEqual(result.positions[1], (-1.0, 0.4))
        self.assertEqual(result.diagnostics.robot_contacts, 1)
        self.assertEqual(result.diagnostics.residual_violation_count, 0)

    def test_static_obstacle_blocks_swept_motion(self):
        result = project_safe_step(
            previous={1: (-1.0, 0.0)},
            proposed={1: (1.0, 0.0)},
            radii={1: 0.3},
            robot_previous=(100.0, 100.0),
            robot_proposed=(100.0, 100.0),
            robot_radius=0.45,
            static_obstacles={1: ((0.0, 0.0),)},
            config=CONFIG,
        )

        self.assertGreaterEqual(math.dist(result.positions[1], (0.0, 0.0)), 0.34009)
        self.assertEqual(result.diagnostics.static_contacts, 1)

    def test_fixed_agent_does_not_move_when_pair_is_constrained(self):
        result = project_safe_step(
            previous={1: (0.0, 0.0), 2: (1.0, 0.0)},
            proposed={1: (0.0, 0.0), 2: (0.2, 0.0)},
            radii={1: 0.3, 2: 0.3},
            robot_previous=(100.0, 100.0),
            robot_proposed=(100.0, 100.0),
            robot_radius=0.45,
            static_obstacles={},
            fixed_ids={1},
            config=CONFIG,
        )

        self.assertEqual(result.positions[1], (0.0, 0.0))
        self.assertGreaterEqual(math.dist(result.positions[1], result.positions[2]), 0.61009)

    def test_initial_overlap_is_projected_deterministically(self):
        kwargs = dict(
            previous={1: (0.0, 0.0), 2: (0.1, 0.0)},
            proposed={1: (-0.1, 0.0), 2: (0.2, 0.0)},
            radii={1: 0.3, 2: 0.3},
            robot_previous=(100.0, 100.0),
            robot_proposed=(100.0, 100.0),
            robot_radius=0.45,
            static_obstacles={},
            config=CONFIG,
        )

        first = project_safe_step(**kwargs)
        second = project_safe_step(**kwargs)

        self.assertEqual(first.positions, second.positions)
        self.assertGreaterEqual(math.dist(first.positions[1], first.positions[2]), 0.61009)


if __name__ == "__main__":
    unittest.main()
