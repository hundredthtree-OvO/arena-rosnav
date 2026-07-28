import unittest

from toilet_benchmark.hunav_phase0_coordination import (
    BottleneckSpec,
    DeterministicBottleneckCoordinator,
)


class TestHuNavPhase0Coordination(unittest.TestCase):
    def setUp(self):
        self.coordinator = DeterministicBottleneckCoordinator(
            routes={
                1: ((-2.4, 0.0), (1.7, 0.0)),
                2: ((2.4, 0.0), (-1.7, 0.0)),
            },
            spec=BottleneckSpec(
                zone_x_min=-1.0,
                zone_x_max=1.0,
                release_clearance_m=0.15,
                priority_order=(1, 2),
            ),
        )

    def test_second_agent_is_held_while_first_owns_zone(self):
        self.coordinator.update({1: (-2.4, 0.0), 2: (2.4, 0.0)})

        self.assertEqual(self.coordinator.active_agent_id, 1)
        self.assertEqual(self.coordinator.held_agent_ids(), (2,))

    def test_owner_is_not_released_before_entering(self):
        self.coordinator.update({1: (-2.0, 0.0), 2: (2.4, 0.0)})

        self.assertEqual(self.coordinator.active_agent_id, 1)

    def test_second_agent_is_released_after_first_clears_zone(self):
        self.coordinator.update({1: (-0.9, 0.0), 2: (2.4, 0.0)})
        self.coordinator.update({1: (1.2, 0.0), 2: (2.4, 0.0)})

        self.assertEqual(self.coordinator.active_agent_id, 2)
        self.assertEqual(self.coordinator.held_agent_ids(), ())
        self.assertEqual(self.coordinator.status()["completed_agent_ids"], [1])

    def test_priority_must_cover_all_routes(self):
        with self.assertRaises(ValueError):
            DeterministicBottleneckCoordinator(
                routes={
                    1: ((-1.0, 0.0), (1.0, 0.0)),
                    2: ((1.0, 0.0), (-1.0, 0.0)),
                },
                spec=BottleneckSpec(-0.5, 0.5, priority_order=(1,)),
            )


if __name__ == "__main__":
    unittest.main()
