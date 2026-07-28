import unittest

from toilet_benchmark.robot_reaction import RobotProximityReactionController


def _agent():
    return {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.4, "vy": 0.0}


def _robot(x):
    return {"x": x, "y": 0.0, "vx": 0.0, "vy": 0.0}


class TestRobotProximityReactionController(unittest.TestCase):
    def test_fixed_yielding_latches_until_release_is_stable(self):
        controller = RobotProximityReactionController(
            {
                "mode": "fixed",
                "reaction": "yielding",
                "trigger": {"distance_m": 0.85},
                "release": {"distance_m": 1.10, "stable_sec": 0.8},
            },
            seed=42,
        )

        entered = controller.update(agent=_agent(), robot=_robot(0.8), now=1.0)
        waiting = controller.update(agent=_agent(), robot=_robot(1.2), now=1.4)
        resumed = controller.update(agent=_agent(), robot=_robot(1.2), now=2.3)

        self.assertEqual(entered.state, "YIELDING_TO_ROBOT")
        self.assertEqual(entered.speed_scale, 0.0)
        self.assertEqual(waiting.state, "YIELDING_TO_ROBOT")
        self.assertEqual(resumed.state, "WALKING")

    def test_reaction_does_not_resample_inside_one_encounter(self):
        controller = RobotProximityReactionController(
            {
                "mode": "weighted",
                "trigger": {"distance_m": 1.0},
                "reactions": {
                    "yielding": {"weight": 1.0},
                    "impatient": {"weight": 1.0},
                },
            },
            seed=7,
        )

        first = controller.update(agent=_agent(), robot=_robot(0.7), now=1.0)
        second = controller.update(agent=_agent(), robot=_robot(0.6), now=2.0)

        self.assertEqual(first.reaction, second.reaction)

    def test_ttc_can_trigger_before_distance_threshold(self):
        controller = RobotProximityReactionController(
            {
                "mode": "fixed",
                "reaction": "yielding",
                "trigger": {
                    "distance_m": 0.85,
                    "time_to_collision_sec": 1.5,
                    "front_half_angle_deg": 90.0,
                },
            },
            seed=42,
        )
        agent = {"x": 0.0, "y": 0.0, "yaw": 0.0, "vx": 0.0, "vy": 0.0}
        approaching_robot = {"x": 1.2, "y": 0.0, "vx": -1.0, "vy": 0.0}

        decision = controller.update(
            agent=agent,
            robot=approaching_robot,
            now=1.0,
        )

        self.assertEqual(decision.state, "YIELDING_TO_ROBOT")
        self.assertLess(decision.ttc_sec, 1.5)

    def test_weighted_selection_is_reproducible(self):
        config = {
            "mode": "weighted",
            "trigger": {"distance_m": 1.0},
            "reactions": {
                "yielding": {"weight": 0.6},
                "impatient": {"weight": 0.3},
                "regular": {"weight": 0.1},
            },
        }
        first = RobotProximityReactionController(config, seed=123)
        second = RobotProximityReactionController(config, seed=123)

        self.assertEqual(
            first.update(agent=_agent(), robot=_robot(0.5), now=1.0).reaction,
            second.update(agent=_agent(), robot=_robot(0.5), now=1.0).reaction,
        )


if __name__ == "__main__":
    unittest.main()
