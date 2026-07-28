import math
import unittest

from hunav_msgs.msg import Agent

from toilet_benchmark.hunav_isaac_mirror import HuNavIsaacMirror
from toilet_benchmark.hunav_phase0_safety import SafetyConfig


def _agent(x, y):
    agent = Agent()
    agent.id = 1
    agent.position.position.x = float(x)
    agent.position.position.y = float(y)
    agent.position.orientation.w = 1.0
    return agent


class TestHuNavIsaacMirrorSafety(unittest.TestCase):
    def setUp(self):
        self.mirror = object.__new__(HuNavIsaacMirror)
        self.mirror.safety_enabled = True
        self.mirror.safety_config = SafetyConfig(clearance_m=0.01)
        self.mirror.agent_radius = 0.2
        self.mirror.robot_radius = 0.2
        self.mirror._safety_robot_previous = {
            "available": True,
            "x": 0.5,
            "y": 0.0,
        }

    def test_raw_step_crossing_robot_is_projected_to_safe_clearance(self):
        previous = _agent(0.0, 0.0)
        proposed = _agent(1.0, 0.0)
        robot = {
            "available": True,
            "x": 0.5,
            "y": 0.0,
        }

        diagnostics = self.mirror._apply_hard_safety(
            proposed,
            previous_shadow=previous,
            robot=robot,
            dt=0.1,
        )

        distance = math.hypot(
            proposed.position.position.x - robot["x"],
            proposed.position.position.y - robot["y"],
        )
        self.assertTrue(diagnostics["intervened"])
        self.assertGreaterEqual(distance, 0.41 - 1e-6)


if __name__ == "__main__":
    unittest.main()
