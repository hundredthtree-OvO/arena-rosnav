import math
import unittest

from toilet_policy.alignment import nearest_index, previous_index, relative_goal


class AlignmentTest(unittest.TestCase):
    def test_timestamp_selection_distinguishes_nearest_and_previous(self):
        timestamps = [1.0, 2.0, 4.0]
        self.assertEqual(nearest_index(timestamps, 3.2), 2)
        self.assertEqual(previous_index(timestamps, 3.2), 1)

    def test_goal_is_rotated_into_robot_frame(self):
        x, y, sin_heading, cos_heading = relative_goal(
            1.0, 2.0, math.pi / 2.0, 1.0, 3.0, math.pi
        )
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(sin_heading, 1.0)
        self.assertAlmostEqual(cos_heading, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()
