import unittest

from toilet_benchmark.pose_utils import parse_semantic_pose


class TestPoseUtils(unittest.TestCase):
    def test_three_value_semantic_pose_uses_yaw_not_z(self):
        pose = parse_semantic_pose([2.88, 1.07, 1.5708])
        self.assertEqual(pose.as_list(), [2.88, 1.07, 0.0])
        self.assertAlmostEqual(pose.yaw, 1.5708)

    def test_four_value_semantic_pose_preserves_explicit_z_and_yaw(self):
        pose = parse_semantic_pose([1.0, 2.0, 0.25, -1.2])
        self.assertEqual(pose.as_list(), [1.0, 2.0, 0.25])
        self.assertAlmostEqual(pose.yaw, -1.2)

    def test_two_value_semantic_pose_falls_back_to_defaults(self):
        pose = parse_semantic_pose([1.5, -0.5], default_yaw=0.75)
        self.assertEqual(pose.as_list(), [1.5, -0.5, 0.0])
        self.assertAlmostEqual(pose.yaw, 0.75)

    def test_with_z_replaces_only_height(self):
        pose = parse_semantic_pose([1.0, 2.0, 0.25, -1.2]).with_z(0.0)
        self.assertEqual(pose.as_list(), [1.0, 2.0, 0.0])
        self.assertAlmostEqual(pose.yaw, -1.2)


if __name__ == "__main__":
    unittest.main()
