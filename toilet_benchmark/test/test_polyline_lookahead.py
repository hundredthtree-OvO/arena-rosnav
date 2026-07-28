import unittest

from toilet_benchmark.polyline_lookahead import PolylineLookaheadTracker


class TestPolylineLookaheadTracker(unittest.TestCase):
    def test_target_is_one_lookahead_point_on_polyline(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]],
            lookahead_m=0.75,
        )

        target = tracker.update(0.0, 0.0)

        self.assertAlmostEqual(target.x, 0.75)
        self.assertAlmostEqual(target.y, 0.0)
        self.assertFalse(target.is_final)

    def test_progress_does_not_regress_under_lateral_pose_noise(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [3.0, 0.0]],
            lookahead_m=0.80,
        )
        progress = []
        for pose in [(0.4, 0.04), (0.9, -0.03), (0.85, 0.06), (1.4, -0.02)]:
            progress.append(tracker.update(*pose).progress_m)

        self.assertEqual(progress, sorted(progress))
        self.assertAlmostEqual(progress[-1], 1.4)

    def test_terminal_window_targets_exact_route_endpoint(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
            lookahead_m=0.80,
        )

        target = tracker.update(1.0, 0.3)

        self.assertTrue(target.is_final)
        self.assertAlmostEqual(target.x, 1.0)
        self.assertAlmostEqual(target.y, 1.0)

    def test_centimeter_tail_is_deduplicated_without_failure(self):
        tracker = PolylineLookaheadTracker(
            [[-3.825, -0.925], [-3.8, -0.91]],
            lookahead_m=0.80,
        )

        target = tracker.update(-3.825, -0.925)

        self.assertTrue(target.is_final)
        self.assertAlmostEqual(target.x, -3.8)
        self.assertAlmostEqual(target.y, -0.91)


if __name__ == "__main__":
    unittest.main()
