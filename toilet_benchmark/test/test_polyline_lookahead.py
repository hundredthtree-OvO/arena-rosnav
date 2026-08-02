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

    def test_nearby_future_segment_cannot_jump_route_progress(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [0.5, 0.0], [0.5, 0.4], [0.0, 0.4]],
            lookahead_m=0.5,
            progress_slack_m=0.10,
        )
        tracker.update(0.0, 0.0)

        target = tracker.update(0.05, 0.35)

        self.assertLessEqual(target.progress_m, 0.46)
        self.assertTrue(target.progress_limited)

    def test_large_cross_track_error_holds_progress(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [3.0, 0.0]],
            lookahead_m=0.5,
            max_cross_track_m=0.4,
        )
        tracker.update(0.0, 0.0)

        target = tracker.update(1.0, 1.0)

        self.assertEqual(target.progress_m, 0.0)
        self.assertGreater(target.cross_track_error_m, 0.4)

    def test_small_existing_progress_keeps_current_segment_when_pose_moves_back(self):
        tracker = PolylineLookaheadTracker(
            [[-3.8, -0.91], [-2.325, -0.825], [-0.825, -0.825]],
            lookahead_m=0.8,
        )
        tracker.update(-3.7998, -0.9098)

        target = tracker.update(-4.035, -0.968)

        self.assertLess(target.cross_track_error_m, 0.30)
        self.assertLess(target.progress_m, 0.01)

    def test_target_at_progress_keeps_tracker_progress_monotonic(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]],
            lookahead_m=0.8,
        )
        nominal = tracker.update(0.6, 0.0)

        visible = tracker.target_at_progress(
            0.9,
            cross_track_error_m=nominal.cross_track_error_m,
            progress_limited=nominal.progress_limited,
        )

        self.assertAlmostEqual(visible.progress_m, 0.6)
        self.assertAlmostEqual(visible.target_progress_m, 0.9)
        self.assertAlmostEqual(visible.x, 0.9)

    def test_project_to_route_does_not_advance_progress(self):
        tracker = PolylineLookaheadTracker(
            [[0.0, 0.0], [2.0, 0.0]],
            lookahead_m=0.8,
        )
        tracker.update(0.25, 0.0)

        projected_x, projected_y, error = tracker.project_to_route(0.8, 0.5)

        self.assertAlmostEqual(projected_x, 0.8)
        self.assertAlmostEqual(projected_y, 0.0)
        self.assertAlmostEqual(error, 0.5)
        self.assertAlmostEqual(tracker.progress_m, 0.25)


if __name__ == "__main__":
    unittest.main()
