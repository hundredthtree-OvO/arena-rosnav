import math
import unittest

from toilet_benchmark.motion.swept_envelope import (
    SweptEnvelope,
    SweptEnvelopeConfig,
)


class _VerticalWallPlanner:
    def __init__(self, wall_x=1.0):
        self.wall_x = float(wall_x)

    def clearance_at(self, point):
        return abs(float(point[0]) - self.wall_x)


class TestSweptEnvelope(unittest.TestCase):
    def setUp(self):
        self.envelope = SweptEnvelope(
            SweptEnvelopeConfig(
                disc_radius_m=0.20,
                half_length_m=0.10,
                clearance_m=0.02,
                axis_sample_spacing_m=0.05,
                sweep_sample_spacing_m=0.01,
                angular_sample_step_rad=math.radians(5.0),
            )
        )
        self.planner = _VerticalWallPlanner()

    def test_capsule_rejects_pose_whose_root_disc_would_fit(self):
        self.assertFalse(
            self.envelope.pose_is_free(
                self.planner,
                (0.75, 0.0),
                0.0,
            )
        )
        self.assertTrue(
            self.envelope.pose_is_free(
                self.planner,
                (0.65, 0.0),
                0.0,
            )
        )

    def test_continuous_sweep_clips_before_first_contact(self):
        result = self.envelope.clip_step(
            self.planner,
            (0.50, 0.0),
            (0.90, 0.0),
            0.0,
            0.0,
        )

        self.assertTrue(result.clipped)
        self.assertGreater(result.fraction, 0.0)
        self.assertLess(result.fraction, 1.0)
        self.assertLessEqual(result.position[0], 0.68 + 1e-3)
        self.assertTrue(
            self.envelope.pose_is_free(
                self.planner,
                result.position,
                0.0,
            )
        )

    def test_turning_sweep_catches_body_axis_collision(self):
        result = self.envelope.clip_step(
            self.planner,
            (0.70, 0.0),
            (0.70, 0.0),
            math.pi / 2.0,
            0.0,
        )

        self.assertTrue(result.clipped)
        self.assertLess(result.fraction, 1.0)

    def test_existing_overlap_may_only_move_toward_greater_clearance(self):
        result = self.envelope.clip_step(
            self.planner,
            (0.75, 0.0),
            (0.50, 0.0),
            0.0,
            0.0,
        )

        self.assertTrue(result.recovering_overlap)
        self.assertFalse(result.clipped)
        self.assertEqual(result.position, (0.50, 0.0))

    def test_existing_overlap_cannot_move_deeper(self):
        result = self.envelope.clip_step(
            self.planner,
            (0.75, 0.0),
            (0.85, 0.0),
            0.0,
            0.0,
        )

        self.assertFalse(result.recovering_overlap)
        self.assertTrue(result.clipped)
        self.assertEqual(result.fraction, 0.0)
        self.assertEqual(result.position, (0.75, 0.0))

    def test_polyline_reports_negative_clearance_when_corner_sweep_hits(self):
        clearance = self.envelope.polyline_minimum_clearance(
            self.planner,
            ((0.50, 0.0), (0.70, 0.0), (0.70, 0.30)),
            start_yaw=0.0,
        )

        self.assertLess(clearance, 0.0)
