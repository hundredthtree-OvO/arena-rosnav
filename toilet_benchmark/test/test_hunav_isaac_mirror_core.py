import math
import unittest

from toilet_benchmark.hunav_isaac_mirror_core import (
    build_motion_intent_payload,
    parse_motion_intent,
    person_yaw,
    point_to_polyline_distance,
    summarize_samples,
    velocity_heading,
)


class _Person:
    def __init__(self, tagnames=(), tags=()):
        self.tagnames = list(tagnames)
        self.tags = list(tags)


class TestHuNavIsaacMirrorCore(unittest.TestCase):
    def test_motion_intent_round_trip_preserves_shadow_route(self):
        payload = build_motion_intent_payload(
            agent_id="toilet_agent_01",
            generation=3,
            phase="WALK_TO_URINAL",
            resource_id="urinal_2",
            goal_pose=[2.0, 1.0, 0.0],
            path_points=[[-1.0, 0.0, 0.0], [0.5, 0.5, 0.0]],
            velocity=0.65,
            final_yaw=math.pi / 2.0,
            stop=False,
            use_direct_pose=False,
            constrain_to_path=False,
        )

        intent = parse_motion_intent(payload)

        self.assertIsNotNone(intent)
        self.assertTrue(intent.shadow_enabled)
        self.assertEqual(intent.generation, 3)
        self.assertEqual(
            intent.goals(),
            (
                (-1.0, 0.0, 0.0),
                (0.5, 0.5, 0.0),
                (2.0, 1.0, 0.0),
            ),
        )

    def test_direct_pose_intent_does_not_start_shadow_motion(self):
        payload = build_motion_intent_payload(
            agent_id="toilet_agent_01",
            generation=1,
            phase="ACTIVATING",
            resource_id="urinal_1",
            goal_pose=[-3.8, -0.9, 0.0],
            path_points=[],
            velocity=0.0,
            final_yaw=0.0,
            stop=True,
            use_direct_pose=True,
            constrain_to_path=False,
        )

        self.assertFalse(parse_motion_intent(payload).shadow_enabled)

    def test_portal_corridor_metadata_round_trip_is_backward_compatible(self):
        payload = build_motion_intent_payload(
            agent_id="toilet_agent_01",
            generation=2,
            phase="WALK_TO_ENTRY_CLEARANCE",
            resource_id="urinal_1",
            goal_pose=[-1.84, -0.9, 0.0],
            path_points=[[-3.8, -0.9, 0.0], [-1.84, -0.9, 0.0]],
            velocity=0.45,
            final_yaw=0.0,
            stop=False,
            use_direct_pose=False,
            constrain_to_path=True,
            portal={
                "portal_id": "entrance_portal",
                "direction": "entering",
                "outside": [-3.8, -0.9, 0.0],
                "inside": [-2.24, -0.9, 0.0],
                "half_width_m": 0.45,
                "clearance_m": 0.40,
                "capacity": 1,
            },
        )

        intent = parse_motion_intent(payload)

        self.assertEqual(payload["route_kind"], "portal_corridor")
        self.assertIsNotNone(intent.portal)
        self.assertEqual(intent.portal.portal_id, "entrance_portal")
        self.assertEqual(intent.portal.direction, "entering")
        self.assertAlmostEqual(intent.portal.half_width_m, 0.45)

    def test_yaw_is_read_by_tag_name_not_position(self):
        person = _Person(
            tagnames=("motion_state", "yaw_valid", "yaw_rad"),
            tags=("executing", "true", "1.25"),
        )

        self.assertEqual(person_yaw(person), 1.25)

    def test_velocity_heading_ignores_idle_noise(self):
        self.assertIsNone(velocity_heading(0.01, 0.01, min_speed=0.1))
        self.assertAlmostEqual(
            velocity_heading(0.0, 1.0, min_speed=0.1),
            math.pi / 2.0,
        )

    def test_point_to_polyline_distance_uses_clamped_segments(self):
        route = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

        self.assertAlmostEqual(
            point_to_polyline_distance(0.5, 0.2, route),
            0.2,
        )
        self.assertAlmostEqual(
            point_to_polyline_distance(1.2, 0.5, route),
            0.2,
        )
        self.assertAlmostEqual(
            point_to_polyline_distance(-0.3, 0.0, route),
            0.3,
        )

    def test_summary_reports_tracking_error_and_segment_local_paths(self):
        samples = [
            {
                "segment_index": 1,
                "source_stamp_sec": 10.0,
                "position_error_m": 0.1,
                "yaw_error_rad": 0.2,
                "motion_heading_error_rad": 0.05,
                "actual_root_to_motion_offset_rad": math.pi / 2.0,
                "actual_cross_track_error_m": 0.02,
                "hunav_cross_track_error_m": 0.03,
                "actual_near_target": True,
                "hunav_near_target": True,
                "actual": {
                    "x": 0.0,
                    "y": 0.0,
                    "guard_blocked": True,
                },
                "hunav": {"x": 0.1, "y": 0.0},
                "robot": {"available": True, "age_sec": 0.01},
                "interaction": {
                    "actual_robot_clearance_m": 0.2,
                    "hunav_robot_clearance_m": 0.3,
                },
            },
            {
                "segment_index": 1,
                "source_stamp_sec": 10.1,
                "position_error_m": 0.2,
                "yaw_error_rad": -0.1,
                "motion_heading_error_rad": -0.1,
                "actual_root_to_motion_offset_rad": math.pi / 2.0,
                "actual_cross_track_error_m": 0.04,
                "hunav_cross_track_error_m": 0.05,
                "actual_near_target": True,
                "hunav_near_target": True,
                "actual": {
                    "x": 0.1,
                    "y": 0.0,
                    "guard_blocked": False,
                },
                "hunav": {"x": 0.3, "y": 0.0},
                "robot": {"available": True, "age_sec": 0.02},
                "interaction": {
                    "actual_robot_clearance_m": 0.1,
                    "hunav_robot_clearance_m": 0.15,
                },
            },
            {
                "segment_index": 2,
                "source_stamp_sec": 10.2,
                "position_error_m": 0.3,
                "yaw_error_rad": None,
                "motion_heading_error_rad": None,
                "actual_root_to_motion_offset_rad": None,
                "actual_cross_track_error_m": 0.06,
                "hunav_cross_track_error_m": 0.07,
                "actual_near_target": False,
                "hunav_near_target": False,
                "actual": {"x": 5.0, "y": 0.0},
                "hunav": {"x": 5.3, "y": 0.0},
                "robot": {"available": False, "age_sec": None},
                "interaction": {
                    "actual_robot_clearance_m": None,
                    "hunav_robot_clearance_m": None,
                },
            },
        ]

        summary = summarize_samples(
            samples, intent_count=2, compute_failures=0
        )

        self.assertEqual(summary["sample_count"], 3)
        self.assertEqual(summary["segment_count"], 2)
        self.assertAlmostEqual(summary["mean_position_error_m"], 0.2)
        self.assertAlmostEqual(summary["actual_path_length_m"], 0.1)
        self.assertAlmostEqual(summary["hunav_path_length_m"], 0.2)
        self.assertAlmostEqual(
            summary["mean_abs_motion_heading_error_rad"],
            0.075,
        )
        self.assertAlmostEqual(
            summary["mean_abs_actual_root_to_motion_offset_rad"],
            math.pi / 2.0,
        )
        self.assertAlmostEqual(summary["guard_blocked_duration_sec"], 0.1)
        self.assertEqual(summary["guard_blocked_sample_count"], 1)
        self.assertEqual(summary["robot_observation_sample_count"], 2)
        self.assertAlmostEqual(summary["min_actual_robot_clearance_m"], 0.1)
        self.assertAlmostEqual(summary["actual_near_target_jitter_m"], 0.1)
        self.assertAlmostEqual(summary["hunav_near_target_jitter_m"], 0.2)

    def test_settled_jitter_excludes_approach_and_hold_window(self):
        samples = []
        for stamp, x, speed in (
            (10.0, 0.00, 0.20),
            (10.2, 0.04, 0.02),
            (10.8, 0.05, 0.02),
            (11.3, 0.06, 0.02),
            (11.5, 0.07, 0.02),
        ):
            samples.append(
                {
                    "segment_index": 1,
                    "source_stamp_sec": stamp,
                    "position_error_m": 0.0,
                    "yaw_error_rad": None,
                    "actual_target_distance_m": 0.1,
                    "actual": {
                        "x": x,
                        "y": 0.0,
                        "vx": speed,
                        "vy": 0.0,
                    },
                    "hunav": {"x": 0.0, "y": 0.0},
                }
            )

        summary = summarize_samples(
            samples,
            intent_count=1,
            compute_failures=0,
            settled_hold_sec=1.0,
        )

        self.assertAlmostEqual(summary["actual_settled_jitter_m"], 0.01)


if __name__ == "__main__":
    unittest.main()
