import unittest

from toilet_benchmark.hunav_phase0_core import (
    aggregate_results,
    analyze_trace,
    available_scenarios,
    build_scenario,
    select_closest_obstacles,
)
from toilet_benchmark.semantic_rules import PortalCorridor


THRESHOLDS = {
    "goal_slack_m": 0.15,
    "completion_speed_mps": 0.05,
    "final_yaw_tolerance_rad": 0.25,
    "overlap_radius_scale": 0.90,
    "pose_jump_min_m": 0.20,
    "pose_jump_slack_m": 0.10,
    "motion_yaw_speed_mps": 0.10,
    "stall_window_sec": 1.0,
    "stall_displacement_m": 0.05,
    "portal_center_stop_max_frames": 2,
    "portal_center_stop_max_sec": 1.5,
    "portal_reverse_progress_epsilon_m": 0.05,
}


def frame(time_sec, *agents):
    return {
        "time_sec": time_sec,
        "agents": [
            {
                "id": agent_id,
                "name": f"agent_{agent_id}",
                "x": x,
                "y": y,
                "yaw": yaw,
                "vx": vx,
                "vy": vy,
                "radius": 0.30,
            }
            for agent_id, x, y, yaw, vx, vy in agents
        ],
        "robot": {
            "x": 100.0,
            "y": 100.0,
            "vx": 0.0,
            "vy": 0.0,
            "radius": 0.45,
        },
    }


class TestHuNavPhase0Core(unittest.TestCase):
    def test_all_documented_scenarios_are_available(self):
        self.assertEqual(
            set(available_scenarios()),
            {
                "single_pause_resume",
                "head_on",
                "crossing",
                "narrow_passage",
                "narrow_gate",
                "toilet_portal_corridor",
                "static_robot",
                "moving_robot",
            },
        )

    def test_seeded_scenario_is_reproducible(self):
        first = build_scenario("crossing", seed=42)
        second = build_scenario("crossing", seed=42)
        different = build_scenario("crossing", seed=43)

        self.assertEqual(first, second)
        self.assertNotEqual(first, different)

    def test_closest_obstacles_are_local_bounded_and_stable(self):
        selected = select_closest_obstacles(
            ((2.0, 0.0), (0.5, 0.0), (-0.5, 0.0), (0.0, 0.4)),
            (0.0, 0.0),
            max_count=2,
            max_distance_m=1.0,
        )

        self.assertEqual(selected, ((0.0, 0.4), (0.5, 0.0)))

    def test_completed_single_agent_trace_passes(self):
        scenario = build_scenario("single_pause_resume", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -2.0, 0.0, 0.0, 0.5, 0.0)),
            frame(1.0, (1, -1.0, 0.0, 0.0, 0.5, 0.0)),
            frame(2.0, (1, 0.0, 0.0, 0.0, 0.5, 0.0)),
            frame(3.0, (1, 1.0, 0.0, 0.0, 0.5, 0.0)),
            frame(4.0, (1, 2.0, 0.0, 0.0, 0.0, 0.0)),
        ]
        thresholds = dict(THRESHOLDS, pose_jump_min_m=1.1)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertTrue(result["passed"])
        self.assertEqual(result["termination_reason"], "completed")
        self.assertEqual(result["completed_agents"], [1])

    def test_available_scenarios_include_toilet_portal_corridor(self):
        self.assertIn("toilet_portal_corridor", available_scenarios())

    def test_toilet_portal_corridor_uses_expected_portal_geometry(self):
        scenario = build_scenario("toilet_portal_corridor", seed=42, jitter_m=0.0)

        self.assertIsInstance(scenario.portal, PortalCorridor)
        self.assertEqual(scenario.portal.outside, (-3.8, -0.9, 0.0))
        self.assertEqual(scenario.portal.inside, (-2.24, -0.9, 0.0))
        self.assertAlmostEqual(
            scenario.portal.traversal_goal("entering")[0],
            -1.84,
        )
        self.assertAlmostEqual(
            scenario.portal.traversal_goal("entering")[1],
            -0.9,
        )

    def test_portal_corridor_trace_reports_portal_entry_clearance_and_deviation(self):
        scenario = build_scenario("toilet_portal_corridor", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -4.10, -0.90, 0.0, 0.40, 0.0)),
            frame(1.0, (1, -3.60, -0.90, 0.0, 0.40, 0.0)),
            frame(2.0, (1, -2.90, -0.88, 0.0, 0.40, 0.0)),
            frame(3.0, (1, -2.20, -0.89, 0.0, 0.40, 0.0)),
            frame(4.0, (1, -1.80, -0.90, 0.0, 0.0, 0.0)),
        ]
        thresholds = dict(THRESHOLDS, pose_jump_min_m=1.1)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertTrue(result["passed"])
        self.assertEqual(result["termination_reason"], "completed")
        self.assertTrue(result["portal_entered"])
        self.assertTrue(result["portal_cleared"])
        self.assertAlmostEqual(result["portal_max_lateral_deviation_m"], 0.02)
        self.assertEqual(result["portal_center_stop_frame_count"], 0)
        self.assertAlmostEqual(result["portal_center_stop_duration_sec"], 0.0)
        self.assertAlmostEqual(result["portal_reverse_progress_m"], 0.0)

    def test_portal_corridor_trace_fails_when_agent_stops_at_center_too_long(self):
        scenario = build_scenario("toilet_portal_corridor", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -4.10, -0.90, 0.0, 0.40, 0.0)),
            frame(1.0, (1, -3.30, -0.90, 0.0, 0.40, 0.0)),
            frame(2.0, (1, -3.02, -0.90, 0.0, 0.0, 0.0)),
            frame(3.0, (1, -3.02, -0.90, 0.0, 0.0, 0.0)),
            frame(4.0, (1, -3.02, -0.90, 0.0, 0.0, 0.0)),
        ]
        thresholds = dict(THRESHOLDS, pose_jump_min_m=1.1)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertFalse(result["passed"])
        self.assertEqual(result["termination_reason"], "portal_center_stop")
        self.assertTrue(result["portal_entered"])
        self.assertFalse(result["portal_cleared"])
        self.assertGreater(result["portal_center_stop_frame_count"], 2)
        self.assertGreater(result["portal_center_stop_duration_sec"], 1.5)

    def test_portal_corridor_trace_does_not_clear_when_beyond_plane_but_outside_width(self):
        scenario = build_scenario("toilet_portal_corridor", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -4.10, -0.90, 0.0, 0.40, 0.0)),
            frame(1.0, (1, -3.40, -0.90, 0.0, 0.40, 0.0)),
            frame(2.0, (1, -2.60, -0.72, 0.0, 0.40, 0.0)),
            frame(3.0, (1, -1.80, -0.30, 0.0, 0.20, 0.0)),
            frame(4.0, (1, -1.80, -0.30, 0.0, 0.0, 0.0)),
        ]
        thresholds = dict(THRESHOLDS, pose_jump_min_m=1.1)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertFalse(result["passed"])
        self.assertFalse(result["portal_cleared"])
        self.assertGreater(result["portal_max_lateral_deviation_m"], 0.45)
        self.assertEqual(result["termination_reason"], "timeout")

    def test_overlap_has_priority_over_completion(self):
        scenario = build_scenario("head_on", seed=42, jitter_m=0.0)
        frames = [
            frame(
                0.0,
                (1, -2.0, -0.12, 0.0, 0.5, 0.0),
                (2, 2.0, 0.12, 3.14, -0.5, 0.0),
            ),
            frame(
                1.0,
                (1, 0.0, 0.0, 0.0, 0.5, 0.0),
                (2, 0.1, 0.0, 3.14, -0.5, 0.0),
            ),
            frame(
                2.0,
                (1, 2.0, -0.12, 0.0, 0.0, 0.0),
                (2, -2.0, 0.12, 3.14, 0.0, 0.0),
            ),
        ]
        thresholds = dict(THRESHOLDS, pose_jump_min_m=2.2)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertFalse(result["passed"])
        self.assertEqual(result["termination_reason"], "pedestrian_overlap")
        self.assertEqual(result["overlap_frame_count"], 1)

    def test_timeout_with_stationary_tail_is_stalled(self):
        scenario = build_scenario("single_pause_resume", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -2.0, 0.0, 0.0, 0.0, 0.0)),
            frame(1.0, (1, -1.5, 0.0, 0.0, 0.0, 0.0)),
            frame(2.0, (1, -1.5, 0.0, 0.0, 0.0, 0.0)),
        ]

        thresholds = dict(THRESHOLDS, pose_jump_min_m=0.6)
        result = analyze_trace(scenario, frames, thresholds)

        self.assertFalse(result["passed"])
        self.assertEqual(result["termination_reason"], "stalled")
        self.assertEqual(result["stalled_agents"], [1])

    def test_robot_overlap_fails_an_otherwise_completed_trace(self):
        scenario = build_scenario("static_robot", seed=42, jitter_m=0.0)
        frames = [
            frame(0.0, (1, -2.0, 0.0, 0.0, 0.5, 0.0)),
            frame(1.0, (1, 0.0, 0.0, 0.0, 0.5, 0.0)),
            frame(2.0, (1, 2.0, 0.0, 0.0, 0.0, 0.0)),
        ]
        for item in frames:
            item["robot"].update({"x": 0.0, "y": 0.0})
        thresholds = dict(THRESHOLDS, pose_jump_min_m=2.1)

        result = analyze_trace(scenario, frames, thresholds)

        self.assertFalse(result["passed"])
        self.assertEqual(result["termination_reason"], "robot_overlap")
        self.assertEqual(result["robot_overlap_frame_count"], 1)

    def test_aggregate_reports_failed_scenarios(self):
        aggregate = aggregate_results(
            [
                {"scenario": "single_pause_resume", "passed": True},
                {"scenario": "head_on", "passed": False},
            ]
        )
        self.assertFalse(aggregate["passed"])
        self.assertEqual(aggregate["passed_count"], 1)
        self.assertEqual(aggregate["failed_scenarios"], ["head_on"])


if __name__ == "__main__":
    unittest.main()
