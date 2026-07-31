import json
import math
import unittest

from toilet_benchmark.domain.agent import AgentSnapshot, DirectedAgentState
from toilet_benchmark.domain.events import (
    BenchmarkEvent,
    decode_json_payload,
    encode_event_payload,
)
from toilet_benchmark.domain.task import MotionCommand, TaskPhase, TerminationReason
from toilet_benchmark.hunav_adapter import DirectedAgentState as LegacyDirectedAgentState


class TestDomainContracts(unittest.TestCase):
    def test_agent_snapshot_round_trip_is_simulator_neutral(self):
        snapshot = AgentSnapshot(
            agent_id="toilet_agent_01",
            timestamp_sec=12.5,
            x=1.0,
            y=2.0,
            z=0.0,
            yaw=0.5,
            vx=0.2,
            vy=-0.1,
            wz=0.05,
            radius_m=0.26,
            source="isaac",
        )

        restored = AgentSnapshot.from_mapping(snapshot.to_dict())

        self.assertEqual(restored, snapshot)
        self.assertNotIn("ros", snapshot.to_dict())
        self.assertNotIn("hunav", snapshot.to_dict())

    def test_agent_snapshot_rejects_non_finite_state(self):
        with self.assertRaises(ValueError):
            AgentSnapshot(
                agent_id="toilet_agent_01",
                timestamp_sec=0.0,
                x=math.nan,
                y=0.0,
                z=0.0,
                yaw=0.0,
                vx=0.0,
                vy=0.0,
                wz=0.0,
                radius_m=0.26,
                source="test",
            )

    def test_legacy_directed_agent_state_is_a_compatibility_reexport(self):
        self.assertIs(LegacyDirectedAgentState, DirectedAgentState)

    def test_motion_command_preserves_existing_fields(self):
        command = MotionCommand(
            agent_id="toilet_agent_01",
            goal_pose=[1.0, 2.0, 0.0],
            path_points=[[0.0, 0.0, 0.0], [1.0, 2.0, 0.0]],
            velocity=0.7,
            orientation=1.2,
            phase=TaskPhase.WALK_TO_URINAL,
        )

        self.assertEqual(command.phase, TaskPhase.WALK_TO_URINAL)
        self.assertEqual(command.goal_pose, [1.0, 2.0, 0.0])
        self.assertEqual(command.velocity, 0.7)

    def test_task_and_termination_enums_remain_string_compatible(self):
        self.assertEqual(TaskPhase.EXITING, "EXITING")
        self.assertEqual(TerminationReason.ROBOT_HUMAN_COLLISION, "robot_human_collision")

    def test_benchmark_event_keeps_current_director_payload_shape(self):
        event = BenchmarkEvent(
            event_type="pedestrian_active",
            agent_id="toilet_agent_01",
            phase=TaskPhase.WALK_TO_URINAL,
            payload={"resource_id": "urinal_3", "reason": "activated"},
        )

        payload = event.to_payload()

        self.assertEqual(
            payload,
            {
                "event": "pedestrian_active",
                "agent_id": "toilet_agent_01",
                "phase": "WALK_TO_URINAL",
                "resource_id": "urinal_3",
                "reason": "activated",
            },
        )
        self.assertEqual(BenchmarkEvent.from_payload(payload), event)

    def test_event_json_helpers_are_deterministic_and_backward_compatible(self):
        payload = {
            "event": "motion_intent",
            "agent_id": "toilet_agent_01",
            "generation": 2,
        }

        encoded = encode_event_payload(payload)

        self.assertEqual(encoded, json.dumps(payload, sort_keys=True))
        self.assertEqual(decode_json_payload(encoded), payload)
        self.assertEqual(decode_json_payload("[1, 2]"), {"value": [1, 2]})
        self.assertEqual(decode_json_payload("not-json"), {"raw": "not-json"})


if __name__ == "__main__":
    unittest.main()

