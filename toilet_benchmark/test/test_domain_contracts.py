import json
import unittest

from toilet_benchmark.domain.events import (
    BenchmarkEvent,
    decode_json_payload,
    encode_event_payload,
)
from toilet_benchmark.domain.task import MotionCommand, TaskPhase, TerminationReason


class TestDomainContracts(unittest.TestCase):
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

    def test_benchmark_event_keeps_current_payload_shape(self):
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
