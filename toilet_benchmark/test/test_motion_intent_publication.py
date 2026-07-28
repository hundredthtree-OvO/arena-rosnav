import unittest
from types import SimpleNamespace

from toilet_benchmark.semantic_rules import PortalCorridor
from toilet_benchmark.toilet_director_node import ToiletDirectorNode


class _Response:
    def __init__(self, accepted):
        self.ret = bool(accepted)


class _Future:
    def __init__(self, accepted):
        self._response = _Response(accepted)

    def result(self):
        return self._response


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class TestMotionIntentPublication(unittest.TestCase):
    def setUp(self):
        self.director = object.__new__(ToiletDirectorNode)
        self.director._status_publisher = _Publisher()
        self.director.walk_plane_z = 0.0
        self.director.portal_crossing_velocity_mps = 0.45
        self.payload = {
            "event": "motion_intent",
            "agent_id": "toilet_agent_01",
            "generation": 2,
        }

    def test_accepted_move_publishes_intent(self):
        self.director._publish_motion_intent_if_accepted(
            self.payload,
            _Future(True),
        )

        self.assertEqual(len(self.director._status_publisher.messages), 1)

    def test_rejected_move_does_not_publish_intent(self):
        self.director._publish_motion_intent_if_accepted(
            self.payload,
            _Future(False),
        )

        self.assertEqual(self.director._status_publisher.messages, [])

    def test_portal_motion_metadata_uses_corridor_contract(self):
        self.director.portal = {
            "id": "entrance_portal",
            "corridor": PortalCorridor(
                outside=(-3.8, -0.9, 0.0),
                inside=(-2.24, -0.9, 0.0),
                half_width_m=0.45,
                clearance_m=0.40,
            ),
        }
        state = SimpleNamespace(portal_direction="entering")

        metadata = self.director._portal_motion_metadata(
            state,
            "WALK_TO_ENTRY_CLEARANCE",
        )

        self.assertEqual(metadata["portal_id"], "entrance_portal")
        self.assertEqual(metadata["direction"], "entering")
        self.assertEqual(metadata["capacity"], 1)

    def test_hunav_portal_path_extends_along_portal_axis(self):
        self.director.motion_backend_name = "hunav"
        self.director.portal = {
            "outside": [-3.8, -0.91, 0.0],
            "center": [-2.24, -0.9, 0.0],
            "inside": [-2.24, -0.9, 0.0],
            "inside_staging": [-2.24, -0.675, 0.0],
            "corridor": PortalCorridor(
                outside=(-3.8, -0.91, 0.0),
                inside=(-2.24, -0.9, 0.0),
                half_width_m=0.45,
                clearance_m=0.40,
            ),
        }

        clearance = self.director._hunav_portal_clearance_pose()
        waypoints = self.director._portal_entry_waypoints()

        self.assertGreater(clearance[0], self.director.portal["inside"][0])
        self.assertAlmostEqual(clearance[1], -0.8974, places=3)
        self.assertEqual(waypoints[-1], clearance)
        self.assertAlmostEqual(self.director._portal_entry_clearance_yaw(), 0.0064, places=3)

    def test_hunav_portal_uses_regular_agent_velocity(self):
        self.director.motion_backend_name = "hunav"
        self.assertEqual(
            self.director._portal_motion_velocity(SimpleNamespace(velocity=0.78)),
            0.78,
        )

    def test_compatibility_portal_retains_velocity_cap(self):
        self.director.motion_backend_name = "isaac_people"
        self.assertEqual(
            self.director._portal_motion_velocity(SimpleNamespace(velocity=0.78)),
            0.45,
        )

    def test_hunav_can_bypass_portal_controller(self):
        self.director.motion_backend_name = "hunav"
        self.director.hunav_use_portal_controller = False
        self.director.portal = {"id": "entrance_portal"}

        self.assertFalse(self.director._uses_portal_controller())

    def test_compatibility_backend_keeps_portal_controller(self):
        self.director.motion_backend_name = "isaac_people"
        self.director.hunav_use_portal_controller = False
        self.director.portal = {"id": "entrance_portal"}

        self.assertTrue(self.director._uses_portal_controller())


if __name__ == "__main__":
    unittest.main()
