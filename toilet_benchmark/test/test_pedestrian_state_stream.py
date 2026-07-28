import unittest
from types import SimpleNamespace

from toilet_benchmark.pedestrian_state_stream import (
    extract_xyz,
    observations_from_message,
)


def _point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


class TestPedestrianStateStream(unittest.TestCase):
    def test_normalizes_people_message_and_guard_metadata(self):
        person = SimpleNamespace(
            stage_prefix="/World/Characters/toilet_agent_01",
            name="ignored",
            reliability=0.75,
            pose=SimpleNamespace(position=_point(1.0, 2.0, 0.0)),
            position=None,
            velocity=_point(0.2, -0.1, 0.0),
            tagnames=["guard_blocked", "guard_block_reason"],
            tags=["true", "robot"],
        )

        observations = observations_from_message(
            SimpleNamespace(people=[person]),
            observed_at=12.5,
        )

        self.assertEqual(len(observations), 1)
        observation = observations[0]
        self.assertEqual(
            observation.identifier,
            "/World/Characters/toilet_agent_01",
        )
        self.assertEqual(observation.position, (1.0, 2.0, 0.0))
        self.assertEqual(observation.observed_at, 12.5)
        self.assertEqual(observation.reliability, 0.75)
        self.assertEqual(observation.metadata["guard_block_reason"], "robot")
        self.assertEqual(observation.velocity, (0.2, -0.1, 0.0))

    def test_accepts_pedestrians_field_and_nested_pose_shape(self):
        nested = SimpleNamespace(pose=SimpleNamespace(position=_point(-1.0, 0.5, 0.2)))
        person = SimpleNamespace(
            id="toilet_agent_02",
            reliability=None,
            pose=None,
            position=nested,
            tagnames=[],
            tags=[],
        )

        observations = observations_from_message(
            SimpleNamespace(people=None, pedestrians=[person]),
            observed_at=2.0,
        )

        self.assertEqual(observations[0].identifier, "toilet_agent_02")
        self.assertEqual(observations[0].position, (-1.0, 0.5, 0.2))
        self.assertEqual(observations[0].reliability, 1.0)

    def test_drops_unreliable_or_unpositioned_entries(self):
        unreliable = SimpleNamespace(
            name="a",
            reliability=0.0,
            pose=_point(0.0, 0.0, 0.0),
            position=None,
        )
        unpositioned = SimpleNamespace(
            name="b",
            reliability=1.0,
            pose=SimpleNamespace(),
            position=None,
        )

        observations = observations_from_message(
            SimpleNamespace(people=[unreliable, unpositioned]),
            observed_at=1.0,
        )

        self.assertEqual(observations, [])

    def test_extract_xyz_supports_direct_vector(self):
        self.assertEqual(extract_xyz(_point(1, 2, 3)), (1.0, 2.0, 3.0))


if __name__ == "__main__":
    unittest.main()
