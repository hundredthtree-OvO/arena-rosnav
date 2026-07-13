import unittest

from toilet_benchmark.resource_manager import QueueSlot, Resource, ResourceManager


class TestResourceManager(unittest.TestCase):
    def setUp(self):
        self.resource = Resource(
            resource_id="urinal_1",
            category="urinals",
            position=[0.0, 0.0, 0.0],
            occupied_by="agent_1",
            queue_slots=[
                QueueSlot("queue_1", [0.0, -0.5, 0.0], occupant_id="agent_2"),
                QueueSlot("queue_2", [0.0, -1.0, 0.0], occupant_id="agent_3"),
            ],
        )
        self.manager = ResourceManager({self.resource.resource_id: self.resource})

    def test_owner_release_promotes_first_queued_agent(self):
        promoted = self.manager.release("agent_1", "urinal_1")

        self.assertEqual(promoted, "agent_2")
        self.assertEqual(self.resource.occupied_by, "agent_2")
        self.assertIsNone(self.resource.queue_slots[0].occupant_id)

    def test_queued_agent_release_does_not_promote_another_agent(self):
        promoted = self.manager.release("agent_3", "urinal_1")

        self.assertIsNone(promoted)
        self.assertEqual(self.resource.occupied_by, "agent_1")
        self.assertEqual(self.resource.queue_slots[0].occupant_id, "agent_2")
        self.assertIsNone(self.resource.queue_slots[1].occupant_id)

    def test_unknown_agent_release_is_a_noop(self):
        promoted = self.manager.release("missing", "urinal_1")

        self.assertIsNone(promoted)
        self.assertEqual(self.resource.occupied_by, "agent_1")
        self.assertEqual(
            [slot.occupant_id for slot in self.resource.queue_slots],
            ["agent_2", "agent_3"],
        )


if __name__ == "__main__":
    unittest.main()
