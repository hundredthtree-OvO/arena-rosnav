from toilet_benchmark.scenario import (
    ClaimStatus,
    ObjectKind,
    SmartObject,
    SmartObjectRegistry,
    SmartObjectSlot,
)


def _urinal(object_id="urinal_1"):
    return SmartObject(
        object_id=object_id,
        kind=ObjectKind.URINAL,
        interaction_slots=(SmartObjectSlot(f"{object_id}:use", (1.0, 2.0, 0.0)),),
        queue_slots=(
            SmartObjectSlot(f"{object_id}:queue:1", (0.5, 2.0, 0.0)),
            SmartObjectSlot(f"{object_id}:queue:2", (0.0, 2.0, 0.0)),
        ),
    )


def test_claim_release_promotes_fifo_and_compacts_queue():
    registry = SmartObjectRegistry((_urinal(),))

    owner = registry.claim("urinal_1", "agent_01")
    first = registry.claim("urinal_1", "agent_02")
    second = registry.claim("urinal_1", "agent_03")

    assert owner.status == ClaimStatus.ACQUIRED
    assert first.status == ClaimStatus.QUEUED
    assert first.queue_index == 0
    assert second.queue_index == 1
    assert registry.claim("urinal_1", "agent_02") == first

    released = registry.release(owner.claim_id, reason="activity_complete")

    assert released.promoted_agent_id == "agent_02"
    assert registry.claim_for_agent("agent_02").status == ClaimStatus.ACQUIRED
    compacted = registry.claim_for_agent("agent_03")
    assert compacted.queue_index == 0
    assert compacted.slot_id == "urinal_1:queue:1"
    assert registry.snapshot() == {
        "urinal_1": {
            "owners": {"urinal_1:use": "agent_02"},
            "queue": ["agent_03"],
        }
    }


def test_queue_capacity_single_claim_and_runtime_registration():
    registry = SmartObjectRegistry((_urinal(),))
    registry.register(_urinal("urinal_2"))
    registry.claim("urinal_1", "agent_01")
    registry.claim("urinal_1", "agent_02")
    registry.claim("urinal_1", "agent_03")

    full = registry.claim("urinal_1", "agent_04")
    duplicate = registry.claim("urinal_2", "agent_01")

    assert full.status == ClaimStatus.REJECTED
    assert full.reason == "queue_full"
    assert duplicate.status == ClaimStatus.REJECTED
    assert "already holds claim" in duplicate.reason

    registry.reset()
    assert registry.claim_for_agent("agent_01") is None
    assert registry.claim("urinal_2", "agent_01").status == ClaimStatus.ACQUIRED


def test_releasing_queued_claim_compacts_remaining_queue():
    registry = SmartObjectRegistry((_urinal(),))
    registry.claim("urinal_1", "agent_01")
    first = registry.claim("urinal_1", "agent_02")
    registry.claim("urinal_1", "agent_03")

    registry.release(first.claim_id)

    remaining = registry.claim_for_agent("agent_03")
    assert remaining.queue_index == 0
    assert remaining.slot_id == "urinal_1:queue:1"
