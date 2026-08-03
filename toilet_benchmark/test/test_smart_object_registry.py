import pytest

from toilet_benchmark.scenario import (
    ClaimStatus,
    ObjectKind,
    SmartObject,
    SmartObjectRegistry,
    SmartObjectSlot,
    OrientedRegion,
    PassageStatus,
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


def _oriented_urinal(object_id, x, *, passage_half_width=0.42):
    return SmartObject(
        object_id=object_id,
        kind=ObjectKind.URINAL,
        interaction_slots=(
            SmartObjectSlot(
                f"{object_id}:use",
                (x, 1.0, 0.5 * 3.141592653589793),
                occupancy_region=OrientedRegion(
                    center_xy=(x, 1.0),
                    yaw=0.5 * 3.141592653589793,
                    half_length=0.42,
                    half_width=0.26,
                ),
                passage_region=OrientedRegion(
                    center_xy=(x, 0.55),
                    yaw=0.5 * 3.141592653589793,
                    half_length=0.45,
                    half_width=passage_half_width,
                ),
            ),
        ),
    )


def test_oriented_station_occupancy_does_not_block_non_overlapping_neighbor():
    registry = SmartObjectRegistry(
        (_oriented_urinal("urinal_1", 0.0), _oriented_urinal("urinal_2", 0.77))
    )

    assert registry.occupancy_conflicts("urinal_1") == ()
    assert registry.claim("urinal_1", "agent_01").status == ClaimStatus.ACQUIRED
    assert registry.claim("urinal_2", "agent_02").status == ClaimStatus.ACQUIRED


def test_overlapping_oriented_station_occupancy_retries_after_owner_release():
    registry = SmartObjectRegistry(
        (_oriented_urinal("urinal_1", 0.0), _oriented_urinal("urinal_2", 0.40))
    )
    owner = registry.claim("urinal_1", "agent_01")

    blocked = registry.claim("urinal_2", "agent_02")
    assert blocked.status == ClaimStatus.REJECTED
    assert blocked.reason == "oriented_occupancy_conflict"

    registry.release(owner.claim_id)
    assert registry.claim("urinal_2", "agent_02").status == ClaimStatus.ACQUIRED


def test_adjacent_ingress_is_concurrent_but_egress_is_serialized():
    registry = SmartObjectRegistry(
        (_oriented_urinal("urinal_1", 0.0), _oriented_urinal("urinal_2", 0.77))
    )

    first = registry.acquire_passage("urinal_1", "agent_01", direction="ingress")
    concurrent = registry.acquire_passage("urinal_2", "agent_02", direction="ingress")

    assert registry.passage_conflicts("urinal_1") == ("urinal_2",)
    assert first.status == PassageStatus.ACQUIRED
    assert concurrent.status == PassageStatus.ACQUIRED

    assert registry.release_passage("agent_01") is True
    assert registry.release_passage("agent_02") is True
    first_exit = registry.acquire_passage("urinal_1", "agent_01", direction="egress")
    blocked = registry.acquire_passage("urinal_2", "agent_02", direction="egress")
    assert first_exit.status == PassageStatus.ACQUIRED
    assert blocked.status == PassageStatus.BLOCKED
    assert blocked.blocked_by_agent_id == "agent_01"

    assert registry.release_passage("agent_01") is True
    resumed = registry.acquire_passage("urinal_2", "agent_02", direction="egress")
    assert resumed.status == PassageStatus.ACQUIRED


def test_passage_holding_pose_is_outside_the_arbitrated_corridor():
    registry = SmartObjectRegistry((_oriented_urinal("urinal_1", 0.0),))

    staging = registry.passage_staging_pose("urinal_1")
    holding = registry.passage_holding_pose("urinal_1", offset_m=0.45)

    assert staging == pytest.approx((0.0, 0.10, 0.5 * 3.141592653589793))
    assert holding == pytest.approx((0.0, -0.35, 0.5 * 3.141592653589793))
