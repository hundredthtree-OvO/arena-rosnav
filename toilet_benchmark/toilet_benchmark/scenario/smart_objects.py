"""Reservation-only Smart Objects for toilet resources and queues."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Iterable


class ObjectKind(str, Enum):
    PORTAL = "portal"
    URINAL = "urinal"
    TOILET_STALL = "toilet_stall"
    WAITING_AREA = "waiting_area"


@dataclass(frozen=True)
class SmartObjectSlot:
    slot_id: str
    pose: tuple[float, float, float]

    def __post_init__(self) -> None:
        if not self.slot_id:
            raise ValueError("slot_id must not be empty")
        if len(self.pose) != 3:
            raise ValueError("slot pose must contain x, y and yaw")
        if not all(math.isfinite(float(value)) for value in self.pose):
            raise ValueError("slot pose values must be finite")


@dataclass(frozen=True)
class SmartObject:
    object_id: str
    kind: ObjectKind
    interaction_slots: tuple[SmartObjectSlot, ...]
    queue_slots: tuple[SmartObjectSlot, ...] = ()
    tags: frozenset[str] = frozenset()

    def __post_init__(self) -> None:
        if not self.object_id:
            raise ValueError("object_id must not be empty")
        if not self.interaction_slots:
            raise ValueError("a Smart Object needs at least one interaction slot")
        slot_ids = [slot.slot_id for slot in (*self.interaction_slots, *self.queue_slots)]
        if len(slot_ids) != len(set(slot_ids)):
            raise ValueError(f"duplicate slot id in {self.object_id}")


class ClaimStatus(str, Enum):
    ACQUIRED = "acquired"
    QUEUED = "queued"
    REJECTED = "rejected"


@dataclass(frozen=True)
class ClaimResult:
    status: ClaimStatus
    object_id: str
    agent_id: str
    claim_id: str | None = None
    slot_id: str | None = None
    queue_index: int | None = None
    reason: str | None = None


@dataclass(frozen=True)
class ReleaseResult:
    released: bool
    object_id: str
    agent_id: str
    promoted_agent_id: str | None = None
    promoted_claim_id: str | None = None


@dataclass
class _Claim:
    claim_id: str
    object_id: str
    agent_id: str
    slot_id: str
    status: ClaimStatus


class SmartObjectRegistry:
    """Deterministic ownership and FIFO queue bookkeeping."""

    def __init__(self, objects: Iterable[SmartObject] = ()):
        self._objects: dict[str, SmartObject] = {}
        self._claims: dict[str, _Claim] = {}
        self._agent_claims: dict[str, str] = {}
        self._owners: dict[str, dict[str, str]] = {}
        self._queues: dict[str, list[str]] = {}
        self._claim_sequence = 0
        for smart_object in objects:
            self.register(smart_object)

    def register(self, smart_object: SmartObject) -> None:
        if smart_object.object_id in self._objects:
            raise ValueError(f"duplicate Smart Object: {smart_object.object_id}")
        self._objects[smart_object.object_id] = smart_object
        self._owners[smart_object.object_id] = {}
        self._queues[smart_object.object_id] = []

    def reset(self) -> None:
        self._claims: dict[str, _Claim] = {}
        self._agent_claims: dict[str, str] = {}
        self._owners: dict[str, dict[str, str]] = {
            object_id: {} for object_id in self._objects
        }
        self._queues: dict[str, list[str]] = {
            object_id: [] for object_id in self._objects
        }
        self._claim_sequence = 0

    def get(self, object_id: str) -> SmartObject:
        try:
            return self._objects[object_id]
        except KeyError as exc:
            raise KeyError(f"unknown Smart Object: {object_id}") from exc

    def query(self, *, kind: ObjectKind | None = None, tags: Iterable[str] = ()) -> tuple[SmartObject, ...]:
        required_tags = frozenset(tags)
        return tuple(
            smart_object
            for smart_object in self._objects.values()
            if (kind is None or smart_object.kind == kind)
            and required_tags.issubset(smart_object.tags)
        )

    def claim(self, object_id: str, agent_id: str) -> ClaimResult:
        if not agent_id:
            raise ValueError("agent_id must not be empty")
        smart_object = self.get(object_id)
        existing_id = self._agent_claims.get(agent_id)
        if existing_id is not None:
            existing = self._claims[existing_id]
            if existing.object_id != object_id:
                return ClaimResult(
                    ClaimStatus.REJECTED,
                    object_id,
                    agent_id,
                    reason=f"agent already holds claim for {existing.object_id}",
                )
            return self._to_result(existing)

        free_slot = next(
            (
                slot
                for slot in smart_object.interaction_slots
                if slot.slot_id not in self._owners[object_id]
            ),
            None,
        )
        if free_slot is not None:
            claim = self._new_claim(object_id, agent_id, free_slot.slot_id, ClaimStatus.ACQUIRED)
            self._owners[object_id][free_slot.slot_id] = claim.claim_id
            return self._to_result(claim)

        queue = self._queues[object_id]
        if len(queue) >= len(smart_object.queue_slots):
            return ClaimResult(
                ClaimStatus.REJECTED,
                object_id,
                agent_id,
                reason="queue_full",
            )
        queue_slot = smart_object.queue_slots[len(queue)]
        claim = self._new_claim(object_id, agent_id, queue_slot.slot_id, ClaimStatus.QUEUED)
        queue.append(claim.claim_id)
        return self._to_result(claim)

    def claim_for_agent(self, agent_id: str) -> ClaimResult | None:
        claim_id = self._agent_claims.get(agent_id)
        return None if claim_id is None else self._to_result(self._claims[claim_id])

    def release(self, claim_id: str, *, reason: str | None = None) -> ReleaseResult:
        del reason  # Reserved for audit adapters; registry semantics do not depend on it.
        claim = self._claims.get(claim_id)
        if claim is None:
            return ReleaseResult(False, "", "")
        smart_object = self._objects[claim.object_id]
        promoted: _Claim | None = None

        if claim.status == ClaimStatus.ACQUIRED:
            self._owners[claim.object_id].pop(claim.slot_id, None)
            queue = self._queues[claim.object_id]
            if queue:
                promoted = self._claims[queue.pop(0)]
                interaction_slot = next(
                    slot
                    for slot in smart_object.interaction_slots
                    if slot.slot_id not in self._owners[claim.object_id]
                )
                promoted.slot_id = interaction_slot.slot_id
                promoted.status = ClaimStatus.ACQUIRED
                self._owners[claim.object_id][interaction_slot.slot_id] = promoted.claim_id
                self._refresh_queue_slots(claim.object_id)
        else:
            self._queues[claim.object_id].remove(claim.claim_id)
            self._refresh_queue_slots(claim.object_id)

        del self._claims[claim.claim_id]
        self._agent_claims.pop(claim.agent_id, None)
        return ReleaseResult(
            True,
            claim.object_id,
            claim.agent_id,
            None if promoted is None else promoted.agent_id,
            None if promoted is None else promoted.claim_id,
        )

    def snapshot(self) -> dict[str, dict[str, object]]:
        return {
            object_id: {
                "owners": {
                    slot_id: self._claims[claim_id].agent_id
                    for slot_id, claim_id in owners.items()
                },
                "queue": [self._claims[claim_id].agent_id for claim_id in self._queues[object_id]],
            }
            for object_id, owners in self._owners.items()
        }

    def _new_claim(
        self,
        object_id: str,
        agent_id: str,
        slot_id: str,
        status: ClaimStatus,
    ) -> _Claim:
        self._claim_sequence += 1
        claim = _Claim(
            claim_id=f"claim_{self._claim_sequence:06d}",
            object_id=object_id,
            agent_id=agent_id,
            slot_id=slot_id,
            status=status,
        )
        self._claims[claim.claim_id] = claim
        self._agent_claims[agent_id] = claim.claim_id
        return claim

    def _refresh_queue_slots(self, object_id: str) -> None:
        queue_slots = self._objects[object_id].queue_slots
        for index, claim_id in enumerate(self._queues[object_id]):
            self._claims[claim_id].slot_id = queue_slots[index].slot_id

    def _to_result(self, claim: _Claim) -> ClaimResult:
        queue_index = None
        if claim.status == ClaimStatus.QUEUED:
            queue_index = self._queues[claim.object_id].index(claim.claim_id)
        return ClaimResult(
            status=claim.status,
            object_id=claim.object_id,
            agent_id=claim.agent_id,
            claim_id=claim.claim_id,
            slot_id=claim.slot_id,
            queue_index=queue_index,
        )
