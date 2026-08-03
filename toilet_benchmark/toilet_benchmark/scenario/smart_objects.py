"""Smart Object ownership plus deterministic oriented-passage arbitration."""

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
class OrientedRegion:
    """Planar oriented box used for semantic occupancy and passage conflicts."""

    center_xy: tuple[float, float]
    yaw: float
    half_length: float
    half_width: float

    def __post_init__(self) -> None:
        values = (*self.center_xy, self.yaw, self.half_length, self.half_width)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("oriented region values must be finite")
        if self.half_length < 0.0 or self.half_width < 0.0:
            raise ValueError("oriented region half extents must be non-negative")

    def intersects(self, other: "OrientedRegion") -> bool:
        axes = (*self._axes(), *other._axes())
        delta = (
            float(other.center_xy[0]) - float(self.center_xy[0]),
            float(other.center_xy[1]) - float(self.center_xy[1]),
        )
        for axis in axes:
            separation = abs(delta[0] * axis[0] + delta[1] * axis[1])
            if separation > self._projection_radius(axis) + other._projection_radius(axis) + 1e-9:
                return False
        return True

    def _axes(self) -> tuple[tuple[float, float], tuple[float, float]]:
        forward = (math.cos(float(self.yaw)), math.sin(float(self.yaw)))
        return forward, (-forward[1], forward[0])

    def _projection_radius(self, axis: tuple[float, float]) -> float:
        forward, lateral = self._axes()
        return (
            self.half_length * abs(forward[0] * axis[0] + forward[1] * axis[1])
            + self.half_width * abs(lateral[0] * axis[0] + lateral[1] * axis[1])
        )


@dataclass(frozen=True)
class SmartObjectSlot:
    slot_id: str
    pose: tuple[float, float, float]
    occupancy_region: OrientedRegion | None = None
    passage_region: OrientedRegion | None = None

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


class PassageStatus(str, Enum):
    ACQUIRED = "acquired"
    BLOCKED = "blocked"


@dataclass(frozen=True)
class PassageResult:
    status: PassageStatus
    object_id: str
    agent_id: str
    direction: str
    blocked_by_agent_id: str | None = None
    blocked_by_object_id: str | None = None


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
        self._occupancy_conflicts: dict[str, set[str]] = {}
        self._passage_conflicts: dict[str, set[str]] = {}
        self._passage_by_object: dict[str, PassageResult] = {}
        self._passage_by_agent: dict[str, PassageResult] = {}
        self._occupied_agents: set[str] = set()
        for smart_object in objects:
            self.register(smart_object)

    def register(self, smart_object: SmartObject) -> None:
        if smart_object.object_id in self._objects:
            raise ValueError(f"duplicate Smart Object: {smart_object.object_id}")
        self._objects[smart_object.object_id] = smart_object
        self._owners[smart_object.object_id] = {}
        self._queues[smart_object.object_id] = []
        self._occupancy_conflicts[smart_object.object_id] = set()
        self._passage_conflicts[smart_object.object_id] = set()
        for other_id, other in self._objects.items():
            if other_id == smart_object.object_id:
                continue
            if self._objects_overlap(smart_object, other, passage=False):
                self._occupancy_conflicts[smart_object.object_id].add(other_id)
                self._occupancy_conflicts[other_id].add(smart_object.object_id)
            if self._objects_overlap(smart_object, other, passage=True):
                self._passage_conflicts[smart_object.object_id].add(other_id)
                self._passage_conflicts[other_id].add(smart_object.object_id)

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
        self._passage_by_object = {}
        self._passage_by_agent = {}
        self._occupied_agents = set()

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

        blocked_by_occupancy = any(
            self._owners[conflicting_id]
            for conflicting_id in self._occupancy_conflicts[object_id]
        )
        if blocked_by_occupancy:
            return ClaimResult(
                ClaimStatus.REJECTED,
                object_id,
                agent_id,
                reason="oriented_occupancy_conflict",
            )
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

    def occupancy_conflicts(self, object_id: str) -> tuple[str, ...]:
        self.get(object_id)
        return tuple(sorted(self._occupancy_conflicts[object_id]))

    def passage_conflicts(self, object_id: str) -> tuple[str, ...]:
        self.get(object_id)
        return tuple(sorted(self._passage_conflicts[object_id]))

    def passage_staging_pose(self, object_id: str) -> tuple[float, float, float] | None:
        """Return the outer centerline of the object's local passage corridor."""
        smart_object = self.get(object_id)
        for slot in smart_object.interaction_slots:
            region = slot.passage_region
            if region is None:
                continue
            return (
                float(region.center_xy[0]) - math.cos(region.yaw) * region.half_length,
                float(region.center_xy[1]) - math.sin(region.yaw) * region.half_length,
                float(region.yaw),
            )
        return None

    def passage_holding_pose(
        self,
        object_id: str,
        *,
        offset_m: float,
    ) -> tuple[float, float, float] | None:
        """Return a waiting point behind the passage, outside its tokenized area."""
        staging = self.passage_staging_pose(object_id)
        if staging is None:
            return None
        offset = max(0.0, float(offset_m))
        return (
            float(staging[0]) - math.cos(float(staging[2])) * offset,
            float(staging[1]) - math.sin(float(staging[2])) * offset,
            float(staging[2]),
        )

    def requires_passage_arbitration(self, object_id: str) -> bool:
        smart_object = self.get(object_id)
        return any(
            slot.passage_region is not None
            for slot in smart_object.interaction_slots
        )

    def mark_occupied(self, agent_id: str) -> None:
        claim = self.claim_for_agent(agent_id)
        if claim is None or claim.status != ClaimStatus.ACQUIRED:
            raise ValueError(f"{agent_id} has no acquired Smart Object claim")
        self._occupied_agents.add(agent_id)

    def mark_vacated(self, agent_id: str) -> None:
        self._occupied_agents.discard(agent_id)

    def acquire_passage(
        self,
        object_id: str,
        agent_id: str,
        *,
        direction: str,
    ) -> PassageResult:
        self.get(object_id)
        if direction not in {"ingress", "egress"}:
            raise ValueError("passage direction must be ingress or egress")
        existing = self._passage_by_agent.get(agent_id)
        if existing is not None:
            if existing.object_id == object_id and existing.direction == direction:
                return existing
            return PassageResult(
                PassageStatus.BLOCKED,
                object_id,
                agent_id,
                direction,
                blocked_by_agent_id=existing.agent_id,
                blocked_by_object_id=existing.object_id,
            )
        # Ingress is independent across resources so waiters are not parked in
        # the shared aisle. Egress below briefly serializes overlapping lanes
        # while bodies back away and turn; continuous local-motion geometry
        # remains responsible for all other cross-object separation.
        conflict_ids = {object_id}
        if direction == "egress":
            # Adjacent users may approach and use their resources in parallel,
            # but two bodies turning/backing out of overlapping station lanes
            # need a short deterministic order.
            conflict_ids.update(self._passage_conflicts[object_id])
        blocker = next(
            (
                self._passage_by_object[conflict_id]
                for conflict_id in sorted(conflict_ids)
                if conflict_id in self._passage_by_object
            ),
            None,
        )
        if blocker is not None:
            return PassageResult(
                PassageStatus.BLOCKED,
                object_id,
                agent_id,
                direction,
                blocked_by_agent_id=blocker.agent_id,
                blocked_by_object_id=blocker.object_id,
            )
        result = PassageResult(
            PassageStatus.ACQUIRED,
            object_id,
            agent_id,
            direction,
        )
        self._passage_by_object[object_id] = result
        self._passage_by_agent[agent_id] = result
        return result

    def release_passage(self, agent_id: str) -> bool:
        result = self._passage_by_agent.pop(agent_id, None)
        if result is None:
            return False
        self._passage_by_object.pop(result.object_id, None)
        return True

    def passage_for_agent(self, agent_id: str) -> PassageResult | None:
        return self._passage_by_agent.get(agent_id)

    def release(self, claim_id: str, *, reason: str | None = None) -> ReleaseResult:
        del reason  # Reserved for audit adapters; registry semantics do not depend on it.
        claim = self._claims.get(claim_id)
        if claim is None:
            return ReleaseResult(False, "", "")
        smart_object = self._objects[claim.object_id]
        self._occupied_agents.discard(claim.agent_id)
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

    @staticmethod
    def _objects_overlap(
        first: SmartObject,
        second: SmartObject,
        *,
        passage: bool,
    ) -> bool:
        first_occupancy = tuple(
            region
            for slot in first.interaction_slots
            if (region := slot.occupancy_region) is not None
        )
        second_occupancy = tuple(
            region
            for slot in second.interaction_slots
            if (region := slot.occupancy_region) is not None
        )
        if not passage:
            return any(
                a.intersects(b)
                for a in first_occupancy
                for b in second_occupancy
            )
        first_passage = tuple(
            region
            for slot in first.interaction_slots
            if (region := slot.passage_region) is not None
        )
        second_passage = tuple(
            region
            for slot in second.interaction_slots
            if (region := slot.passage_region) is not None
        )
        return any(
            a.intersects(b)
            for first_regions, second_regions in (
                (first_passage, second_passage),
                (first_passage, second_occupancy),
                (first_occupancy, second_passage),
            )
            for a in first_regions
            for b in second_regions
        )
