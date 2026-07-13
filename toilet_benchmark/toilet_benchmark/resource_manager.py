from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class QueueSlot:
    slot_id: str
    position: list[float]
    yaw: float = 0.0
    occupant_id: str | None = None


@dataclass
class Resource:
    resource_id: str
    category: str
    position: list[float]
    yaw: float = 0.0
    scene_prim: str = ""
    queue_slots: list[QueueSlot] = field(default_factory=list)
    occupied_by: str | None = None


class ResourceManager:
    """Minimal placeholder for toilet resource ownership and queue bookkeeping."""

    def __init__(self, resources: dict[str, Resource]):
        self.resources = resources

    def acquire_or_queue(
        self,
        agent_id: str,
        category: str,
        candidate_ids: list[str] | None = None,
    ) -> tuple[str | None, str]:
        if candidate_ids:
            candidates = [
                self.resources[resource_id]
                for resource_id in candidate_ids
                if resource_id in self.resources and self.resources[resource_id].category == category
            ]
        else:
            candidates = [res for res in self.resources.values() if res.category == category]
        if not candidates:
            return None, "missing_resource"

        # Free resource wins immediately.
        for resource in candidates:
            if resource.occupied_by is None and all(slot.occupant_id is None for slot in resource.queue_slots):
                resource.occupied_by = agent_id
                return resource.resource_id, "resource"

        # Otherwise choose the shortest queue with an empty slot.
        queueable = []
        for resource in candidates:
            queue_len = sum(1 for slot in resource.queue_slots if slot.occupant_id is not None)
            has_space = any(slot.occupant_id is None for slot in resource.queue_slots)
            if has_space:
                queueable.append((queue_len, resource))
        if not queueable:
            return None, "queue_full"

        queueable.sort(key=lambda item: item[0])
        resource = queueable[0][1]
        for slot in resource.queue_slots:
            if slot.occupant_id is None:
                slot.occupant_id = agent_id
                return resource.resource_id, slot.slot_id
        return None, "queue_full"

    def release(self, agent_id: str, resource_id: str) -> str | None:
        resource = self.resources.get(resource_id)
        if resource is None:
            return None
        if resource.occupied_by != agent_id:
            for slot in resource.queue_slots:
                if slot.occupant_id == agent_id:
                    slot.occupant_id = None
                    break
            return None

        resource.occupied_by = None
        promoted = None
        for slot in resource.queue_slots:
            if slot.occupant_id:
                promoted = slot.occupant_id
                resource.occupied_by = promoted
                slot.occupant_id = None
                break
        return promoted

    def resource_pose(self, resource_id: str) -> list[float]:
        return list(self.resources[resource_id].position)

    def resource_yaw(self, resource_id: str) -> float:
        return float(self.resources[resource_id].yaw)

    def queue_slot_pose(self, resource_id: str, slot_id: str) -> list[float] | None:
        resource = self.resources.get(resource_id)
        if resource is None:
            return None
        for slot in resource.queue_slots:
            if slot.slot_id == slot_id:
                return list(slot.position)
        return None

    def queue_slot_yaw(self, resource_id: str, slot_id: str) -> float | None:
        resource = self.resources.get(resource_id)
        if resource is None:
            return None
        for slot in resource.queue_slots:
            if slot.slot_id == slot_id:
                return float(slot.yaw)
        return None

    def snapshot(self) -> dict[str, dict]:
        return {
            key: {
                "category": value.category,
                "scene_prim": value.scene_prim,
                "occupied_by": value.occupied_by,
                "queue": [slot.occupant_id for slot in value.queue_slots],
            }
            for key, value in self.resources.items()
        }
