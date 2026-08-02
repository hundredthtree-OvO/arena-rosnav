"""Compatibility boundary between the ROS director and ScenarioRuntime."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Mapping, Protocol

from toilet_benchmark.domain.events import BenchmarkEvent
from toilet_benchmark.domain.intent import DirectiveType, TaskDirective
from toilet_benchmark.domain.world import AgentTaskFeedback, WorldSnapshot
from toilet_benchmark.episodes.schema import (
    EpisodeSpec,
    PedestrianEpisodeSpec,
    RobotEpisodeSpec,
    TerminationSpec,
    TrackType,
)

from .runtime import RuntimeTickResult, ScenarioRuntime
from .smart_objects import ObjectKind, SmartObject, SmartObjectRegistry, SmartObjectSlot
from .templates import EnterUseExit


class ScenarioRuntimeMode(str, Enum):
    LEGACY = "legacy"
    SHADOW = "shadow"
    TAKEOVER = "takeover"


@dataclass(frozen=True)
class ScenarioAgentAssignment:
    agent_id: str
    object_id: str


@dataclass(frozen=True)
class ShadowMismatch:
    agent_id: str
    legacy_phase: str
    directive_type: DirectiveType
    reason: str


class LegacyQueueSlot(Protocol):
    slot_id: str
    position: list[float]
    yaw: float
    occupant_id: str | None


class LegacyResource(Protocol):
    resource_id: str
    category: str
    position: list[float]
    yaw: float
    queue_slots: list[LegacyQueueSlot]
    occupied_by: str | None


class LegacyResourceManager(Protocol):
    resources: dict[str, LegacyResource]


class DirectorScenarioAdapter:
    """Own semantic runtime state while legacy objects remain motion-facing mirrors."""

    def __init__(
        self,
        resources: Iterable[SmartObject],
        *,
        exit_target_id: str,
        mode: ScenarioRuntimeMode | str = ScenarioRuntimeMode.TAKEOVER,
    ):
        self.mode = mode if isinstance(mode, ScenarioRuntimeMode) else ScenarioRuntimeMode(mode)
        self.runtime = ScenarioRuntime(
            SmartObjectRegistry(resources),
            EnterUseExit(exit_target_id=exit_target_id, activity_duration_sec=None),
        )
        self._directives: dict[str, TaskDirective] = {}
        self._started = False

    @classmethod
    def from_legacy_resources(
        cls,
        manager: LegacyResourceManager,
        *,
        exit_target_id: str,
        mode: ScenarioRuntimeMode | str = ScenarioRuntimeMode.TAKEOVER,
    ) -> "DirectorScenarioAdapter":
        objects = []
        for resource in manager.resources.values():
            kind = {
                "urinals": ObjectKind.URINAL,
                "stalls": ObjectKind.TOILET_STALL,
            }.get(resource.category, ObjectKind.WAITING_AREA)
            objects.append(
                SmartObject(
                    object_id=resource.resource_id,
                    kind=kind,
                    interaction_slots=(
                        SmartObjectSlot(
                            f"{resource.resource_id}:interaction",
                            (
                                float(resource.position[0]),
                                float(resource.position[1]),
                                float(resource.yaw),
                            ),
                        ),
                    ),
                    queue_slots=tuple(
                        SmartObjectSlot(
                            slot.slot_id,
                            (
                                float(slot.position[0]),
                                float(slot.position[1]),
                                float(slot.yaw),
                            ),
                        )
                        for slot in resource.queue_slots
                    ),
                    tags=frozenset((resource.category,)),
                )
            )
        return cls(objects, exit_target_id=exit_target_id, mode=mode)

    def reset(
        self,
        assignments: Iterable[ScenarioAgentAssignment],
        *,
        episode_id: str,
        seed: int,
        timestamp_sec: float,
    ) -> RuntimeTickResult:
        pedestrians = tuple(
            PedestrianEpisodeSpec(
                agent_id=assignment.agent_id,
                semantic_goal=assignment.object_id,
            )
            for assignment in assignments
        )
        episode = EpisodeSpec(
            episode_id=episode_id,
            scene_id="runtime_scene",
            task_type="enter_use_exit",
            track=TrackType.INTERACTIVE,
            seed=int(seed),
            robot=RobotEpisodeSpec(
                model="runtime_robot",
                start_pose=(0.0, 0.0, 0.0, 0.0),
                goal_pose=(0.0, 0.0, 0.0),
            ),
            pedestrians=pedestrians,
            termination=TerminationSpec(timeout_sec=0.0, goal_tolerance_m=0.0),
        )
        self.runtime.reset(episode)
        self._started = True
        result = self.runtime.tick(WorldSnapshot(timestamp_sec))
        self._remember(result)
        return result

    def advance(
        self,
        timestamp_sec: float,
        feedback: Mapping[str, AgentTaskFeedback] | None = None,
    ) -> RuntimeTickResult:
        self._require_started()
        result = self.runtime.tick(
            WorldSnapshot(timestamp_sec, task_feedback=dict(feedback or {}))
        )
        self._remember(result)
        return result

    def cancel_agent(
        self,
        agent_id: str,
        timestamp_sec: float,
        *,
        reason: str,
    ) -> RuntimeTickResult:
        self._require_started()
        result = self.runtime.cancel_agent(agent_id, timestamp_sec, reason=reason)
        self._remember(result)
        return result

    def directive_for(self, agent_id: str) -> TaskDirective | None:
        return self._directives.get(agent_id)

    def sync_legacy_resources(self, manager: LegacyResourceManager) -> None:
        snapshot = self.runtime.registry.snapshot()
        for resource_id, legacy in manager.resources.items():
            state = snapshot.get(resource_id, {"owners": {}, "queue": []})
            owners = list(dict(state["owners"]).values())
            legacy.occupied_by = owners[0] if owners else None
            queued_agents = list(state["queue"])
            for index, slot in enumerate(legacy.queue_slots):
                slot.occupant_id = queued_agents[index] if index < len(queued_agents) else None

    def compare_legacy_phases(self, phases: Mapping[str, str]) -> tuple[ShadowMismatch, ...]:
        mismatches = []
        for agent_id, legacy_phase in phases.items():
            directive = self._directives.get(agent_id)
            if directive is None:
                continue
            allowed = _LEGACY_PHASES_BY_DIRECTIVE[directive.directive_type]
            if str(legacy_phase) not in allowed:
                mismatches.append(
                    ShadowMismatch(
                        agent_id=agent_id,
                        legacy_phase=str(legacy_phase),
                        directive_type=directive.directive_type,
                        reason="legacy phase does not match semantic directive",
                    )
                )
        return tuple(mismatches)

    def compare_legacy_resources(
        self,
        manager: LegacyResourceManager,
    ) -> tuple[ShadowMismatch, ...]:
        semantic = self.runtime.registry.snapshot()
        mismatches = []
        for object_id, legacy in manager.resources.items():
            state = semantic.get(object_id, {"owners": {}, "queue": []})
            owners = list(dict(state["owners"]).values())
            semantic_owner = owners[0] if owners else None
            if legacy.occupied_by != semantic_owner:
                mismatches.append(
                    ShadowMismatch(
                        agent_id=str(legacy.occupied_by or semantic_owner or ""),
                        legacy_phase="RESOURCE",
                        directive_type=DirectiveType.WAIT_AT_QUEUE,
                        reason=(
                            f"owner mismatch for {object_id}: "
                            f"legacy={legacy.occupied_by}, scenario={semantic_owner}"
                        ),
                    )
                )
            legacy_queue = [
                slot.occupant_id
                for slot in legacy.queue_slots
                if slot.occupant_id is not None
            ]
            semantic_queue = list(state["queue"])
            if legacy_queue != semantic_queue:
                mismatches.append(
                    ShadowMismatch(
                        agent_id=str((legacy_queue or semantic_queue or [""])[0]),
                        legacy_phase="RESOURCE",
                        directive_type=DirectiveType.WAIT_AT_QUEUE,
                        reason=(
                            f"queue mismatch for {object_id}: "
                            f"legacy={legacy_queue}, scenario={semantic_queue}"
                        ),
                    )
                )
        return tuple(mismatches)

    def _remember(self, result: RuntimeTickResult) -> None:
        self._directives.update(
            (directive.agent_id, directive) for directive in result.directives
        )

    def _require_started(self) -> None:
        if not self._started:
            raise RuntimeError("DirectorScenarioAdapter.reset() must be called first")


_LEGACY_PHASES_BY_DIRECTIVE = {
    DirectiveType.MOVE_TO_OBJECT: frozenset(
        {
            "ACTIVATING",
            "WALK_TO_ENTRY_CLEARANCE",
            "WALK_TO_URINAL",
            "QUEUEING",
            "FINAL_ALIGN_URINAL",
        }
    ),
    DirectiveType.WAIT_AT_QUEUE: frozenset(
        {"ACTIVATING", "WALK_TO_ENTRY_CLEARANCE", "QUEUEING"}
    ),
    DirectiveType.START_ACTIVITY: frozenset({"USING_URINAL"}),
    DirectiveType.MOVE_TO_EXIT: frozenset({"WALK_TO_EXIT_STAGING", "EXITING"}),
    DirectiveType.RETIRE: frozenset({"DESPAWNING", "DONE"}),
}


def scenario_events(result: RuntimeTickResult) -> tuple[BenchmarkEvent, ...]:
    return result.events
