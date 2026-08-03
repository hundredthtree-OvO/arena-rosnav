"""Per-agent EnterUseExit task executive."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from toilet_benchmark.domain.events import BenchmarkEvent
from toilet_benchmark.domain.intent import DirectiveType, TaskDirective
from toilet_benchmark.domain.task import TaskPhase
from toilet_benchmark.domain.world import AgentTaskFeedback
from toilet_benchmark.episodes.schema import PedestrianEpisodeSpec

from .smart_objects import ClaimStatus, PassageStatus, SmartObjectRegistry
from .templates import EnterUseExit


class ExecutiveState(str, Enum):
    SEEK_OBJECT = "seek_object"
    WAIT_OBJECT = "wait_object"
    APPROACH_PASSAGE = "approach_passage"
    WAIT_INGRESS = "wait_ingress"
    APPROACH_OBJECT = "approach_object"
    INTERACT = "interact"
    WAIT_EGRESS = "wait_egress"
    CLEAR_PASSAGE = "clear_passage"
    SEEK_EXIT = "seek_exit"
    RETIRED = "retired"


@dataclass(frozen=True)
class ExecutiveTickResult:
    directive: TaskDirective
    events: tuple[BenchmarkEvent, ...] = ()


class AgentExecutive:
    def __init__(
        self,
        spec: PedestrianEpisodeSpec,
        template: EnterUseExit,
        *,
        episode_id: str,
    ):
        self.spec = spec
        self.template = template
        self.episode_id = episode_id
        self.reset()

    def reset(self) -> None:
        self.state = ExecutiveState.SEEK_OBJECT
        self.claim_id: str | None = None
        self.activity_started_at: float | None = None

    def tick(
        self,
        registry: SmartObjectRegistry,
        feedback: AgentTaskFeedback,
        timestamp_sec: float,
    ) -> ExecutiveTickResult:
        events: list[BenchmarkEvent] = []

        if self.state == ExecutiveState.SEEK_OBJECT:
            claim = registry.claim(self.spec.semantic_goal, self.spec.agent_id)
            if claim.status == ClaimStatus.REJECTED:
                return ExecutiveTickResult(
                    self._directive(DirectiveType.WAIT_AT_QUEUE, TaskPhase.QUEUEING),
                    (self._event("smart_object_unavailable", timestamp_sec, reason=claim.reason),),
                )
            self.claim_id = claim.claim_id
            if claim.status == ClaimStatus.QUEUED:
                self.state = ExecutiveState.WAIT_OBJECT
                events.append(
                    self._event(
                        "smart_object_queued",
                        timestamp_sec,
                        slot_id=claim.slot_id,
                        queue_index=claim.queue_index,
                    )
                )
            else:
                events.append(self._event("smart_object_claimed", timestamp_sec, slot_id=claim.slot_id))
                events.extend(self._admit_or_hold_ingress(registry, timestamp_sec))

        elif self.state == ExecutiveState.WAIT_OBJECT:
            claim = registry.claim_for_agent(self.spec.agent_id)
            if claim is not None and claim.status == ClaimStatus.ACQUIRED:
                events.append(self._event("smart_object_promoted", timestamp_sec, slot_id=claim.slot_id))
                events.extend(self._admit_or_hold_ingress(registry, timestamp_sec))

        elif self.state == ExecutiveState.APPROACH_PASSAGE:
            passage_target_id = f"{self.spec.semantic_goal}:passage"
            if feedback.reached_target_id == passage_target_id:
                passage = registry.acquire_passage(
                    self.spec.semantic_goal,
                    self.spec.agent_id,
                    direction="ingress",
                )
                if passage.status == PassageStatus.ACQUIRED:
                    self.state = ExecutiveState.APPROACH_OBJECT
                else:
                    self.state = ExecutiveState.WAIT_INGRESS
                    events.append(
                        self._event(
                            "smart_object_ingress_blocked",
                            timestamp_sec,
                            blocked_by_agent_id=passage.blocked_by_agent_id,
                            blocked_by_object_id=passage.blocked_by_object_id,
                        )
                    )

        elif self.state == ExecutiveState.WAIT_INGRESS:
            passage = registry.acquire_passage(
                self.spec.semantic_goal,
                self.spec.agent_id,
                direction="ingress",
            )
            if passage.status == PassageStatus.ACQUIRED:
                self.state = ExecutiveState.APPROACH_OBJECT
                if feedback.reached_target_id == self.spec.semantic_goal:
                    events.extend(
                        self._begin_interaction(registry, timestamp_sec)
                    )

        elif self.state == ExecutiveState.APPROACH_OBJECT:
            if feedback.reached_target_id == self.spec.semantic_goal:
                events.extend(self._begin_interaction(registry, timestamp_sec))

        elif self.state == ExecutiveState.INTERACT:
            duration_complete = (
                self.template.activity_duration_sec is not None
                and self.activity_started_at is not None
                and timestamp_sec - self.activity_started_at
                >= self.template.activity_duration_sec
            )
            if feedback.activity_complete or duration_complete:
                events.append(self._event("activity_completed", timestamp_sec))
                if not registry.requires_passage_arbitration(self.spec.semantic_goal):
                    events.extend(self._release_object_for_exit(registry, timestamp_sec))
                    return ExecutiveTickResult(
                        self._current_directive(registry), tuple(events)
                    )
                passage = registry.acquire_passage(
                    self.spec.semantic_goal,
                    self.spec.agent_id,
                    direction="egress",
                )
                if passage.status == PassageStatus.ACQUIRED:
                    events.extend(self._release_object_for_passage(registry, timestamp_sec))
                else:
                    self.state = ExecutiveState.WAIT_EGRESS
                    events.append(
                        self._event(
                            "smart_object_egress_blocked",
                            timestamp_sec,
                            blocked_by_agent_id=passage.blocked_by_agent_id,
                            blocked_by_object_id=passage.blocked_by_object_id,
                        )
                    )

        elif self.state == ExecutiveState.WAIT_EGRESS:
            passage = registry.acquire_passage(
                self.spec.semantic_goal,
                self.spec.agent_id,
                direction="egress",
            )
            if passage.status == PassageStatus.ACQUIRED:
                events.extend(self._release_object_for_passage(registry, timestamp_sec))

        elif self.state == ExecutiveState.CLEAR_PASSAGE:
            passage_target_id = f"{self.spec.semantic_goal}:passage:egress"
            if feedback.reached_target_id == passage_target_id:
                registry.release_passage(self.spec.agent_id)
                self.state = ExecutiveState.SEEK_EXIT
                events.append(self._event("smart_object_passage_cleared", timestamp_sec))

        elif self.state == ExecutiveState.SEEK_EXIT:
            if feedback.reached_target_id == self.template.exit_target_id:
                registry.release_passage(self.spec.agent_id)
                self.state = ExecutiveState.RETIRED
                events.append(self._event("agent_completed", timestamp_sec))

        return ExecutiveTickResult(self._current_directive(registry), tuple(events))

    def cancel(
        self,
        registry: SmartObjectRegistry,
        timestamp_sec: float,
        *,
        reason: str,
    ) -> ExecutiveTickResult:
        events: list[BenchmarkEvent] = []
        if self.state != ExecutiveState.RETIRED:
            registry.release_passage(self.spec.agent_id)
            if self.claim_id is not None:
                released = registry.release(self.claim_id, reason=reason)
                if released.released:
                    events.append(
                        self._event(
                            "smart_object_released",
                            timestamp_sec,
                            promoted_agent_id=released.promoted_agent_id,
                            reason=reason,
                        )
                    )
            self.claim_id = None
            self.state = ExecutiveState.RETIRED
            events.append(self._event("agent_cancelled", timestamp_sec, reason=reason))
        return ExecutiveTickResult(self._current_directive(registry), tuple(events))

    def _current_directive(self, registry: SmartObjectRegistry) -> TaskDirective:
        if self.state == ExecutiveState.APPROACH_PASSAGE:
            return self._directive(
                DirectiveType.MOVE_TO_PASSAGE,
                TaskPhase.WALK_TO_URINAL,
                target_id=f"{self.spec.semantic_goal}:passage",
            )
        if self.state == ExecutiveState.WAIT_INGRESS:
            return self._directive(
                DirectiveType.WAIT_FOR_PASSAGE,
                TaskPhase.QUEUEING,
            )
        if self.state == ExecutiveState.WAIT_OBJECT:
            claim = registry.claim_for_agent(self.spec.agent_id)
            return self._directive(
                DirectiveType.WAIT_AT_QUEUE,
                TaskPhase.QUEUEING,
                target_id=None if claim is None else claim.slot_id,
                slot_id=None if claim is None else claim.slot_id,
            )
        if self.state == ExecutiveState.APPROACH_OBJECT:
            claim = registry.claim_for_agent(self.spec.agent_id)
            return self._directive(
                DirectiveType.MOVE_TO_OBJECT,
                TaskPhase.WALK_TO_URINAL,
                target_id=self.spec.semantic_goal,
                slot_id=None if claim is None else claim.slot_id,
            )
        if self.state == ExecutiveState.INTERACT:
            return self._directive(
                DirectiveType.START_ACTIVITY,
                TaskPhase.USING_URINAL,
                target_id=self.spec.semantic_goal,
            )
        if self.state == ExecutiveState.WAIT_EGRESS:
            return self._directive(
                DirectiveType.START_ACTIVITY,
                TaskPhase.USING_URINAL,
                target_id=self.spec.semantic_goal,
            )
        if self.state == ExecutiveState.CLEAR_PASSAGE:
            return self._directive(
                DirectiveType.MOVE_TO_PASSAGE,
                TaskPhase.EXITING,
                target_id=f"{self.spec.semantic_goal}:passage:egress",
            )
        if self.state == ExecutiveState.SEEK_EXIT:
            return self._directive(
                DirectiveType.MOVE_TO_EXIT,
                TaskPhase.EXITING,
                target_id=self.template.exit_target_id,
            )
        if self.state == ExecutiveState.RETIRED:
            return self._directive(DirectiveType.RETIRE, TaskPhase.DONE)
        return self._directive(DirectiveType.WAIT_AT_QUEUE, TaskPhase.QUEUEING)

    def _directive(
        self,
        directive_type: DirectiveType,
        phase: TaskPhase,
        *,
        target_id: str | None = None,
        slot_id: str | None = None,
    ) -> TaskDirective:
        object_id = self.spec.semantic_goal if directive_type in {
            DirectiveType.MOVE_TO_PASSAGE,
            DirectiveType.MOVE_TO_OBJECT,
            DirectiveType.WAIT_AT_QUEUE,
            DirectiveType.WAIT_FOR_PASSAGE,
            DirectiveType.START_ACTIVITY,
        } else None
        return TaskDirective(
            agent_id=self.spec.agent_id,
            directive_type=directive_type,
            phase=phase,
            target_id=target_id,
            object_id=object_id,
            slot_id=slot_id,
        )

    def _admit_or_hold_ingress(
        self,
        registry: SmartObjectRegistry,
        timestamp_sec: float,
    ) -> tuple[BenchmarkEvent, ...]:
        if not registry.requires_passage_arbitration(self.spec.semantic_goal):
            self.state = ExecutiveState.APPROACH_OBJECT
            return ()
        passage = registry.acquire_passage(
            self.spec.semantic_goal,
            self.spec.agent_id,
            direction="ingress",
        )
        if passage.status == PassageStatus.ACQUIRED:
            self.state = ExecutiveState.APPROACH_OBJECT
            return ()
        self.state = ExecutiveState.APPROACH_PASSAGE
        return (
            self._event(
                "smart_object_ingress_deferred",
                timestamp_sec,
                blocked_by_agent_id=passage.blocked_by_agent_id,
                blocked_by_object_id=passage.blocked_by_object_id,
            ),
        )

    def _event(self, event_type: str, timestamp_sec: float, **payload) -> BenchmarkEvent:
        return BenchmarkEvent(
            event_type=event_type,
            timestamp_sec=timestamp_sec,
            episode_id=self.episode_id,
            agent_id=self.spec.agent_id,
            phase=self._phase_for_state(),
            payload={
                "object_id": self.spec.semantic_goal,
                **{key: value for key, value in payload.items() if value is not None},
            },
        )

    def _release_object_for_passage(
        self,
        registry: SmartObjectRegistry,
        timestamp_sec: float,
    ) -> tuple[BenchmarkEvent, ...]:
        events: list[BenchmarkEvent] = []
        if self.claim_id is not None:
            released = registry.release(self.claim_id)
            if released.released:
                events.append(
                    self._event(
                        "smart_object_released",
                        timestamp_sec,
                        promoted_agent_id=released.promoted_agent_id,
                    )
                )
        self.claim_id = None
        self.state = ExecutiveState.CLEAR_PASSAGE
        return tuple(events)

    def _release_object_for_exit(
        self,
        registry: SmartObjectRegistry,
        timestamp_sec: float,
    ) -> tuple[BenchmarkEvent, ...]:
        events = self._release_object_for_passage(registry, timestamp_sec)
        self.state = ExecutiveState.SEEK_EXIT
        return events

    def _begin_interaction(
        self,
        registry: SmartObjectRegistry,
        timestamp_sec: float,
    ) -> tuple[BenchmarkEvent, ...]:
        registry.mark_occupied(self.spec.agent_id)
        registry.release_passage(self.spec.agent_id)
        self.state = ExecutiveState.INTERACT
        self.activity_started_at = timestamp_sec
        return (
            self._event("smart_object_reached", timestamp_sec),
            self._event("activity_started", timestamp_sec),
        )

    def _phase_for_state(self) -> TaskPhase:
        return {
            ExecutiveState.SEEK_OBJECT: TaskPhase.ACTIVATING,
            ExecutiveState.WAIT_OBJECT: TaskPhase.QUEUEING,
            ExecutiveState.APPROACH_PASSAGE: TaskPhase.WALK_TO_URINAL,
            ExecutiveState.WAIT_INGRESS: TaskPhase.QUEUEING,
            ExecutiveState.APPROACH_OBJECT: TaskPhase.WALK_TO_URINAL,
            ExecutiveState.INTERACT: TaskPhase.USING_URINAL,
            ExecutiveState.WAIT_EGRESS: TaskPhase.USING_URINAL,
            ExecutiveState.CLEAR_PASSAGE: TaskPhase.EXITING,
            ExecutiveState.SEEK_EXIT: TaskPhase.EXITING,
            ExecutiveState.RETIRED: TaskPhase.DONE,
        }[self.state]
