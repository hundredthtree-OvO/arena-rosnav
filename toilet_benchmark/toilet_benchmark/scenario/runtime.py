"""Multi-agent scenario runtime with deterministic tick ordering."""

from __future__ import annotations

from dataclasses import dataclass

from toilet_benchmark.domain.events import BenchmarkEvent
from toilet_benchmark.domain.intent import TaskDirective
from toilet_benchmark.domain.world import WorldSnapshot
from toilet_benchmark.episodes.schema import EpisodeSpec

from .executive import AgentExecutive, ExecutiveState
from .smart_objects import ClaimStatus, SmartObjectRegistry
from .templates import EnterUseExit


@dataclass(frozen=True)
class RuntimeTickResult:
    directives: tuple[TaskDirective, ...]
    events: tuple[BenchmarkEvent, ...]
    completed: bool


class ScenarioRuntime:
    def __init__(self, registry: SmartObjectRegistry, template: EnterUseExit):
        self.registry = registry
        self.template = template
        self.episode: EpisodeSpec | None = None
        self.executives: dict[str, AgentExecutive] = {}

    def reset(self, episode: EpisodeSpec) -> None:
        self.registry.reset()
        self.episode = episode
        self.executives = {
            pedestrian.agent_id: AgentExecutive(
                pedestrian,
                self.template,
                episode_id=episode.episode_id,
            )
            for pedestrian in episode.pedestrians
        }

    def tick(self, snapshot: WorldSnapshot) -> RuntimeTickResult:
        if self.episode is None:
            raise RuntimeError("ScenarioRuntime.reset() must be called before tick()")
        directives: dict[str, TaskDirective] = {}
        events: list[BenchmarkEvent] = []
        for agent_id in sorted(self.executives):
            result = self.executives[agent_id].tick(
                self.registry,
                snapshot.feedback_for(agent_id),
                snapshot.timestamp_sec,
            )
            directives[agent_id] = result.directive
            events.extend(result.events)
        # A release may promote an agent that was already ticked earlier in the
        # deterministic order. Refresh only those waiters in the same world tick.
        for agent_id in sorted(self.executives):
            executive = self.executives[agent_id]
            if executive.state != ExecutiveState.WAIT_OBJECT:
                continue
            claim = self.registry.claim_for_agent(agent_id)
            if claim is None or claim.status != ClaimStatus.ACQUIRED:
                continue
            result = executive.tick(
                self.registry,
                snapshot.feedback_for(agent_id),
                snapshot.timestamp_sec,
            )
            directives[agent_id] = result.directive
            events.extend(result.events)
        return RuntimeTickResult(
            directives=tuple(directives[agent_id] for agent_id in sorted(directives)),
            events=tuple(events),
            completed=bool(self.executives)
            and all(
                executive.state == ExecutiveState.RETIRED
                for executive in self.executives.values()
            ),
        )

    def cancel_agent(
        self,
        agent_id: str,
        timestamp_sec: float,
        *,
        reason: str,
    ) -> RuntimeTickResult:
        if self.episode is None:
            raise RuntimeError("ScenarioRuntime.reset() must be called before cancel_agent()")
        executive = self.executives.get(agent_id)
        if executive is None:
            raise KeyError(f"unknown scenario agent: {agent_id}")
        cancelled = executive.cancel(
            self.registry,
            timestamp_sec,
            reason=reason,
        )
        refreshed = self.tick(WorldSnapshot(timestamp_sec))
        return RuntimeTickResult(
            directives=refreshed.directives,
            events=(*cancelled.events, *refreshed.events),
            completed=refreshed.completed,
        )
