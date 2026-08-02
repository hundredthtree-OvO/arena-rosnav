"""Runtime storage helpers extracted for HuNav per-agent session isolation."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from typing import Any, Iterator, Mapping


RUNTIME_FIELDS = (
    "_command",
    "_shadow",
    "_generation",
    "_active_generation",
    "_hunav_active",
    "_settled_generation",
    "_settled_agent_id",
    "_terminal_align_active",
    "_terminal_align_stable_since",
    "_frozen_present",
    "_lookahead_tracker",
    "_lookahead_target",
    "_last_visible_route_target",
    "_route_visibility_lost_since",
    "_avoidance_active_route_infeasible_ticks",
    "_avoidance_recovery_attempts",
    "_avoidance_side",
    "_avoidance_target",
    "_avoidance_encounter_active",
    "_last_avoidance_candidates",
    "_local_route_mode",
    "_local_route_points",
    "_local_route_index",
    "_local_route_best_distance",
    "_local_route_last_progress_at",
    "_avoidance_failed_side",
    "_static_hold_yaw",
    "_static_clear_ticks",
    "_last_global_replan_at",
    "_pending_rebase_replan",
    "_yield_hold_yaw",
    "_pedestrian_yield_blocker",
    "_pedestrian_yield_clear_ticks",
    "_route_hold_active",
    "_route_hold_yaw",
    "_last_external_motion_mode",
    "_reaction_controller",
    "_reaction_decision",
    "_external_future",
    "_external_future_started_at",
    "_external_retry_after",
    "_pending_external_command",
    "_external_request_token",
    "_last_state_wait_log",
    "_last_diagnostic_log",
    "_last_native_behavior_signature",
    "_last_local_motion_shadow",
)


class HuNavRuntimeRegistry:
    """Small per-agent runtime store preserving isolated mutable slices."""

    def __init__(self, runtime_fields: tuple[str, ...] = RUNTIME_FIELDS):
        self.runtime_fields = tuple(runtime_fields)
        self.runtime_states: dict[str, dict[str, Any]] = {}
        self.commands: dict[str, Any] = {}
        self._runtime_template: dict[str, Any] = {}
        self._active_agent_id: str | None = None

    def ensure_runtime_template(self, owner: object) -> None:
        if not self._runtime_template:
            self._runtime_template = self._capture(owner)

    def capture(self, owner: object) -> dict[str, Any]:
        return self._capture(owner)

    def _capture(self, owner: object) -> dict[str, Any]:
        return {
            field: getattr(owner, field)
            for field in self.runtime_fields
            if hasattr(owner, field)
        }

    def capture_active(self, owner: object) -> None:
        agent_id = self._active_agent_id
        if agent_id is None:
            return
        self.runtime_states[agent_id] = self._capture(owner)
        command = self.runtime_states[agent_id].get("_command")
        if command is None:
            self.commands.pop(agent_id, None)
        else:
            self.commands[agent_id] = command

    def restore(self, owner: object, agent_id: str) -> None:
        state = self.runtime_states[str(agent_id)]
        for field, value in state.items():
            setattr(owner, field, value)
        self._active_agent_id = str(agent_id)

    @contextmanager
    def context(self, owner: object, agent_id: str) -> Iterator[None]:
        self.ensure_runtime_template(owner)
        previous = self._active_agent_id
        self.capture_active(owner)
        self.restore(owner, agent_id)
        try:
            yield
        finally:
            self.capture_active(owner)
            if previous is not None and previous in self.runtime_states:
                self.restore(owner, previous)
            else:
                self._active_agent_id = None

    def create(
        self,
        owner: object,
        agent_id: str,
        *,
        reaction_seed: int,
        runtime_initializer,
        configure: Mapping[str, Any] | None = None,
    ) -> None:
        agent_id = str(agent_id)
        state = deepcopy(self._runtime_template)
        runtime_initializer(owner, state, reaction_seed=reaction_seed, **dict(configure or {}))
        self.runtime_states[agent_id] = state

    def select(self, owner: object, agent_id: str | None) -> None:
        self.capture_active(owner)
        if agent_id is None or str(agent_id) not in self.runtime_states:
            self._active_agent_id = None
            owner._command = None
            owner._shadow = None
            return
        self.restore(owner, str(agent_id))

    def remove(self, owner: object, agent_id: str) -> None:
        if str(agent_id) not in self.runtime_states:
            return
        self.capture_active(owner)
        removed = self.runtime_states.pop(str(agent_id), None)
        self.commands.pop(str(agent_id), None)
        if removed is not None and self._active_agent_id == str(agent_id):
            self._active_agent_id = None
            owner._runtime_agent_id = None

    def active_agent_id(self) -> str | None:
        return self._active_agent_id

    @property
    def runtime_count(self) -> int:
        return len(self.runtime_states)
