"""Deterministic runtime reactions to robot proximity."""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
from typing import Mapping


REACTION_STATES = {
    "yielding": "YIELDING_TO_ROBOT",
    "impatient": "IMPATIENT_TO_ROBOT",
    "regular": "REGULAR_TO_ROBOT",
}


@dataclass(frozen=True)
class ReactionDecision:
    state: str
    reaction: str | None
    speed_scale: float
    distance_m: float | None
    ttc_sec: float | None
    transition: str | None = None


class RobotProximityReactionController:
    """Latch one reaction per proximity encounter and release with hysteresis."""

    def __init__(self, config: Mapping, *, seed: int):
        self._config = dict(config or {})
        self.enabled = bool(self._config.get("enabled", True))
        self.mode = str(self._config.get("mode", "fixed")).strip().lower()
        self.fixed_reaction = str(
            self._config.get("reaction", "yielding")
        ).strip().lower()
        trigger = dict(self._config.get("trigger", {}) or {})
        release = dict(self._config.get("release", {}) or {})
        self.trigger_distance_m = max(
            0.0, float(trigger.get("distance_m", 0.85))
        )
        self.ttc_sec = max(0.0, float(trigger.get("time_to_collision_sec", 1.5)))
        self.front_half_angle_rad = math.radians(
            max(0.0, min(180.0, float(trigger.get("front_half_angle_deg", 180.0))))
        )
        self.release_distance_m = max(
            self.trigger_distance_m,
            float(release.get("distance_m", 1.10)),
        )
        self.release_stable_sec = max(0.0, float(release.get("stable_sec", 0.8)))
        self.reactions = dict(self._config.get("reactions", {}) or {})
        self._rng = random.Random(int(seed))
        self._reaction: str | None = None
        self._release_since = 0.0

    @property
    def reaction(self) -> str | None:
        return self._reaction

    def reset(self) -> None:
        self._reaction = None
        self._release_since = 0.0

    def force_reaction(self, reaction: str) -> None:
        """Override and latch the current encounter after feasibility checks."""
        self._reaction = self._validated_reaction(str(reaction).strip().lower())
        self._release_since = 0.0

    def update(
        self,
        *,
        agent: Mapping[str, float],
        robot: Mapping[str, float] | None,
        now: float,
    ) -> ReactionDecision:
        distance, ttc, in_front = self._geometry(agent, robot)
        transition = None
        triggered = (
            distance is not None
            and (
                distance <= self.trigger_distance_m
                or (in_front and ttc is not None and ttc <= self.ttc_sec)
            )
        )
        if not self.enabled:
            self.reset()
        elif self._reaction is None:
            if triggered:
                self._reaction = self._choose_reaction()
                transition = f"WALKING->{REACTION_STATES[self._reaction]}"
        else:
            release_ready = not triggered and (
                distance is None or distance >= self.release_distance_m
            )
            if release_ready:
                if self._release_since <= 0.0:
                    self._release_since = float(now)
                elif float(now) - self._release_since >= self.release_stable_sec:
                    previous = REACTION_STATES[self._reaction]
                    self._reaction = None
                    self._release_since = 0.0
                    transition = f"{previous}->WALKING"
            else:
                self._release_since = 0.0

        reaction = self._reaction
        return ReactionDecision(
            state=REACTION_STATES.get(reaction, "WALKING"),
            reaction=reaction,
            speed_scale=self._speed_scale(reaction),
            distance_m=distance,
            ttc_sec=ttc,
            transition=transition,
        )

    def _choose_reaction(self) -> str:
        if self.mode != "weighted":
            return self._validated_reaction(self.fixed_reaction)
        choices = []
        weights = []
        for name, values in self.reactions.items():
            reaction = self._validated_reaction(str(name).strip().lower())
            weight = max(0.0, float(dict(values or {}).get("weight", 0.0)))
            if weight > 0.0:
                choices.append(reaction)
                weights.append(weight)
        if not choices:
            return "yielding"
        return self._rng.choices(choices, weights=weights, k=1)[0]

    @staticmethod
    def _validated_reaction(reaction: str) -> str:
        return reaction if reaction in REACTION_STATES else "yielding"

    def _speed_scale(self, reaction: str | None) -> float:
        if reaction is None:
            return 1.0
        values = dict(self.reactions.get(reaction, {}) or {})
        default = 0.0 if reaction == "yielding" else 1.35 if reaction == "impatient" else 1.0
        return max(0.0, float(values.get("speed_scale", default)))

    def _geometry(
        self,
        agent: Mapping[str, float],
        robot: Mapping[str, float] | None,
    ) -> tuple[float | None, float | None, bool]:
        if robot is None:
            return None, None, False
        dx = float(robot["x"]) - float(agent["x"])
        dy = float(robot["y"]) - float(agent["y"])
        distance = math.hypot(dx, dy)
        yaw = float(agent.get("yaw", 0.0) or 0.0)
        bearing_error = math.atan2(
            math.sin(math.atan2(dy, dx) - yaw),
            math.cos(math.atan2(dy, dx) - yaw),
        )
        in_front = abs(bearing_error) <= self.front_half_angle_rad
        relative_vx = float(robot.get("vx", 0.0)) - float(agent.get("vx", 0.0))
        relative_vy = float(robot.get("vy", 0.0)) - float(agent.get("vy", 0.0))
        closing_speed = (
            -(dx * relative_vx + dy * relative_vy) / distance
            if distance > 1e-6
            else math.inf
        )
        ttc = distance / closing_speed if closing_speed > 1e-3 else None
        return distance, ttc, in_front
