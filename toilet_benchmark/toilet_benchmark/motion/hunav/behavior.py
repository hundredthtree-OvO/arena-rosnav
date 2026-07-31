"""HuNav behavior profile resolution extracted from backend hot-path logic."""

from __future__ import annotations

import math
from typing import Any, Mapping

from hunav_msgs.msg import AgentBehavior

HUNAV_BEHAVIOR_ORDER = (
    "regular",
    "impassive",
    "surprised",
    "scared",
    "curious",
    "threatening",
)

HUNAV_BEHAVIOR_TYPE_BY_NAME = {
    "regular": AgentBehavior.BEH_REGULAR,
    "impassive": AgentBehavior.BEH_IMPASSIVE,
    "surprised": AgentBehavior.BEH_SURPRISED,
    "scared": AgentBehavior.BEH_SCARED,
    "curious": AgentBehavior.BEH_CURIOUS,
    "threatening": AgentBehavior.BEH_THREATENING,
}

HUNAV_BEHAVIOR_FIELD_NAMES = (
    "duration",
    "once",
    "vel",
    "dist",
    "social_force_factor",
    "goal_force_factor",
    "obstacle_force_factor",
    "other_force_factor",
)

HUNAV_BEHAVIOR_DEFAULTS = {
    "configuration": "custom",
    "duration": 0.0,
    "once": False,
    "dist": 1.0,
    "social_force_factor": 5.0,
    "goal_force_factor": 2.0,
    "obstacle_force_factor": 10.0,
    "other_force_factor": 20.0,
}

HUNAV_CONFIGURATION_BY_NAME = {
    "default": AgentBehavior.BEH_CONF_DEFAULT,
    "custom": AgentBehavior.BEH_CONF_CUSTOM,
    "random_normal": AgentBehavior.BEH_CONF_RANDOM_NORMAL,
    "random_uniform": AgentBehavior.BEH_CONF_RANDOM_UNIFORM,
}

HUNAV_BEHAVIOR_NAME_BY_TYPE = {
    value: name for name, value in HUNAV_BEHAVIOR_TYPE_BY_NAME.items()
}
HUNAV_CONFIGURATION_NAME_BY_TYPE = {
    value: name for name, value in HUNAV_CONFIGURATION_BY_NAME.items()
}
HUNAV_BEHAVIOR_STATE_NAME_BY_TYPE = {
    AgentBehavior.BEH_NO_ACTIVE: "inactive",
    AgentBehavior.BEH_ACTIVE_1: "active_1",
    AgentBehavior.BEH_ACTIVE_2: "active_2",
}


def resolve_hunav_behavior_type(name: str) -> int:
    return HUNAV_BEHAVIOR_TYPE_BY_NAME[_normalize_behavior_name(name)]


def _normalize_behavior_name(name: str | None) -> str:
    normalized = str(name or "").strip().lower()
    return normalized if normalized in HUNAV_BEHAVIOR_TYPE_BY_NAME else "regular"


def _to_config_block(
    behavior_config: Mapping[str, Any] | None,
    behavior_name: str,
) -> Mapping[str, Any]:
    behavior_config = dict(behavior_config or {})
    global_defaults = {
        key: behavior_config.get(key)
        for key in HUNAV_BEHAVIOR_FIELD_NAMES
        if behavior_config.get(key) is not None
    }
    if behavior_config.get("configuration") is not None:
        global_defaults["configuration"] = behavior_config["configuration"]

    profile = behavior_config.get(behavior_name, {})
    if isinstance(profile, Mapping):
        merged = dict(global_defaults)
        for key, value in profile.items():
            if (
                key in HUNAV_BEHAVIOR_FIELD_NAMES
                or key == "configuration"
            ) and value is not None:
                merged[key] = value
        if merged:
            return merged

    if global_defaults:
        return global_defaults
    return {}


def _finite_float(value: Any, *, field_name: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError(f"HuNav behavior {field_name} must be finite")
    return parsed


def _resolve_configuration(value: Any) -> int:
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized not in HUNAV_CONFIGURATION_BY_NAME:
            raise ValueError(f"unknown HuNav behavior configuration: {value}")
        return HUNAV_CONFIGURATION_BY_NAME[normalized]
    parsed = int(value)
    if parsed not in HUNAV_CONFIGURATION_BY_NAME.values():
        raise ValueError(f"unknown HuNav behavior configuration: {value}")
    return parsed


def build_hunav_behavior(
    *,
    behavior_name: str,
    command_velocity: float,
    speed_scale: float,
    behavior_config: Mapping[str, Any] | None,
):
    """Build an AgentBehavior message from a pure config map.

    Pure behavior config keeps existing keys for backward compatibility:
      - top-level force-factor fields apply to all behaviors
      - optional per-behavior override blocks under keys above (regular/...)
    """

    behavior_name = _normalize_behavior_name(behavior_name)
    profile = {
        **HUNAV_BEHAVIOR_DEFAULTS,
        **_to_config_block(behavior_config, behavior_name),
    }

    behavior = AgentBehavior()
    behavior.type = resolve_hunav_behavior_type(behavior_name)
    behavior.state = AgentBehavior.BEH_NO_ACTIVE
    behavior.configuration = _resolve_configuration(profile["configuration"])
    behavior.duration = _finite_float(profile["duration"], field_name="duration")
    behavior.once = bool(profile["once"])
    base_velocity = profile.get("vel", command_velocity)
    behavior.vel = _finite_float(base_velocity, field_name="vel") * _finite_float(
        speed_scale,
        field_name="speed_scale",
    )
    behavior.dist = _finite_float(profile["dist"], field_name="dist")
    for key in (
        "social_force_factor",
        "goal_force_factor",
        "obstacle_force_factor",
        "other_force_factor",
    ):
        setattr(behavior, key, _finite_float(profile[key], field_name=key))
    return behavior


def describe_hunav_behavior(profile_name: str, behavior: AgentBehavior) -> dict[str, Any]:
    """Return stable diagnostic fields for the message sent to HuNav."""

    return {
        "profile": _normalize_behavior_name(profile_name),
        "native_type": HUNAV_BEHAVIOR_NAME_BY_TYPE.get(
            int(behavior.type),
            f"unknown:{int(behavior.type)}",
        ),
        "configuration": HUNAV_CONFIGURATION_NAME_BY_TYPE.get(
            int(behavior.configuration),
            f"unknown:{int(behavior.configuration)}",
        ),
        "state": HUNAV_BEHAVIOR_STATE_NAME_BY_TYPE.get(
            int(behavior.state),
            f"unknown:{int(behavior.state)}",
        ),
        "duration": float(behavior.duration),
        "once": bool(behavior.once),
        "vel": float(behavior.vel),
        "dist": float(behavior.dist),
        "social_force_factor": float(behavior.social_force_factor),
        "goal_force_factor": float(behavior.goal_force_factor),
        "obstacle_force_factor": float(behavior.obstacle_force_factor),
        "other_force_factor": float(behavior.other_force_factor),
    }
