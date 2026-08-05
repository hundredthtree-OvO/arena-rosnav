from __future__ import annotations

import hashlib
import json
import random
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


def _require_mapping(value: Any, *, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise TypeError(f"{name} must be a mapping, got {type(value).__name__}")
    return value


def _require_sequence(value: Any, *, name: str) -> Sequence[Any]:
    if isinstance(value, (str, bytes)) or not isinstance(value, Sequence):
        raise TypeError(f"{name} must be a sequence, got {type(value).__name__}")
    return value


def _require_str(value: Any, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _require_bool(value: Any, *, name: str) -> bool:
    if not isinstance(value, bool):
        raise TypeError(f"{name} must be a bool, got {type(value).__name__}")
    return value


def _require_float(value: Any, *, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{name} must be a number, got {type(value).__name__}")
    return float(value)


def _require_int(value: Any, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TypeError(f"{name} must be an int, got {type(value).__name__}")
    return int(value)


def _parse_pose(value: Any, *, name: str, dims: int) -> tuple[float, ...]:
    sequence = _require_sequence(value, name=name)
    if len(sequence) != dims:
        raise ValueError(f"{name} must contain exactly {dims} values, got {len(sequence)}")
    return tuple(_require_float(item, name=f"{name}[{index}]") for index, item in enumerate(sequence))


@dataclass(frozen=True)
class SessionConfig:
    output_root: Path
    operator_id: str
    seed: int
    selection_mode: str = "round_robin"
    max_episodes: int = 10
    fixed_scenario_id: str | None = None


@dataclass(frozen=True)
class EpisodeConfig:
    timeout_sec: float
    goal_tolerance_m: float
    goal_stop_speed_mps: float
    settle_timeout_sec: float
    settle_linear_speed_mps: float
    settle_angular_speed_rps: float


@dataclass(frozen=True)
class PedestrianConfig:
    agent_id: str
    count: int = 1
    character_pool: tuple[str, ...] = ()

    @property
    def agent_ids(self) -> tuple[str, ...]:
        match = re.fullmatch(r"(.*?)(\d+)", self.agent_id)
        if match is None:
            if self.count == 1:
                return (self.agent_id,)
            raise ValueError("pedestrian.agent_id must end in digits when pedestrian.count is greater than one")
        prefix, number = match.groups()
        start = int(number)
        width = len(number)
        return tuple(f"{prefix}{index:0{width}d}" for index in range(start, start + self.count))


@dataclass(frozen=True)
class ScenarioConfig:
    id: str
    enabled: bool
    weight: float
    robot_start: tuple[float, float, float, float]
    robot_goal: tuple[float, float, float]
    pedestrian_target_urinal_ids: tuple[str, ...]
    source_mode: str = "legacy_enter_use_exit"
    episode_path: str | None = None
    episode_sha256: str | None = None
    pedestrian_agent_ids: tuple[str, ...] = ()

    @property
    def pedestrian_target_urinal_id(self) -> str:
        """Backward-compatible primary target for single-pedestrian callers."""
        if not self.pedestrian_target_urinal_ids:
            raise ValueError("authored-route scenarios do not define a urinal target")
        return self.pedestrian_target_urinal_ids[0]

    @property
    def as_manifest_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class ManualCollectionConfig:
    session: SessionConfig
    episode: EpisodeConfig
    pedestrian: PedestrianConfig
    scenarios: tuple[ScenarioConfig, ...] = field(default_factory=tuple)

    def enabled_scenarios(self) -> tuple[ScenarioConfig, ...]:
        return tuple(scenario for scenario in self.scenarios if scenario.enabled)


@dataclass(frozen=True)
class ScenarioSelection:
    scenario: ScenarioConfig
    selection_index: int
    enabled_index: int
    selector_mode: str
    seed: int


def _parse_session(raw: Mapping[str, Any]) -> SessionConfig:
    max_episodes = _require_int(raw.get("max_episodes", 10), name="session.max_episodes")
    if max_episodes < 0:
        raise ValueError("session.max_episodes must be >= 0")
    return SessionConfig(
        output_root=Path(_require_str(raw.get("output_root"), name="session.output_root")),
        operator_id=_require_str(raw.get("operator_id"), name="session.operator_id"),
        seed=_require_int(raw.get("seed"), name="session.seed"),
        selection_mode=_require_str(raw.get("selection_mode", "round_robin"), name="session.selection_mode"),
        max_episodes=max_episodes,
        fixed_scenario_id=(
            None
            if raw.get("fixed_scenario_id") is None
            else _require_str(raw.get("fixed_scenario_id"), name="session.fixed_scenario_id")
        ),
    )


def _parse_episode(raw: Mapping[str, Any]) -> EpisodeConfig:
    return EpisodeConfig(
        timeout_sec=_require_float(raw.get("timeout_sec"), name="episode.timeout_sec"),
        goal_tolerance_m=_require_float(raw.get("goal_tolerance_m"), name="episode.goal_tolerance_m"),
        goal_stop_speed_mps=_require_float(raw.get("goal_stop_speed_mps"), name="episode.goal_stop_speed_mps"),
        settle_timeout_sec=_require_float(raw.get("settle_timeout_sec"), name="episode.settle_timeout_sec"),
        settle_linear_speed_mps=_require_float(raw.get("settle_linear_speed_mps"), name="episode.settle_linear_speed_mps"),
        settle_angular_speed_rps=_require_float(raw.get("settle_angular_speed_rps"), name="episode.settle_angular_speed_rps"),
    )


def _parse_pedestrian(raw: Mapping[str, Any]) -> PedestrianConfig:
    count = _require_int(raw.get("count", 1), name="pedestrian.count")
    if count <= 0:
        raise ValueError("pedestrian.count must be greater than zero")
    character_pool_raw = raw.get("character_pool", [])
    character_pool = tuple(
        _require_str(value, name=f"pedestrian.character_pool[{index}]")
        for index, value in enumerate(
            _require_sequence(character_pool_raw, name="pedestrian.character_pool")
        )
    )
    config = PedestrianConfig(
        agent_id=_require_str(raw.get("agent_id"), name="pedestrian.agent_id"),
        count=count,
        character_pool=character_pool,
    )
    config.agent_ids
    return config


def _parse_target_urinal_ids(raw: Mapping[str, Any], *, index: int) -> tuple[str, ...]:
    plural = raw.get("pedestrian_target_urinal_ids")
    if plural is None:
        return (
            _require_str(
                raw.get("pedestrian_target_urinal_id"),
                name=f"scenarios[{index}].pedestrian_target_urinal_id",
            ),
        )
    values = _require_sequence(plural, name=f"scenarios[{index}].pedestrian_target_urinal_ids")
    if not values:
        raise ValueError(f"scenarios[{index}].pedestrian_target_urinal_ids must not be empty")
    return tuple(
        _require_str(value, name=f"scenarios[{index}].pedestrian_target_urinal_ids[{target_index}]")
        for target_index, value in enumerate(values)
    )


def _load_authored_scenario(path_value: Any, *, index: int) -> tuple:
    path = Path(
        _require_str(path_value, name=f"scenarios[{index}].episode_path")
    ).expanduser().resolve()
    payload = json.loads(path.read_text(encoding="utf-8"))
    root = _require_mapping(payload, name=f"scenarios[{index}].episode")
    if root.get("task_type") != "authored_route":
        raise ValueError(f"scenarios[{index}].episode must use task_type=authored_route")
    robot = _require_mapping(root.get("robot"), name=f"scenarios[{index}].episode.robot")
    pedestrians = _require_sequence(
        root.get("pedestrians"), name=f"scenarios[{index}].episode.pedestrians"
    )
    if not pedestrians:
        raise ValueError(f"scenarios[{index}].episode.pedestrians must not be empty")
    agent_ids = tuple(
        _require_str(
            _require_mapping(item, name=f"scenarios[{index}].episode.pedestrians[{ped_index}]").get("agent_id"),
            name=f"scenarios[{index}].episode.pedestrians[{ped_index}].agent_id",
        )
        for ped_index, item in enumerate(pedestrians)
    )
    if len(set(agent_ids)) != len(agent_ids):
        raise ValueError(f"scenarios[{index}].episode contains duplicate pedestrian agent_id values")
    return (
        str(path),
        hashlib.sha256(path.read_bytes()).hexdigest(),
        _parse_pose(robot.get("start_pose"), name=f"scenarios[{index}].episode.robot.start_pose", dims=4),
        _parse_pose(robot.get("goal_pose"), name=f"scenarios[{index}].episode.robot.goal_pose", dims=3),
        agent_ids,
    )


def _parse_scenario(raw: Mapping[str, Any], *, index: int, source_mode: str) -> ScenarioConfig:
    scenario_id = _require_str(raw.get("id"), name=f"scenarios[{index}].id")
    enabled = _require_bool(raw.get("enabled", True), name=f"scenarios[{index}].enabled")
    weight = _require_float(raw.get("weight", 1.0), name=f"scenarios[{index}].weight")
    if weight <= 0.0:
        raise ValueError(f"scenarios[{index}].weight must be greater than zero")
    if source_mode == "authored_route":
        episode_path, episode_sha256, robot_start, robot_goal, agent_ids = _load_authored_scenario(
            raw.get("episode_path"), index=index
        )
        return ScenarioConfig(
            id=scenario_id,
            enabled=enabled,
            weight=weight,
            robot_start=robot_start,
            robot_goal=robot_goal,
            pedestrian_target_urinal_ids=(),
            source_mode=source_mode,
            episode_path=episode_path,
            episode_sha256=episode_sha256,
            pedestrian_agent_ids=agent_ids,
        )
    if source_mode != "legacy_enter_use_exit":
        raise ValueError(
            "scenario_source.mode must be authored_route or legacy_enter_use_exit"
        )
    return ScenarioConfig(
        id=scenario_id,
        enabled=enabled,
        weight=weight,
        robot_start=_parse_pose(raw.get("robot_start"), name=f"scenarios[{index}].robot_start", dims=4),
        robot_goal=_parse_pose(raw.get("robot_goal"), name=f"scenarios[{index}].robot_goal", dims=3),
        pedestrian_target_urinal_ids=_parse_target_urinal_ids(raw, index=index),
        source_mode=source_mode,
    )


def load_manual_collection_config(source: str | Path | Mapping[str, Any]) -> ManualCollectionConfig:
    if isinstance(source, (str, Path)):
        raw = yaml.safe_load(Path(source).read_text(encoding="utf-8"))
    else:
        raw = source
    root = _require_mapping(raw or {}, name="manual_collection_config")
    session = _parse_session(_require_mapping(root.get("session"), name="session"))
    episode = _parse_episode(_require_mapping(root.get("episode"), name="episode"))
    pedestrian = _parse_pedestrian(_require_mapping(root.get("pedestrian"), name="pedestrian"))
    source_raw = _require_mapping(root.get("scenario_source", {}), name="scenario_source")
    source_mode = _require_str(
        source_raw.get("mode", "legacy_enter_use_exit"), name="scenario_source.mode"
    )
    scenario_items = _require_sequence(root.get("scenarios"), name="scenarios")
    scenarios = tuple(
        _parse_scenario(
            _require_mapping(item, name=f"scenarios[{index}]"),
            index=index,
            source_mode=source_mode,
        )
        for index, item in enumerate(scenario_items)
    )
    if not scenarios:
        raise ValueError("scenarios must not be empty")
    if not any(scenario.enabled for scenario in scenarios):
        raise ValueError("at least one scenario must be enabled")
    return ManualCollectionConfig(session=session, episode=episode, pedestrian=pedestrian, scenarios=scenarios)


class ScenarioSelector:
    def __init__(
        self,
        scenarios: Sequence[ScenarioConfig],
        *,
        selection_mode: str,
        seed: int,
        fixed_scenario_id: str | None = None,
    ) -> None:
        self._scenarios = tuple(scenario for scenario in scenarios if scenario.enabled)
        if not self._scenarios:
            raise ValueError("At least one enabled scenario is required")
        self._mode = _require_str(selection_mode, name="selection_mode").lower()
        if self._mode not in {"fixed", "round_robin", "seeded_random"}:
            raise ValueError(f"Unsupported selection_mode: {selection_mode!r}")
        self._seed = _require_int(seed, name="seed")
        self._fixed_scenario_id = fixed_scenario_id.strip() if fixed_scenario_id else None
        if self._fixed_scenario_id is not None and not any(scenario.id == self._fixed_scenario_id for scenario in self._scenarios):
            raise ValueError(f"Unknown fixed_scenario_id: {self._fixed_scenario_id!r}")
        self._rr_index = 0
        self._selection_count = 0
        self._rng = random.Random(self._seed)
        self._coverage_counts = {scenario.id: 0 for scenario in self._scenarios}

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def seed(self) -> int:
        return self._seed

    @property
    def coverage_counts(self) -> dict[str, int]:
        return dict(self._coverage_counts)

    def reset(self) -> None:
        self._rr_index = 0
        self._selection_count = 0
        self._rng = random.Random(self._seed)
        for key in self._coverage_counts:
            self._coverage_counts[key] = 0

    def select(self) -> ScenarioSelection:
        self._selection_count += 1
        if self._mode == "fixed":
            scenario = self._select_fixed()
        elif self._mode == "round_robin":
            scenario = self._select_round_robin()
        else:
            scenario = self._select_seeded_random()
        enabled_index = self._scenarios.index(scenario)
        self._coverage_counts[scenario.id] += 1
        return ScenarioSelection(
            scenario=scenario,
            selection_index=self._selection_count,
            enabled_index=enabled_index,
            selector_mode=self._mode,
            seed=self._seed,
        )

    def _select_fixed(self) -> ScenarioConfig:
        if self._fixed_scenario_id is None:
            return self._scenarios[0]
        for scenario in self._scenarios:
            if scenario.id == self._fixed_scenario_id:
                return scenario
        raise AssertionError("fixed_scenario_id validation should prevent this branch")

    def _select_round_robin(self) -> ScenarioConfig:
        scenario = self._scenarios[self._rr_index % len(self._scenarios)]
        self._rr_index += 1
        return scenario

    def _select_seeded_random(self) -> ScenarioConfig:
        total_weight = sum(scenario.weight for scenario in self._scenarios)
        draw = self._rng.random() * total_weight
        cumulative = 0.0
        for scenario in self._scenarios:
            cumulative += scenario.weight
            if draw < cumulative:
                return scenario
        return self._scenarios[-1]


def dump_manual_collection_config(config: ManualCollectionConfig) -> dict[str, Any]:
    return {
        "session": {
            "output_root": str(config.session.output_root),
            "operator_id": config.session.operator_id,
            "seed": config.session.seed,
            "selection_mode": config.session.selection_mode,
            "max_episodes": config.session.max_episodes,
            **(
                {"fixed_scenario_id": config.session.fixed_scenario_id}
                if config.session.fixed_scenario_id is not None
                else {}
            ),
        },
        "episode": asdict(config.episode),
        "pedestrian": {
            "agent_id": config.pedestrian.agent_id,
            "count": config.pedestrian.count,
            "character_pool": list(config.pedestrian.character_pool),
        },
        "scenarios": [
            {
                "id": scenario.id,
                "enabled": scenario.enabled,
                "weight": scenario.weight,
                "robot_start": list(scenario.robot_start),
                "robot_goal": list(scenario.robot_goal),
                "pedestrian_target_urinal_ids": list(scenario.pedestrian_target_urinal_ids),
            }
            for scenario in config.scenarios
        ],
    }
