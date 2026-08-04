"""Portable episode specification for replay, interactive, and dataset tracks."""

from __future__ import annotations

import json
from dataclasses import dataclass, field, replace
from enum import Enum
from pathlib import Path
from typing import Any, Mapping, Sequence

from toilet_benchmark.collection_scenarios import ManualCollectionConfig, ScenarioSelection


BENCHMARK_SCHEMA_VERSION = "toilet-social-nav-0.1"


class StringEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class TrackType(StringEnum):
    REPLAY = "replay"
    INTERACTIVE = "interactive"
    DATASET = "dataset"


class CollisionPolicy(StringEnum):
    TERMINATE = "terminate"
    CONTAIN = "contain"


def _numeric_tuple(value: Sequence[Any], length: int, field_name: str) -> tuple[float, ...]:
    if len(value) != length:
        raise ValueError(f"{field_name} must contain {length} values")
    return tuple(float(item) for item in value)


@dataclass(frozen=True)
class PedestrianHoldSpec:
    waypoint_index: int
    duration_sec: float

    def __post_init__(self) -> None:
        if int(self.waypoint_index) < 0:
            raise ValueError("pedestrian hold waypoint_index must be non-negative")
        if float(self.duration_sec) <= 0.0:
            raise ValueError("pedestrian hold duration_sec must be positive")

    def to_dict(self) -> dict[str, Any]:
        return {
            "waypoint_index": int(self.waypoint_index),
            "duration_sec": float(self.duration_sec),
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "PedestrianHoldSpec":
        return cls(
            waypoint_index=int(value["waypoint_index"]),
            duration_sec=float(value["duration_sec"]),
        )


@dataclass(frozen=True)
class RobotEpisodeSpec:
    model: str
    start_pose: tuple[float, float, float, float]
    goal_pose: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "model": self.model,
            "start_pose": list(self.start_pose),
            "goal_pose": list(self.goal_pose),
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "RobotEpisodeSpec":
        return cls(
            model=str(value["model"]),
            start_pose=_numeric_tuple(value["start_pose"], 4, "robot.start_pose"),
            goal_pose=_numeric_tuple(value["goal_pose"], 3, "robot.goal_pose"),
        )


@dataclass(frozen=True)
class PedestrianBehaviorSpec:
    behavior_type: str = "regular"
    configuration: str = "custom"
    walking_speed_mps: float | None = None
    group_id: int = -1

    def to_dict(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "type": self.behavior_type,
            "configuration": self.configuration,
            "group_id": self.group_id,
        }
        if self.walking_speed_mps is not None:
            value["walking_speed_mps"] = self.walking_speed_mps
        return value

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "PedestrianBehaviorSpec":
        speed = value.get("walking_speed_mps")
        return cls(
            behavior_type=str(value.get("type", "regular")),
            configuration=str(value.get("configuration", "custom")),
            walking_speed_mps=float(speed) if speed is not None else None,
            group_id=int(value.get("group_id", -1)),
        )


@dataclass(frozen=True)
class PedestrianEpisodeSpec:
    agent_id: str
    semantic_goal: str
    character: str | None = None
    start_reference: str | None = None
    start_pose: tuple[float, float, float] | None = None
    start_yaw: float | None = None
    route_waypoints: tuple[tuple[float, float, float], ...] = ()
    holds: tuple[PedestrianHoldSpec, ...] = ()
    constrain_to_path: bool = False
    behavior: PedestrianBehaviorSpec = field(default_factory=PedestrianBehaviorSpec)

    def to_dict(self) -> dict[str, Any]:
        value: dict[str, Any] = {
            "agent_id": self.agent_id,
            "semantic_goal": self.semantic_goal,
            "behavior": self.behavior.to_dict(),
        }
        if self.character is not None:
            value["character"] = self.character
        if self.start_reference is not None:
            value["start_reference"] = self.start_reference
        if self.start_pose is not None:
            value["start_pose"] = list(self.start_pose)
        if self.start_yaw is not None:
            value["start_yaw"] = float(self.start_yaw)
        if self.route_waypoints:
            value["route_waypoints"] = [list(point) for point in self.route_waypoints]
        if self.holds:
            value["holds"] = [hold.to_dict() for hold in self.holds]
        if self.constrain_to_path:
            value["constrain_to_path"] = True
        return value

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "PedestrianEpisodeSpec":
        start_pose = value.get("start_pose")
        route_waypoints = value.get("route_waypoints", ())
        return cls(
            agent_id=str(value["agent_id"]),
            semantic_goal=str(value["semantic_goal"]),
            character=str(value["character"]) if value.get("character") is not None else None,
            start_reference=(
                str(value["start_reference"])
                if value.get("start_reference") is not None
                else None
            ),
            start_pose=(
                _numeric_tuple(start_pose, 3, "pedestrian.start_pose")
                if start_pose is not None
                else None
            ),
            start_yaw=(
                float(value["start_yaw"])
                if value.get("start_yaw") is not None
                else None
            ),
            route_waypoints=tuple(
                _numeric_tuple(point, 3, "pedestrian.route_waypoint")
                for point in route_waypoints
            ),
            holds=tuple(
                PedestrianHoldSpec.from_mapping(item)
                for item in value.get("holds", ())
            ),
            constrain_to_path=bool(value.get("constrain_to_path", False)),
            behavior=PedestrianBehaviorSpec.from_mapping(value.get("behavior", {})),
        )


@dataclass(frozen=True)
class TerminationSpec:
    timeout_sec: float
    goal_tolerance_m: float
    collision_policy: CollisionPolicy = CollisionPolicy.TERMINATE

    def to_dict(self) -> dict[str, Any]:
        return {
            "timeout_sec": self.timeout_sec,
            "goal_tolerance_m": self.goal_tolerance_m,
            "collision_policy": self.collision_policy.value,
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "TerminationSpec":
        return cls(
            timeout_sec=float(value["timeout_sec"]),
            goal_tolerance_m=float(value["goal_tolerance_m"]),
            collision_policy=CollisionPolicy(
                value.get("collision_policy", CollisionPolicy.TERMINATE.value)
            ),
        )


@dataclass(frozen=True)
class EpisodeSpec:
    episode_id: str
    scene_id: str
    task_type: str
    track: TrackType
    seed: int
    robot: RobotEpisodeSpec
    pedestrians: tuple[PedestrianEpisodeSpec, ...]
    termination: TerminationSpec
    schema_version: str = BENCHMARK_SCHEMA_VERSION
    benchmark_version: str = "0.1.0"
    difficulty: Mapping[str, Any] = field(default_factory=dict)
    assets: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "benchmark_version": self.benchmark_version,
            "episode_id": self.episode_id,
            "scene_id": self.scene_id,
            "task_type": self.task_type,
            "track": self.track.value,
            "seed": self.seed,
            "robot": self.robot.to_dict(),
            "pedestrians": [pedestrian.to_dict() for pedestrian in self.pedestrians],
            "termination": self.termination.to_dict(),
            "difficulty": dict(self.difficulty),
            "assets": dict(self.assets),
            "metadata": dict(self.metadata),
        }

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "EpisodeSpec":
        return cls(
            schema_version=str(value.get("schema_version", BENCHMARK_SCHEMA_VERSION)),
            benchmark_version=str(value.get("benchmark_version", "0.1.0")),
            episode_id=str(value["episode_id"]),
            scene_id=str(value["scene_id"]),
            task_type=str(value["task_type"]),
            track=TrackType(value["track"]),
            seed=int(value["seed"]),
            robot=RobotEpisodeSpec.from_mapping(value["robot"]),
            pedestrians=tuple(
                PedestrianEpisodeSpec.from_mapping(item)
                for item in value.get("pedestrians", ())
            ),
            termination=TerminationSpec.from_mapping(value["termination"]),
            difficulty=dict(value.get("difficulty", {})),
            assets=dict(value.get("assets", {})),
            metadata=dict(value.get("metadata", {})),
        )


def episode_from_manual_selection(
    config: ManualCollectionConfig,
    selection: ScenarioSelection,
    *,
    episode_id: str,
    scene_id: str,
    track: TrackType | str = TrackType.DATASET,
    benchmark_version: str = "0.1.0",
    task_type: str = "enter_exit",
    robot_model: str = "xms_mecanum",
    pedestrian_start_reference: str = "entrance_main",
) -> EpisodeSpec:
    track_type = track if isinstance(track, TrackType) else TrackType(track)
    if selection.scenario.source_mode == "authored_route":
        if selection.scenario.episode_path is None:
            raise ValueError("authored-route scenario is missing episode_path")
        authored = EpisodeSpec.from_mapping(
            json.loads(Path(selection.scenario.episode_path).read_text(encoding="utf-8"))
        )
        return replace(
            authored,
            benchmark_version=benchmark_version,
            episode_id=episode_id,
            scene_id=scene_id,
            track=track_type,
            seed=selection.seed,
            termination=TerminationSpec(
                timeout_sec=config.episode.timeout_sec,
                goal_tolerance_m=config.episode.goal_tolerance_m,
                collision_policy=CollisionPolicy.TERMINATE,
            ),
            metadata={
                **authored.metadata,
                "source": "manual_collection_authored_route",
                "source_episode_path": selection.scenario.episode_path,
                "source_episode_sha256": selection.scenario.episode_sha256,
                "selection_index": selection.selection_index,
            },
        )
    targets = selection.scenario.pedestrian_target_urinal_ids
    characters = config.pedestrian.character_pool
    pedestrians = tuple(
        PedestrianEpisodeSpec(
            agent_id=agent_id,
            semantic_goal=targets[index % len(targets)],
            character=characters[index % len(characters)] if characters else None,
            start_reference=pedestrian_start_reference,
        )
        for index, agent_id in enumerate(config.pedestrian.agent_ids)
    )
    return EpisodeSpec(
        schema_version=BENCHMARK_SCHEMA_VERSION,
        benchmark_version=benchmark_version,
        episode_id=episode_id,
        scene_id=scene_id,
        task_type=task_type,
        track=track_type,
        seed=selection.seed,
        robot=RobotEpisodeSpec(
            model=robot_model,
            start_pose=selection.scenario.robot_start,
            goal_pose=selection.scenario.robot_goal,
        ),
        pedestrians=pedestrians,
        termination=TerminationSpec(
            timeout_sec=config.episode.timeout_sec,
            goal_tolerance_m=config.episode.goal_tolerance_m,
            collision_policy=CollisionPolicy.TERMINATE,
        ),
        difficulty={
            "pedestrian_count": len(pedestrians),
            "scenario_id": selection.scenario.id,
        },
        metadata={
            "source": "manual_collection",
            "selection_index": selection.selection_index,
        },
    )


def episodes_from_manual_config(
    config: ManualCollectionConfig,
    *,
    scene_id: str,
    track: TrackType | str = TrackType.DATASET,
    episode_prefix: str = "manual",
    benchmark_version: str = "0.1.0",
    seed: int | None = None,
) -> tuple[EpisodeSpec, ...]:
    """Convert every enabled manual scenario into a stable episode list."""

    episodes = []
    for enabled_index, scenario in enumerate(config.enabled_scenarios()):
        selection = ScenarioSelection(
            scenario=scenario,
            selection_index=enabled_index + 1,
            enabled_index=enabled_index,
            selector_mode="manifest",
            seed=config.session.seed if seed is None else int(seed),
        )
        episodes.append(
            episode_from_manual_selection(
                config,
                selection,
                episode_id=f"{episode_prefix}_{scenario.id}",
                scene_id=scene_id,
                track=track,
                benchmark_version=benchmark_version,
            )
        )
    return tuple(episodes)
