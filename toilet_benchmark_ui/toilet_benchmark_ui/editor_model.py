"""Simulator-neutral map and route editing models."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Iterable, Sequence

from toilet_benchmark.episodes.schema import (
    CollisionPolicy,
    EpisodeSpec,
    PedestrianBehaviorSpec,
    PedestrianEpisodeSpec,
    PedestrianHoldSpec,
    PedestrianTerminalBehavior,
    RobotEpisodeSpec,
    TerminationSpec,
    TrackType,
)
from toilet_benchmark.tracks.authored_scenario_core import (
    ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD,
    initial_character_root_yaw,
    normalize_yaw,
    route_heading_to_character_root_yaw,
)


@dataclass(frozen=True)
class ValidationResult:
    valid: bool
    reason: str = ""
    point_index: int | None = None


@dataclass(frozen=True)
class MapSnapshot:
    """An occupancy grid with an explicit world-frame transform."""

    frame_id: str
    resolution: float
    width: int
    height: int
    origin: tuple[float, float]
    data: tuple[int, ...]

    def __post_init__(self) -> None:
        if not str(self.frame_id).strip():
            raise ValueError("frame_id must not be empty")
        if not math.isfinite(float(self.resolution)) or self.resolution <= 0.0:
            raise ValueError("resolution must be positive and finite")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("occupancy data length does not match map dimensions")

    @classmethod
    def from_payload(cls, payload: dict[str, Any], *, frame_id: str = "map") -> "MapSnapshot":
        origin = payload.get("origin", (0.0, 0.0))
        return cls(
            frame_id=str(frame_id),
            resolution=float(payload["resolution"]),
            width=int(payload["width"]),
            height=int(payload["height"]),
            origin=(float(origin[0]), float(origin[1])),
            data=tuple(int(value) for value in payload["data"]),
        )

    @classmethod
    def from_occupancy_grid(cls, message) -> "MapSnapshot":
        return cls(
            frame_id=str(message.header.frame_id or "map"),
            resolution=float(message.info.resolution),
            width=int(message.info.width),
            height=int(message.info.height),
            origin=(
                float(message.info.origin.position.x),
                float(message.info.origin.position.y),
            ),
            data=tuple(int(value) for value in message.data),
        )

    @property
    def world_width(self) -> float:
        return self.width * self.resolution

    @property
    def world_height(self) -> float:
        return self.height * self.resolution

    def cell_for_world(self, x: float, y: float) -> tuple[int, int] | None:
        ix = math.floor((float(x) - self.origin[0]) / self.resolution)
        iy = math.floor((float(y) - self.origin[1]) / self.resolution)
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return int(ix), int(iy)

    def value_at_cell(self, ix: int, iy: int) -> int | None:
        if ix < 0 or iy < 0 or ix >= self.width or iy >= self.height:
            return None
        return int(self.data[iy * self.width + ix])

    def value_at_world(self, x: float, y: float) -> int | None:
        cell = self.cell_for_world(x, y)
        if cell is None:
            return None
        return self.value_at_cell(*cell)

    def is_free(self, x: float, y: float) -> bool:
        return self.value_at_world(x, y) == 0

    def world_to_pixel(self, x: float, y: float) -> tuple[float, float]:
        """Map world coordinates to a top-left-origin canvas in pixel units."""
        return (
            (float(x) - self.origin[0]) / self.resolution,
            self.height - (float(y) - self.origin[1]) / self.resolution,
        )

    def pixel_to_world(self, x: float, y: float) -> tuple[float, float]:
        return (
            self.origin[0] + float(x) * self.resolution,
            self.origin[1] + (self.height - float(y)) * self.resolution,
        )

    def _disk_is_free(self, x: float, y: float, radius_m: float) -> bool:
        if radius_m <= 0.0:
            return self.is_free(x, y)
        cells = max(1, math.ceil(radius_m / self.resolution))
        center = self.cell_for_world(x, y)
        if center is None:
            return False
        cx, cy = center
        for iy in range(cy - cells, cy + cells + 1):
            for ix in range(cx - cells, cx + cells + 1):
                dx = ((ix + 0.5) * self.resolution + self.origin[0]) - x
                dy = ((iy + 0.5) * self.resolution + self.origin[1]) - y
                if dx * dx + dy * dy <= radius_m * radius_m:
                    if self.value_at_cell(ix, iy) != 0:
                        return False
        return True

    def validate_polyline(
        self,
        points: Iterable[Sequence[float]],
        *,
        radius_m: float = 0.22,
    ) -> ValidationResult:
        points = tuple(tuple(float(value) for value in point[:2]) for point in points)
        if not points:
            return ValidationResult(False, "route has no waypoints")
        if radius_m < 0.0 or not math.isfinite(float(radius_m)):
            return ValidationResult(False, "route radius must be finite and non-negative")
        for index, (x, y) in enumerate(points):
            if not self._disk_is_free(x, y, radius_m):
                return ValidationResult(False, "waypoint is outside or occupied", index)
            if index == 0:
                continue
            px, py = points[index - 1]
            distance = math.hypot(x - px, y - py)
            steps = max(1, math.ceil(distance / max(self.resolution * 0.5, 0.02)))
            for step in range(1, steps + 1):
                ratio = step / steps
                sx = px + (x - px) * ratio
                sy = py + (y - py) * ratio
                if not self._disk_is_free(sx, sy, radius_m):
                    return ValidationResult(False, "route segment crosses occupied space", index)
        return ValidationResult(True)


def _route_point(point: Sequence[float]) -> tuple[float, float, float]:
    if len(point) < 2:
        raise ValueError("route point needs x and y")
    values = (
        float(point[0]),
        float(point[1]),
        float(point[2]) if len(point) > 2 else 0.0,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("route point must be finite")
    return values


@dataclass
class HoldDraft:
    waypoint_index: int
    duration_sec: float | None = 2.0
    yaw: float | None = None

    def __post_init__(self) -> None:
        if int(self.waypoint_index) < 0:
            raise ValueError("hold waypoint_index must be non-negative")
        if self.duration_sec is not None and (
            not math.isfinite(float(self.duration_sec)) or float(self.duration_sec) <= 0.0
        ):
            raise ValueError("hold duration_sec must be positive and finite")


@dataclass
class ActorDraft:
    actor_id: str
    kind: str
    spawn_pose: tuple[float, float, float, float] | None = None
    goal_pose: tuple[float, float, float] | None = None
    route: list[tuple[float, float, float]] = field(default_factory=list)
    holds: list[HoldDraft] = field(default_factory=list)
    character: str | None = None
    auto_start_yaw: bool = True
    start_hold_duration_sec: float | None = 0.0
    terminal_behavior: PedestrianTerminalBehavior = PedestrianTerminalBehavior.RETIRE
    terminal_yaw: float | None = None
    velocity_mps: float = 0.8
    constrain_to_path: bool = True

    @classmethod
    def pedestrian(cls, actor_id: str) -> "ActorDraft":
        return cls(
            actor_id=str(actor_id),
            kind="pedestrian",
            character="original_female_adult_business_02",
        )

    @classmethod
    def robot(cls) -> "ActorDraft":
        return cls(
            actor_id="xms_mecanum",
            kind="robot",
            spawn_pose=(0.0, -1.2, 0.03, 0.0),
            constrain_to_path=False,
        )

    def set_route(self, points: Iterable[Sequence[float]]) -> None:
        self.route = [_route_point(point) for point in points]
        self.holds = [hold for hold in self.holds if hold.waypoint_index < len(self.route)]

    def authored_points(self) -> list[tuple[float, float, float]]:
        """Return UI route points with pedestrian spawn represented as point 0."""
        if self.kind != "pedestrian" or self.spawn_pose is None:
            return list(self.route)
        return [self.spawn_pose[:3], *self.route]

    def sync_walking_heading(self, *, epsilon_m: float = 1e-6) -> bool:
        """Update the UI walking heading from spawn to the first effective target."""
        if (
            self.kind != "pedestrian"
            or self.spawn_pose is None
            or not self.auto_start_yaw
        ):
            return False
        start_x, start_y = self.spawn_pose[:2]
        for target in self.route:
            dx = float(target[0]) - start_x
            dy = float(target[1]) - start_y
            if math.hypot(dx, dy) > epsilon_m:
                self.spawn_pose = (*self.spawn_pose[:3], math.atan2(dy, dx))
                return True
        return False


@dataclass
class ScenarioDraft:
    scenario_id: str = "narrow_head_on_001"
    scene_id: str = "shenxinfu_841837"
    seed: int = 42
    robot: ActorDraft = field(default_factory=ActorDraft.robot)
    pedestrians: list[ActorDraft] = field(default_factory=list)
    timeout_sec: float = 90.0
    goal_tolerance_m: float = 0.30

    @classmethod
    def default(cls) -> "ScenarioDraft":
        return cls()

    def actor(self, actor_id: str) -> ActorDraft:
        if actor_id == self.robot.actor_id:
            return self.robot
        for actor in self.pedestrians:
            if actor.actor_id == actor_id:
                return actor
        raise KeyError(actor_id)

    def add_pedestrian(self, actor_id: str) -> ActorDraft:
        actor_id = str(actor_id).strip()
        if not actor_id:
            raise ValueError("pedestrian id must not be empty")
        if actor_id == self.robot.actor_id or any(actor.actor_id == actor_id for actor in self.pedestrians):
            raise ValueError(f"duplicate actor id: {actor_id}")
        actor = ActorDraft.pedestrian(actor_id)
        self.pedestrians.append(actor)
        return actor

    def remove_pedestrian(self, actor_id: str) -> bool:
        for index, actor in enumerate(self.pedestrians):
            if actor.actor_id == actor_id:
                self.pedestrians.pop(index)
                return True
        return False

    def validate(self, grid: MapSnapshot | None, *, radius_m: float = 0.22) -> list[str]:
        errors: list[str] = []
        if not self.pedestrians:
            errors.append("scenario needs at least one pedestrian")
        for actor in [self.robot, *self.pedestrians]:
            if actor.spawn_pose is None:
                errors.append(f"{actor.actor_id}: spawn pose is missing")
            elif grid is not None:
                spawn_result = grid.validate_polyline((actor.spawn_pose,), radius_m=radius_m)
                if not spawn_result.valid:
                    errors.append(f"{actor.actor_id}: spawn is outside or occupied")
            if actor.kind != "pedestrian":
                continue
            if (
                not actor.route
                and actor.terminal_behavior != PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
            ):
                errors.append(
                    f"{actor.actor_id}: route needs a target unless terminal behavior keeps it active"
                )
            if actor.start_hold_duration_sec is None and actor.route:
                errors.append(f"{actor.actor_id}: route is unreachable while spawn holds until episode end")
            elif grid is not None:
                for point_index, point in enumerate(actor.route):
                    point_result = grid.validate_polyline((point,), radius_m=radius_m)
                    if not point_result.valid:
                        errors.append(
                            f"{actor.actor_id}: target {point_index} is outside or occupied"
                        )
            for hold in actor.holds:
                if hold.waypoint_index >= len(actor.route):
                    errors.append(
                        f"{actor.actor_id}: hold waypoint {hold.waypoint_index} is outside route"
                    )
                elif hold.waypoint_index == len(actor.route) - 1:
                    errors.append(
                        f"{actor.actor_id}: terminal point behavior must use terminal settings"
                    )
                elif hold.duration_sec is None:
                    errors.append(
                        f"{actor.actor_id}: intermediate hold must have a finite duration"
                    )
        if self.robot.goal_pose is None:
            errors.append(f"{self.robot.actor_id}: robot goal pose is missing")
        else:
            if grid is not None:
                goal_result = grid.validate_polyline((self.robot.goal_pose,), radius_m=radius_m)
                if not goal_result.valid:
                    errors.append(f"{self.robot.actor_id}: robot goal is outside or occupied")
            if self.robot.spawn_pose is not None and math.hypot(
                self.robot.goal_pose[0] - self.robot.spawn_pose[0],
                self.robot.goal_pose[1] - self.robot.spawn_pose[1],
            ) <= self.goal_tolerance_m:
                errors.append(
                    f"{self.robot.actor_id}: robot start-goal distance must exceed "
                    f"goal tolerance {self.goal_tolerance_m:g}m"
                )
        for index, first in enumerate(self.pedestrians):
            if first.spawn_pose is None:
                continue
            for second in self.pedestrians[index + 1 :]:
                if second.spawn_pose is None:
                    continue
                if math.hypot(
                    first.spawn_pose[0] - second.spawn_pose[0],
                    first.spawn_pose[1] - second.spawn_pose[1],
                ) < 2.0 * radius_m:
                    errors.append(f"{first.actor_id}/{second.actor_id}: spawn footprints overlap")
        return errors

    def to_episode_spec(self, *, map_path: str) -> EpisodeSpec:
        if self.robot.spawn_pose is None:
            raise ValueError("robot spawn pose is required")
        if self.robot.goal_pose is None:
            raise ValueError("robot goal pose is required")
        pedestrians = []
        for actor in self.pedestrians:
            if actor.spawn_pose is None:
                raise ValueError(f"{actor.actor_id} spawn pose is required")
            start_yaw = (
                initial_character_root_yaw(actor.spawn_pose, actor.route)
                if actor.auto_start_yaw and actor.route
                else route_heading_to_character_root_yaw(actor.spawn_pose[3])
            )
            pedestrians.append(
                PedestrianEpisodeSpec(
                    agent_id=actor.actor_id,
                    semantic_goal="route_terminal",
                    character=actor.character,
                    start_pose=actor.spawn_pose[:3],
                    start_yaw=start_yaw,
                    auto_start_yaw=actor.auto_start_yaw,
                    start_hold_duration_sec=actor.start_hold_duration_sec,
                    route_waypoints=tuple(actor.route),
                    holds=tuple(
                        PedestrianHoldSpec(
                            waypoint_index=hold.waypoint_index,
                            duration_sec=hold.duration_sec,
                            yaw=(
                                None
                                if hold.yaw is None
                                else route_heading_to_character_root_yaw(hold.yaw)
                            ),
                        )
                        for hold in actor.holds
                    ),
                    terminal_behavior=actor.terminal_behavior,
                    terminal_yaw=(
                        None
                        if actor.terminal_yaw is None
                        else route_heading_to_character_root_yaw(actor.terminal_yaw)
                    ),
                    constrain_to_path=actor.constrain_to_path,
                    behavior=PedestrianBehaviorSpec(
                        walking_speed_mps=actor.velocity_mps,
                    ),
                )
            )
        return EpisodeSpec(
            episode_id=self.scenario_id,
            scene_id=self.scene_id,
            task_type="authored_route",
            track=TrackType.INTERACTIVE,
            seed=int(self.seed),
            robot=RobotEpisodeSpec(
                model=self.robot.actor_id,
                start_pose=self.robot.spawn_pose,
                goal_pose=self.robot.goal_pose,
            ),
            pedestrians=tuple(pedestrians),
            termination=TerminationSpec(
                timeout_sec=float(self.timeout_sec),
                goal_tolerance_m=float(self.goal_tolerance_m),
                collision_policy=CollisionPolicy.TERMINATE,
            ),
            difficulty={"pedestrian_count": len(pedestrians)},
            assets={"walkable_map": str(map_path)},
            metadata={"source": "toilet_benchmark_ui"},
        )

    @classmethod
    def from_episode_spec(cls, episode: EpisodeSpec) -> "ScenarioDraft":
        robot = ActorDraft.robot()
        robot.actor_id = episode.robot.model
        robot.spawn_pose = episode.robot.start_pose
        robot.goal_pose = episode.robot.goal_pose
        pedestrians = []
        for spec in episode.pedestrians:
            actor = ActorDraft.pedestrian(spec.agent_id)
            actor.character = spec.character
            actor.auto_start_yaw = spec.auto_start_yaw
            actor.start_hold_duration_sec = spec.start_hold_duration_sec
            actor.terminal_behavior = spec.terminal_behavior
            actor.terminal_yaw = (
                None
                if spec.terminal_yaw is None
                else normalize_yaw(
                    spec.terminal_yaw - ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD
                )
            )
            if spec.start_pose is not None:
                ui_yaw = 0.0 if spec.start_yaw is None else spec.start_yaw
                if not spec.auto_start_yaw:
                    ui_yaw = normalize_yaw(
                        ui_yaw - ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD
                    )
                actor.spawn_pose = (
                    spec.start_pose[0],
                    spec.start_pose[1],
                    spec.start_pose[2],
                    ui_yaw,
                )
            actor.route = list(spec.route_waypoints)
            actor.holds = []
            terminal_index = len(spec.route_waypoints) - 1
            for hold in spec.holds:
                hold_yaw = (
                    None
                    if hold.yaw is None
                    else normalize_yaw(hold.yaw - ISAAC_CHARACTER_ROOT_YAW_OFFSET_RAD)
                )
                if (
                    hold.waypoint_index == terminal_index
                    and hold.duration_sec is None
                    and spec.terminal_behavior
                    == PedestrianTerminalBehavior.HOLD_UNTIL_EPISODE_END
                ):
                    if actor.terminal_yaw is None:
                        actor.terminal_yaw = hold_yaw
                    continue
                actor.holds.append(
                    HoldDraft(hold.waypoint_index, hold.duration_sec, hold_yaw)
                )
            if spec.behavior.walking_speed_mps is not None:
                actor.velocity_mps = spec.behavior.walking_speed_mps
            actor.constrain_to_path = spec.constrain_to_path
            actor.sync_walking_heading()
            pedestrians.append(actor)
        return cls(
            scenario_id=episode.episode_id,
            scene_id=episode.scene_id,
            seed=episode.seed,
            robot=robot,
            pedestrians=pedestrians,
            timeout_sec=episode.termination.timeout_sec,
            goal_tolerance_m=episode.termination.goal_tolerance_m,
        )
