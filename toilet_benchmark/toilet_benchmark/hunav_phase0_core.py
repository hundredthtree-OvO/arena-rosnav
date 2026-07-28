"""Deterministic scenarios and offline metrics for the HuNav Phase 0 smoke test."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field, replace
import math
import random
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

from toilet_benchmark.hunav_phase0_coordination import BottleneckSpec
from toilet_benchmark.semantic_rules import PortalCorridor


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class AgentSpec:
    agent_id: int
    name: str
    start: Point2D
    goal: Point2D
    desired_velocity: float = 0.65
    radius: float = 0.30
    goal_radius: float = 0.20
    final_yaw: float | None = None
    obstacles: Tuple[Point2D, ...] = ()

    def expected_final_yaw(self) -> float:
        if self.final_yaw is not None:
            return self.final_yaw
        return math.atan2(self.goal[1] - self.start[1], self.goal[0] - self.start[0])


@dataclass(frozen=True)
class RobotSpec:
    start: Point2D = (100.0, 100.0)
    velocity: Point2D = (0.0, 0.0)
    radius: float = 0.45
    motion_start_sec: float = 0.0
    motion_end_sec: float = 0.0

    def state_at(self, time_sec: float) -> Tuple[float, float, float, float]:
        active_time = max(
            0.0,
            min(time_sec, self.motion_end_sec) - self.motion_start_sec,
        )
        return (
            self.start[0] + self.velocity[0] * active_time,
            self.start[1] + self.velocity[1] * active_time,
            self.velocity[0] if self.motion_start_sec <= time_sec <= self.motion_end_sec else 0.0,
            self.velocity[1] if self.motion_start_sec <= time_sec <= self.motion_end_sec else 0.0,
        )


@dataclass(frozen=True)
class Scenario:
    name: str
    agents: Tuple[AgentSpec, ...]
    robot: RobotSpec = field(default_factory=RobotSpec)
    pause_windows: Tuple[Tuple[float, float], ...] = ()
    bottleneck: BottleneckSpec | None = None
    portal: PortalCorridor | None = None
    description: str = ""

    def paused_at(self, time_sec: float) -> bool:
        return any(start <= time_sec < end for start, end in self.pause_windows)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def _wall_points(
    x_min: float,
    x_max: float,
    y: float,
    spacing: float = 0.20,
) -> Tuple[Point2D, ...]:
    count = max(1, int(round((x_max - x_min) / spacing)))
    return tuple((x_min + (x_max - x_min) * i / count, y) for i in range(count + 1))


def _scenario_templates() -> Dict[str, Scenario]:
    corridor_obstacles = _wall_points(-2.5, 2.5, -0.75) + _wall_points(
        -2.5, 2.5, 0.75
    )
    gate_obstacles = _wall_points(-1.0, 1.0, -0.50) + _wall_points(
        -1.0, 1.0, 0.50
    )
    toilet_portal = PortalCorridor(
        outside=(-3.80, -0.90, 0.0),
        inside=(-2.24, -0.90, 0.0),
        half_width_m=0.45,
        clearance_m=0.40,
    )
    toilet_portal_walls = _wall_points(-3.80, -1.84, -1.40, 0.10) + _wall_points(
        -3.80,
        -1.84,
        -0.40,
        0.10,
    )
    return {
        "single_pause_resume": Scenario(
            name="single_pause_resume",
            description="One pedestrian walks, is externally paused, then resumes.",
            agents=(
                AgentSpec(1, "phase0_agent_01", (-2.0, 0.0), (2.0, 0.0)),
            ),
            pause_windows=((2.0, 3.0),),
        ),
        "head_on": Scenario(
            name="head_on",
            description="Two pedestrians pass each other in opposite directions.",
            agents=(
                AgentSpec(1, "phase0_agent_01", (-2.0, -0.12), (2.0, -0.12)),
                AgentSpec(2, "phase0_agent_02", (2.0, 0.12), (-2.0, 0.12)),
            ),
        ),
        "crossing": Scenario(
            name="crossing",
            description="Two pedestrians cross at right angles.",
            agents=(
                AgentSpec(1, "phase0_agent_01", (-2.0, 0.0), (2.0, 0.0)),
                AgentSpec(2, "phase0_agent_02", (0.0, -2.0), (0.0, 2.0)),
            ),
        ),
        "narrow_passage": Scenario(
            name="narrow_passage",
            description="Two pedestrians meet inside a bounded narrow corridor.",
            agents=(
                AgentSpec(
                    1,
                    "phase0_agent_01",
                    (-2.0, -0.10),
                    (2.0, -0.10),
                    obstacles=corridor_obstacles,
                ),
                AgentSpec(
                    2,
                    "phase0_agent_02",
                    (2.0, 0.10),
                    (-2.0, 0.10),
                    obstacles=corridor_obstacles,
                ),
            ),
        ),
        "narrow_gate": Scenario(
            name="narrow_gate",
            description="Two pedestrians serialize through a capacity-one corridor.",
            agents=(
                AgentSpec(
                    1,
                    "phase0_agent_01",
                    (-2.4, 0.0),
                    (1.7, 0.0),
                    obstacles=gate_obstacles,
                ),
                AgentSpec(
                    2,
                    "phase0_agent_02",
                    (2.4, 0.0),
                    (-1.7, 0.0),
                    obstacles=gate_obstacles,
                ),
            ),
            bottleneck=BottleneckSpec(
                zone_x_min=-1.0,
                zone_x_max=1.0,
                release_clearance_m=0.15,
                priority_order=(1, 2),
            ),
        ),
        "toilet_portal_corridor": Scenario(
            name="toilet_portal_corridor",
            description=(
                "One pedestrian traverses the toilet portal continuously to its "
                "directional clear plane."
            ),
            agents=(
                AgentSpec(
                    1,
                    "phase0_agent_01",
                    toilet_portal.outside[:2],
                    tuple(
                        toilet_portal.traversal_goal(
                            "entering",
                            overshoot_m=0.35,
                        )[:2]
                    ),
                    final_yaw=toilet_portal.yaw,
                    obstacles=toilet_portal_walls,
                ),
            ),
            portal=toilet_portal,
        ),
        "static_robot": Scenario(
            name="static_robot",
            description="A stationary robot blocks a pedestrian's nominal route.",
            agents=(
                AgentSpec(1, "phase0_agent_01", (-2.0, 0.0), (2.0, 0.0)),
            ),
            robot=RobotSpec(start=(0.0, 0.0)),
        ),
        "moving_robot": Scenario(
            name="moving_robot",
            description="A moving robot crosses a pedestrian's nominal route.",
            agents=(
                AgentSpec(1, "phase0_agent_01", (-2.0, 0.0), (2.0, 0.0)),
            ),
            robot=RobotSpec(
                start=(0.0, -2.0),
                velocity=(0.0, 0.50),
                motion_start_sec=0.0,
                motion_end_sec=8.0,
            ),
        ),
    }


def available_scenarios() -> Tuple[str, ...]:
    return tuple(_scenario_templates())


def build_scenario(name: str, seed: int, jitter_m: float = 0.02) -> Scenario:
    """Instantiate a scenario with deterministic, seed-controlled perturbations."""
    templates = _scenario_templates()
    if name not in templates:
        raise ValueError(
            f"Unknown scenario {name!r}; available: {', '.join(templates)}"
        )
    scenario = templates[name]
    if jitter_m <= 0.0:
        return scenario

    rng = random.Random(f"{seed}:{name}")
    agents: List[AgentSpec] = []
    for spec in scenario.agents:
        dx = rng.uniform(-jitter_m, jitter_m)
        dy = rng.uniform(-jitter_m, jitter_m)
        agents.append(
            replace(
                spec,
                start=(spec.start[0] + dx, spec.start[1] + dy),
                goal=(spec.goal[0] + dx, spec.goal[1] + dy),
            )
        )
    return replace(scenario, agents=tuple(agents))


def _distance(a: Point2D, b: Point2D) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def select_closest_obstacles(
    obstacles: Sequence[Point2D],
    position: Point2D,
    *,
    max_count: int,
    max_distance_m: float,
) -> Tuple[Point2D, ...]:
    """Select a bounded local obstacle sample for HuNav's closest_obs field."""
    ranked = sorted(
        enumerate(obstacles),
        key=lambda item: (_distance(item[1], position), item[0]),
    )
    selected = [
        obstacle
        for _, obstacle in ranked
        if _distance(obstacle, position) <= max_distance_m
    ]
    return tuple(selected[: max(0, int(max_count))])


def _agent_map(frame: Mapping[str, Any]) -> Dict[int, Mapping[str, Any]]:
    return {int(agent["id"]): agent for agent in frame["agents"]}


def analyze_trace(
    scenario: Scenario,
    frames: Sequence[Mapping[str, Any]],
    thresholds: Mapping[str, float],
) -> Dict[str, Any]:
    """Compute conservative, machine-readable smoke-test metrics."""
    result: Dict[str, Any] = {
        "scenario": scenario.name,
        "passed": False,
        "termination_reason": "no_frames",
        "frame_count": len(frames),
        "completed_agents": [],
        "stalled_agents": [],
        "nonfinite_state_count": 0,
        "pose_jump_count": 0,
        "overlap_frame_count": 0,
        "robot_overlap_frame_count": 0,
        "min_pair_distance_m": None,
        "min_robot_distance_m": None,
        "max_speed_mps": 0.0,
        "max_motion_yaw_error_rad": 0.0,
        "path_length_m": {},
        "final_goal_distance_m": {},
        "final_speed_mps": {},
        "final_yaw_error_rad": {},
        "safety_intervention_frames": 0,
        "safety_correction_count": 0,
        "safety_pair_contacts": 0,
        "safety_robot_contacts": 0,
        "safety_static_contacts": 0,
        "max_safety_correction_m": 0.0,
        "safety_residual_violation_count": 0,
        "coordination_hold_frames": 0,
        "terminal_alignment_count": 0,
        "portal_entered_agents": [],
        "portal_cleared_agents": [],
        "portal_entered": False,
        "portal_cleared": False,
        "portal_center_stop_frames": 0,
        "portal_center_stop_frame_count": 0,
        "portal_center_stop_duration_sec": 0.0,
        "portal_reverse_progress_count": 0,
        "portal_reverse_progress_m": 0.0,
        "portal_max_lateral_offset_m": None,
        "portal_max_lateral_deviation_m": None,
    }
    if not frames:
        return result

    specs = {spec.agent_id: spec for spec in scenario.agents}
    path_lengths = {agent_id: 0.0 for agent_id in specs}
    min_pair_distance = math.inf
    min_robot_distance = math.inf
    overlap_frames = 0
    robot_overlap_frames = 0
    nonfinite_count = 0
    pose_jump_count = 0
    max_speed = 0.0
    max_yaw_error = 0.0
    safety_intervention_frames = 0
    safety_correction_count = 0
    safety_pair_contacts = 0
    safety_robot_contacts = 0
    safety_static_contacts = 0
    max_safety_correction = 0.0
    safety_residual_violations = 0
    coordination_hold_frames = 0
    terminal_alignment_count = 0
    portal_entered: set[int] = set()
    portal_cleared: set[int] = set()
    portal_center_stop_frames = 0
    portal_center_stop_duration = 0.0
    portal_reverse_progress_count = 0
    portal_reverse_progress_m = 0.0
    portal_max_lateral_offset = 0.0
    portal_previous_progress: Dict[int, float] = {}

    previous: Mapping[str, Any] | None = None
    for frame in frames:
        frame_dt = (
            0.0
            if previous is None
            else max(
                0.0,
                float(frame["time_sec"]) - float(previous["time_sec"]),
            )
        )
        safety = frame.get("safety", {})
        safety_intervention_frames += int(bool(safety.get("intervened", False)))
        safety_correction_count += int(safety.get("correction_count", 0))
        safety_pair_contacts += int(safety.get("pedestrian_pair_contacts", 0))
        safety_robot_contacts += int(safety.get("robot_contacts", 0))
        safety_static_contacts += int(safety.get("static_contacts", 0))
        max_safety_correction = max(
            max_safety_correction,
            float(safety.get("max_correction_m", 0.0)),
        )
        safety_residual_violations += int(
            safety.get("residual_violation_count", 0)
        )
        coordination = frame.get("coordination", {})
        coordination_hold_frames += int(
            bool(coordination.get("held_agent_ids", ()))
        )
        terminal_alignment_count += len(
            frame.get("terminal_alignment_agents", ())
        )
        agents = _agent_map(frame)
        frame_overlap = False
        for agent_id, agent in agents.items():
            values = (
                float(agent["x"]),
                float(agent["y"]),
                float(agent["yaw"]),
                float(agent["vx"]),
                float(agent["vy"]),
            )
            if not all(math.isfinite(value) for value in values):
                nonfinite_count += 1
                continue
            speed = math.hypot(values[3], values[4])
            max_speed = max(max_speed, speed)
            if scenario.portal is not None:
                longitudinal, lateral = scenario.portal.coordinates(values)
                if (
                    longitudinal >= 0.0
                    and scenario.portal.contains_laterally(values)
                ):
                    portal_entered.add(agent_id)
                if scenario.portal.cleared(values, "entering"):
                    portal_cleared.add(agent_id)
                if longitudinal >= 0.0:
                    portal_max_lateral_offset = max(
                        portal_max_lateral_offset,
                        abs(lateral),
                    )
                center_band = float(thresholds.get("portal_center_band_m", 0.20))
                stop_speed = float(
                    thresholds.get(
                        "portal_stop_speed_mps",
                        thresholds["completion_speed_mps"],
                    )
                )
                if (
                    abs(longitudinal - 0.5 * scenario.portal.length_m)
                    <= center_band
                    and abs(lateral) <= scenario.portal.half_width_m
                    and speed <= stop_speed
                ):
                    portal_center_stop_frames += 1
                    portal_center_stop_duration += frame_dt
                previous_progress = portal_previous_progress.get(agent_id)
                progress_epsilon = float(
                    thresholds.get(
                        "portal_reverse_progress_epsilon_m",
                        thresholds.get("portal_progress_epsilon_m", 0.01),
                    )
                )
                if (
                    previous_progress is not None
                    and 0.0 <= longitudinal <= scenario.portal.length_m
                    and longitudinal < previous_progress - progress_epsilon
                ):
                    portal_reverse_progress_count += 1
                    portal_reverse_progress_m += previous_progress - longitudinal
                portal_previous_progress[agent_id] = longitudinal
            if speed >= thresholds["motion_yaw_speed_mps"]:
                velocity_yaw = math.atan2(values[4], values[3])
                yaw_error = abs(
                    math.atan2(
                        math.sin(values[2] - velocity_yaw),
                        math.cos(values[2] - velocity_yaw),
                    )
                )
                max_yaw_error = max(max_yaw_error, yaw_error)

        agent_ids = sorted(agents)
        robot = frame["robot"]
        frame_robot_overlap = False
        for agent_id, agent in agents.items():
            robot_distance = _distance(
                (float(agent["x"]), float(agent["y"])),
                (float(robot["x"]), float(robot["y"])),
            )
            min_robot_distance = min(min_robot_distance, robot_distance)
            robot_overlap_limit = (
                specs[agent_id].radius + float(robot["radius"])
            ) * thresholds["overlap_radius_scale"]
            if robot_distance < robot_overlap_limit:
                frame_robot_overlap = True
        for index, agent_id in enumerate(agent_ids):
            for other_id in agent_ids[index + 1 :]:
                left = agents[agent_id]
                right = agents[other_id]
                distance = _distance(
                    (float(left["x"]), float(left["y"])),
                    (float(right["x"]), float(right["y"])),
                )
                min_pair_distance = min(min_pair_distance, distance)
                overlap_limit = (
                    specs[agent_id].radius + specs[other_id].radius
                ) * thresholds["overlap_radius_scale"]
                if distance < overlap_limit:
                    frame_overlap = True
        overlap_frames += int(frame_overlap)
        robot_overlap_frames += int(frame_robot_overlap)

        if previous is not None:
            dt = float(frame["time_sec"]) - float(previous["time_sec"])
            previous_agents = _agent_map(previous)
            if dt > 0.0:
                for agent_id, agent in agents.items():
                    if agent_id not in previous_agents:
                        continue
                    prior = previous_agents[agent_id]
                    step_distance = _distance(
                        (float(agent["x"]), float(agent["y"])),
                        (float(prior["x"]), float(prior["y"])),
                    )
                    path_lengths[agent_id] += step_distance
                    prior_speed = math.hypot(
                        float(prior["vx"]), float(prior["vy"])
                    )
                    allowed_jump = max(
                        thresholds["pose_jump_min_m"],
                        prior_speed * dt + thresholds["pose_jump_slack_m"],
                    )
                    if step_distance > allowed_jump:
                        pose_jump_count += 1
        previous = frame

    final_agents = _agent_map(frames[-1])
    completed: List[int] = []
    stalled: List[int] = []
    stall_window_sec = thresholds["stall_window_sec"]
    window_start = float(frames[-1]["time_sec"]) - stall_window_sec
    trailing_frames = [
        frame for frame in frames if float(frame["time_sec"]) >= window_start
    ]
    for agent_id, spec in specs.items():
        final = final_agents[agent_id]
        goal_distance = _distance(
            (float(final["x"]), float(final["y"])),
            spec.goal,
        )
        final_speed = math.hypot(float(final["vx"]), float(final["vy"]))
        final_yaw_error = abs(
            math.atan2(
                math.sin(float(final["yaw"]) - spec.expected_final_yaw()),
                math.cos(float(final["yaw"]) - spec.expected_final_yaw()),
            )
        )
        result["final_goal_distance_m"][str(agent_id)] = goal_distance
        result["final_speed_mps"][str(agent_id)] = final_speed
        result["final_yaw_error_rad"][str(agent_id)] = final_yaw_error
        if (
            goal_distance <= spec.goal_radius + thresholds["goal_slack_m"]
            and final_speed <= thresholds["completion_speed_mps"]
        ):
            completed.append(agent_id)
            continue

        if len(trailing_frames) >= 2:
            first = _agent_map(trailing_frames[0])[agent_id]
            last = _agent_map(trailing_frames[-1])[agent_id]
            trailing_displacement = _distance(
                (float(first["x"]), float(first["y"])),
                (float(last["x"]), float(last["y"])),
            )
            portal_longitudinal = (
                None
                if scenario.portal is None
                else scenario.portal.coordinates(
                    (float(final["x"]), float(final["y"]))
                )[0]
            )
            has_passed_portal_longitudinally = (
                scenario.portal is not None
                and portal_longitudinal is not None
                and portal_longitudinal
                >= scenario.portal.length_m + scenario.portal.clearance_m
            )
            if (
                trailing_displacement < thresholds["stall_displacement_m"]
                and not has_passed_portal_longitudinally
            ):
                stalled.append(agent_id)

    result.update(
        {
            "completed_agents": completed,
            "stalled_agents": stalled,
            "nonfinite_state_count": nonfinite_count,
            "pose_jump_count": pose_jump_count,
            "overlap_frame_count": overlap_frames,
            "robot_overlap_frame_count": robot_overlap_frames,
            "min_pair_distance_m": (
                None if math.isinf(min_pair_distance) else min_pair_distance
            ),
            "min_robot_distance_m": (
                None if math.isinf(min_robot_distance) else min_robot_distance
            ),
            "max_speed_mps": max_speed,
            "max_motion_yaw_error_rad": max_yaw_error,
            "path_length_m": {
                str(agent_id): value for agent_id, value in path_lengths.items()
            },
            "safety_intervention_frames": safety_intervention_frames,
            "safety_correction_count": safety_correction_count,
            "safety_pair_contacts": safety_pair_contacts,
            "safety_robot_contacts": safety_robot_contacts,
            "safety_static_contacts": safety_static_contacts,
            "max_safety_correction_m": max_safety_correction,
            "safety_residual_violation_count": safety_residual_violations,
            "coordination_hold_frames": coordination_hold_frames,
            "terminal_alignment_count": terminal_alignment_count,
            "portal_entered_agents": sorted(portal_entered),
            "portal_cleared_agents": sorted(portal_cleared),
            "portal_entered": bool(portal_entered),
            "portal_cleared": (
                scenario.portal is not None
                and set(portal_cleared) == set(specs)
            ),
            "portal_center_stop_frames": portal_center_stop_frames,
            "portal_center_stop_frame_count": portal_center_stop_frames,
            "portal_center_stop_duration_sec": portal_center_stop_duration,
            "portal_reverse_progress_count": portal_reverse_progress_count,
            "portal_reverse_progress_m": portal_reverse_progress_m,
            "portal_max_lateral_offset_m": (
                portal_max_lateral_offset if scenario.portal is not None else None
            ),
            "portal_max_lateral_deviation_m": (
                portal_max_lateral_offset if scenario.portal is not None else None
            ),
        }
    )
    all_complete = len(completed) == len(specs)
    final_yaw_ok = all(
        error <= thresholds["final_yaw_tolerance_rad"]
        for error in result["final_yaw_error_rad"].values()
    )
    portal_clear_ok = (
        scenario.portal is None
        or set(result["portal_cleared_agents"]) == set(specs)
    )
    portal_stop_ok = (
        scenario.portal is None
        or (
            result["portal_center_stop_duration_sec"]
            <= float(thresholds.get("portal_center_stop_max_sec", 0.25))
            and result["portal_center_stop_frame_count"]
            <= int(thresholds.get("portal_center_stop_max_frames", 5))
        )
    )
    portal_progress_ok = (
        scenario.portal is None
        or result["portal_reverse_progress_count"] == 0
    )
    passed = (
        all_complete
        and final_yaw_ok
        and portal_clear_ok
        and portal_stop_ok
        and portal_progress_ok
        and not stalled
        and nonfinite_count == 0
        and pose_jump_count == 0
        and overlap_frames == 0
        and robot_overlap_frames == 0
        and safety_residual_violations == 0
    )
    result["passed"] = passed
    if passed:
        result["termination_reason"] = "completed"
    elif nonfinite_count:
        result["termination_reason"] = "nonfinite_state"
    elif safety_residual_violations:
        result["termination_reason"] = "safety_violation"
    elif pose_jump_count:
        result["termination_reason"] = "pose_jump"
    elif overlap_frames:
        result["termination_reason"] = "pedestrian_overlap"
    elif robot_overlap_frames:
        result["termination_reason"] = "robot_overlap"
    elif not portal_stop_ok:
        result["termination_reason"] = "portal_center_stop"
    elif not portal_progress_ok:
        result["termination_reason"] = "portal_reverse_progress"
    elif all_complete and not portal_clear_ok:
        result["termination_reason"] = "portal_not_cleared"
    elif stalled:
        result["termination_reason"] = "stalled"
    elif not final_yaw_ok:
        result["termination_reason"] = "final_yaw"
    else:
        result["termination_reason"] = "timeout"
    return result


def aggregate_results(results: Iterable[Mapping[str, Any]]) -> Dict[str, Any]:
    scenario_results = list(results)
    failed = [result for result in scenario_results if not result["passed"]]
    return {
        "passed": bool(scenario_results)
        and all(bool(result["passed"]) for result in scenario_results),
        "scenario_count": len(scenario_results),
        "passed_count": sum(bool(result["passed"]) for result in scenario_results),
        "failed_scenarios": [result["scenario"] for result in failed],
        "failed_cases": [
            f"seed={result.get('seed', 'unknown')}:{result['scenario']}"
            for result in failed
        ],
        "scenarios": scenario_results,
    }
