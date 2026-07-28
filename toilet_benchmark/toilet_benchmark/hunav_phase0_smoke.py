"""Standalone HuNav Phase 0 smoke runner.

This module deliberately does not import or modify the toilet director. It drives
the HuNav agent manager through its public services and records deterministic
traces that can be inspected without Isaac Sim.
"""

from __future__ import annotations

import argparse
from contextlib import suppress
from copy import deepcopy
from datetime import datetime
import json
import logging
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any, Dict, List, Mapping, Sequence

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from hunav_msgs.msg import Agent, AgentBehavior, Agents
from hunav_msgs.srv import ComputeAgents, ResetAgents
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
import yaml

from toilet_benchmark.hunav_phase0_core import (
    AgentSpec,
    Scenario,
    aggregate_results,
    analyze_trace,
    available_scenarios,
    build_scenario,
    select_closest_obstacles,
)
from toilet_benchmark.hunav_phase0_coordination import (
    DeterministicBottleneckCoordinator,
)
from toilet_benchmark.hunav_phase0_safety import (
    SafetyConfig,
    project_safe_step,
)


LOGGER = logging.getLogger("hunav_phase0")


def _default_config_path() -> Path:
    with suppress(Exception):
        return (
            Path(get_package_share_directory("toilet_benchmark"))
            / "config"
            / "hunav_phase0_smoke.yaml"
        )
    return Path(__file__).resolve().parents[1] / "config" / "hunav_phase0_smoke.yaml"


def _load_config(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError(f"Phase 0 config must be a mapping: {path}")
    return config


def _stamp(time_sec: float):
    seconds = int(math.floor(time_sec))
    nanoseconds = int(round((time_sec - seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        seconds += 1
        nanoseconds -= 1_000_000_000
    from builtin_interfaces.msg import Time

    return Time(sec=seconds, nanosec=nanoseconds)


def _set_pose(agent: Agent, x: float, y: float, yaw: float) -> None:
    agent.position.position.x = float(x)
    agent.position.position.y = float(y)
    agent.position.position.z = 0.0
    agent.position.orientation.z = math.sin(yaw * 0.5)
    agent.position.orientation.w = math.cos(yaw * 0.5)
    agent.yaw = float(yaw)


def _build_agent(spec: AgentSpec, behavior_cfg: Mapping[str, float]) -> Agent:
    message = Agent()
    message.id = spec.agent_id
    message.type = Agent.PERSON
    message.name = spec.name
    message.group_id = -1
    yaw = math.atan2(spec.goal[1] - spec.start[1], spec.goal[0] - spec.start[0])
    _set_pose(message, spec.start[0], spec.start[1], yaw)
    message.desired_velocity = float(spec.desired_velocity)
    message.radius = float(spec.radius)
    message.goal_radius = float(spec.goal_radius)
    message.cyclic_goals = False

    goal = Pose()
    goal.position.x = float(spec.goal[0])
    goal.position.y = float(spec.goal[1])
    goal.orientation.w = 1.0
    message.goals = [goal]
    message.closest_obs = []

    behavior = AgentBehavior()
    behavior.type = AgentBehavior.BEH_REGULAR
    behavior.state = AgentBehavior.BEH_NO_ACTIVE
    behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
    behavior.duration = 0.0
    behavior.once = False
    behavior.vel = float(spec.desired_velocity)
    behavior.dist = 1.0
    behavior.social_force_factor = float(behavior_cfg["social_force_factor"])
    behavior.goal_force_factor = float(behavior_cfg["goal_force_factor"])
    behavior.obstacle_force_factor = float(
        behavior_cfg["obstacle_force_factor"]
    )
    behavior.other_force_factor = float(behavior_cfg["other_force_factor"])
    message.behavior = behavior
    return message


def _build_robot(scenario: Scenario, time_sec: float) -> Agent:
    x, y, vx, vy = scenario.robot.state_at(time_sec)
    robot = Agent()
    robot.id = 0
    robot.type = Agent.ROBOT
    robot.name = "phase0_robot"
    robot.group_id = -1
    yaw = math.atan2(vy, vx) if math.hypot(vx, vy) > 1e-6 else 0.0
    _set_pose(robot, x, y, yaw)
    robot.velocity.linear.x = float(vx)
    robot.velocity.linear.y = float(vy)
    robot.desired_velocity = math.hypot(vx, vy)
    robot.radius = float(scenario.robot.radius)
    return robot


def _restore_request_fields(
    agents: Agents,
    specs: Mapping[int, AgentSpec],
    behavior_cfg: Mapping[str, float],
    obstacle_cfg: Mapping[str, Any],
) -> None:
    """Restore fields omitted by HuNav's getUpdatedAgentMsg response."""
    for agent in agents.agents:
        spec = specs[int(agent.id)]
        agent.desired_velocity = float(spec.desired_velocity)
        agent.radius = float(spec.radius)
        agent.goal_radius = float(spec.goal_radius)
        agent.cyclic_goals = False
        closest = select_closest_obstacles(
            spec.obstacles,
            (
                float(agent.position.position.x),
                float(agent.position.position.y),
            ),
            max_count=int(obstacle_cfg["nearest_count"]),
            max_distance_m=float(obstacle_cfg["max_distance_m"]),
        )
        agent.closest_obs = [Point(x=x, y=y, z=0.0) for x, y in closest]
        agent.behavior.configuration = AgentBehavior.BEH_CONF_CUSTOM
        agent.behavior.social_force_factor = float(
            behavior_cfg["social_force_factor"]
        )
        agent.behavior.goal_force_factor = float(
            behavior_cfg["goal_force_factor"]
        )
        agent.behavior.obstacle_force_factor = float(
            behavior_cfg["obstacle_force_factor"]
        )
        agent.behavior.other_force_factor = float(
            behavior_cfg["other_force_factor"]
        )


def _freeze_agent_ids(
    previous: Agents,
    updated: Agents,
    frozen_ids: Sequence[int],
) -> Agents:
    frozen = deepcopy(updated)
    previous_by_id = {int(agent.id): agent for agent in previous.agents}
    for agent in frozen.agents:
        if int(agent.id) not in frozen_ids:
            continue
        prior = previous_by_id[int(agent.id)]
        agent.position = deepcopy(prior.position)
        agent.yaw = float(prior.yaw)
        agent.velocity.linear.x = 0.0
        agent.velocity.linear.y = 0.0
        agent.velocity.angular.z = 0.0
        agent.linear_vel = 0.0
        agent.angular_vel = 0.0
    return frozen


def _freeze_agents(previous: Agents, updated: Agents) -> Agents:
    return _freeze_agent_ids(
        previous,
        updated,
        [int(agent.id) for agent in previous.agents],
    )


def _restore_terminal_holds(
    agents: Agents,
    terminal_holds: Dict[int, Agent],
) -> None:
    """Restore terminal agents before applying multi-agent safety constraints."""
    for agent in agents.agents:
        agent_id = int(agent.id)
        if agent_id not in terminal_holds:
            continue
        held = terminal_holds[agent_id]
        agent.position = deepcopy(held.position)
        agent.yaw = float(held.yaw)
        agent.velocity.linear.x = 0.0
        agent.velocity.linear.y = 0.0
        agent.velocity.angular.z = 0.0
        agent.linear_vel = 0.0
        agent.angular_vel = 0.0


def _capture_terminal_holds(
    agents: Agents,
    terminal_holds: Dict[int, Agent],
    specs: Mapping[int, AgentSpec],
    alignment_enabled: bool,
) -> List[int]:
    """Capture and stop an agent once HuNav consumes its final goal."""
    aligned: List[int] = []
    for agent in agents.agents:
        agent_id = int(agent.id)
        if agent_id not in terminal_holds and not agent.goals:
            if alignment_enabled:
                _set_pose(
                    agent,
                    float(agent.position.position.x),
                    float(agent.position.position.y),
                    specs[agent_id].expected_final_yaw(),
                )
                aligned.append(agent_id)
            terminal_holds[agent_id] = deepcopy(agent)
        if agent_id not in terminal_holds:
            continue
        agent.velocity.linear.x = 0.0
        agent.velocity.linear.y = 0.0
        agent.velocity.angular.z = 0.0
        agent.linear_vel = 0.0
        agent.angular_vel = 0.0
    return aligned


def _apply_hard_safety(
    previous: Agents,
    proposed: Agents,
    scenario: Scenario,
    time_sec: float,
    dt: float,
    safety_cfg: Mapping[str, Any],
    fixed_ids: Sequence[int],
) -> Dict[str, Any]:
    if not bool(safety_cfg.get("enabled", True)):
        return {
            "enabled": False,
            "intervened": False,
            "correction_count": 0,
            "pedestrian_pair_contacts": 0,
            "robot_contacts": 0,
            "static_contacts": 0,
            "max_correction_m": 0.0,
            "constrained_agents": [],
            "residual_violation_count": 0,
        }

    previous_by_id = {int(agent.id): agent for agent in previous.agents}
    proposed_by_id = {int(agent.id): agent for agent in proposed.agents}
    specs = {spec.agent_id: spec for spec in scenario.agents}
    result = project_safe_step(
        previous={
            agent_id: (
                float(agent.position.position.x),
                float(agent.position.position.y),
            )
            for agent_id, agent in previous_by_id.items()
        },
        proposed={
            agent_id: (
                float(agent.position.position.x),
                float(agent.position.position.y),
            )
            for agent_id, agent in proposed_by_id.items()
        },
        radii={agent_id: float(spec.radius) for agent_id, spec in specs.items()},
        robot_previous=scenario.robot.state_at(max(0.0, time_sec - dt))[:2],
        robot_proposed=scenario.robot.state_at(time_sec)[:2],
        robot_radius=float(scenario.robot.radius),
        static_obstacles={
            agent_id: spec.obstacles for agent_id, spec in specs.items()
        },
        fixed_ids=fixed_ids,
        config=SafetyConfig(
            clearance_m=float(safety_cfg["clearance_m"]),
            contact_epsilon_m=float(safety_cfg["contact_epsilon_m"]),
            static_obstacle_radius_m=float(
                safety_cfg["static_obstacle_radius_m"]
            ),
            max_iterations=int(safety_cfg["max_iterations"]),
        ),
    )
    safe_dt = max(float(dt), 1e-6)
    for agent_id, safe_position in result.positions.items():
        agent = proposed_by_id[agent_id]
        prior = previous_by_id[agent_id]
        vx = (safe_position[0] - float(prior.position.position.x)) / safe_dt
        vy = (safe_position[1] - float(prior.position.position.y)) / safe_dt
        agent.position.position.x = float(safe_position[0])
        agent.position.position.y = float(safe_position[1])
        agent.velocity.linear.x = vx
        agent.velocity.linear.y = vy
        agent.linear_vel = math.hypot(vx, vy)
        if agent.linear_vel > 1e-5 and agent_id not in fixed_ids:
            yaw = math.atan2(vy, vx)
            _set_pose(agent, safe_position[0], safe_position[1], yaw)

    diagnostics = result.diagnostics.to_dict()
    diagnostics["enabled"] = True
    return diagnostics


def _frame_from_messages(
    time_sec: float,
    agents: Agents,
    scenario: Scenario,
) -> Dict[str, Any]:
    specs = {spec.agent_id: spec for spec in scenario.agents}
    x, y, vx, vy = scenario.robot.state_at(time_sec)
    return {
        "time_sec": round(float(time_sec), 9),
        "agents": [
            {
                "id": int(agent.id),
                "name": agent.name,
                "x": float(agent.position.position.x),
                "y": float(agent.position.position.y),
                "yaw": float(agent.yaw),
                "vx": float(agent.velocity.linear.x),
                "vy": float(agent.velocity.linear.y),
                "radius": float(specs[int(agent.id)].radius),
            }
            for agent in sorted(agents.agents, key=lambda item: int(item.id))
        ],
        "robot": {
            "x": x,
            "y": y,
            "vx": vx,
            "vy": vy,
            "radius": scenario.robot.radius,
        },
    }


class HuNavSmokeClient(Node):
    def __init__(self, namespace: str):
        super().__init__("hunav_phase0_smoke", namespace=namespace)
        self.compute_client = self.create_client(ComputeAgents, "compute_agents")
        self.reset_client = self.create_client(ResetAgents, "reset_agents")

    def wait_ready(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            if self.compute_client.wait_for_service(
                timeout_sec=min(0.5, remaining)
            ) and self.reset_client.wait_for_service(timeout_sec=0.1):
                return True
        return False

    def reset(self, agents: Agents, robot: Agent, timeout_sec: float) -> bool:
        request = ResetAgents.Request()
        request.current_agents = agents
        request.robot = robot
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done() or future.exception() is not None:
            return False
        return bool(future.result().ok)

    def compute(
        self,
        agents: Agents,
        robot: Agent,
        timeout_sec: float,
    ) -> Agents:
        request = ComputeAgents.Request()
        request.current_agents = agents
        request.robot = robot
        future = self.compute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError("Timed out waiting for HuNav compute_agents")
        if future.exception() is not None:
            raise RuntimeError(f"HuNav compute_agents failed: {future.exception()}")
        return future.result().updated_agents


class ManagedHuNavProcess:
    def __init__(self, namespace: str, log_path: Path):
        self.namespace = namespace
        self.log_path = log_path
        self.process: subprocess.Popen[str] | None = None
        self._stream = None

    def start(self) -> None:
        ros2 = shutil.which("ros2")
        if not ros2:
            raise RuntimeError("ros2 executable not found; source the workspace first")
        self._stream = self.log_path.open("w", encoding="utf-8")
        command = [
            ros2,
            "run",
            "hunav_agent_manager",
            "hunav_agent_manager",
            "--ros-args",
            "-r",
            f"__ns:={self.namespace}",
            "-p",
            "publish_tf:=false",
            "-p",
            "publish_sfm_forces:=false",
            "-p",
            "hunav_loader.publish_people:=false",
        ]
        LOGGER.info("Starting HuNav manager: %s", " ".join(command))
        self.process = subprocess.Popen(
            command,
            stdout=self._stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )

    def assert_running(self) -> None:
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError(
                f"HuNav manager exited early with code {self.process.returncode}; "
                f"see {self.log_path}"
            )

    def stop(self) -> None:
        if self.process is not None and self.process.poll() is None:
            with suppress(ProcessLookupError):
                os.killpg(self.process.pid, signal.SIGINT)
            try:
                self.process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                with suppress(ProcessLookupError):
                    os.killpg(self.process.pid, signal.SIGTERM)
                try:
                    self.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    with suppress(ProcessLookupError):
                        os.killpg(self.process.pid, signal.SIGKILL)
                    self.process.wait(timeout=2.0)
        if self._stream is not None:
            self._stream.close()


def _run_scenario(
    node: HuNavSmokeClient,
    scenario: Scenario,
    config: Mapping[str, Any],
    trace_path: Path,
) -> Dict[str, Any]:
    simulation = config["simulation"]
    behavior = config["behavior"]
    thresholds = config["thresholds"]
    safety_cfg = config["safety"]
    coordination_cfg = config["coordination"]
    alignment_cfg = config["terminal_alignment"]
    obstacle_cfg = config["obstacle_sampling"]
    dt = float(simulation["dt_sec"])
    duration = float(simulation["duration_sec"])
    service_timeout = float(simulation["service_timeout_sec"])
    specs = {spec.agent_id: spec for spec in scenario.agents}

    current = Agents()
    current.header.frame_id = "map"
    current.header.stamp = _stamp(1.0)
    current.agents = [_build_agent(spec, behavior) for spec in scenario.agents]
    _restore_request_fields(current, specs, behavior, obstacle_cfg)
    robot = _build_robot(scenario, 0.0)
    if not node.reset(current, robot, service_timeout):
        raise RuntimeError(f"HuNav reset failed before scenario {scenario.name}")

    frames: List[Dict[str, Any]] = []
    terminal_holds: Dict[int, Agent] = {}
    coordinator = None
    if bool(coordination_cfg.get("enabled", True)) and scenario.bottleneck is not None:
        coordinator = DeterministicBottleneckCoordinator(
            routes={
                spec.agent_id: (spec.start, spec.goal) for spec in scenario.agents
            },
            spec=scenario.bottleneck,
        )
    with trace_path.open("w", encoding="utf-8") as trace:
        steps = int(math.ceil(duration / dt)) + 1
        for step in range(steps):
            time_sec = step * dt
            current.header.frame_id = "map"
            current.header.stamp = _stamp(1.0 + time_sec)
            _restore_request_fields(current, specs, behavior, obstacle_cfg)
            current_positions = {
                int(agent.id): (
                    float(agent.position.position.x),
                    float(agent.position.position.y),
                )
                for agent in current.agents
            }
            if coordinator is not None:
                coordinator.update(current_positions)
                held_ids = coordinator.held_agent_ids()
                coordination = coordinator.status()
            else:
                held_ids = ()
                coordination = {
                    "enabled": False,
                    "active_agent_id": None,
                    "held_agent_ids": [],
                    "completed_agent_ids": [],
                }
            robot = _build_robot(scenario, time_sec)
            updated = node.compute(current, robot, service_timeout)
            if scenario.paused_at(time_sec):
                updated = _freeze_agents(current, updated)
            elif held_ids:
                updated = _freeze_agent_ids(current, updated, held_ids)
            _restore_terminal_holds(updated, terminal_holds)
            fixed_ids = tuple(sorted(set(terminal_holds) | set(held_ids)))
            safety = _apply_hard_safety(
                current,
                updated,
                scenario,
                time_sec,
                dt,
                safety_cfg,
                fixed_ids,
            )
            aligned_agents = _capture_terminal_holds(
                updated,
                terminal_holds,
                specs,
                bool(alignment_cfg.get("enabled", True)),
            )
            updated.header.frame_id = "map"
            updated.header.stamp = current.header.stamp
            _restore_request_fields(updated, specs, behavior, obstacle_cfg)

            frame = _frame_from_messages(time_sec, updated, scenario)
            frame["terminal_agents"] = sorted(terminal_holds)
            frame["safety"] = safety
            frame["coordination"] = coordination
            frame["terminal_alignment_agents"] = aligned_agents
            frames.append(frame)
            trace.write(json.dumps(frame, sort_keys=True) + "\n")
            current = updated

            if step > 1:
                interim = analyze_trace(scenario, frames, thresholds)
                if len(interim["completed_agents"]) == len(scenario.agents):
                    break

    result = analyze_trace(scenario, frames, thresholds)
    result["duration_sec"] = frames[-1]["time_sec"] if frames else 0.0
    result["trace_path"] = str(trace_path)
    return result


def _configure_logging(log_path: Path) -> None:
    LOGGER.setLevel(logging.INFO)
    LOGGER.handlers.clear()
    formatter = logging.Formatter(
        "%(asctime)s %(levelname)s %(message)s",
        datefmt="%Y-%m-%dT%H:%M:%S",
    )
    console = logging.StreamHandler()
    console.setFormatter(formatter)
    file_handler = logging.FileHandler(log_path, encoding="utf-8")
    file_handler.setFormatter(formatter)
    LOGGER.addHandler(console)
    LOGGER.addHandler(file_handler)


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=_default_config_path())
    parser.add_argument(
        "--scenarios",
        default="",
        help="Comma-separated scenario names; defaults to config selection.",
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--seed-count",
        type=int,
        default=1,
        help="Run consecutive seeds starting at --seed in one manager process.",
    )
    parser.add_argument("--log-dir", type=Path, default=None)
    parser.add_argument(
        "--no-start-manager",
        action="store_true",
        help="Connect to an already-running manager in --namespace.",
    )
    parser.add_argument("--namespace", default="")
    parser.add_argument(
        "--list-scenarios",
        action="store_true",
        help="Print available scenarios and exit.",
    )
    parser.add_argument(
        "--disable-hard-safety",
        action="store_true",
        help="Run raw HuNav proposals without geometric projection.",
    )
    parser.add_argument(
        "--disable-coordination",
        action="store_true",
        help="Disable deterministic bottleneck ownership and yielding.",
    )
    parser.add_argument(
        "--disable-terminal-alignment",
        action="store_true",
        help="Keep HuNav's final yaw instead of aligning after goal consumption.",
    )
    return parser.parse_args(argv)


def main(args: Sequence[str] | None = None) -> int:
    cli_args = list(args) if args is not None else remove_ros_args(sys.argv)[1:]
    options = _parse_args(cli_args)
    if options.list_scenarios:
        print("\n".join(available_scenarios()))
        return 0

    config = _load_config(options.config)
    if options.disable_hard_safety:
        config = deepcopy(config)
        config.setdefault("safety", {})["enabled"] = False
    if options.disable_coordination:
        config = deepcopy(config)
        config.setdefault("coordination", {})["enabled"] = False
    if options.disable_terminal_alignment:
        config = deepcopy(config)
        config.setdefault("terminal_alignment", {})["enabled"] = False
    seed = int(options.seed if options.seed is not None else config["seed"])
    seed_count = max(1, int(options.seed_count))
    seeds = list(range(seed, seed + seed_count))
    selected = (
        [item.strip() for item in options.scenarios.split(",") if item.strip()]
        if options.scenarios
        else list(config["scenarios"])
    )
    unknown = sorted(set(selected) - set(available_scenarios()))
    if unknown:
        raise ValueError(f"Unknown Phase 0 scenarios: {', '.join(unknown)}")

    seed_suffix = f"seed{seed}" if seed_count == 1 else f"seed{seed}_n{seed_count}"
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f") + f"_{seed_suffix}"
    log_dir = options.log_dir or Path("/tmp/toilet_hunav_debug") / run_id
    log_dir.mkdir(parents=True, exist_ok=False)
    ros_log_dir = log_dir / "ros_logs"
    ros_log_dir.mkdir()
    os.environ.setdefault("ROS_LOG_DIR", str(ros_log_dir))
    _configure_logging(log_dir / "runner.log")
    namespace = options.namespace.strip() or f"/toilet_hunav_phase0_{os.getpid()}"
    if not namespace.startswith("/"):
        namespace = "/" + namespace

    manager = ManagedHuNavProcess(namespace, log_dir / "hunav.log")
    node: HuNavSmokeClient | None = None
    results: List[Dict[str, Any]] = []
    manifest = {
        "run_id": run_id,
        "seed": seed,
        "seeds": seeds,
        "namespace": namespace,
        "config_path": str(options.config.resolve()),
        "scenarios": selected,
        "manager_started_by_runner": not options.no_start_manager,
        "config": config,
    }
    (log_dir / "manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    exit_code = 1
    try:
        if not options.no_start_manager:
            manager.start()
        rclpy.init(args=None)
        node = HuNavSmokeClient(namespace)
        if not node.wait_ready(float(config["simulation"]["startup_timeout_sec"])):
            manager.assert_running()
            raise TimeoutError(
                f"HuNav services did not become ready in namespace {namespace}"
            )
        LOGGER.info(
            "HuNav services ready; scenarios=%s seeds=%s log_dir=%s",
            ",".join(selected),
            seeds,
            log_dir,
        )

        for scenario_seed in seeds:
            for name in selected:
                scenario = build_scenario(
                    name,
                    scenario_seed,
                    jitter_m=float(config["simulation"]["seed_jitter_m"]),
                )
                LOGGER.info("Running scenario %s seed=%d", name, scenario_seed)
                trace_name = (
                    f"trace_{name}.jsonl"
                    if seed_count == 1
                    else f"trace_seed{scenario_seed}_{name}.jsonl"
                )
                result = _run_scenario(
                    node,
                    scenario,
                    config,
                    log_dir / trace_name,
                )
                result["seed"] = scenario_seed
                result["scenario_definition"] = scenario.to_dict()
                results.append(result)
                LOGGER.info(
                    "Scenario %s seed=%d: passed=%s reason=%s frames=%d",
                    name,
                    scenario_seed,
                    result["passed"],
                    result["termination_reason"],
                    result["frame_count"],
                )
                manager.assert_running()

        aggregate = aggregate_results(results)
        aggregate.update(
            {
                "run_id": run_id,
                "seed": seed,
                "seeds": seeds,
                "log_dir": str(log_dir),
            }
        )
        (log_dir / "result.json").write_text(
            json.dumps(aggregate, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        exit_code = 0 if aggregate["passed"] else 2
        LOGGER.info(
            "Phase 0 complete: passed=%s (%d/%d), result=%s",
            aggregate["passed"],
            aggregate["passed_count"],
            aggregate["scenario_count"],
            log_dir / "result.json",
        )
    except KeyboardInterrupt:
        LOGGER.warning("Phase 0 interrupted by user")
        exit_code = 130
    except Exception:
        LOGGER.exception("Phase 0 runner failed")
        failure = aggregate_results(results)
        failure.update(
            {
                "run_id": run_id,
                "seed": seed,
                "seeds": seeds,
                "log_dir": str(log_dir),
                "runner_error": True,
            }
        )
        (log_dir / "result.json").write_text(
            json.dumps(failure, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if not options.no_start_manager:
            manager.stop()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
