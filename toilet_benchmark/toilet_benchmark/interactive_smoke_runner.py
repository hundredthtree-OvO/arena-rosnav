"""Run the Isaac bridge, scene spawn, and one HuNav director smoke test."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Mapping, Sequence

import yaml


DEFAULT_REQUIRED_SERVICES = (
    "/isaac/import_usd",
    "/isaac/move_pedestrians",
    "/isaac/spawn_pedestrian",
    "/isaac/urdf_to_usd",
)
BRIDGE_STARTUP_MARKER = "Simulation App Startup Complete"
PEDESTRIAN_CHARACTERIZATION_OVERLAP_DISTANCE_M = 0.60


@dataclass(frozen=True)
class PipelinePaths:
    workspace: Path
    arena_isaac: Path
    profile: Path
    benchmark: Path
    output_root: Path


@dataclass(frozen=True)
class RobotIntervention:
    trigger_x: float
    hold_sec: float
    away_pose: tuple[float, float, float, float]
    blocking_pose: tuple[float, float, float, float]


@dataclass(frozen=True)
class DynamicCrossingProfile:
    name: str
    reset_pose: tuple[float, float, float, float]
    post_pose: tuple[float, float, float, float] | None
    trigger_x: float
    topic: str
    linear_x: float
    motion_sec: float


@dataclass(frozen=True)
class ScenarioProfile:
    name: str
    initial_agents: int | None = None
    target_resource: str | None = None
    initial_spawn_interval_sec: float | None = None
    urinal_service_time_sec: tuple[float, float] | None = None
    robot_intervention: str | None = None
    robot_blocking_pose: tuple[float, float, float, float] | None = None
    robot_away_pose: tuple[float, float, float, float] | None = None
    trigger_marker: str | None = None
    release_marker: str | None = None


_BASE_DYNAMIC_CROSSING_PROFILE = DynamicCrossingProfile(
    name="custom",
    reset_pose=(0.0, -0.8, 0.03, math.pi / 2.0),
    post_pose=None,
    trigger_x=-1.20,
    topic="/cmd_vel_gamepad_diff",
    linear_x=0.20,
    motion_sec=5.0,
)

DYNAMIC_CROSSING_PROFILES: dict[str, DynamicCrossingProfile] = {
    "custom": _BASE_DYNAMIC_CROSSING_PROFILE,
    "crossing_clear": DynamicCrossingProfile(
        name="crossing_clear",
        reset_pose=(1.0, -0.8, 0.03, math.pi / 2.0),
        post_pose=(4.0, -3.0, 0.03, 0.0),
        trigger_x=-1.50,
        topic="/cmd_vel_gamepad_diff",
        linear_x=0.20,
        motion_sec=3.0,
    ),
    "crossing_conflict": DynamicCrossingProfile(
        name="crossing_conflict",
        # Calibrated stock People locomotion reaches the crossing more slowly
        # than its task-level desired speed, so trigger after the doorway.
        reset_pose=(0.0, 0.15, 0.03, math.pi / 2.0),
        post_pose=(4.0, -3.0, 0.03, 0.0),
        trigger_x=-2.00,
        topic="/cmd_vel_gamepad_diff",
        linear_x=0.20,
        motion_sec=5.0,
    ),
}

SCENARIO_PROFILES: dict[str, ScenarioProfile] = {
    "standard": ScenarioProfile(name="standard"),
    "occupied_passage": ScenarioProfile(
        name="occupied_passage",
        initial_agents=2,
        target_resource="urinal_2,urinal_1",
        # Agent 01 reaches urinal_2 before agent 02 is activated. The fixed
        # service window then keeps it present throughout agent 02's passage.
        initial_spawn_interval_sec=40.0,
        urinal_service_time_sec=(55.0, 55.0),
        robot_intervention="occupied_passage",
        robot_blocking_pose=(2.05, 0.18, 0.03, 0.0),
        robot_away_pose=(4.0, -3.0, 0.03, 0.0),
        trigger_marker="toilet_agent_01 started using urinal_2",
        release_marker="toilet_agent_01 leaving urinal_2",
    ),
    "passable_passage_u1": ScenarioProfile(
        name="passable_passage_u1",
        initial_agents=2,
        target_resource="urinal_3,urinal_1",
        initial_spawn_interval_sec=40.0,
        urinal_service_time_sec=(70.0, 70.0),
        robot_intervention="occupied_passage",
        robot_blocking_pose=(0.0, 0.0, 0.03, 0.0),
        robot_away_pose=(4.0, -3.0, 0.03, 0.0),
        trigger_marker="toilet_agent_01 started using urinal_3",
        release_marker="toilet_agent_01 leaving urinal_3",
    ),
    "passable_passage_u2": ScenarioProfile(
        name="passable_passage_u2",
        initial_agents=2,
        target_resource="urinal_3,urinal_2",
        initial_spawn_interval_sec=40.0,
        urinal_service_time_sec=(70.0, 70.0),
        robot_intervention="occupied_passage",
        robot_blocking_pose=(0.0, 0.0, 0.03, 0.0),
        robot_away_pose=(4.0, -3.0, 0.03, 0.0),
        trigger_marker="toilet_agent_01 started using urinal_3",
        release_marker="toilet_agent_01 leaving urinal_3",
    ),
}


def discover_paths() -> PipelinePaths:
    package_root = Path(__file__).resolve().parents[1]
    workspace = package_root.parents[3]
    arena_isaac = workspace / "src" / "arena" / "arena-isaac"
    return PipelinePaths(
        workspace=workspace,
        arena_isaac=arena_isaac,
        profile=arena_isaac / "scripts" / "profiles" / "shenxinfu_841837.yaml",
        benchmark=package_root / "config" / "toilet_benchmark.yaml",
        output_root=Path("/tmp/toilet_pipeline_runs"),
    )


def make_benchmark_variant(
    source: Path,
    destination: Path,
    *,
    diagnostic_dir: Path,
    reaction_override: bool | None,
    avoidance_override: bool | None = None,
    local_motion_shadow_override: bool | None = None,
    seed_override: int | None = None,
    scenario_profile: ScenarioProfile | None = None,
) -> Mapping:
    payload = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    if seed_override is not None:
        payload.setdefault("arrival", {})["seed"] = int(seed_override)
    hunav = payload.setdefault("motion_backend", {}).setdefault("hunav", {})
    hunav["diagnostic_log_dir"] = str(diagnostic_dir)
    if reaction_override is not None:
        reaction = hunav.setdefault("robot_proximity_reaction", {})
        reaction["enabled"] = bool(reaction_override)
    if avoidance_override is not None:
        avoidance = hunav.setdefault("regular_avoidance", {})
        avoidance["enabled"] = bool(avoidance_override)
    if local_motion_shadow_override is not None:
        local_motion_shadow = hunav.setdefault("local_motion_shadow", {})
        local_motion_shadow["enabled"] = bool(local_motion_shadow_override)
    if scenario_profile is not None:
        if scenario_profile.initial_spawn_interval_sec is not None:
            payload.setdefault("director", {})["initial_spawn_interval_sec"] = float(
                scenario_profile.initial_spawn_interval_sec
            )
        if scenario_profile.urinal_service_time_sec is not None:
            payload.setdefault("service_time_sec", {})["urinal"] = [
                float(value)
                for value in scenario_profile.urinal_service_time_sec
            ]
    destination.write_text(
        yaml.safe_dump(payload, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return payload


def _diagnostic_samples(text: str) -> dict[str, list[dict[str, object]]]:
    samples: dict[str, list[dict[str, object]]] = {}
    pattern = re.compile(
        r"\[(?P<time>\d+\.\d+)\].*HuNav diagnostics: "
        r"agent=(?P<agent>[^,]+), generation=(?P<generation>\d+), "
        r"phase=(?P<phase>[^,]+), pose=\((?P<x>-?[\d.]+),(?P<y>-?[\d.]+)\), "
        r"yaw=(?P<yaw>-?[\d.]+), speed=(?P<speed>[\d.]+), "
        r"route_progress_m=(?P<progress>[\d.]+).*?"
        r"behavior_profile=(?P<profile>[^,]+), native_behavior=(?P<behavior>[^,]+), "
        r".*?native_behavior_state=(?P<behavior_state>[^,]+).*?"
        r"pedestrian_pair_contacts=(?P<pair_contacts>\d+)"
    )
    for match in pattern.finditer(text):
        avoidance_match = re.search(
            r"avoidance_side=([+-]?\d+)",
            match.group(0),
        )
        shadow_enabled_match = re.search(
            r"local_shadow_enabled=(True|False)",
            match.group(0),
        )
        shadow_feasible_match = re.search(
            r"local_shadow_feasible=(True|False|None)",
            match.group(0),
        )
        shadow_error_match = re.search(
            r"local_shadow_velocity_error_mps=([\d.]+)",
            match.group(0),
        )
        shadow_mode_match = re.search(
            r"local_shadow_behavior_mode=([^, ]+)",
            match.group(0),
        )
        shadow_speed_match = re.search(
            r"local_shadow_selected_speed_mps=([\d.]+)",
            match.group(0),
        )
        shadow_avoidance_match = re.search(
            r"local_shadow_avoidance_active=(True|False|None)",
            match.group(0),
        )
        sample = {
            "time": float(match.group("time")),
            "generation": int(match.group("generation")),
            "phase": match.group("phase"),
            "x": float(match.group("x")),
            "y": float(match.group("y")),
            "yaw": float(match.group("yaw")),
            "speed": float(match.group("speed")),
            "route_progress": float(match.group("progress")),
            "avoidance_side": (
                int(avoidance_match.group(1))
                if avoidance_match is not None
                else 0
            ),
            "behavior_profile": match.group("profile"),
            "native_behavior": match.group("behavior"),
            "native_behavior_state": match.group("behavior_state"),
            "pedestrian_pair_contacts": int(match.group("pair_contacts")),
            "local_shadow_enabled": (
                shadow_enabled_match is not None
                and shadow_enabled_match.group(1) == "True"
            ),
            "local_shadow_feasible": (
                shadow_feasible_match.group(1) == "True"
                if shadow_feasible_match is not None
                and shadow_feasible_match.group(1) != "None"
                else None
            ),
            "local_shadow_velocity_error_mps": (
                float(shadow_error_match.group(1))
                if shadow_error_match is not None
                else None
            ),
            "local_shadow_behavior_mode": (
                shadow_mode_match.group(1)
                if shadow_mode_match is not None
                else None
            ),
            "local_shadow_selected_speed_mps": (
                float(shadow_speed_match.group(1))
                if shadow_speed_match is not None
                else None
            ),
            "local_shadow_avoidance_active": (
                shadow_avoidance_match.group(1) == "True"
                if shadow_avoidance_match is not None
                and shadow_avoidance_match.group(1) != "None"
                else None
            ),
        }
        samples.setdefault(match.group("agent"), []).append(sample)
    return samples


def _activation_targets(text: str) -> dict[str, tuple[float, float]]:
    targets: dict[str, tuple[float, float]] = {}
    pattern = re.compile(
        r"Activating (?P<agent>toilet_agent_\d+) at shared entrance pose "
        r"\[(?P<x>-?[\d.]+), (?P<y>-?[\d.]+),"
    )
    for match in pattern.finditer(text):
        targets[match.group("agent")] = (
            float(match.group("x")),
            float(match.group("y")),
        )
    return targets


def _activation_times(text: str) -> dict[str, float]:
    times: dict[str, float] = {}
    pattern = re.compile(
        r"\[(?P<time>\d+\.\d+)\].*Activating "
        r"(?P<agent>toilet_agent_\d+) at shared entrance pose"
    )
    for match in pattern.finditer(text):
        times[match.group("agent")] = float(match.group("time"))
    return times


def summarize_multi_agent_diagnostics(text: str) -> dict[str, object]:
    samples_by_agent = _diagnostic_samples(text)
    activation_targets = _activation_targets(text)
    activation_times = _activation_times(text)
    per_agent: dict[str, dict[str, object]] = {}
    moving_phases = {"WALK_TO_URINAL", "QUEUEING", "EXITING"}

    for agent_id, samples in sorted(samples_by_agent.items()):
        samples.sort(key=lambda item: float(item["time"]))
        stop_episodes = 0
        stopped_duration_sec = 0.0
        was_stopped = False
        route_regressions = 0
        behavior_transitions = 0
        behavior_profile_mismatches = 0
        avoidance_episode_count = 0
        avoidance_side_switch_count = 0
        avoidance_active_sample_count = 0
        previous_avoidance_side = 0
        previous = None
        previous_behavior = None
        for sample in samples:
            stopped = (
                str(sample["phase"]) in moving_phases
                and float(sample["speed"]) <= 0.05
            )
            if stopped and not was_stopped:
                stop_episodes += 1
            if stopped and was_stopped and previous is not None:
                stopped_duration_sec += min(
                    2.0,
                    max(0.0, float(sample["time"]) - float(previous["time"])),
                )
            was_stopped = stopped
            if (
                previous is not None
                and sample["generation"] == previous["generation"]
                and sample["phase"] == previous["phase"]
                and float(sample["route_progress"])
                < float(previous["route_progress"]) - 0.05
            ):
                route_regressions += 1
            behavior_key = (
                str(sample["native_behavior"]),
                str(sample["native_behavior_state"]),
            )
            if previous_behavior is not None and behavior_key != previous_behavior:
                behavior_transitions += 1
            previous_behavior = behavior_key
            if sample["native_behavior"] != sample["behavior_profile"]:
                behavior_profile_mismatches += 1
            avoidance_side = int(sample["avoidance_side"])
            if avoidance_side:
                avoidance_active_sample_count += 1
                if previous_avoidance_side == 0:
                    avoidance_episode_count += 1
                elif avoidance_side != previous_avoidance_side:
                    avoidance_side_switch_count += 1
                previous_avoidance_side = avoidance_side
            else:
                previous_avoidance_side = 0
            previous = sample

        shadow_samples = [
            sample
            for sample in samples
            if bool(sample["local_shadow_enabled"])
            and sample["local_shadow_feasible"] is not None
        ]
        shadow_errors = sorted(
            float(sample["local_shadow_velocity_error_mps"])
            for sample in shadow_samples
            if sample["local_shadow_velocity_error_mps"] is not None
        )
        shadow_modes: dict[str, int] = {}
        for sample in shadow_samples:
            mode = sample["local_shadow_behavior_mode"]
            if mode is None:
                continue
            key = str(mode)
            shadow_modes[key] = shadow_modes.get(key, 0) + 1
        shadow_feasible_count = sum(
            sample["local_shadow_feasible"] is True
            for sample in shadow_samples
        )
        shadow_selected_speeds = [
            float(sample["local_shadow_selected_speed_mps"])
            for sample in shadow_samples
            if sample["local_shadow_selected_speed_mps"] is not None
        ]
        shadow_avoidance_count = sum(
            sample["local_shadow_avoidance_active"] is True
            for sample in shadow_samples
        )
        p95_index = (
            max(0, math.ceil(0.95 * len(shadow_errors)) - 1)
            if shadow_errors
            else 0
        )

        activation_error_m = None
        target = activation_targets.get(agent_id)
        if target is not None and samples:
            activation_error_m = math.hypot(
                float(samples[0]["x"]) - target[0],
                float(samples[0]["y"]) - target[1],
            )
        peer_activation_displacement_m = None
        peer_activation_jump_excess_m = None
        for peer_id, activation_time in activation_times.items():
            if peer_id == agent_id:
                continue
            before = [
                sample
                for sample in samples
                if float(sample["time"]) <= activation_time
            ]
            after = [
                sample
                for sample in samples
                if float(sample["time"]) >= activation_time
            ]
            if not before or not after:
                continue
            previous_sample = before[-1]
            next_sample = after[0]
            dt_sec = float(next_sample["time"]) - float(previous_sample["time"])
            if dt_sec <= 0.0 or dt_sec > 2.5:
                continue
            displacement_m = math.hypot(
                float(next_sample["x"]) - float(previous_sample["x"]),
                float(next_sample["y"]) - float(previous_sample["y"]),
            )
            expected_motion_m = (
                max(
                    float(previous_sample["speed"]),
                    float(next_sample["speed"]),
                )
                * dt_sec
                + 0.10
            )
            jump_excess_m = max(0.0, displacement_m - expected_motion_m)
            peer_activation_displacement_m = max(
                float(peer_activation_displacement_m or 0.0),
                displacement_m,
            )
            peer_activation_jump_excess_m = max(
                float(peer_activation_jump_excess_m or 0.0),
                jump_excess_m,
            )
        per_agent[agent_id] = {
            "sample_count": len(samples),
            "activation_error_m": activation_error_m,
            "peer_activation_bracketing_displacement_m": (
                peer_activation_displacement_m
            ),
            "peer_activation_jump_excess_m": peer_activation_jump_excess_m,
            "stop_episode_count": stop_episodes,
            "stopped_duration_sec": stopped_duration_sec,
            "route_progress_regression_count": route_regressions,
            "behavior_transition_count": behavior_transitions,
            "behavior_profile_mismatch_count": behavior_profile_mismatches,
            "avoidance_episode_count": avoidance_episode_count,
            "avoidance_active_sample_count": avoidance_active_sample_count,
            "avoidance_side_switch_count": avoidance_side_switch_count,
            "pedestrian_pair_contact_samples": sum(
                int(sample["pedestrian_pair_contacts"]) > 0 for sample in samples
            ),
            "local_shadow_sample_count": len(shadow_samples),
            "local_shadow_feasible_sample_count": shadow_feasible_count,
            "local_shadow_infeasible_sample_count": (
                len(shadow_samples) - shadow_feasible_count
            ),
            "local_shadow_feasible_ratio": (
                shadow_feasible_count / len(shadow_samples)
                if shadow_samples
                else None
            ),
            "local_shadow_velocity_error_mean_mps": (
                sum(shadow_errors) / len(shadow_errors)
                if shadow_errors
                else None
            ),
            "local_shadow_velocity_error_p95_mps": (
                shadow_errors[p95_index] if shadow_errors else None
            ),
            "local_shadow_velocity_error_max_mps": (
                shadow_errors[-1] if shadow_errors else None
            ),
            "local_shadow_behavior_modes": dict(sorted(shadow_modes.items())),
            "local_shadow_avoidance_active_sample_count": shadow_avoidance_count,
            "local_shadow_selected_speed_mean_mps": (
                sum(shadow_selected_speeds) / len(shadow_selected_speeds)
                if shadow_selected_speeds
                else None
            ),
        }

    pair_distances: list[float] = []
    overlap_samples = 0
    simultaneous_stop_duration_sec = 0.0
    agent_ids = sorted(samples_by_agent)
    for first_index, first_id in enumerate(agent_ids):
        for second_id in agent_ids[first_index + 1 :]:
            first_samples = samples_by_agent[first_id]
            second_samples = samples_by_agent[second_id]
            second_cursor = 0
            previous_pair_time = None
            previous_both_stopped = False
            for first in first_samples:
                while (
                    second_cursor + 1 < len(second_samples)
                    and abs(
                        float(second_samples[second_cursor + 1]["time"])
                        - float(first["time"])
                    )
                    <= abs(
                        float(second_samples[second_cursor]["time"])
                        - float(first["time"])
                    )
                ):
                    second_cursor += 1
                second = second_samples[second_cursor]
                pair_time_delta = abs(float(second["time"]) - float(first["time"]))
                if pair_time_delta > 0.25:
                    continue
                distance = math.hypot(
                    float(first["x"]) - float(second["x"]),
                    float(first["y"]) - float(second["y"]),
                )
                pair_distances.append(distance)
                if distance < PEDESTRIAN_CHARACTERIZATION_OVERLAP_DISTANCE_M:
                    overlap_samples += 1
                both_stopped = (
                    str(first["phase"]) in moving_phases
                    and str(second["phase"]) in moving_phases
                    and float(first["speed"]) <= 0.05
                    and float(second["speed"]) <= 0.05
                )
                pair_time = max(float(first["time"]), float(second["time"]))
                if (
                    both_stopped
                    and previous_both_stopped
                    and previous_pair_time is not None
                ):
                    simultaneous_stop_duration_sec += min(
                        2.0,
                        max(0.0, pair_time - previous_pair_time),
                    )
                previous_pair_time = pair_time
                previous_both_stopped = both_stopped

    activation_errors = [
        float(item["activation_error_m"])
        for item in per_agent.values()
        if item["activation_error_m"] is not None
    ]
    local_shadow_sample_count = sum(
        int(item["local_shadow_sample_count"])
        for item in per_agent.values()
    )
    local_shadow_infeasible_sample_count = sum(
        int(item["local_shadow_infeasible_sample_count"])
        for item in per_agent.values()
    )
    return {
        "diagnostic_agent_count": len(samples_by_agent),
        "diagnostic_samples_by_agent": per_agent,
        "activation_max_error_m": max(activation_errors) if activation_errors else None,
        "pedestrian_pair_sample_count": len(pair_distances),
        "pedestrian_pair_min_distance_m": min(pair_distances) if pair_distances else None,
        "pedestrian_pair_overlap_threshold_m": (
            PEDESTRIAN_CHARACTERIZATION_OVERLAP_DISTANCE_M
        ),
        "pedestrian_pair_overlap_samples": overlap_samples,
        "simultaneous_stop_duration_sec": simultaneous_stop_duration_sec,
        "local_shadow_sample_count": local_shadow_sample_count,
        "local_shadow_infeasible_sample_count": (
            local_shadow_infeasible_sample_count
        ),
        "local_shadow_feasible_ratio": (
            (local_shadow_sample_count - local_shadow_infeasible_sample_count)
            / local_shadow_sample_count
            if local_shadow_sample_count
            else None
        ),
    }


def summarize_local_motion_diagnostics(text: str) -> dict[str, object]:
    """Summarize the backend-neutral E2 takeover diagnostics."""

    number = r"(?:-?(?:\d+(?:\.\d*)?|\.\d+)|inf|nan)"
    pattern = re.compile(
        rf"\[(?P<time>\d+\.\d+)\].*Local motion diagnostics: "
        rf"agent=(?P<agent>[^,]+), phase=(?P<phase>[^,]+), "
        rf"feasible=(?P<feasible>True|False), "
        rf"command_speed=(?P<command_speed>{number}), "
        rf"actual_speed=(?P<actual_speed>{number}), "
        rf"remaining_m=(?P<remaining>{number}), "
        rf"static_clearance_m=(?P<static_clearance>{number}), "
        rf"service_inflight=(?P<service_inflight>True|False), "
        rf"static_rejected=(?P<static_rejected>\d+), "
        rf"dynamic_rejected=(?P<dynamic_rejected>\d+)"
        rf"(?:, dynamic_clearance_m=(?P<dynamic_clearance>{number}))?"
        rf"(?:, peer_clearance_m=(?P<peer_clearance>{number}))?"
        rf"(?:, robot_clearance_m=(?P<robot_clearance>{number}))?"
        rf"(?:, soft_clearance_target_m=(?P<soft_clearance_target>{number}))?"
        rf"(?:, overlap_recovery_neighbors=(?P<overlap_recovery>\d+))?"
    )
    samples_by_agent: dict[str, list[dict[str, object]]] = {}
    for match in pattern.finditer(text):
        sample = {
            "time": float(match.group("time")),
            "phase": match.group("phase"),
            "feasible": match.group("feasible") == "True",
            "command_speed": float(match.group("command_speed")),
            "actual_speed": float(match.group("actual_speed")),
            "remaining": float(match.group("remaining")),
            "static_clearance": float(match.group("static_clearance")),
            "service_inflight": match.group("service_inflight") == "True",
            "static_rejected": int(match.group("static_rejected")),
            "dynamic_rejected": int(match.group("dynamic_rejected")),
            "dynamic_clearance": (
                float(match.group("dynamic_clearance"))
                if match.group("dynamic_clearance") is not None
                else math.nan
            ),
            "peer_clearance": (
                float(match.group("peer_clearance"))
                if match.group("peer_clearance") is not None
                else math.nan
            ),
            "robot_clearance": (
                float(match.group("robot_clearance"))
                if match.group("robot_clearance") is not None
                else math.nan
            ),
            "soft_clearance_target": (
                float(match.group("soft_clearance_target"))
                if match.group("soft_clearance_target") is not None
                else math.nan
            ),
            "overlap_recovery": int(match.group("overlap_recovery") or 0),
        }
        samples_by_agent.setdefault(match.group("agent"), []).append(sample)

    per_agent: dict[str, dict[str, object]] = {}
    all_tracking_errors: list[float] = []
    all_finite_clearances: list[float] = []
    all_finite_dynamic_clearances: list[float] = []
    all_finite_peer_clearances: list[float] = []
    all_finite_robot_clearances: list[float] = []
    total_overlap_recovery_samples = 0
    total_samples = 0
    total_infeasible = 0
    total_inflight = 0
    for agent_id, samples in sorted(samples_by_agent.items()):
        samples.sort(key=lambda sample: float(sample["time"]))
        tracking_errors = [
            abs(float(sample["command_speed"]) - float(sample["actual_speed"]))
            for sample in samples
        ]
        finite_clearances = [
            float(sample["static_clearance"])
            for sample in samples
            if math.isfinite(float(sample["static_clearance"]))
        ]
        finite_dynamic_clearances = [
            float(sample["dynamic_clearance"])
            for sample in samples
            if math.isfinite(float(sample["dynamic_clearance"]))
        ]
        finite_peer_clearances = [
            float(sample["peer_clearance"])
            for sample in samples
            if math.isfinite(float(sample["peer_clearance"]))
        ]
        finite_robot_clearances = [
            float(sample["robot_clearance"])
            for sample in samples
            if math.isfinite(float(sample["robot_clearance"]))
        ]
        overlap_recovery_count = sum(
            int(sample["overlap_recovery"]) > 0 for sample in samples
        )
        infeasible_count = sum(not bool(sample["feasible"]) for sample in samples)
        inflight_count = sum(bool(sample["service_inflight"]) for sample in samples)
        remaining_regressions = sum(
            float(current["remaining"]) > float(previous["remaining"]) + 0.05
            for previous, current in zip(samples, samples[1:])
            if current["phase"] == previous["phase"]
        )
        per_agent[agent_id] = {
            "sample_count": len(samples),
            "feasible_sample_count": len(samples) - infeasible_count,
            "infeasible_sample_count": infeasible_count,
            "feasible_ratio": (
                (len(samples) - infeasible_count) / len(samples)
                if samples
                else None
            ),
            "speed_tracking_error_mean_mps": (
                sum(tracking_errors) / len(tracking_errors)
                if tracking_errors
                else None
            ),
            "speed_tracking_error_max_mps": (
                max(tracking_errors) if tracking_errors else None
            ),
            "min_static_clearance_m": (
                min(finite_clearances) if finite_clearances else None
            ),
            "min_dynamic_clearance_m": (
                min(finite_dynamic_clearances)
                if finite_dynamic_clearances
                else None
            ),
            "min_peer_clearance_m": (
                min(finite_peer_clearances) if finite_peer_clearances else None
            ),
            "min_robot_clearance_m": (
                min(finite_robot_clearances) if finite_robot_clearances else None
            ),
            "overlap_recovery_sample_count": overlap_recovery_count,
            "service_inflight_sample_count": inflight_count,
            "remaining_distance_regression_count": remaining_regressions,
            "max_static_rejected_candidates": max(
                int(sample["static_rejected"]) for sample in samples
            ),
            "max_dynamic_rejected_candidates": max(
                int(sample["dynamic_rejected"]) for sample in samples
            ),
        }
        total_samples += len(samples)
        total_infeasible += infeasible_count
        total_inflight += inflight_count
        all_tracking_errors.extend(tracking_errors)
        all_finite_clearances.extend(finite_clearances)
        all_finite_dynamic_clearances.extend(finite_dynamic_clearances)
        all_finite_peer_clearances.extend(finite_peer_clearances)
        all_finite_robot_clearances.extend(finite_robot_clearances)
        total_overlap_recovery_samples += overlap_recovery_count

    return {
        "local_motion_diagnostic_agent_count": len(samples_by_agent),
        "local_motion_diagnostic_samples_by_agent": per_agent,
        "local_motion_sample_count": total_samples,
        "local_motion_infeasible_sample_count": total_infeasible,
        "local_motion_feasible_ratio": (
            (total_samples - total_infeasible) / total_samples
            if total_samples
            else None
        ),
        "local_motion_speed_tracking_error_mean_mps": (
            sum(all_tracking_errors) / len(all_tracking_errors)
            if all_tracking_errors
            else None
        ),
        "local_motion_speed_tracking_error_max_mps": (
            max(all_tracking_errors) if all_tracking_errors else None
        ),
        "local_motion_min_static_clearance_m": (
            min(all_finite_clearances) if all_finite_clearances else None
        ),
        "local_motion_min_dynamic_clearance_m": (
            min(all_finite_dynamic_clearances)
            if all_finite_dynamic_clearances
            else None
        ),
        "local_motion_min_peer_clearance_m": (
            min(all_finite_peer_clearances)
            if all_finite_peer_clearances
            else None
        ),
        "local_motion_min_robot_clearance_m": (
            min(all_finite_robot_clearances)
            if all_finite_robot_clearances
            else None
        ),
        "local_motion_overlap_recovery_sample_count": (
            total_overlap_recovery_samples
        ),
        "local_motion_service_inflight_sample_count": total_inflight,
    }


def summarize_director_log(text: str, *, exit_code: int) -> dict[str, object]:
    behavior_samples: dict[str, int] = {}
    active_samples: dict[str, int] = {}
    for line in text.splitlines():
        fields = {
            key: value
            for key, value in re.findall(
                r"(native_behavior|native_behavior_state)=([^, ]+)",
                line,
            )
        }
        behavior = fields.get("native_behavior")
        if behavior is None:
            continue
        behavior_samples[behavior] = behavior_samples.get(behavior, 0) + 1
        if fields.get("native_behavior_state") in {"active_1", "active_2"}:
            active_samples[behavior] = active_samples.get(behavior, 0) + 1
    summary = {
        "exit_code": int(exit_code),
        "completed": "All toilet benchmark agents completed" in text,
        "native_behavior_samples": behavior_samples,
        "native_behavior_active_samples_by_profile": active_samples,
        "native_surprised_samples": text.count("native_behavior=surprised"),
        "native_behavior_active_samples": (
            text.count("native_behavior_state=active_1")
            + text.count("native_behavior_state=active_2")
        ),
        "reaction_regular_samples": text.count("reaction=regular"),
        "reaction_yielding_samples": text.count("reaction=yielding"),
        "regular_avoidance_latched": text.count("HuNav regular avoidance latched"),
        "robot_contact_events": text.count("robot_contacts=1"),
        "static_clip_events": text.count("static_clip=True"),
        "hard_safety_events": text.count("HuNav hard safety:"),
        "stall_events": text.count("stalled"),
        "recovery_exhausted_events": text.count("exceeded 3"),
        "scenario_shadow_mismatch_events": text.count(
            "E1 scenario shadow mismatch:"
        ),
        "urinal_arrivals": text.count("reached urinal_"),
        "exit_arrivals": text.count("aligned at EXITING"),
    }
    summary.update(summarize_multi_agent_diagnostics(text))
    summary.update(summarize_local_motion_diagnostics(text))
    return summary


def summarize_visual_envelope_log(
    path: Path,
    *,
    expected_agent_ids: Sequence[str] = (),
) -> dict[str, object]:
    """Summarize read-only Isaac skeleton-envelope diagnostics."""

    expected_agents = {str(agent_id) for agent_id in expected_agent_ids}
    frame_samples = 0
    agent_samples = 0
    available_samples = 0
    overlap_samples = 0
    max_overlap_ratio = 0.0
    overlap_paths: dict[str, int] = {}
    unavailable_reasons: dict[str, int] = {}
    available_by_agent: dict[str, int] = {}
    consecutive_by_agent: dict[str, int] = {}
    max_consecutive_by_agent: dict[str, int] = {}

    if not path.is_file():
        return {
            "visual_envelope_frame_samples": 0,
            "visual_envelope_agent_samples": 0,
            "visual_envelope_available_samples": 0,
            "visual_envelope_available_samples_by_agent": {},
            "visual_envelope_overlap_samples": 0,
            "visual_envelope_max_overlap_ratio": 0.0,
            "visual_envelope_max_consecutive_overlap_samples": 0,
            "visual_envelope_overlap_paths": {},
            "visual_envelope_unavailable_reasons": {},
        }

    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        try:
            payload = json.loads(line)
        except (TypeError, ValueError, json.JSONDecodeError):
            continue
        agents = payload.get("agents", ())
        if not isinstance(agents, list):
            continue
        frame_samples += 1
        observed_agents: set[str] = set()
        for item in agents:
            if not isinstance(item, Mapping):
                continue
            agent_id = str(item.get("agent_id", "") or "")
            if not agent_id:
                continue
            if expected_agents and agent_id not in expected_agents:
                continue
            observed_agents.add(agent_id)
            agent_samples += 1
            if bool(item.get("available", False)):
                available_samples += 1
                available_by_agent[agent_id] = (
                    available_by_agent.get(agent_id, 0) + 1
                )
            else:
                reason = str(
                    item.get("unavailable_reason", "skeleton_unavailable")
                    or "skeleton_unavailable"
                )
                unavailable_reasons[reason] = unavailable_reasons.get(reason, 0) + 1
            overlap_count = max(0, int(item.get("overlap_sample_count", 0) or 0))
            if overlap_count > 0:
                overlap_samples += 1
                consecutive_by_agent[agent_id] = (
                    consecutive_by_agent.get(agent_id, 0) + 1
                )
                max_consecutive_by_agent[agent_id] = max(
                    max_consecutive_by_agent.get(agent_id, 0),
                    consecutive_by_agent[agent_id],
                )
            else:
                consecutive_by_agent[agent_id] = 0
            try:
                max_overlap_ratio = max(
                    max_overlap_ratio,
                    float(item.get("overlap_ratio", 0.0) or 0.0),
                )
            except (TypeError, ValueError):
                pass
            for prim_path in item.get("overlap_paths", ()) or ():
                key = str(prim_path)
                overlap_paths[key] = overlap_paths.get(key, 0) + 1
        for agent_id in set(consecutive_by_agent) - observed_agents:
            consecutive_by_agent[agent_id] = 0

    return {
        "visual_envelope_frame_samples": frame_samples,
        "visual_envelope_agent_samples": agent_samples,
        "visual_envelope_available_samples": available_samples,
        "visual_envelope_available_samples_by_agent": dict(
            sorted(available_by_agent.items())
        ),
        "visual_envelope_overlap_samples": overlap_samples,
        "visual_envelope_max_overlap_ratio": round(max_overlap_ratio, 6),
        "visual_envelope_max_consecutive_overlap_samples": max(
            max_consecutive_by_agent.values(),
            default=0,
        ),
        "visual_envelope_overlap_paths": dict(
            sorted(overlap_paths.items(), key=lambda item: (-item[1], item[0]))
        ),
        "visual_envelope_unavailable_reasons": dict(
            sorted(
                unavailable_reasons.items(),
                key=lambda item: (-item[1], item[0]),
            )
        ),
    }


def apply_visual_envelope_gate(
    summary: dict[str, object],
    *,
    expected_agent_ids: Sequence[str] = (),
) -> bool:
    """Require observed animated skeletons and reject any static overlap."""

    frame_samples = int(summary.get("visual_envelope_frame_samples", 0) or 0)
    available_samples = int(
        summary.get("visual_envelope_available_samples", 0) or 0
    )
    overlap_samples = int(
        summary.get("visual_envelope_overlap_samples", 0) or 0
    )
    available_by_agent = dict(
        summary.get("visual_envelope_available_samples_by_agent", {}) or {}
    )
    missing_agents = sorted(
        str(agent_id)
        for agent_id in expected_agent_ids
        if int(available_by_agent.get(str(agent_id), 0) or 0) <= 0
    )
    reasons = []
    if frame_samples <= 0 or available_samples <= 0:
        reasons.append("visual_envelope_unavailable")
    if missing_agents:
        reasons.append("visual_envelope_agent_missing")
    if overlap_samples > 0:
        reasons.append("visual_envelope_overlap")
    summary["visual_envelope_validated"] = not (
        frame_samples <= 0 or available_samples <= 0
    )
    summary["visual_envelope_collision_free"] = overlap_samples == 0
    summary["visual_envelope_gate_passed"] = not reasons
    summary["visual_envelope_failure_reasons"] = reasons
    summary["visual_envelope_missing_agents"] = missing_agents
    return not reasons


def _active_agent_windows(
    director_text: str,
) -> dict[str, tuple[float, float | None]]:
    starts = _activation_times(director_text)
    ends: dict[str, float] = {}
    pattern = re.compile(
        r"\[(?P<time>\d+\.\d+)\].*"
        r"(?P<agent>toilet_agent_\d+) aligned at EXITING:"
    )
    for match in pattern.finditer(director_text):
        ends[match.group("agent")] = float(match.group("time"))
    return {
        agent_id: (start_time, ends.get(agent_id))
        for agent_id, start_time in starts.items()
    }


def _moving_agent_windows(
    director_text: str,
) -> dict[str, list[tuple[float, float | None]]]:
    active = _active_agent_windows(director_text)
    walk_ends: dict[str, float] = {}
    exit_starts: dict[str, float] = {}
    exit_ends: dict[str, float] = {}
    patterns = (
        (
            walk_ends,
            re.compile(
                r"\[(?P<time>\d+\.\d+)\].*"
                r"(?P<agent>toilet_agent_\d+) aligned at WALK_TO_URINAL:"
            ),
        ),
        (
            exit_starts,
            re.compile(
                r"\[(?P<time>\d+\.\d+)\].*"
                r"(?P<agent>toilet_agent_\d+) leaving urinal_[^;]+;"
            ),
        ),
        (
            exit_ends,
            re.compile(
                r"\[(?P<time>\d+\.\d+)\].*"
                r"(?P<agent>toilet_agent_\d+) aligned at EXITING:"
            ),
        ),
    )
    for destination, pattern in patterns:
        for match in pattern.finditer(director_text):
            destination[match.group("agent")] = float(match.group("time"))

    windows: dict[str, list[tuple[float, float | None]]] = {}
    for agent_id, (active_start, active_end) in active.items():
        walk_end = walk_ends.get(agent_id)
        if walk_end is None:
            windows[agent_id] = [(active_start, active_end)]
            continue
        agent_windows = [(active_start, walk_end)]
        exit_start = exit_starts.get(agent_id)
        exit_end = exit_ends.get(agent_id, active_end)
        if exit_start is not None:
            agent_windows.append((exit_start, exit_end))
        windows[agent_id] = agent_windows
    return windows


def _in_time_windows(
    timestamp: float,
    windows: Sequence[tuple[float, float | None]],
) -> bool:
    return any(
        timestamp >= start and (end is None or timestamp <= end)
        for start, end in windows
    )


def _pedestrian_yield_windows(
    director_text: str,
) -> dict[str, list[tuple[float, float | None]]]:
    event_pattern = re.compile(
        r"\[(?P<time>\d+\.\d+)\].*HuNav pedestrian corridor "
        r"(?P<event>blocked|released): agent=(?P<agent>toilet_agent_\d+)"
    )
    starts: dict[str, float] = {}
    windows: dict[str, list[tuple[float, float | None]]] = {}
    for match in event_pattern.finditer(director_text):
        agent_id = match.group("agent")
        timestamp = float(match.group("time"))
        if match.group("event") == "blocked":
            starts.setdefault(agent_id, timestamp)
            continue
        start = starts.pop(agent_id, None)
        if start is not None:
            windows.setdefault(agent_id, []).append((start, timestamp))
    for agent_id, start in starts.items():
        windows.setdefault(agent_id, []).append((start, None))
    return windows


def _yield_stability_metrics(
    samples: Sequence[Mapping[str, float]],
    windows: Sequence[tuple[float, float | None]],
) -> dict[str, object]:
    selected = [
        sample
        for sample in samples
        if _in_time_windows(float(sample["time"]), windows)
    ]
    speeds = [
        math.hypot(float(sample["vx"]), float(sample["vy"]))
        for sample in selected
    ]
    max_xy_span = 0.0
    max_yaw_span = 0.0
    for start, end in windows:
        window_samples = [
            sample
            for sample in selected
            if float(sample["time"]) >= start
            and (end is None or float(sample["time"]) <= end)
        ]
        if not window_samples:
            continue
        xs = [float(sample["x"]) for sample in window_samples]
        ys = [float(sample["y"]) for sample in window_samples]
        max_xy_span = max(
            max_xy_span,
            math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        )
        first_yaw = float(window_samples[0]["yaw"])
        max_yaw_span = max(
            max_yaw_span,
            max(
                abs(
                    math.atan2(
                        math.sin(float(sample["yaw"]) - first_yaw),
                        math.cos(float(sample["yaw"]) - first_yaw),
                    )
                )
                for sample in window_samples
            ),
        )
    return {
        "window_count": len(windows),
        "sample_count": len(selected),
        "max_xy_span_m": max_xy_span,
        "max_yaw_span_rad": max_yaw_span,
        "max_speed_mps": max(speeds, default=0.0),
        "mean_speed_mps": (
            sum(speeds) / len(speeds) if speeds else 0.0
        ),
    }


def _locomotion_metrics(
    samples: Sequence[Mapping[str, float]],
    windows: Sequence[tuple[float, float | None]],
) -> dict[str, object]:
    moving = [
        sample
        for sample in samples
        if _in_time_windows(float(sample["time"]), windows)
    ]
    stop_episode_count = 0
    stopped_duration_sec = 0.0
    stop_started_at: float | None = None
    previous_time: float | None = None
    for sample in moving:
        timestamp = float(sample["time"])
        speed = math.hypot(float(sample["vx"]), float(sample["vy"]))
        stopped = speed <= 0.05
        if stopped and stop_started_at is None:
            stop_started_at = timestamp
        elif not stopped and stop_started_at is not None:
            duration = max(0.0, (previous_time or timestamp) - stop_started_at)
            if duration >= 0.30:
                stop_episode_count += 1
                stopped_duration_sec += duration
            stop_started_at = None
        previous_time = timestamp
    if stop_started_at is not None and previous_time is not None:
        duration = max(0.0, previous_time - stop_started_at)
        if duration >= 0.30:
            stop_episode_count += 1
            stopped_duration_sec += duration

    headings: list[tuple[float, float, float]] = []
    for previous, current in zip(moving, moving[1:]):
        dt = float(current["time"]) - float(previous["time"])
        dx = float(current["x"]) - float(previous["x"])
        dy = float(current["y"]) - float(previous["y"])
        distance = math.hypot(dx, dy)
        if dt <= 0.0 or dt > 0.50 or distance < 0.015:
            continue
        headings.append(
            (float(current["time"]), math.atan2(dy, dx), distance)
        )

    turn_reversals = 0
    total_heading_change = 0.0
    last_turn_sign = 0
    distance_since_sign = 0.0
    previous_heading: float | None = None
    for _, heading, distance in headings:
        distance_since_sign += distance
        if previous_heading is None:
            previous_heading = heading
            continue
        delta = math.atan2(
            math.sin(heading - previous_heading),
            math.cos(heading - previous_heading),
        )
        previous_heading = heading
        total_heading_change += abs(delta)
        if abs(delta) < 0.10:
            continue
        turn_sign = 1 if delta > 0.0 else -1
        if (
            last_turn_sign
            and turn_sign != last_turn_sign
            and distance_since_sign >= 0.10
        ):
            turn_reversals += 1
            distance_since_sign = 0.0
        last_turn_sign = turn_sign

    return {
        "sample_count": len(moving),
        "stop_episode_count": stop_episode_count,
        "stopped_duration_sec": stopped_duration_sec,
        "turn_direction_reversal_count": turn_reversals,
        "absolute_heading_change_rad": total_heading_change,
    }


def summarize_raw_pose_log(
    path: Path,
    *,
    director_text: str,
    overlap_distance_m: float = PEDESTRIAN_CHARACTERIZATION_OVERLAP_DISTANCE_M,
) -> dict[str, object]:
    windows = _active_agent_windows(director_text)
    moving_windows = _moving_agent_windows(director_text)
    samples_by_agent: dict[str, list[dict[str, float]]] = {}
    pair_distances: list[float] = []
    pair_samples: dict[str, list[dict[str, float | bool]]] = {}
    overlap_frames = 0
    active_frame_times: list[float] = []

    if path.is_file():
        for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
            try:
                payload = json.loads(line)
            except (TypeError, ValueError, json.JSONDecodeError):
                continue
            stamp = payload.get("source_stamp_sec")
            if not isinstance(stamp, (int, float)) or not math.isfinite(float(stamp)):
                continue
            stamp = float(stamp)
            active_agents: list[dict[str, float]] = []
            for item in payload.get("agents", ()) or ():
                if not isinstance(item, Mapping):
                    continue
                agent_id = str(item.get("name", "") or "").strip()
                window = windows.get(agent_id)
                if window is None:
                    continue
                start_time, end_time = window
                if stamp < start_time or (
                    end_time is not None and stamp > end_time
                ):
                    continue
                try:
                    sample = {
                        "time": stamp,
                        "x": float(item["x"]),
                        "y": float(item["y"]),
                        "vx": float(item.get("vx", 0.0)),
                        "vy": float(item.get("vy", 0.0)),
                        "yaw": float(item.get("yaw", 0.0)),
                    }
                except (KeyError, TypeError, ValueError):
                    continue
                if not all(math.isfinite(value) for value in sample.values()):
                    continue
                samples_by_agent.setdefault(agent_id, []).append(sample)
                active_agents.append({"agent_id": agent_id, **sample})
            if active_agents:
                active_frame_times.append(stamp)
            frame_overlaps = 0
            for first_index, first in enumerate(active_agents):
                for second in active_agents[first_index + 1 :]:
                    distance = math.hypot(
                        first["x"] - second["x"],
                        first["y"] - second["y"],
                    )
                    pair_distances.append(distance)
                    pair_key = "|".join(sorted((first["agent_id"], second["agent_id"])))
                    first_moving = any(
                        start <= stamp and (end is None or stamp <= end)
                        for start, end in moving_windows.get(first["agent_id"], ())
                    )
                    second_moving = any(
                        start <= stamp and (end is None or stamp <= end)
                        for start, end in moving_windows.get(second["agent_id"], ())
                    )
                    pair_samples.setdefault(pair_key, []).append(
                        {
                            "time": stamp,
                            "distance": distance,
                            "overlap": distance < float(overlap_distance_m),
                            "both_stopped": (
                                first_moving
                                and second_moving
                                and math.hypot(first["vx"], first["vy"]) <= 0.05
                                and math.hypot(second["vx"], second["vy"]) <= 0.05
                            ),
                        }
                    )
                    if distance < float(overlap_distance_m):
                        frame_overlaps += 1
            if frame_overlaps:
                overlap_frames += 1

    pose_rate_hz = None
    unique_frame_times = sorted(set(active_frame_times))
    if len(unique_frame_times) >= 2:
        duration_sec = unique_frame_times[-1] - unique_frame_times[0]
        if duration_sec > 0.0:
            pose_rate_hz = (len(unique_frame_times) - 1) / duration_sec

    peer_activation_jump_excess: dict[str, float] = {}
    activation_times = _activation_times(director_text)
    for agent_id, samples in samples_by_agent.items():
        samples.sort(key=lambda item: item["time"])
        for peer_id, activation_time in activation_times.items():
            if peer_id == agent_id:
                continue
            before = [sample for sample in samples if sample["time"] <= activation_time]
            after = [sample for sample in samples if sample["time"] >= activation_time]
            if not before or not after:
                continue
            previous_sample = before[-1]
            next_sample = after[0]
            dt_sec = next_sample["time"] - previous_sample["time"]
            if dt_sec <= 0.0 or dt_sec > 0.5:
                continue
            displacement_m = math.hypot(
                next_sample["x"] - previous_sample["x"],
                next_sample["y"] - previous_sample["y"],
            )
            expected_motion_m = (
                max(
                    math.hypot(previous_sample["vx"], previous_sample["vy"]),
                    math.hypot(next_sample["vx"], next_sample["vy"]),
                )
                * dt_sec
                + 0.05
            )
            peer_activation_jump_excess[agent_id] = max(
                peer_activation_jump_excess.get(agent_id, 0.0),
                max(0.0, displacement_m - expected_motion_m),
            )

    overlap_duration_sec = None
    if pose_rate_hz is not None and pose_rate_hz > 0.0:
        overlap_duration_sec = overlap_frames / pose_rate_hz
    pair_metrics: dict[str, dict[str, object]] = {}
    for pair_key, samples in sorted(pair_samples.items()):
        samples.sort(key=lambda item: float(item["time"]))
        overlap_count = sum(bool(item["overlap"]) for item in samples)
        simultaneous_stop_duration = 0.0
        previous = None
        for item in samples:
            if (
                previous is not None
                and bool(previous["both_stopped"])
                and bool(item["both_stopped"])
            ):
                simultaneous_stop_duration += min(
                    0.25,
                    max(0.0, float(item["time"]) - float(previous["time"])),
                )
            previous = item
        pair_metrics[pair_key] = {
            "sample_count": len(samples),
            "min_distance_m": min(float(item["distance"]) for item in samples),
            "overlap_frame_count": overlap_count,
            "overlap_duration_sec": (
                overlap_count / pose_rate_hz
                if pose_rate_hz is not None and pose_rate_hz > 0.0
                else None
            ),
            "simultaneous_stop_duration_sec": simultaneous_stop_duration,
        }
    locomotion_by_agent = {
        agent_id: _locomotion_metrics(
            samples,
            moving_windows.get(agent_id, ()),
        )
        for agent_id, samples in sorted(samples_by_agent.items())
    }
    yield_windows = _pedestrian_yield_windows(director_text)
    yield_stability_by_agent = {
        agent_id: _yield_stability_metrics(
            samples_by_agent.get(agent_id, ()),
            windows,
        )
        for agent_id, windows in sorted(yield_windows.items())
    }
    return {
        "raw_pose_frame_count": len(unique_frame_times),
        "raw_pose_agent_sample_count": sum(
            len(samples) for samples in samples_by_agent.values()
        ),
        "raw_pose_rate_hz": pose_rate_hz,
        "raw_pose_samples_by_agent": {
            agent_id: len(samples)
            for agent_id, samples in sorted(samples_by_agent.items())
        },
        "raw_pose_pair_sample_count": len(pair_distances),
        "raw_pose_pair_min_distance_m": (
            min(pair_distances) if pair_distances else None
        ),
        "raw_pose_pair_overlap_threshold_m": float(overlap_distance_m),
        "raw_pose_pair_overlap_frames": overlap_frames,
        "raw_pose_pair_overlap_duration_sec": overlap_duration_sec,
        "raw_pose_pair_metrics": pair_metrics,
        "raw_pose_pair_max_simultaneous_stop_duration_sec": max(
            (
                float(metrics["simultaneous_stop_duration_sec"])
                for metrics in pair_metrics.values()
            ),
            default=0.0,
        ),
        "raw_pose_peer_activation_jump_excess_m": peer_activation_jump_excess,
        "raw_pose_peer_activation_max_jump_excess_m": (
            max(peer_activation_jump_excess.values())
            if peer_activation_jump_excess
            else None
        ),
        "raw_pose_locomotion_by_agent": locomotion_by_agent,
        "raw_pose_pedestrian_yield_by_agent": yield_stability_by_agent,
        "raw_pose_max_turn_direction_reversals": max(
            (
                int(metrics["turn_direction_reversal_count"])
                for metrics in locomotion_by_agent.values()
            ),
            default=0,
        ),
        "raw_pose_total_stop_episodes": sum(
            int(metrics["stop_episode_count"])
            for metrics in locomotion_by_agent.values()
        ),
    }


def _tail(path: Path, lines: int = 30) -> str:
    try:
        return "\n".join(path.read_text(encoding="utf-8", errors="replace").splitlines()[-lines:])
    except OSError:
        return ""


def _service_names(env: Mapping[str, str]) -> set[str]:
    result = subprocess.run(
        ["ros2", "service", "list"],
        env=dict(env),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=10.0,
        check=False,
    )
    if result.returncode != 0:
        return set()
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def wait_for_robot_reset_service(
    env: Mapping[str, str],
    *,
    timeout_sec: float = 20.0,
) -> None:
    required_service = "/isaac/reset_mecanum_episode"
    deadline = time.monotonic() + float(timeout_sec)
    while required_service not in _service_names(env):
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"robot intervention service unavailable: {required_service}"
            )
        time.sleep(0.5)


def wait_for_services(
    process: subprocess.Popen | None,
    *,
    required: Sequence[str],
    timeout_sec: float,
    env: Mapping[str, str],
) -> None:
    deadline = time.monotonic() + float(timeout_sec)
    required_set = set(required)
    last_missing = required_set
    while time.monotonic() < deadline:
        if process is not None and process.poll() is not None:
            raise RuntimeError(f"bridge exited before services were ready: {process.returncode}")
        available = _service_names(env)
        last_missing = required_set - available
        if not last_missing:
            return
        time.sleep(2.0)
    missing = ", ".join(sorted(last_missing))
    raise TimeoutError(f"bridge service readiness timed out; missing: {missing}")


def wait_for_log_marker(
    path: Path,
    marker: str,
    *,
    timeout_sec: float,
    process: subprocess.Popen | None = None,
) -> bool:
    deadline = time.monotonic() + float(timeout_sec)
    while time.monotonic() < deadline:
        if marker in _tail(path, 200):
            return True
        if process is not None and process.poll() is not None:
            return False
        time.sleep(1.0)
    return False


def run_logged(
    command: Sequence[str],
    *,
    cwd: Path,
    env: Mapping[str, str],
    log_path: Path,
    timeout_sec: float,
) -> int:
    with log_path.open("w", encoding="utf-8") as stream:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd),
            env=dict(env),
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=float(timeout_sec),
            check=False,
        )
    return int(completed.returncode)


def build_robot_reset_command(
    pose: Sequence[float],
    *,
    robot_name: str = "xms_mecanum",
) -> list[str]:
    x, y, z, yaw = (float(value) for value in pose)
    request = {
        "name": str(robot_name),
        "pose": {
            "position": {"x": x, "y": y, "z": z},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(0.5 * yaw),
                "w": math.cos(0.5 * yaw),
            },
        },
    }
    return [
        "ros2",
        "service",
        "call",
        "/isaac/reset_mecanum_episode",
        "isaacsim_msgs/srv/ResetRobot",
        yaml.safe_dump(request, default_flow_style=True).strip(),
    ]


def _append_jsonl(path: Path, payload: Mapping[str, object]) -> None:
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(dict(payload), sort_keys=True) + "\n")


def build_dynamic_crossing_command(
    *,
    topic: str,
    linear_x: float,
    duration_sec: float,
    rate_hz: float = 20.0,
) -> list[str]:
    return [
        "ros2",
        "run",
        "ros2isaacsim",
        "cmd_vel_pulse",
        "--topic",
        str(topic),
        "--motion",
        "forward",
        "--duration",
        str(max(0.0, float(duration_sec))),
        "--rate",
        str(max(1.0, float(rate_hz))),
        "--linear",
        str(float(linear_x)),
    ]


def resolve_dynamic_crossing_profile(
    profile_name: str,
    *,
    reset_pose: Sequence[float] | None = None,
    post_pose: Sequence[float] | None = None,
    trigger_x: float | None = None,
    topic: str | None = None,
    linear_x: float | None = None,
    motion_sec: float | None = None,
) -> tuple[DynamicCrossingProfile, str]:
    base = DYNAMIC_CROSSING_PROFILES[str(profile_name)]
    resolved = DynamicCrossingProfile(
        name=base.name if (
            reset_pose is None
            and post_pose is None
            and trigger_x is None
            and topic is None
            and linear_x is None
            and motion_sec is None
        ) else "custom",
        reset_pose=tuple(float(value) for value in reset_pose)
        if reset_pose is not None
        else base.reset_pose,
        post_pose=tuple(float(value) for value in post_pose)
        if post_pose is not None
        else base.post_pose,
        trigger_x=float(trigger_x) if trigger_x is not None else base.trigger_x,
        topic=str(topic) if topic is not None else base.topic,
        linear_x=float(linear_x) if linear_x is not None else base.linear_x,
        motion_sec=float(motion_sec) if motion_sec is not None else base.motion_sec,
    )
    return resolved, base.name if resolved == base else "custom"


def _run_dynamic_pulse(
    command: Sequence[str],
    *,
    cwd: Path,
    env: Mapping[str, str],
    timeout_sec: float,
):
    return subprocess.run(
        list(command),
        cwd=str(cwd),
        env=dict(env),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=float(timeout_sec),
        check=False,
    )


def stop_dynamic_crossing(
    *,
    cwd: Path,
    env: Mapping[str, str],
    event_log: Path,
    topic: str,
) -> bool:
    command = build_dynamic_crossing_command(
        topic=topic,
        linear_x=0.0,
        duration_sec=0.0,
    )
    try:
        completed = _run_dynamic_pulse(
            command,
            cwd=cwd,
            env=env,
            timeout_sec=15.0,
        )
        accepted = completed.returncode == 0
        response = completed.stdout.strip()
        returncode = int(completed.returncode)
    except (OSError, subprocess.SubprocessError) as exc:
        accepted = False
        response = str(exc)
        returncode = -1
    _append_jsonl(
        event_log,
        {
            "event": "robot_dynamic_crossing_zero_command",
            "topic": str(topic),
            "command": command,
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "accepted": bool(accepted),
            "returncode": returncode,
            "response": response,
        },
    )
    return bool(accepted)


def run_director_with_dynamic_crossing(
    command: Sequence[str],
    *,
    cwd: Path,
    env: Mapping[str, str],
    log_path: Path,
    event_log: Path,
    reset_pose: Sequence[float],
    trigger_x: float,
    topic: str,
    linear_x: float,
    motion_sec: float,
    timeout_sec: float,
    post_pose: Sequence[float] | None = None,
) -> int:
    wait_for_robot_reset_service(env)
    initial_pose = post_pose if post_pose is not None else reset_pose
    if not reset_robot_for_intervention(
        initial_pose,
        cwd=cwd,
        env=env,
        event_log=event_log,
        event="robot_dynamic_crossing_initial_pose_requested",
    ):
        raise RuntimeError("robot dynamic-crossing initial-pose intervention was rejected")

    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            env=dict(env),
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        started_at = time.monotonic()
        triggered = False
        zero_sent = False
        try:
            while process.poll() is None:
                now = time.monotonic()
                if now - started_at > float(timeout_sec):
                    raise subprocess.TimeoutExpired(list(command), timeout_sec)
                agent_x = _latest_agent_x(log_path)
                if not triggered and agent_x is not None and agent_x >= float(trigger_x):
                    triggered = True
                    if post_pose is not None and not reset_robot_for_intervention(
                        reset_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_dynamic_crossing_reset_pose_requested",
                    ):
                        raise RuntimeError(
                            "robot dynamic-crossing reset-pose intervention was rejected"
                        )
                    motion_command = build_dynamic_crossing_command(
                        topic=topic,
                        linear_x=linear_x,
                        duration_sec=motion_sec,
                    )
                    _append_jsonl(
                        event_log,
                        {
                            "event": "robot_dynamic_crossing_triggered",
                            "trigger_x": float(trigger_x),
                            "agent_x": float(agent_x),
                            "topic": str(topic),
                            "linear_x": float(linear_x),
                            "motion_sec": float(motion_sec),
                            "command": motion_command,
                            "wall_time": time.time(),
                            "monotonic_time": time.monotonic(),
                        },
                    )
                    completed = _run_dynamic_pulse(
                        motion_command,
                        cwd=cwd,
                        env=env,
                        timeout_sec=max(15.0, float(motion_sec) + 15.0),
                    )
                    _append_jsonl(
                        event_log,
                        {
                            "event": "robot_dynamic_crossing_cmd_vel_pulse_completed",
                            "topic": str(topic),
                            "linear_x": float(linear_x),
                            "motion_sec": float(motion_sec),
                            "command": motion_command,
                            "wall_time": time.time(),
                            "monotonic_time": time.monotonic(),
                            "accepted": completed.returncode == 0,
                            "returncode": int(completed.returncode),
                            "response": completed.stdout.strip(),
                        },
                    )
                    if completed.returncode != 0:
                        raise RuntimeError(
                            "robot dynamic-crossing cmd_vel_pulse command failed"
                        )
                    zero_sent = stop_dynamic_crossing(
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        topic=topic,
                    )
                    if not zero_sent:
                        raise RuntimeError(
                            "robot dynamic-crossing explicit zero command failed"
                        )
                    if post_pose is not None and not reset_robot_for_intervention(
                        post_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_dynamic_crossing_post_pose_requested",
                    ):
                        raise RuntimeError(
                            "robot dynamic-crossing post-pose intervention was rejected"
                        )
                time.sleep(0.2)
            return int(process.returncode or 0)
        finally:
            if triggered and not zero_sent:
                stop_dynamic_crossing(
                    cwd=cwd,
                    env=env,
                    event_log=event_log,
                    topic=topic,
                )
            stop_process_group(process, timeout_sec=5.0)


def reset_robot_for_intervention(
    pose: Sequence[float],
    *,
    cwd: Path,
    env: Mapping[str, str],
    event_log: Path,
    event: str,
    timeout_sec: float = 15.0,
) -> bool:
    command = build_robot_reset_command(pose)
    completed = subprocess.run(
        command,
        cwd=str(cwd),
        env=dict(env),
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=float(timeout_sec),
        check=False,
    )
    output = completed.stdout.strip()
    accepted = completed.returncode == 0 and (
        "accepted=True" in output or "accepted: true" in output.lower()
    )
    _append_jsonl(
        event_log,
        {
            "event": str(event),
            "wall_time": time.time(),
            "monotonic_time": time.monotonic(),
            "pose": [float(value) for value in pose],
            "accepted": bool(accepted),
            "returncode": int(completed.returncode),
            "response": output,
        },
    )
    return accepted


def _latest_agent_x(path: Path) -> float | None:
    matches = re.findall(
        r"(?:HuNav|Local motion) diagnostics:.*?pose=\(([-+]?[0-9]*\.?[0-9]+),",
        _tail(path, 250),
    )
    return float(matches[-1]) if matches else None


def run_director_with_intervention(
    command: Sequence[str],
    *,
    cwd: Path,
    env: Mapping[str, str],
    log_path: Path,
    event_log: Path,
    intervention: RobotIntervention,
    timeout_sec: float,
) -> int:
    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            env=dict(env),
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        started_at = time.monotonic()
        blocked_at: float | None = None
        released = False
        try:
            while process.poll() is None:
                now = time.monotonic()
                if now - started_at > float(timeout_sec):
                    raise subprocess.TimeoutExpired(list(command), timeout_sec)
                agent_x = _latest_agent_x(log_path)
                if blocked_at is None and agent_x is not None and agent_x >= intervention.trigger_x:
                    if not reset_robot_for_intervention(
                        intervention.blocking_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_blocking_pose_requested",
                    ):
                        raise RuntimeError("robot blocking-pose intervention was rejected")
                    blocked_at = time.monotonic()
                elif (
                    blocked_at is not None
                    and not released
                    and now - blocked_at >= intervention.hold_sec
                ):
                    if not reset_robot_for_intervention(
                        intervention.away_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_release_pose_requested",
                    ):
                        raise RuntimeError("robot release-pose intervention was rejected")
                    released = True
                time.sleep(0.2)
            return int(process.returncode or 0)
        except Exception:
            stop_process_group(process, timeout_sec=5.0)
            raise


def run_director_with_occupied_passage(
    command: Sequence[str],
    *,
    cwd: Path,
    env: Mapping[str, str],
    log_path: Path,
    event_log: Path,
    trigger_marker: str,
    release_marker: str,
    blocking_pose: Sequence[float],
    away_pose: Sequence[float],
    timeout_sec: float,
) -> int:
    wait_for_robot_reset_service(env)
    if not reset_robot_for_intervention(
        away_pose,
        cwd=cwd,
        env=env,
        event_log=event_log,
        event="robot_occupied_passage_initial_away_pose_requested",
    ):
        raise RuntimeError("occupied-passage initial-away intervention was rejected")

    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            list(command),
            cwd=str(cwd),
            env=dict(env),
            stdout=stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        started_at = time.monotonic()
        blocking_applied = False
        blocking_released = False
        try:
            while process.poll() is None:
                if time.monotonic() - started_at > float(timeout_sec):
                    raise subprocess.TimeoutExpired(list(command), timeout_sec)
                recent_log = _tail(log_path, 300)
                if not blocking_applied and trigger_marker in recent_log:
                    if not reset_robot_for_intervention(
                        blocking_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_occupied_passage_blocking_pose_requested",
                    ):
                        raise RuntimeError(
                            "occupied-passage blocking intervention was rejected"
                        )
                    blocking_applied = True
                if (
                    blocking_applied
                    and not blocking_released
                    and release_marker in recent_log
                ):
                    if not reset_robot_for_intervention(
                        away_pose,
                        cwd=cwd,
                        env=env,
                        event_log=event_log,
                        event="robot_occupied_passage_release_pose_requested",
                    ):
                        raise RuntimeError(
                            "occupied-passage release intervention was rejected"
                        )
                    blocking_released = True
                time.sleep(0.2)
            if not blocking_applied:
                raise RuntimeError(
                    "occupied-passage trigger was never observed: "
                    f"{trigger_marker!r}"
                )
            return int(process.returncode or 0)
        finally:
            stop_process_group(process, timeout_sec=5.0)
            if not blocking_released:
                reset_robot_for_intervention(
                    away_pose,
                    cwd=cwd,
                    env=env,
                    event_log=event_log,
                    event="robot_occupied_passage_release_pose_requested",
                )


def stop_process_group(process: subprocess.Popen, timeout_sec: float = 20.0) -> None:
    if process.poll() is not None:
        return
    for sig, wait_sec in (
        (signal.SIGINT, timeout_sec),
        (signal.SIGTERM, 5.0),
        (signal.SIGKILL, 2.0),
    ):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=wait_sec)
            return
        except subprocess.TimeoutExpired:
            continue


def _reaction_override(value: str) -> bool | None:
    if value == "source":
        return None
    return value == "enabled"


def _optional_bool_override(value: str) -> bool | None:
    return _reaction_override(value)


def _pose_arg(value: str) -> tuple[float, float, float, float]:
    parts = tuple(float(item.strip()) for item in str(value).split(","))
    if len(parts) != 4 or not all(math.isfinite(item) for item in parts):
        raise argparse.ArgumentTypeError("pose must contain four finite x,y,z,yaw values")
    return parts


def _robot_intervention_arg(value: str) -> str:
    normalized = str(value).strip().lower().replace("-", "_")
    allowed = {
        "none",
        "parked_away",
        "fixed_origin",
        "crossing",
        "dynamic_crossing",
        "occupied_passage",
    }
    if normalized not in allowed:
        raise argparse.ArgumentTypeError(
            "robot intervention must be one of: "
            "none, parked_away, fixed_origin, crossing, dynamic_crossing, occupied_passage"
        )
    return normalized


def build_commands(
    paths: PipelinePaths,
    *,
    benchmark: Path,
    motion_backend: str = "hunav",
    behavior: str,
    target_resource: str,
    initial_agents: int,
    scenario_runtime: str = "source",
) -> tuple[list[str], list[str], list[str]]:
    profile_script = paths.arena_isaac / "scripts" / "arena_scene_profile.py"
    bridge = [
        sys.executable,
        str(profile_script),
        "--profile",
        str(paths.profile),
        "bridge",
        "physx_diff_contact",
    ]
    spawn = [
        sys.executable,
        str(profile_script),
        "--profile",
        str(paths.profile),
        "spawn",
        "--phase",
        "physx_diff_contact",
    ]
    director = [
        "ros2",
        "run",
        "toilet_benchmark",
        "toilet_director_node",
        "--benchmark",
        str(benchmark),
        "--motion-backend",
        str(motion_backend),
        "--profile",
        str(behavior),
        "--initial-agents",
        str(initial_agents),
    ]
    for resource_id in (
        item.strip() for item in str(target_resource).split(",") if item.strip()
    ):
        director.extend(["--target-resource", resource_id])
    if scenario_runtime != "source":
        director.extend(["--scenario-runtime", scenario_runtime])
    return bridge, spawn, director


def _build_parser() -> argparse.ArgumentParser:
    defaults = discover_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=defaults.profile)
    parser.add_argument("--benchmark", type=Path, default=defaults.benchmark)
    parser.add_argument("--output-root", type=Path, default=defaults.output_root)
    parser.add_argument(
        "--scenario-profile",
        choices=tuple(SCENARIO_PROFILES),
        default="standard",
        help="Apply a frozen characterization scenario on top of the benchmark YAML.",
    )
    parser.add_argument("--behavior", default="surprised")
    parser.add_argument(
        "--motion-backend",
        choices=("hunav", "local_motion"),
        default="hunav",
        help=(
            "Select the continuous pedestrian motion backend under test; "
            "the default preserves the existing HuNav smoke workflow."
        ),
    )
    parser.add_argument("--reaction", choices=("source", "enabled", "disabled"), default="source")
    parser.add_argument(
        "--legacy-avoidance",
        choices=("source", "enabled", "disabled"),
        default="source",
        help="Override the legacy regular_avoidance layer for native-BT comparisons.",
    )
    parser.add_argument(
        "--local-motion-shadow",
        choices=("source", "enabled", "disabled"),
        default="source",
        help="Override E2-B sampled-RVO comparison without changing motion authority.",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Override arrival.seed in the isolated per-run benchmark YAML.",
    )
    parser.add_argument(
        "--target-resource",
        default="urinal_1,urinal_2,urinal_3,urinal_4",
        help=(
            "Comma-separated deterministic resource order. Multiple agents rotate "
            "through this list; pass one id explicitly for a queue stress test."
        ),
    )
    parser.add_argument("--initial-agents", type=int, default=1)
    parser.add_argument(
        "--scenario-runtime",
        choices=("source", "legacy", "shadow", "takeover"),
        default="source",
        help="Override the director E1 scenario runtime mode.",
    )
    parser.add_argument(
        "--robot-intervention",
        type=_robot_intervention_arg,
        default="none",
        help=(
            "Use deterministic reset poses to park, place, hold, and remove "
            "a robot during a smoke run."
        ),
    )
    parser.add_argument("--intervention-trigger-x", type=float, default=-1.20)
    parser.add_argument("--intervention-hold-sec", type=float, default=6.0)
    parser.add_argument("--intervention-settle-sec", type=float, default=2.0)
    parser.add_argument(
        "--intervention-away-pose",
        type=_pose_arg,
        default=(0.0, -2.0, 0.03, 0.0),
    )
    parser.add_argument(
        "--intervention-blocking-pose",
        type=_pose_arg,
        default=(0.0, 0.0, 0.03, 0.0),
    )
    parser.add_argument(
        "--dynamic-crossing-topic",
        default=argparse.SUPPRESS,
        help="Override the selected dynamic-crossing profile topic.",
    )
    parser.add_argument(
        "--dynamic-crossing-profile",
        choices=tuple(DYNAMIC_CROSSING_PROFILES),
        default="custom",
        help="Resolved dynamic-crossing profile when robot-intervention is dynamic_crossing.",
    )
    parser.add_argument(
        "--dynamic-crossing-linear-x",
        type=float,
        default=argparse.SUPPRESS,
        help="Override the selected dynamic-crossing profile linear_x.",
    )
    parser.add_argument(
        "--dynamic-crossing-motion-sec",
        type=float,
        default=argparse.SUPPRESS,
        help="Override the selected dynamic-crossing profile motion_sec.",
    )
    parser.add_argument(
        "--dynamic-crossing-trigger-x",
        type=float,
        default=argparse.SUPPRESS,
        help="Override the selected dynamic-crossing profile trigger_x.",
    )
    parser.add_argument(
        "--dynamic-crossing-reset-pose",
        type=_pose_arg,
        default=argparse.SUPPRESS,
        help="Override the selected dynamic-crossing profile reset_pose; yaw should make +x traverse the corridor.",
    )
    parser.add_argument(
        "--dynamic-crossing-post-pose",
        type=_pose_arg,
        default=argparse.SUPPRESS,
        help="Override the pose applied after the velocity pulse; omit to leave the robot in place.",
    )
    parser.add_argument("--bridge-timeout-sec", type=float, default=300.0)
    parser.add_argument("--spawn-timeout-sec", type=float, default=300.0)
    parser.add_argument("--director-timeout-sec", type=float, default=240.0)
    parser.add_argument("--settling-timeout-sec", type=float, default=60.0)
    parser.add_argument("--post-service-stable-sec", type=float, default=5.0)
    parser.add_argument(
        "--reuse-existing-bridge",
        action="store_true",
        help="Use an already running Isaac bridge and never stop it.",
    )
    parser.add_argument(
        "--skip-spawn",
        action="store_true",
        help="Assume the scene and robot are already imported.",
    )
    return parser


def main(args: Sequence[str] | None = None) -> int:
    parsed = _build_parser().parse_args(args)
    scenario_profile = SCENARIO_PROFILES[parsed.scenario_profile]
    initial_agents = (
        int(scenario_profile.initial_agents)
        if scenario_profile.initial_agents is not None
        else int(parsed.initial_agents)
    )
    target_resource = (
        str(scenario_profile.target_resource)
        if scenario_profile.target_resource is not None
        else str(parsed.target_resource)
    )
    robot_intervention = (
        str(scenario_profile.robot_intervention)
        if scenario_profile.robot_intervention is not None
        else str(parsed.robot_intervention)
    )
    defaults = discover_paths()
    paths = PipelinePaths(
        workspace=defaults.workspace,
        arena_isaac=defaults.arena_isaac,
        profile=parsed.profile.expanduser().resolve(),
        benchmark=parsed.benchmark.expanduser().resolve(),
        output_root=parsed.output_root.expanduser().resolve(),
    )
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = paths.output_root / f"{stamp}_{parsed.behavior}_{parsed.reaction}"
    run_dir.mkdir(parents=True, exist_ok=False)
    benchmark_variant = run_dir / "toilet_benchmark.yaml"
    benchmark_payload = make_benchmark_variant(
        paths.benchmark,
        benchmark_variant,
        diagnostic_dir=run_dir,
        reaction_override=_reaction_override(parsed.reaction),
        avoidance_override=_optional_bool_override(parsed.legacy_avoidance),
        local_motion_shadow_override=_optional_bool_override(
            parsed.local_motion_shadow
        ),
        seed_override=parsed.seed,
        scenario_profile=scenario_profile,
    )
    bridge_cmd, spawn_cmd, director_cmd = build_commands(
        paths,
        benchmark=benchmark_variant,
        motion_backend=parsed.motion_backend,
        behavior=parsed.behavior,
        target_resource=target_resource,
        initial_agents=initial_agents,
        scenario_runtime=parsed.scenario_runtime,
    )
    dynamic_crossing_profile, resolved_dynamic_crossing_profile = resolve_dynamic_crossing_profile(
        parsed.dynamic_crossing_profile,
        reset_pose=getattr(parsed, "dynamic_crossing_reset_pose", None),
        post_pose=getattr(parsed, "dynamic_crossing_post_pose", None),
        trigger_x=getattr(parsed, "dynamic_crossing_trigger_x", None),
        topic=getattr(parsed, "dynamic_crossing_topic", None),
        linear_x=getattr(parsed, "dynamic_crossing_linear_x", None),
        motion_sec=getattr(parsed, "dynamic_crossing_motion_sec", None),
    )
    metadata = {
        "behavior": parsed.behavior,
        "motion_backend": parsed.motion_backend,
        "scenario_profile": scenario_profile.name,
        "reaction": parsed.reaction,
        "legacy_avoidance": parsed.legacy_avoidance,
        "seed": int(benchmark_payload.get("arrival", {}).get("seed", 12345)),
        "robot_intervention": robot_intervention,
        "target_resource": target_resource,
        "initial_agents": initial_agents,
        "scenario_runtime": parsed.scenario_runtime,
        "dynamic_crossing": {
            "selected_profile": parsed.dynamic_crossing_profile,
            "resolved_profile": resolved_dynamic_crossing_profile,
            "topic": dynamic_crossing_profile.topic,
            "linear_x": float(dynamic_crossing_profile.linear_x),
            "motion_sec": float(dynamic_crossing_profile.motion_sec),
            "trigger_x": float(dynamic_crossing_profile.trigger_x),
            "reset_pose": [float(value) for value in dynamic_crossing_profile.reset_pose],
            "post_pose": (
                [float(value) for value in dynamic_crossing_profile.post_pose]
                if dynamic_crossing_profile.post_pose is not None
                else None
            ),
        },
        "commands": {
            "bridge": bridge_cmd,
            "spawn": spawn_cmd,
            "director": director_cmd,
        },
        "reuse_existing_bridge": bool(parsed.reuse_existing_bridge),
        "skip_spawn": bool(parsed.skip_spawn),
    }
    (run_dir / "metadata.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    env = os.environ.copy()
    ros_log_dir = run_dir / "ros_log"
    ros_log_dir.mkdir()
    env["ROS_LOG_DIR"] = str(ros_log_dir)
    cached_experience = (
        Path.home()
        / ".cache"
        / "arena_rosnav"
        / "isaacsim45"
        / "isaacsim_exp_base_python_animgraph.kit"
    )
    if cached_experience.is_file():
        env["ISAACSIM_ROSNAV_EXPERIENCE"] = str(cached_experience)
    bridge_log = run_dir / "bridge.log"
    spawn_log = run_dir / "spawn.log"
    director_log = run_dir / "director.log"
    intervention_log = run_dir / "robot_intervention.jsonl"
    visual_envelope_log = run_dir / "pedestrian_visual_envelope.jsonl"
    raw_pose_log = run_dir / "pedestrian_states.jsonl"
    diagnostic_recorder_log = run_dir / "pedestrian_diagnostic_recorder.log"
    env["ARENA_ISAAC_ENABLE_PEDESTRIAN_VISUAL_ENVELOPE"] = "true"
    env["ARENA_ISAAC_PEDESTRIAN_VISUAL_ENVELOPE_LOG_PATH"] = str(
        visual_envelope_log
    )
    print(f"run directory: {run_dir}", flush=True)
    bridge_stream = None
    bridge = None
    diagnostic_recorder = None
    diagnostic_recorder_stream = None
    if not parsed.reuse_existing_bridge:
        bridge_stream = bridge_log.open("w", encoding="utf-8")
        bridge = subprocess.Popen(
            bridge_cmd,
            cwd=str(paths.arena_isaac),
            env=env,
            stdout=bridge_stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
    try:
        if bridge is not None:
            if not wait_for_log_marker(
                bridge_log,
                BRIDGE_STARTUP_MARKER,
                timeout_sec=parsed.bridge_timeout_sec,
                process=bridge,
            ):
                if bridge.poll() is not None:
                    raise RuntimeError(
                        f"bridge exited before startup completed: {bridge.returncode}"
                    )
                raise TimeoutError(
                    "bridge startup marker timed out: "
                    f"{BRIDGE_STARTUP_MARKER!r}"
                )
            print("current bridge startup complete", flush=True)
        else:
            print("reusing existing bridge; checking services", flush=True)
        wait_for_services(
            bridge,
            required=DEFAULT_REQUIRED_SERVICES,
            timeout_sec=parsed.bridge_timeout_sec,
            env=env,
        )
        print("bridge services ready", flush=True)
        time.sleep(max(0.0, parsed.post_service_stable_sec))

        if not parsed.skip_spawn:
            spawn_code = run_logged(
                spawn_cmd,
                cwd=paths.arena_isaac,
                env=env,
                log_path=spawn_log,
                timeout_sec=parsed.spawn_timeout_sec,
            )
            if spawn_code != 0:
                raise RuntimeError(f"spawn failed with code {spawn_code}:\n{_tail(spawn_log)}")
            if bridge is not None:
                settled = wait_for_log_marker(
                    bridge_log,
                    "spawn settling complete",
                    timeout_sec=parsed.settling_timeout_sec,
                    process=bridge,
                )
                print(
                    "robot settling marker observed"
                    if settled
                    else "robot settling marker not observed; continuing after successful spawn",
                    flush=True,
                )
        else:
            print("scene spawn skipped", flush=True)

        diagnostic_recorder_stream = diagnostic_recorder_log.open(
            "w", encoding="utf-8"
        )
        diagnostic_recorder = subprocess.Popen(
            [
                "ros2",
                "run",
                "toilet_benchmark",
                "pedestrian_diagnostic_recorder",
                "--pose-output",
                str(raw_pose_log),
                "--envelope-output",
                str(visual_envelope_log),
            ],
            cwd=str(paths.workspace),
            env=env,
            stdout=diagnostic_recorder_stream,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        time.sleep(0.5)
        if diagnostic_recorder.poll() is not None:
            raise RuntimeError(
                "pedestrian diagnostic recorder exited before the director started"
            )

        if robot_intervention == "parked_away":
            wait_for_robot_reset_service(env)
            if not reset_robot_for_intervention(
                parsed.intervention_away_pose,
                cwd=paths.workspace,
                env=env,
                event_log=intervention_log,
                event="robot_parked_away_pose_requested",
            ):
                raise RuntimeError("robot parked-away intervention was rejected")
            time.sleep(max(0.0, parsed.intervention_settle_sec))
            director_code = run_logged(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                timeout_sec=parsed.director_timeout_sec,
            )
        elif robot_intervention == "fixed_origin":
            wait_for_robot_reset_service(env)
            if not reset_robot_for_intervention(
                parsed.intervention_blocking_pose,
                cwd=paths.workspace,
                env=env,
                event_log=intervention_log,
                event="robot_fixed_origin_pose_requested",
            ):
                raise RuntimeError("robot fixed-origin intervention was rejected")
            time.sleep(max(0.0, parsed.intervention_settle_sec))
            director_code = run_logged(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                timeout_sec=parsed.director_timeout_sec,
            )
        elif robot_intervention == "crossing":
            wait_for_robot_reset_service(env)
            if not reset_robot_for_intervention(
                parsed.intervention_away_pose,
                cwd=paths.workspace,
                env=env,
                event_log=intervention_log,
                event="robot_initial_away_pose_requested",
            ):
                raise RuntimeError("robot initial-away intervention was rejected")
            time.sleep(max(0.0, parsed.intervention_settle_sec))
            director_code = run_director_with_intervention(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                event_log=intervention_log,
                intervention=RobotIntervention(
                    trigger_x=float(parsed.intervention_trigger_x),
                    hold_sec=max(0.0, float(parsed.intervention_hold_sec)),
                    away_pose=parsed.intervention_away_pose,
                    blocking_pose=parsed.intervention_blocking_pose,
                ),
                timeout_sec=parsed.director_timeout_sec,
            )
        elif robot_intervention == "dynamic_crossing":
            director_code = run_director_with_dynamic_crossing(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                event_log=intervention_log,
                reset_pose=dynamic_crossing_profile.reset_pose,
                trigger_x=float(dynamic_crossing_profile.trigger_x),
                topic=str(dynamic_crossing_profile.topic),
                linear_x=float(dynamic_crossing_profile.linear_x),
                motion_sec=max(0.0, float(dynamic_crossing_profile.motion_sec)),
                timeout_sec=parsed.director_timeout_sec,
                post_pose=dynamic_crossing_profile.post_pose,
            )
        elif robot_intervention == "occupied_passage":
            if (
                scenario_profile.trigger_marker is None
                or scenario_profile.release_marker is None
                or scenario_profile.robot_blocking_pose is None
                or scenario_profile.robot_away_pose is None
            ):
                raise RuntimeError(
                    "occupied-passage intervention requires a complete scenario profile"
                )
            director_code = run_director_with_occupied_passage(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                event_log=intervention_log,
                trigger_marker=scenario_profile.trigger_marker,
                release_marker=scenario_profile.release_marker,
                blocking_pose=scenario_profile.robot_blocking_pose,
                away_pose=scenario_profile.robot_away_pose,
                timeout_sec=parsed.director_timeout_sec,
            )
        else:
            director_code = run_logged(
                director_cmd,
                cwd=paths.workspace,
                env=env,
                log_path=director_log,
                timeout_sec=parsed.director_timeout_sec,
            )
        if diagnostic_recorder is not None:
            stop_process_group(diagnostic_recorder, timeout_sec=5.0)
            diagnostic_recorder = None
        if diagnostic_recorder_stream is not None:
            diagnostic_recorder_stream.close()
            diagnostic_recorder_stream = None
        director_text = director_log.read_text(encoding="utf-8", errors="replace")
        summary = summarize_director_log(director_text, exit_code=director_code)
        summary["robot_intervention_events"] = (
            len(intervention_log.read_text(encoding="utf-8").splitlines())
            if intervention_log.exists()
            else 0
        )
        summary["native_behavior_transition_events"] = sum(
            len(path.read_text(encoding="utf-8", errors="replace").splitlines())
            for path in run_dir.glob("hunav_behavior_events_*.jsonl")
        )
        expected_agent_ids = tuple(sorted(_activation_times(director_text)))
        summary.update(
            summarize_visual_envelope_log(
                visual_envelope_log,
                expected_agent_ids=expected_agent_ids,
            )
        )
        visual_envelope_passed = apply_visual_envelope_gate(
            summary,
            expected_agent_ids=expected_agent_ids,
        )
        summary.update(
            summarize_raw_pose_log(
                raw_pose_log,
                director_text=director_text,
            )
        )
        (run_dir / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
        return (
            0
            if (
                director_code == 0
                and bool(summary["completed"])
                and visual_envelope_passed
            )
            else 1
        )
    except (RuntimeError, TimeoutError, subprocess.TimeoutExpired) as exc:
        (run_dir / "runner_error.txt").write_text(f"{exc}\n", encoding="utf-8")
        print(f"pipeline failed: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if diagnostic_recorder is not None:
            stop_process_group(diagnostic_recorder, timeout_sec=5.0)
        if diagnostic_recorder_stream is not None:
            diagnostic_recorder_stream.close()
        if bridge is not None:
            stop_process_group(bridge)
        if bridge_stream is not None:
            bridge_stream.close()


if __name__ == "__main__":
    raise SystemExit(main())
