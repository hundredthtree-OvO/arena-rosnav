"""Compare fresh and pooled Isaac People locomotion across repeated routes."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Iterable, Sequence

from .episodes.schema import EpisodeSpec, PedestrianEpisodeSpec
from .pedestrian_state_stream import PedestrianObservation, observations_from_message


@dataclass(frozen=True)
class ProbeSample:
    elapsed_sec: float
    x: float
    y: float
    yaw: float | None
    speed_mps: float
    motion_state: str
    generation: int
    command_generation: int

    def to_mapping(self) -> dict:
        return {
            "elapsed_sec": self.elapsed_sec,
            "x": self.x,
            "y": self.y,
            "yaw": self.yaw,
            "speed_mps": self.speed_mps,
            "motion_state": self.motion_state,
            "generation": self.generation,
            "command_generation": self.command_generation,
        }


def _angle_delta(lhs: float, rhs: float) -> float:
    return math.atan2(math.sin(lhs - rhs), math.cos(lhs - rhs))


def _segment_distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_sq = dx * dx + dy * dy
    if length_sq <= 1e-12:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    progress = max(
        0.0,
        min(
            1.0,
            ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy)
            / length_sq,
        ),
    )
    projected = (start[0] + progress * dx, start[1] + progress * dy)
    return math.hypot(point[0] - projected[0], point[1] - projected[1])


def _polyline_distance(point, route) -> float:
    return min(
        _segment_distance(point, start, end)
        for start, end in zip(route, route[1:])
    )


def analyze_probe_samples(
    samples: Sequence[ProbeSample],
    route: Sequence[Sequence[float]],
) -> dict:
    executing = [sample for sample in samples if sample.motion_state == "executing"]
    if not executing:
        return {"valid": False, "reason": "no_executing_samples", "sample_count": len(samples)}
    started_at = executing[0].elapsed_sec
    first_window = [sample for sample in executing if sample.elapsed_sec - started_at <= 2.0]
    start, forward = route[:2]
    dx, dy = forward[0] - start[0], forward[1] - start[1]
    length = math.hypot(dx, dy)
    ux, uy = (dx / length, dy / length) if length > 1e-9 else (1.0, 0.0)
    lateral = [
        abs(-(sample.x - start[0]) * uy + (sample.y - start[1]) * ux)
        for sample in first_window
    ]
    route_errors = [_polyline_distance((sample.x, sample.y), route) for sample in executing]
    yaw_deltas = []
    turn_signs = []
    previous_yaw = None
    for sample in executing:
        if sample.yaw is None:
            continue
        if previous_yaw is not None:
            delta = _angle_delta(sample.yaw, previous_yaw)
            yaw_deltas.append(abs(delta))
            if abs(delta) > 1e-3:
                turn_signs.append(1 if delta > 0.0 else -1)
        previous_yaw = sample.yaw
    reversals = sum(a != b for a, b in zip(turn_signs, turn_signs[1:]))
    stopped = [sample for sample in executing if sample.speed_mps < 0.03]
    return {
        "valid": True,
        "sample_count": len(samples),
        "executing_sample_count": len(executing),
        "generation": executing[0].generation,
        "command_generation": executing[0].command_generation,
        "first_2s_max_lateral_m": max(lateral, default=0.0),
        "route_error_mean_m": sum(route_errors) / len(route_errors),
        "route_error_max_m": max(route_errors),
        "absolute_heading_change_rad": sum(yaw_deltas),
        "turn_direction_reversal_count": reversals,
        "stopped_sample_count": len(stopped),
        "animgraph_clip_phase_observable": False,
        "animgraph_clip_phase_note": (
            "Isaac People exposes Action/Walk commands but not locomotion clip or foot phase; "
            "fresh-vs-pooled trajectory differences are the isolation signal."
        ),
    }


def time_aligned_root_rmse(
    reference: Sequence[ProbeSample],
    candidate: Sequence[ProbeSample],
    *,
    duration_sec: float = 2.0,
) -> float | None:
    """Compare root trajectories at candidate timestamps after execution starts."""

    reference = [sample for sample in reference if sample.motion_state == "executing"]
    candidate = [sample for sample in candidate if sample.motion_state == "executing"]
    if len(reference) < 2 or len(candidate) < 2:
        return None
    reference_start = reference[0].elapsed_sec
    candidate_start = candidate[0].elapsed_sec
    reference_timed = [
        (sample.elapsed_sec - reference_start, sample)
        for sample in reference
        if sample.elapsed_sec - reference_start <= duration_sec
    ]
    errors = []
    for sample in candidate:
        elapsed = sample.elapsed_sec - candidate_start
        if elapsed < 0.0 or elapsed > duration_sec:
            continue
        nearest = min(reference_timed, key=lambda item: abs(item[0] - elapsed))[1]
        errors.append((sample.x - nearest.x) ** 2 + (sample.y - nearest.y) ** 2)
    if not errors:
        return None
    return math.sqrt(sum(errors) / len(errors))


def build_probe_episode(
    source: EpisodeSpec,
    *,
    case: str,
    source_agent_id: str | None,
    runtime_agent_id: str,
) -> EpisodeSpec:
    candidates = list(source.pedestrians)
    if source_agent_id:
        candidates = [item for item in candidates if item.agent_id == source_agent_id]
    if not candidates:
        raise ValueError(f"source pedestrian {source_agent_id!r} was not found")
    pedestrian = candidates[0]
    if len(pedestrian.route_waypoints) < 2:
        raise ValueError("phase probe source route needs at least two authored targets")
    if case == "straight":
        pedestrian = replace(
            pedestrian,
            route_waypoints=(pedestrian.route_waypoints[1],),
            holds=(),
        )
    elif case == "turn":
        pedestrian = replace(pedestrian, holds=())
    elif case == "stop_resume":
        holds = tuple(pedestrian.holds[:1])
        if not holds:
            raise ValueError("stop_resume requires a hold on the source route")
        pedestrian = replace(pedestrian, holds=holds)
    else:
        raise ValueError(f"unsupported phase probe case: {case}")
    pedestrian = replace(pedestrian, agent_id=runtime_agent_id)
    return replace(
        source,
        episode_id=f"animgraph_{case}_{runtime_agent_id}",
        pedestrians=(pedestrian,),
        difficulty={"pedestrian_count": 1, "probe_case": case},
        metadata={**source.metadata, "source": "animgraph_phase_probe", "probe_case": case},
    )


class _LiveRecorder:
    def __init__(self, node):
        from people_msgs.msg import People

        self._lock = threading.Lock()
        self._active_agent = ""
        self._started_at = 0.0
        self._samples: list[ProbeSample] = []
        self._subscription = node.create_subscription(
            People,
            "/isaac/pedestrian_states",
            self._callback,
            50,
        )

    def begin(self, agent_id: str) -> None:
        with self._lock:
            self._active_agent = str(agent_id)
            self._started_at = time.monotonic()
            self._samples = []

    def finish(self) -> list[ProbeSample]:
        with self._lock:
            result = list(self._samples)
            self._active_agent = ""
            return result

    def _callback(self, message) -> None:
        observed_at = time.monotonic()
        with self._lock:
            active_agent = self._active_agent
            started_at = self._started_at
        if not active_agent:
            return
        for observation in observations_from_message(message, observed_at=observed_at):
            if observation.identifier != active_agent:
                continue
            sample = _sample_from_observation(observation, observed_at - started_at)
            with self._lock:
                if self._active_agent == active_agent:
                    self._samples.append(sample)


def _sample_from_observation(
    observation: PedestrianObservation,
    elapsed_sec: float,
) -> ProbeSample:
    metadata = observation.metadata
    try:
        yaw = float(metadata.get("yaw_rad", "nan"))
    except (TypeError, ValueError):
        yaw = math.nan
    velocity = observation.velocity or (0.0, 0.0, 0.0)
    return ProbeSample(
        elapsed_sec=float(elapsed_sec),
        x=observation.position[0],
        y=observation.position[1],
        yaw=yaw if math.isfinite(yaw) else None,
        speed_mps=math.hypot(velocity[0], velocity[1]),
        motion_state=str(metadata.get("motion_state", "unknown")),
        generation=int(metadata.get("embodiment_generation", "0") or 0),
        command_generation=int(metadata.get("command_generation", "0") or 0),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", required=True)
    parser.add_argument("--agent-id")
    parser.add_argument("--case", choices=("straight", "turn", "stop_resume"), default="turn")
    parser.add_argument("--lifecycle", choices=("pooled", "fresh"), default="pooled")
    parser.add_argument("--repetitions", type=int, default=10)
    parser.add_argument("--planner-radius-m", type=float, default=0.30)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--output", type=Path, default=Path("/tmp/toilet_animgraph_phase_probe"))
    return parser


def _run_authored(command: Sequence[str], log_path: Path, timeout_sec: float) -> int:
    with log_path.open("w", encoding="utf-8") as stream:
        process = subprocess.Popen(
            list(command),
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            return process.wait(timeout=float(timeout_sec))
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
            return 124


def main(args: Sequence[str] | None = None) -> int:
    from .tracks.authored_scenario import load_authored_episode, plan_episode_routes

    namespace = _parser().parse_args(args)
    if namespace.repetitions < 1:
        raise SystemExit("--repetitions must be positive")
    source = load_authored_episode(namespace.episode)
    run_dir = namespace.output.expanduser().resolve() / (
        time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000:06d}"
    )
    run_dir.mkdir(parents=True, exist_ok=False)

    import rclpy

    rclpy.init(args=None)
    node = rclpy.create_node("animgraph_phase_probe")
    recorder = _LiveRecorder(node)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    results = []
    reference_samples: list[ProbeSample] | None = None
    try:
        for index in range(namespace.repetitions):
            runtime_agent_id = (
                namespace.agent_id or source.pedestrians[0].agent_id
                if namespace.lifecycle == "pooled"
                else f"animgraph_probe_{index + 1:02d}"
            )
            episode = build_probe_episode(
                source,
                case=namespace.case,
                source_agent_id=namespace.agent_id,
                runtime_agent_id=runtime_agent_id,
            )
            episode_path = run_dir / f"episode_{index + 1:03d}.json"
            episode_path.write_text(
                json.dumps(episode.to_dict(), indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            planned = plan_episode_routes(episode, planner_radius_m=namespace.planner_radius_m)[0]
            recorder.begin(runtime_agent_id)
            log_path = run_dir / f"episode_{index + 1:03d}.log"
            command = [
                sys.executable,
                "-m",
                "toilet_benchmark.tracks.authored_scenario",
                "--episode",
                str(episode_path),
                "--skip-robot-reset",
                "--planner-radius-m",
                str(namespace.planner_radius_m),
            ]
            exit_code = _run_authored(command, log_path, namespace.timeout_sec)
            samples = recorder.finish()
            raw_path = run_dir / f"episode_{index + 1:03d}.jsonl"
            raw_path.write_text(
                "".join(json.dumps(sample.to_mapping(), sort_keys=True) + "\n" for sample in samples),
                encoding="utf-8",
            )
            metrics = analyze_probe_samples(samples, planned.route_waypoints)
            metrics["first_2s_root_rmse_vs_run1_m"] = (
                0.0
                if reference_samples is None
                else time_aligned_root_rmse(reference_samples, samples)
            )
            metrics.update(
                {
                    "episode_index": index + 1,
                    "agent_id": runtime_agent_id,
                    "exit_code": exit_code,
                }
            )
            results.append(metrics)
            if reference_samples is None:
                reference_samples = samples
            print(json.dumps(metrics, sort_keys=True))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    summary = {
        "case": namespace.case,
        "lifecycle": namespace.lifecycle,
        "repetitions": namespace.repetitions,
        "source_episode": str(Path(namespace.episode).expanduser().resolve()),
        "results": results,
    }
    (run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"probe directory: {run_dir}")
    return 0 if all(item.get("valid") and item.get("exit_code") == 0 for item in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
