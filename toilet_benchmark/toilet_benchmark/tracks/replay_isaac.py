"""Replay-to-Isaac external motion adapter and deterministic playback core."""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import dataclass, field, replace
from typing import Any, Callable, Mapping, Sequence

from toilet_benchmark.domain.task import (
    EXTERNAL_MOTION_FREEZE,
    EXTERNAL_MOTION_LOCOMOTION,
    EXTERNAL_MOTION_REPLAY_TRACK,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
)

from .replay import (
    ReplayBundle,
    ReplayRuntime,
    ReplayStateFrame,
    compute_replay_bundle_hash,
    load_replay_bundle,
    require_valid_replay_bundle,
)
from ..hunav_isaac_mirror_core import person_yaw, point_to_polyline_distance

try:  # pragma: no cover - optional runtime dependency for live ROS integration
    from ..motion_backend import IsaacPeopleBackend, MotionCommand
except Exception:  # pragma: no cover - import guard for core-only test runs
    IsaacPeopleBackend = None
    MotionCommand = None

try:  # pragma: no cover - optional runtime dependency for live ROS integration
    from people_msgs.msg import People
except Exception:  # pragma: no cover - import guard for core-only test runs
    People = None


REPLAY_ISAAC_DEFAULT_RATE_HZ = 10.0
REPLAY_ISAAC_DEFAULT_TIME_SCALE = 1.0
REPLAY_ISAAC_DEFAULT_EXTERNAL_TIMEOUT_SEC = 0.35
REPLAY_ISAAC_DEFAULT_SPAWN_TIMEOUT_SEC = 30.0
REPLAY_ISAAC_DEFAULT_LIVE_POSE_TOPIC = "/isaac/pedestrian_states"
REPLAY_ISAAC_DEFAULT_LIVE_POSE_STALE_SEC = 0.5
REPLAY_ISAAC_DEFAULT_CATCH_UP_SEC = 0.35
REPLAY_ISAAC_STALL_GRACE_SEC = 1.0
REPLAY_ISAAC_MAX_STALL_LEAD_SEC = 2.0
REPLAY_ISAAC_DEFAULT_TERMINAL_STABLE_SAMPLES = 3
REPLAY_ISAAC_DEFAULT_TERMINAL_TIMEOUT_SEC = 4.0
REPLAY_ISAAC_DEFAULT_TERMINAL_POSITION_TOLERANCE_M = 0.35
REPLAY_ISAAC_DEFAULT_TERMINAL_YAW_TOLERANCE_RAD = 0.15
REPLAY_ISAAC_DEFAULT_TERMINAL_MAX_SPEED_MPS = 0.05
REPLAY_ISAAC_TERMINAL_ALIGN = "align"
REPLAY_ISAAC_TERMINAL_FREEZE = "freeze"
REPLAY_ISAAC_TERMINAL_MODES = (
    REPLAY_ISAAC_TERMINAL_ALIGN,
    REPLAY_ISAAC_TERMINAL_FREEZE,
)


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _choose_clock_now(
    physics_now_sec: float | None,
    wall_now_sec: float | None,
) -> tuple[str, float]:
    if _is_finite(physics_now_sec):
        return "physics", float(physics_now_sec)
    if _is_finite(wall_now_sec):
        return "wall", float(wall_now_sec)
    raise ValueError("either physics_now_sec or wall_now_sec must be finite")


def _motion_backend_required() -> tuple[type[Any], type[Any]]:
    if IsaacPeopleBackend is None or MotionCommand is None:
        raise RuntimeError(
            "isaacsim_msgs / motion backend dependencies are unavailable; "
            "core playback and dry-run remain usable, but live Isaac dispatch "
            "requires the package to be importable"
        )
    return IsaacPeopleBackend, MotionCommand


@dataclass(frozen=True)
class ReplayIsaacCommandSpec:
    agent_id: str
    goal_pose: tuple[float, float, float]
    direct_pose: tuple[float, float, float]
    orientation: float
    velocity: float
    external_velocity: tuple[float, float, float]
    external_timeout_sec: float
    use_direct_pose: bool = False
    use_external_motion: bool = True
    external_freeze_pose: bool = False
    external_motion_mode: int = EXTERNAL_MOTION_LOCOMOTION
    stop: bool = False
    constrain_to_path: bool = False
    path_points: tuple[tuple[float, float, float], ...] = field(default_factory=tuple)

    def to_dict(self) -> dict[str, Any]:
        return {
            "agent_id": self.agent_id,
            "goal_pose": [float(value) for value in self.goal_pose],
            "path_points": [
                [float(value) for value in point] for point in self.path_points
            ],
            "velocity": float(self.velocity),
            "orientation": float(self.orientation),
            "stop": bool(self.stop),
            "use_direct_pose": bool(self.use_direct_pose),
            "direct_pose": [float(value) for value in self.direct_pose],
            "use_external_motion": bool(self.use_external_motion),
            "external_velocity": [
                float(value) for value in self.external_velocity
            ],
            "external_timeout_sec": float(self.external_timeout_sec),
            "external_freeze_pose": bool(self.external_freeze_pose),
            "external_motion_mode": int(self.external_motion_mode),
            "constrain_to_path": bool(self.constrain_to_path),
        }


@dataclass(frozen=True)
class ReplayIsaacLivePose:
    agent_id: str
    x: float
    y: float
    z: float
    yaw: float | None
    vx: float
    vy: float
    source_stamp_sec: float | None
    received_monotonic_sec: float
    reference_playback_sec: float | None = None

    def to_dict(self) -> dict[str, Any]:
        payload = {
            "agent_id": self.agent_id,
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "vx": float(self.vx),
            "vy": float(self.vy),
            "received_monotonic_sec": float(self.received_monotonic_sec),
        }
        if self.yaw is not None:
            payload["yaw"] = float(self.yaw)
        if self.source_stamp_sec is not None:
            payload["source_stamp_sec"] = float(self.source_stamp_sec)
        if self.reference_playback_sec is not None:
            payload["reference_playback_sec"] = float(self.reference_playback_sec)
        return payload


@dataclass(frozen=True)
class ReplayIsaacTerminalResult:
    result: str
    reason: str
    stable_samples: int
    required_stable_samples: int
    elapsed_sec: float
    timeout_sec: float
    position_error_m: float | None = None
    yaw_error_rad: float | None = None
    live_pose_age_sec: float | None = None
    live_pose: ReplayIsaacLivePose | None = None

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "result": self.result,
            "reason": self.reason,
            "stable_samples": int(self.stable_samples),
            "required_stable_samples": int(self.required_stable_samples),
            "elapsed_sec": float(self.elapsed_sec),
            "timeout_sec": float(self.timeout_sec),
        }
        if self.position_error_m is not None:
            payload["position_error_m"] = float(self.position_error_m)
        if self.yaw_error_rad is not None:
            payload["yaw_error_rad"] = float(self.yaw_error_rad)
        if self.live_pose_age_sec is not None:
            payload["live_pose_age_sec"] = float(self.live_pose_age_sec)
        if self.live_pose is not None:
            payload["live_pose"] = self.live_pose.to_dict()
        return payload


@dataclass(frozen=True)
class ReplayIsaacTrackingMetrics:
    live_sample_count: int
    reference_sample_count: int
    time_aligned_sample_count: int
    time_aligned_position_rmse_m: float | None
    path_lateral_error_rmse_m: float | None
    path_lateral_error_p95_m: float | None
    path_lateral_error_max_m: float | None
    terminal_position_error_m: float | None
    terminal_yaw_error_rad: float | None

    def to_dict(self) -> dict[str, Any]:
        return {
            "live_sample_count": int(self.live_sample_count),
            "reference_sample_count": int(self.reference_sample_count),
            "time_aligned_sample_count": int(self.time_aligned_sample_count),
            "time_aligned_position_rmse_m": (
                None
                if self.time_aligned_position_rmse_m is None
                else float(self.time_aligned_position_rmse_m)
            ),
            "path_lateral_error_rmse_m": (
                None
                if self.path_lateral_error_rmse_m is None
                else float(self.path_lateral_error_rmse_m)
            ),
            "path_lateral_error_p95_m": (
                None
                if self.path_lateral_error_p95_m is None
                else float(self.path_lateral_error_p95_m)
            ),
            "path_lateral_error_max_m": (
                None
                if self.path_lateral_error_max_m is None
                else float(self.path_lateral_error_max_m)
            ),
            "terminal_position_error_m": (
                None
                if self.terminal_position_error_m is None
                else float(self.terminal_position_error_m)
            ),
            "terminal_yaw_error_rad": (
                None
                if self.terminal_yaw_error_rad is None
                else float(self.terminal_yaw_error_rad)
            ),
        }


def _command_spec_from_frame(
    frame: ReplayStateFrame,
    *,
    external_timeout_sec: float,
    terminal_mode: str,
) -> ReplayIsaacCommandSpec:
    snapshot = frame.snapshot
    terminal = bool(frame.finished)
    if terminal_mode not in REPLAY_ISAAC_TERMINAL_MODES:
        raise ValueError(
            f"terminal_mode must be one of {REPLAY_ISAAC_TERMINAL_MODES}"
        )
    if terminal and terminal_mode == REPLAY_ISAAC_TERMINAL_FREEZE:
        mode = EXTERNAL_MOTION_FREEZE
        freeze_pose = True
        external_velocity = (0.0, 0.0, 0.0)
    elif terminal:
        mode = EXTERNAL_MOTION_TERMINAL_ALIGN
        freeze_pose = False
        external_velocity = (0.0, 0.0, 0.0)
    else:
        mode = EXTERNAL_MOTION_REPLAY_TRACK
        freeze_pose = False
        external_velocity = (float(snapshot.vx), float(snapshot.vy), 0.0)
    return ReplayIsaacCommandSpec(
        agent_id=snapshot.agent_id,
        goal_pose=(float(snapshot.x), float(snapshot.y), float(snapshot.z)),
        direct_pose=(float(snapshot.x), float(snapshot.y), float(snapshot.z)),
        orientation=float(snapshot.yaw),
        velocity=0.0 if terminal else math.hypot(float(snapshot.vx), float(snapshot.vy)),
        external_velocity=external_velocity,
        external_timeout_sec=float(external_timeout_sec),
        external_freeze_pose=freeze_pose,
        external_motion_mode=mode,
    )


def _angle_delta_rad(target: float, actual: float) -> float:
    return math.atan2(
        math.sin(float(target) - float(actual)),
        math.cos(float(target) - float(actual)),
    )


def _percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(float(value) for value in values)
    index = max(
        0,
        min(len(ordered) - 1, int(math.ceil(float(fraction) * len(ordered))) - 1),
    )
    return ordered[index]


def _rmse(values: Sequence[float]) -> float | None:
    if not values:
        return None
    return math.sqrt(sum(float(value) * float(value) for value in values) / len(values))


def _live_pose_time_sec(
    live_pose: ReplayIsaacLivePose,
    *,
    base_received_monotonic_sec: float,
    base_source_stamp_sec: float | None,
    time_scale: float,
) -> float:
    if _is_finite(live_pose.source_stamp_sec) and base_source_stamp_sec is not None:
        elapsed_sec = float(live_pose.source_stamp_sec) - float(base_source_stamp_sec)
    else:
        elapsed_sec = (
            float(live_pose.received_monotonic_sec)
            - float(base_received_monotonic_sec)
        )
    return max(0.0, elapsed_sec) * float(time_scale)


def _tracking_metrics_from_live_samples(
    runtime: ReplayRuntime,
    live_poses: Sequence[ReplayIsaacLivePose],
    terminal_result: ReplayIsaacTerminalResult | None,
    *,
    time_scale: float,
) -> ReplayIsaacTrackingMetrics:
    reference_trajectory = runtime.agent.trajectory
    reference_path = [
        (float(sample.x), float(sample.y)) for sample in reference_trajectory
    ]
    if live_poses:
        base_received_monotonic_sec = float(live_poses[0].received_monotonic_sec)
        base_source_stamp_sec = next(
            (
                float(sample.source_stamp_sec)
                for sample in live_poses
                if _is_finite(sample.source_stamp_sec)
            ),
            None,
        )
    else:
        base_received_monotonic_sec = 0.0
        base_source_stamp_sec = None

    time_aligned_errors: list[float] = []
    lateral_errors: list[float] = []
    for live_pose in live_poses:
        if _is_finite(live_pose.reference_playback_sec):
            live_time_sec = float(live_pose.reference_playback_sec)
        else:
            live_time_sec = _live_pose_time_sec(
                live_pose,
                base_received_monotonic_sec=base_received_monotonic_sec,
                base_source_stamp_sec=base_source_stamp_sec,
                time_scale=time_scale,
            )
        reference_snapshot = runtime.sample(live_time_sec)
        time_aligned_errors.append(
            math.hypot(
                float(live_pose.x) - float(reference_snapshot.x),
                float(live_pose.y) - float(reference_snapshot.y),
            )
        )
        lateral_error = point_to_polyline_distance(
            float(live_pose.x),
            float(live_pose.y),
            reference_path,
        )
        if lateral_error is not None and math.isfinite(float(lateral_error)):
            lateral_errors.append(float(lateral_error))

    terminal_live_pose = (
        terminal_result.live_pose
        if terminal_result is not None and terminal_result.live_pose is not None
        else (live_poses[-1] if live_poses else None)
    )
    terminal_reference_snapshot = runtime.sample(runtime.terminal_timestamp_sec)
    terminal_position_error_m = None
    terminal_yaw_error_rad = None
    if terminal_live_pose is not None:
        terminal_position_error_m = math.hypot(
            float(terminal_live_pose.x) - float(terminal_reference_snapshot.x),
            float(terminal_live_pose.y) - float(terminal_reference_snapshot.y),
        )
        if terminal_live_pose.yaw is not None and _is_finite(terminal_live_pose.yaw):
            terminal_yaw_error_rad = _angle_delta_rad(
                float(terminal_reference_snapshot.yaw),
                float(terminal_live_pose.yaw),
            )

    return ReplayIsaacTrackingMetrics(
        live_sample_count=len(live_poses),
        reference_sample_count=len(reference_trajectory),
        time_aligned_sample_count=len(time_aligned_errors),
        time_aligned_position_rmse_m=_rmse(time_aligned_errors),
        path_lateral_error_rmse_m=_rmse(lateral_errors),
        path_lateral_error_p95_m=_percentile(lateral_errors, 0.95),
        path_lateral_error_max_m=max(lateral_errors) if lateral_errors else None,
        terminal_position_error_m=terminal_position_error_m,
        terminal_yaw_error_rad=terminal_yaw_error_rad,
    )


def _motion_command_from_spec(spec: ReplayIsaacCommandSpec):
    _, MotionCommandType = _motion_backend_required()
    return MotionCommandType(
        agent_id=spec.agent_id,
        goal_pose=spec.goal_pose,
        path_points=spec.path_points,
        velocity=spec.velocity,
        orientation=spec.orientation,
        stop=spec.stop,
        use_direct_pose=spec.use_direct_pose,
        direct_pose=spec.direct_pose,
        use_external_motion=spec.use_external_motion,
        external_velocity=spec.external_velocity,
        external_timeout_sec=spec.external_timeout_sec,
        external_freeze_pose=spec.external_freeze_pose,
        external_motion_mode=spec.external_motion_mode,
        constrain_to_path=spec.constrain_to_path,
    )


@dataclass(frozen=True)
class ReplayIsaacDispatch:
    sample_index: int
    clock_source: str
    clock_sec: float
    playback_sec: float
    finished: bool
    frame: ReplayStateFrame
    command_spec: ReplayIsaacCommandSpec

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_index": int(self.sample_index),
            "clock_source": self.clock_source,
            "clock_sec": float(self.clock_sec),
            "playback_sec": float(self.playback_sec),
            "finished": bool(self.finished),
            "frame": self.frame.to_dict(),
            "command": self.command_spec.to_dict(),
        }


@dataclass(frozen=True)
class ReplayIsaacDryRunResult:
    bundle_hash: str
    dispatches: tuple[ReplayIsaacDispatch, ...]
    rate_hz: float
    time_scale: float
    terminal_mode: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "bundle_hash": self.bundle_hash,
            "rate_hz": float(self.rate_hz),
            "time_scale": float(self.time_scale),
            "terminal_mode": self.terminal_mode,
            "dispatches": [dispatch.to_dict() for dispatch in self.dispatches],
        }


class ReplayIsaacPlayback:
    """Deterministic replay-time sampler for Isaac external motion."""

    def __init__(
        self,
        bundle_or_runtime: ReplayBundle | ReplayRuntime,
        *,
        rate_hz: float = REPLAY_ISAAC_DEFAULT_RATE_HZ,
        time_scale: float = REPLAY_ISAAC_DEFAULT_TIME_SCALE,
        external_timeout_sec: float = REPLAY_ISAAC_DEFAULT_EXTERNAL_TIMEOUT_SEC,
        terminal_mode: str = REPLAY_ISAAC_TERMINAL_ALIGN,
    ):
        if isinstance(bundle_or_runtime, ReplayRuntime):
            self._runtime = bundle_or_runtime
        else:
            self._runtime = ReplayRuntime(bundle_or_runtime)
        if not _is_finite(rate_hz) or float(rate_hz) <= 0.0:
            raise ValueError("rate_hz must be a positive finite number")
        if not _is_finite(time_scale) or float(time_scale) <= 0.0:
            raise ValueError("time_scale must be a positive finite number")
        if not _is_finite(external_timeout_sec) or float(external_timeout_sec) <= 0.0:
            raise ValueError(
                "external_timeout_sec must be a positive finite number"
            )
        if terminal_mode not in REPLAY_ISAAC_TERMINAL_MODES:
            raise ValueError(
                f"terminal_mode must be one of {REPLAY_ISAAC_TERMINAL_MODES}"
            )
        self._rate_hz = float(rate_hz)
        self._time_scale = float(time_scale)
        self._external_timeout_sec = float(external_timeout_sec)
        self._terminal_mode = terminal_mode
        self._clock_origin_sec: float | None = None
        self._last_playback_sec: float | None = None
        self._dispatch_index = 0
        self._finished = False

    @property
    def runtime(self) -> ReplayRuntime:
        return self._runtime

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    @property
    def time_scale(self) -> float:
        return self._time_scale

    @property
    def terminal_mode(self) -> str:
        return self._terminal_mode

    @property
    def finished(self) -> bool:
        return self._finished

    def _dispatch_from_playback_sec(
        self,
        *,
        sample_index: int,
        clock_source: str,
        clock_sec: float,
        playback_sec: float,
    ) -> ReplayIsaacDispatch:
        frame = ReplayStateFrame(
            sample_index=sample_index,
            timestamp_sec=playback_sec,
            snapshot=self._runtime.sample(playback_sec),
            finished=self._runtime.is_finished(playback_sec),
        )
        return ReplayIsaacDispatch(
            sample_index=frame.sample_index,
            clock_source=clock_source,
            clock_sec=clock_sec,
            playback_sec=playback_sec,
            finished=frame.finished,
            frame=frame,
            command_spec=_command_spec_from_frame(
                frame,
                external_timeout_sec=self._external_timeout_sec,
                terminal_mode=self._terminal_mode,
            ),
        )

    def step(self, clock_now_sec: float) -> ReplayIsaacDispatch | None:
        if self._finished:
            return None
        if not _is_finite(clock_now_sec):
            raise ValueError("clock_now_sec must be finite")
        current_sec = float(clock_now_sec)
        if self._clock_origin_sec is None:
            self._clock_origin_sec = current_sec
        elapsed_sec = max(0.0, current_sec - self._clock_origin_sec)
        playback_sec = min(
            elapsed_sec * self._time_scale,
            self._runtime.terminal_timestamp_sec,
        )
        if (
            self._last_playback_sec is not None
            and playback_sec <= self._last_playback_sec
            and playback_sec < self._runtime.terminal_timestamp_sec
        ):
            return None
        dispatch = self._dispatch_from_playback_sec(
            sample_index=self._dispatch_index,
            clock_source="wall",
            clock_sec=current_sec,
            playback_sec=playback_sec,
        )
        self._dispatch_index += 1
        self._last_playback_sec = playback_sec
        if dispatch.finished:
            self._finished = True
        return dispatch

    def step_with_fallback(
        self,
        *,
        physics_now_sec: float | None = None,
        wall_now_sec: float | None = None,
    ) -> ReplayIsaacDispatch | None:
        clock_source, clock_now_sec = _choose_clock_now(
            physics_now_sec,
            wall_now_sec,
        )
        dispatch = self.step(clock_now_sec)
        if dispatch is None:
            return None
        return replace(dispatch, clock_source=clock_source)

    def dry_run(self) -> ReplayIsaacDryRunResult:
        step_interval_sec = 1.0 / self._rate_hz
        clock_sec = 0.0
        dispatches: list[ReplayIsaacDispatch] = []
        guard = 0
        max_dispatches = max(
            2,
            int(
                math.ceil(
                    self._runtime.terminal_timestamp_sec
                    * self._rate_hz
                    / self._time_scale
                )
            )
            + 2,
        )
        while not self._finished and guard < max_dispatches:
            dispatch = self.step(clock_sec)
            if dispatch is not None:
                dispatches.append(dispatch)
            clock_sec += step_interval_sec
            guard += 1
        if not self._finished:
            raise RuntimeError("dry_run did not reach the terminal replay frame")
        return ReplayIsaacDryRunResult(
            bundle_hash=compute_replay_bundle_hash(self._runtime.bundle),
            dispatches=tuple(dispatches),
            rate_hz=self._rate_hz,
            time_scale=self._time_scale,
            terminal_mode=self._terminal_mode,
        )


class ReplayIsaacAdapter:
    """Thin backend adapter that converts dispatch specs into MotionCommand."""

    def __init__(self, backend):
        self._backend = backend

    @classmethod
    def from_node(cls, node, service_name: str) -> "ReplayIsaacAdapter":
        backend_type, _ = _motion_backend_required()
        return cls(backend_type(node, service_name))

    def send(self, dispatch: ReplayIsaacDispatch, done_callback=None):
        command = _motion_command_from_spec(dispatch.command_spec)
        return self._backend.send(command, done_callback=done_callback)


class ReplayIsaacRunner:
    """Live runner that couples playback timing to an Isaac backend."""

    def __init__(
        self,
        bundle_or_runtime: ReplayBundle | ReplayRuntime,
        *,
        service_name: str,
        rate_hz: float = REPLAY_ISAAC_DEFAULT_RATE_HZ,
        time_scale: float = REPLAY_ISAAC_DEFAULT_TIME_SCALE,
        external_timeout_sec: float = REPLAY_ISAAC_DEFAULT_EXTERNAL_TIMEOUT_SEC,
        terminal_mode: str = REPLAY_ISAAC_TERMINAL_ALIGN,
        live_pose_topic: str = REPLAY_ISAAC_DEFAULT_LIVE_POSE_TOPIC,
        catch_up_sec: float = REPLAY_ISAAC_DEFAULT_CATCH_UP_SEC,
        live_pose_stale_sec: float = REPLAY_ISAAC_DEFAULT_LIVE_POSE_STALE_SEC,
        terminal_stable_samples: int = REPLAY_ISAAC_DEFAULT_TERMINAL_STABLE_SAMPLES,
        terminal_timeout_sec: float = REPLAY_ISAAC_DEFAULT_TERMINAL_TIMEOUT_SEC,
        terminal_position_tolerance_m: float = REPLAY_ISAAC_DEFAULT_TERMINAL_POSITION_TOLERANCE_M,
        terminal_yaw_tolerance_rad: float = REPLAY_ISAAC_DEFAULT_TERMINAL_YAW_TOLERANCE_RAD,
        terminal_max_speed_mps: float = REPLAY_ISAAC_DEFAULT_TERMINAL_MAX_SPEED_MPS,
        node=None,
        adapter: ReplayIsaacAdapter | None = None,
        monotonic_fn: Callable[[], float] = time.monotonic,
    ):
        self.playback = ReplayIsaacPlayback(
            bundle_or_runtime,
            rate_hz=rate_hz,
            time_scale=time_scale,
            external_timeout_sec=external_timeout_sec,
            terminal_mode=terminal_mode,
        )
        if adapter is not None:
            self.adapter = adapter
        else:
            if node is None:
                raise ValueError("node is required when adapter is not provided")
            self.adapter = ReplayIsaacAdapter.from_node(node, service_name)
        self._service_name = str(service_name)
        self._timer = None
        self._node = node
        if not _is_finite(catch_up_sec) or float(catch_up_sec) < 0.0:
            raise ValueError("catch_up_sec must be a non-negative finite number")
        if not _is_finite(live_pose_stale_sec) or float(live_pose_stale_sec) <= 0.0:
            raise ValueError("live_pose_stale_sec must be a positive finite number")
        if not isinstance(terminal_stable_samples, int) or terminal_stable_samples <= 0:
            raise ValueError("terminal_stable_samples must be a positive integer")
        if not _is_finite(terminal_timeout_sec) or float(terminal_timeout_sec) <= 0.0:
            raise ValueError("terminal_timeout_sec must be a positive finite number")
        if not _is_finite(terminal_position_tolerance_m) or float(terminal_position_tolerance_m) <= 0.0:
            raise ValueError(
                "terminal_position_tolerance_m must be a positive finite number"
            )
        if not _is_finite(terminal_yaw_tolerance_rad) or float(terminal_yaw_tolerance_rad) <= 0.0:
            raise ValueError(
                "terminal_yaw_tolerance_rad must be a positive finite number"
            )
        if not _is_finite(terminal_max_speed_mps) or float(terminal_max_speed_mps) < 0.0:
            raise ValueError("terminal_max_speed_mps must be a non-negative finite number")
        self._live_pose_topic = str(live_pose_topic)
        self._catch_up_sec = float(catch_up_sec)
        self._live_pose_stale_sec = float(live_pose_stale_sec)
        self._terminal_stable_samples = int(terminal_stable_samples)
        self._terminal_timeout_sec = float(terminal_timeout_sec)
        self._terminal_position_tolerance_m = float(terminal_position_tolerance_m)
        self._terminal_yaw_tolerance_rad = float(terminal_yaw_tolerance_rad)
        self._terminal_max_speed_mps = float(terminal_max_speed_mps)
        self._monotonic = monotonic_fn
        self._live_progress_window_sec = max(2.0, self._catch_up_sec * 8.0)
        self._clock_origin_sec: float | None = None
        self._last_playback_sec: float | None = None
        self._last_live_progress_sec: float | None = None
        self._last_live_progress_advance_monotonic_sec: float | None = None
        self._dispatch_index = 0
        self._terminal_started_sec: float | None = None
        self._terminal_stable_count = 0
        self._terminal_result: ReplayIsaacTerminalResult | None = None
        self._terminal_target_snapshot: ReplayStateFrame | None = None
        self._live_pose: ReplayIsaacLivePose | None = None
        self._live_poses: list[ReplayIsaacLivePose] = []
        self._people_subscription = None
        if node is not None and People is not None:
            subscribe = getattr(node, "create_subscription", None)
            if callable(subscribe):
                self._people_subscription = subscribe(
                    People,
                    self._live_pose_topic,
                    self._people_cb,
                    20,
                )

    @property
    def finished(self) -> bool:
        return self._terminal_result is not None

    @property
    def terminal_result(self) -> ReplayIsaacTerminalResult | None:
        return self._terminal_result

    def tracking_metrics(self) -> ReplayIsaacTrackingMetrics:
        return _tracking_metrics_from_live_samples(
            self.playback.runtime,
            self._live_poses,
            self._terminal_result,
            time_scale=self.playback.time_scale,
        )

    def final_report(self) -> dict[str, Any]:
        terminal_result = self._terminal_result
        if terminal_result is None:
            raise RuntimeError("replay finished without a terminal result")
        return {
            "bundle_hash": compute_replay_bundle_hash(self.playback.runtime.bundle),
            "finished": terminal_result.result == "converged",
            "service": self._service_name,
            "terminal_mode": self.playback.terminal_mode,
            "terminal_result": terminal_result.to_dict(),
            "tracking_metrics": self.tracking_metrics().to_dict(),
        }

    def update_live_pose(
        self,
        pose: Mapping[str, Any],
        *,
        agent_id: str | None = None,
        received_monotonic_sec: float | None = None,
    ) -> None:
        self._live_pose = ReplayIsaacLivePose(
            agent_id=str(agent_id or ""),
            x=float(pose.get("x", 0.0)),
            y=float(pose.get("y", 0.0)),
            z=float(pose.get("z", 0.0)),
            yaw=(
                float(pose["yaw"])
                if _is_finite(pose.get("yaw"))
                else None
            ),
            vx=float(pose.get("vx", 0.0)),
            vy=float(pose.get("vy", 0.0)),
            source_stamp_sec=(
                float(pose["source_stamp_sec"])
                if pose.get("source_stamp_sec") is not None
                else None
            ),
            received_monotonic_sec=(
                float(received_monotonic_sec)
                if received_monotonic_sec is not None
                else float(self._monotonic())
            ),
            reference_playback_sec=(
                float(pose["reference_playback_sec"])
                if _is_finite(pose.get("reference_playback_sec"))
                else (
                    float(self._last_playback_sec)
                    if _is_finite(self._last_playback_sec)
                    else None
                )
            ),
        )
        self._live_poses.append(self._live_pose)

    def _people_cb(self, message) -> None:
        primary_agent = self.playback.runtime.agent.agent_id
        for person in getattr(message, "people", []) or []:
            name = str(getattr(person, "name", "") or "").strip()
            if name != primary_agent and not name.rstrip("/").endswith(
                "/" + primary_agent
            ):
                continue
            reliability = float(getattr(person, "reliability", 0.0))
            if reliability <= 0.0:
                continue
            position = getattr(person, "position", None)
            velocity = getattr(person, "velocity", None)
            header = getattr(message, "header", None)
            stamp = getattr(header, "stamp", None)
            source_stamp_sec = None
            if stamp is not None:
                source_stamp_sec = (
                    float(getattr(stamp, "sec", 0.0))
                    + float(getattr(stamp, "nanosec", 0.0)) * 1e-9
                )
            self.update_live_pose(
                {
                    "x": float(getattr(position, "x", 0.0)),
                    "y": float(getattr(position, "y", 0.0)),
                    "z": float(getattr(position, "z", 0.0)),
                    "yaw": person_yaw(person),
                    "vx": float(getattr(velocity, "x", 0.0)),
                    "vy": float(getattr(velocity, "y", 0.0)),
                    "source_stamp_sec": source_stamp_sec,
                },
                agent_id=name,
                received_monotonic_sec=float(self._monotonic()),
            )
            return

    def _live_pose_is_fresh(self, now_sec: float) -> bool:
        return (
            self._live_pose is not None
            and now_sec - self._live_pose.received_monotonic_sec
            <= self._live_pose_stale_sec
        )

    def _estimate_live_progress_sec(self) -> float | None:
        live_pose = self._live_pose
        if live_pose is None:
            return None
        reference_progress_sec = max(
            0.0,
            float(self._last_live_progress_sec or 0.0),
        )
        maximum_progress_sec = min(
            self.playback.runtime.terminal_timestamp_sec,
            reference_progress_sec + self._live_progress_window_sec,
        )
        if maximum_progress_sec < reference_progress_sec:
            return None
        trajectory = self.playback.runtime.agent.trajectory
        if len(trajectory) == 1:
            only = trajectory[0]
            distance = math.hypot(
                float(only.x) - float(live_pose.x),
                float(only.y) - float(live_pose.y),
            )
            if distance > max(self._terminal_position_tolerance_m * 2.0, 1.0):
                return None
            return reference_progress_sec
        best_distance_sq = math.inf
        best_progress = reference_progress_sec
        for start, end in zip(trajectory, trajectory[1:]):
            segment_start_progress = float(start.timestamp_sec)
            segment_end_progress = float(end.timestamp_sec)
            if segment_end_progress + 1e-6 < reference_progress_sec:
                continue
            if segment_start_progress > maximum_progress_sec + 1e-6:
                break
            ax = float(start.x)
            ay = float(start.y)
            bx = float(end.x)
            by = float(end.y)
            dx = bx - ax
            dy = by - ay
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                if not (reference_progress_sec <= segment_start_progress <= maximum_progress_sec):
                    continue
                progress = segment_start_progress
                projected_x = ax
                projected_y = ay
            else:
                projection = max(
                    0.0,
                    min(
                        1.0,
                        ((float(live_pose.x) - ax) * dx + (float(live_pose.y) - ay) * dy)
                        / length_sq,
                    ),
                )
                minimum_ratio = max(
                    0.0,
                    (
                        reference_progress_sec - segment_start_progress
                    )
                    / max(
                        segment_end_progress - segment_start_progress,
                        1e-9,
                    ),
                )
                maximum_ratio = min(
                    1.0,
                    (
                        maximum_progress_sec - segment_start_progress
                    )
                    / max(
                        segment_end_progress - segment_start_progress,
                        1e-9,
                    ),
                )
                if maximum_ratio + 1e-12 < minimum_ratio:
                    continue
                ratio = max(minimum_ratio, min(maximum_ratio, projection))
                progress = segment_start_progress + ratio * (
                    segment_end_progress - segment_start_progress
                )
                projected_x = ax + ratio * dx
                projected_y = ay + ratio * dy
            distance_sq = (
                (float(live_pose.x) - projected_x) ** 2
                + (float(live_pose.y) - projected_y) ** 2
            )
            if (
                distance_sq + 1e-12 < best_distance_sq
                or (
                    abs(distance_sq - best_distance_sq) <= 1e-12
                    and progress < best_progress
                )
            ):
                best_distance_sq = distance_sq
                best_progress = progress
        if best_distance_sq == math.inf:
            return None
        if best_distance_sq > max(self._terminal_position_tolerance_m * 2.0, 1.0) ** 2:
            return None
        return max(reference_progress_sec, best_progress)

    def _compute_playback_sec(self, clock_now_sec: float) -> float:
        if self._clock_origin_sec is None:
            self._clock_origin_sec = float(clock_now_sec)
        elapsed_sec = max(0.0, float(clock_now_sec) - self._clock_origin_sec)
        playback_sec = min(
            elapsed_sec * self.playback.time_scale,
            self.playback.runtime.terminal_timestamp_sec,
        )
        if self._live_pose_is_fresh(float(self._monotonic())):
            monotonic_now = float(self._monotonic())
            live_progress_sec = self._estimate_live_progress_sec()
            if live_progress_sec is not None:
                previous_live_progress = float(self._last_live_progress_sec or 0.0)
                self._last_live_progress_sec = max(
                    previous_live_progress,
                    float(live_progress_sec),
                )
                if (
                    self._last_live_progress_advance_monotonic_sec is None
                    or self._last_live_progress_sec
                    > previous_live_progress + 0.02
                ):
                    self._last_live_progress_advance_monotonic_sec = monotonic_now
                stalled_for_sec = max(
                    0.0,
                    monotonic_now
                    - float(
                        self._last_live_progress_advance_monotonic_sec
                        if self._last_live_progress_advance_monotonic_sec is not None
                        else monotonic_now
                    )
                    - REPLAY_ISAAC_STALL_GRACE_SEC,
                )
                stall_lead_sec = min(
                    REPLAY_ISAAC_MAX_STALL_LEAD_SEC,
                    stalled_for_sec,
                )
                playback_sec = min(
                    playback_sec,
                    self._last_live_progress_sec
                    + self._catch_up_sec
                    + stall_lead_sec,
                )
        return playback_sec

    def _record_terminal_start(self) -> None:
        if self._terminal_started_sec is None:
            self._terminal_started_sec = float(self._monotonic())
            self._terminal_stable_count = 0
            self._terminal_target_snapshot = ReplayStateFrame(
                sample_index=self._dispatch_index,
                timestamp_sec=self.playback.runtime.terminal_timestamp_sec,
                snapshot=self.playback.runtime.sample(
                    self.playback.runtime.terminal_timestamp_sec
                ),
                finished=True,
            )

    def _terminal_alignment_status(
        self, now_sec: float
    ) -> ReplayIsaacTerminalResult | None:
        started = self._terminal_started_sec
        if started is None:
            return None
        elapsed_sec = max(0.0, float(now_sec) - started)
        live_pose = self._live_pose
        if elapsed_sec >= self._terminal_timeout_sec:
            return ReplayIsaacTerminalResult(
                result="timeout",
                reason="terminal convergence timed out",
                stable_samples=self._terminal_stable_count,
                required_stable_samples=self._terminal_stable_samples,
                elapsed_sec=elapsed_sec,
                timeout_sec=self._terminal_timeout_sec,
                live_pose_age_sec=(
                    None
                    if live_pose is None
                    else max(0.0, float(now_sec) - live_pose.received_monotonic_sec)
                ),
                live_pose=live_pose,
            )
        if not self._live_pose_is_fresh(now_sec):
            self._terminal_stable_count = 0
            return None
        if live_pose is None:
            self._terminal_stable_count = 0
            return None
        if live_pose.yaw is None or not _is_finite(live_pose.yaw):
            self._terminal_stable_count = 0
            return None
        target_snapshot = self.playback.runtime.sample(
            self.playback.runtime.terminal_timestamp_sec
        )
        position_error_m = math.hypot(
            float(live_pose.x) - float(target_snapshot.x),
            float(live_pose.y) - float(target_snapshot.y),
        )
        yaw_error_rad = _angle_delta_rad(target_snapshot.yaw, live_pose.yaw)
        speed_mps = math.hypot(float(live_pose.vx), float(live_pose.vy))
        aligned = (
            position_error_m <= self._terminal_position_tolerance_m
            and abs(yaw_error_rad) <= self._terminal_yaw_tolerance_rad
            and speed_mps <= self._terminal_max_speed_mps
        )
        if aligned:
            self._terminal_stable_count += 1
        else:
            self._terminal_stable_count = 0
        if self._terminal_stable_count < self._terminal_stable_samples:
            return None
        return ReplayIsaacTerminalResult(
            result="converged",
            reason="terminal pose/yaw converged",
            stable_samples=self._terminal_stable_count,
            required_stable_samples=self._terminal_stable_samples,
            elapsed_sec=elapsed_sec,
            timeout_sec=self._terminal_timeout_sec,
            position_error_m=position_error_m,
            yaw_error_rad=yaw_error_rad,
            live_pose_age_sec=max(0.0, float(now_sec) - live_pose.received_monotonic_sec),
            live_pose=live_pose,
        )

    def tick(self, *, physics_now_sec: float | None = None) -> ReplayIsaacDispatch | None:
        if self._terminal_result is not None:
            return None
        clock_source, clock_now_sec = _choose_clock_now(
            physics_now_sec,
            self._monotonic(),
        )
        if self._terminal_started_sec is not None:
            result = self._terminal_alignment_status(float(self._monotonic()))
            if result is not None:
                self._terminal_result = result
                return None
            if self._terminal_target_snapshot is None:
                self._record_terminal_start()
            dispatch = self.playback._dispatch_from_playback_sec(
                sample_index=self._dispatch_index,
                clock_source=clock_source,
                clock_sec=clock_now_sec,
                playback_sec=self.playback.runtime.terminal_timestamp_sec,
            )
            self._dispatch_index += 1
            self.adapter.send(dispatch)
            return dispatch
        playback_sec = self._compute_playback_sec(clock_now_sec)
        if (
            self._last_playback_sec is not None
            and playback_sec <= self._last_playback_sec
            and playback_sec < self.playback.runtime.terminal_timestamp_sec
        ):
            return None
        dispatch = self.playback._dispatch_from_playback_sec(
            sample_index=self._dispatch_index,
            clock_source=clock_source,
            clock_sec=clock_now_sec,
            playback_sec=playback_sec,
        )
        self._dispatch_index += 1
        self._last_playback_sec = playback_sec
        if dispatch.finished:
            self._record_terminal_start()
        self.adapter.send(dispatch)
        if self._terminal_target_snapshot is not None:
            result = self._terminal_alignment_status(float(self._monotonic()))
            if result is not None:
                self._terminal_result = result
        return dispatch

    def start_timer(self) -> None:
        if self._node is None:
            raise ValueError("node is required for live timer execution")

        period = 1.0 / self.playback.rate_hz

        def _on_timer() -> None:
            if self.finished:
                if self._timer is not None:
                    self._timer.cancel()
                return
            clock_now = None
            clock = getattr(self._node, "get_clock", None)
            if callable(clock):
                try:
                    clock_now = float(clock().now().nanoseconds) / 1e9
                except Exception:
                    clock_now = None
            dispatch = self.tick(physics_now_sec=clock_now)
            if dispatch is None:
                if self.finished and self._timer is not None:
                    self._timer.cancel()
                return
            if self.finished and self._timer is not None:
                self._timer.cancel()

        self._timer = self._node.create_timer(period, _on_timer)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_replay_isaac", description=__doc__)
    parser.add_argument("bundle", help="replay bundle JSON or YAML path")
    parser.add_argument(
        "--service",
        default="/isaac/move_pedestrians",
        help="Isaac MovePed service name",
    )
    parser.add_argument(
        "--spawn-character",
        help="Optionally spawn the bundle agent at its first pose before replay.",
    )
    parser.add_argument(
        "--spawn-service",
        default="/isaac/spawn_pedestrian",
        help="Isaac pedestrian spawn service used with --spawn-character.",
    )
    parser.add_argument(
        "--spawn-timeout-sec",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_SPAWN_TIMEOUT_SEC,
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_RATE_HZ,
        help="timer rate in Hz for live playback or dry-run scheduling",
    )
    parser.add_argument(
        "--time-scale",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_TIME_SCALE,
        help="playback time scale relative to the active timer clock",
    )
    parser.add_argument(
        "--external-timeout-sec",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_EXTERNAL_TIMEOUT_SEC,
        help="timeout to write into MovePed external motion commands",
    )
    parser.add_argument(
        "--terminal-mode",
        choices=REPLAY_ISAAC_TERMINAL_MODES,
        default=REPLAY_ISAAC_TERMINAL_ALIGN,
        help="terminal external-motion mode",
    )
    parser.add_argument(
        "--live-pose-topic",
        default=REPLAY_ISAAC_DEFAULT_LIVE_POSE_TOPIC,
        help="fresh live pedestrian state topic used for replay catch-up",
    )
    parser.add_argument(
        "--catch-up-sec",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_CATCH_UP_SEC,
        help="max replay lead over the latest fresh live pedestrian pose",
    )
    parser.add_argument(
        "--live-pose-stale-sec",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_LIVE_POSE_STALE_SEC,
        help="age threshold for considering live pedestrian pose fresh",
    )
    parser.add_argument(
        "--terminal-stable-samples",
        type=int,
        default=REPLAY_ISAAC_DEFAULT_TERMINAL_STABLE_SAMPLES,
        help="fresh live samples required before replay completion",
    )
    parser.add_argument(
        "--terminal-timeout-sec",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_TERMINAL_TIMEOUT_SEC,
        help="timeout for terminal pose/yaw convergence",
    )
    parser.add_argument(
        "--terminal-position-tolerance-m",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_TERMINAL_POSITION_TOLERANCE_M,
        help="terminal pose tolerance in meters",
    )
    parser.add_argument(
        "--terminal-yaw-tolerance-rad",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_TERMINAL_YAW_TOLERANCE_RAD,
        help="terminal yaw tolerance in radians",
    )
    parser.add_argument(
        "--terminal-max-speed-mps",
        type=float,
        default=REPLAY_ISAAC_DEFAULT_TERMINAL_MAX_SPEED_MPS,
        help="terminal speed ceiling used during convergence handshake",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="emit deterministic JSON command plans without calling Isaac",
    )
    return parser


def _dry_run_json(bundle: ReplayBundle, namespace: argparse.Namespace) -> str:
    playback = ReplayIsaacPlayback(
        bundle,
        rate_hz=namespace.rate,
        time_scale=namespace.time_scale,
        external_timeout_sec=namespace.external_timeout_sec,
        terminal_mode=namespace.terminal_mode,
    )
    result = playback.dry_run()
    return json.dumps(result.to_dict(), sort_keys=True, ensure_ascii=False)


def _spawn_bundle_agent(node, bundle: ReplayBundle, namespace: argparse.Namespace) -> None:
    try:
        from isaacsim_msgs.msg import Person
        from isaacsim_msgs.srv import Pedestrian
        import rclpy
    except ImportError as exc:  # pragma: no cover - live ROS integration
        raise RuntimeError("isaacsim_msgs is required to spawn a replay actor") from exc

    agent = bundle.primary_agent()
    first = agent.trajectory[0]
    client = node.create_client(Pedestrian, namespace.spawn_service)
    if not client.wait_for_service(timeout_sec=float(namespace.spawn_timeout_sec)):
        raise RuntimeError(
            f"spawn service {namespace.spawn_service} was not ready within "
            f"{namespace.spawn_timeout_sec:.1f}s"
        )
    request = Pedestrian.Request()
    person = Person()
    person.stage_prefix = agent.agent_id
    person.character_name = str(namespace.spawn_character)
    spawn_pose = [float(first.x), float(first.y), float(first.z)]
    person.initial_pose = spawn_pose
    # Reading a bounded float sequence back from rosidl yields numpy.float32
    # values, which the setter for another sequence field rejects.
    person.goal_pose = spawn_pose
    person.path_points_flat = []
    person.loop_path = False
    person.orientation = float(first.yaw)
    person.controller_stats = False
    person.velocity = 0.0
    request.people.append(person)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(
        node,
        future,
        timeout_sec=float(namespace.spawn_timeout_sec),
    )
    if not future.done():
        raise RuntimeError("replay actor spawn request timed out")
    response = future.result()
    if response is None or not bool(response.ret):
        raise RuntimeError("Isaac rejected replay actor spawn request")


def main(args: Sequence[str] | None = None) -> int:
    namespace = _build_parser().parse_args(args)
    try:
        bundle = load_replay_bundle(namespace.bundle)
        require_valid_replay_bundle(bundle)
        if namespace.dry_run:
            print(_dry_run_json(bundle, namespace))
            return 0
        _, _ = _motion_backend_required()
        try:
            import rclpy  # pragma: no cover - live ROS integration
        except Exception as exc:  # pragma: no cover - live ROS integration
            raise RuntimeError(
                "rclpy is required for live replay execution; use --dry-run for "
                "core-only validation"
            ) from exc

        rclpy.init(args=None)
        node = rclpy.create_node("toilet_replay_isaac")
        try:
            if namespace.spawn_character:
                _spawn_bundle_agent(node, bundle, namespace)
            runner = ReplayIsaacRunner(
                bundle,
                service_name=namespace.service,
                rate_hz=namespace.rate,
                time_scale=namespace.time_scale,
                external_timeout_sec=namespace.external_timeout_sec,
                terminal_mode=namespace.terminal_mode,
                live_pose_topic=namespace.live_pose_topic,
                catch_up_sec=namespace.catch_up_sec,
                live_pose_stale_sec=namespace.live_pose_stale_sec,
                terminal_stable_samples=namespace.terminal_stable_samples,
                terminal_timeout_sec=namespace.terminal_timeout_sec,
                terminal_position_tolerance_m=namespace.terminal_position_tolerance_m,
                terminal_yaw_tolerance_rad=namespace.terminal_yaw_tolerance_rad,
                terminal_max_speed_mps=namespace.terminal_max_speed_mps,
                node=node,
            )
            runner.start_timer()
            while rclpy.ok() and not runner.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
            terminal_result = runner.terminal_result
            if terminal_result is None:
                raise RuntimeError("replay finished without a terminal result")
            print(json.dumps(runner.final_report(), sort_keys=True, ensure_ascii=False))
            return 0 if terminal_result.result == "converged" else 2
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    except (OSError, TypeError, ValueError, KeyError, RuntimeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
