from __future__ import annotations

import json
import os
import signal
import subprocess
import tempfile
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence

import yaml

from .collection_scenarios import ManualCollectionConfig, ScenarioSelection, dump_manual_collection_config


def _utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _yaml_safe_dump_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.safe_dump(dict(payload), handle, sort_keys=False, allow_unicode=True)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, path)
    except Exception:
        try:
            tmp_path.unlink()
        except FileNotFoundError:
            pass
        raise


def _jsonl_append(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True))
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())


class _SubprocessModule(Protocol):
    TimeoutExpired: type[BaseException]

    def Popen(self, args: Sequence[str], **kwargs: Any) -> Any: ...


@dataclass(frozen=True)
class SessionManifest:
    session_id: str
    output_root: str
    operator_id: str
    seed: int
    selection_mode: str
    scenario_count: int
    enabled_scenario_count: int
    status: str
    started_at: str
    ended_at: str | None = None
    config: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class EpisodeManifest:
    session_id: str
    episode_id: str
    episode_index: int
    status: str
    started_at: str
    ended_at: str | None
    operator_id: str
    seed: int
    selection_mode: str
    scenario: Mapping[str, Any]
    event_count: int
    rosbag_started: bool
    rosbag_command: Sequence[str] | None = None
    termination_reason: str | None = None
    extra: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class EpisodeContext:
    session_id: str
    episode_id: str
    episode_index: int
    episode_dir: Path
    selection: ScenarioSelection


class EpisodeRecorder:
    def __init__(
        self,
        config: ManualCollectionConfig,
        *,
        session_id: str,
        subprocess_module: _SubprocessModule = subprocess,
        rosbag_command_factory: Callable[[EpisodeContext], Sequence[str] | None] | None = None,
        signal_module: Any = signal,
    ) -> None:
        self._config = config
        self._session_id = session_id
        self._subprocess_module = subprocess_module
        self._rosbag_command_factory = rosbag_command_factory
        self._signal_module = signal_module
        self._session_dir = config.session.output_root / session_id
        self._session_manifest_path = self._session_dir / "session_manifest.yaml"
        self._current_episode: EpisodeContext | None = None
        self._episode_manifest: EpisodeManifest | None = None
        self._episode_manifest_path: Path | None = None
        self._events_path: Path | None = None
        self._events_count = 0
        self._bag_process: Any | None = None
        self._bag_command: Sequence[str] | None = None
        self._previous_sigint_handler: Any | None = None
        self._signal_installed = False
        self._interrupted = False
        self._session_started_at: str | None = None

    @classmethod
    def from_config(
        cls,
        config: ManualCollectionConfig,
        *,
        session_id: str,
        subprocess_module: _SubprocessModule = subprocess,
        rosbag_command_factory: Callable[[EpisodeContext], Sequence[str] | None] | None = None,
        signal_module: Any = signal,
    ) -> "EpisodeRecorder":
        return cls(
            config,
            session_id=session_id,
            subprocess_module=subprocess_module,
            rosbag_command_factory=rosbag_command_factory,
            signal_module=signal_module,
        )

    @property
    def session_dir(self) -> Path:
        return self._session_dir

    @property
    def interrupted(self) -> bool:
        return self._interrupted

    def prepare_session(self) -> Path:
        self._session_dir.mkdir(parents=True, exist_ok=True)
        if self._session_started_at is None:
            self._session_started_at = _utc_now_iso()
        _yaml_safe_dump_atomic(self._session_manifest_path, self._build_session_manifest(status="running"))
        return self._session_dir

    @property
    def has_active_episode(self) -> bool:
        return self._current_episode is not None

    @property
    def bag_process_running(self) -> bool:
        return self._bag_process is not None and self._bag_process.poll() is None

    def install_sigint_handler(self) -> None:
        if self._signal_installed:
            return
        self._previous_sigint_handler = self._signal_module.signal(self._signal_module.SIGINT, self.handle_sigint)
        self._signal_installed = True

    def restore_sigint_handler(self) -> None:
        if not self._signal_installed:
            return
        self._signal_module.signal(self._signal_module.SIGINT, self._previous_sigint_handler)
        self._previous_sigint_handler = None
        self._signal_installed = False

    def start_episode(self, selection: ScenarioSelection) -> EpisodeContext:
        if self._current_episode is not None:
            raise RuntimeError("An episode is already active")
        self.prepare_session()
        episode_index = self._next_episode_index()
        episode_id = f"episode_{episode_index:06d}"
        episode_dir = self._session_dir / episode_id
        episode_dir.mkdir(parents=True, exist_ok=False)
        context = EpisodeContext(
            session_id=self._session_id,
            episode_id=episode_id,
            episode_index=episode_index,
            episode_dir=episode_dir,
            selection=selection,
        )
        self._current_episode = context
        self._episode_manifest_path = episode_dir / "metadata.yaml"
        self._events_path = episode_dir / "events.jsonl"
        self._events_count = 0
        self._bag_process = None
        self._bag_command = None
        if self._rosbag_command_factory is not None:
            command = self._rosbag_command_factory(context)
            if command is not None:
                self._bag_command = tuple(command)
                self._bag_process = self._subprocess_module.Popen(
                    list(self._bag_command),
                    cwd=str(episode_dir),
                    start_new_session=True,
                )
        self._episode_manifest = self._build_episode_manifest(
            context=context,
            status="running",
            ended_at=None,
            termination_reason=None,
            extra={},
        )
        _yaml_safe_dump_atomic(self._episode_manifest_path, asdict(self._episode_manifest))
        return context

    def record_event(self, event_type: str, payload: Mapping[str, Any] | None = None, *, timestamp: str | None = None) -> None:
        if self._current_episode is None or self._events_path is None:
            raise RuntimeError("No active episode")
        event = {
            "episode_id": self._current_episode.episode_id,
            "episode_index": self._current_episode.episode_index,
            "event_type": event_type,
            "payload": dict(payload or {}),
            "session_id": self._session_id,
            "timestamp": timestamp or _utc_now_iso(),
        }
        _jsonl_append(self._events_path, event)
        self._events_count += 1

    def finalize_episode(
        self,
        *,
        status: str,
        termination_reason: str | None = None,
        extra: Mapping[str, Any] | None = None,
    ) -> Path:
        if self._current_episode is None or self._episode_manifest is None or self._episode_manifest_path is None:
            raise RuntimeError("No active episode")
        self._stop_bag_process()
        manifest = self._build_episode_manifest(
            context=self._current_episode,
            status=status,
            ended_at=_utc_now_iso(),
            termination_reason=termination_reason,
            extra=dict(extra or {}),
        )
        _yaml_safe_dump_atomic(self._episode_manifest_path, asdict(manifest))
        episode_path = self._episode_manifest_path
        self._current_episode = None
        self._episode_manifest = None
        self._episode_manifest_path = None
        self._events_path = None
        self._events_count = 0
        self._bag_process = None
        self._bag_command = None
        return episode_path

    def finalize_session(self, *, status: str = "completed") -> Path:
        self._stop_bag_process()
        _yaml_safe_dump_atomic(
            self._session_manifest_path,
            self._build_session_manifest(status=status, ended_at=_utc_now_iso()),
        )
        return self._session_manifest_path

    def handle_sigint(self, signum: int, frame: Any) -> None:  # noqa: ARG002 - handler signature is intentional
        self._interrupted = True
        try:
            if self._current_episode is not None:
                try:
                    self.finalize_episode(status="aborted_by_signal", termination_reason="sigint")
                except Exception:
                    pass
            try:
                self.finalize_session(status="interrupted")
            except Exception:
                pass
        finally:
            self.restore_sigint_handler()

    def close(self) -> None:
        try:
            if self._current_episode is not None:
                self.finalize_episode(status="aborted", termination_reason="close")
        finally:
            if self._session_dir.exists():
                try:
                    self.finalize_session(status="closed")
                except Exception:
                    pass
            self.restore_sigint_handler()

    def _build_session_manifest(self, *, status: str, ended_at: str | None = None) -> Mapping[str, Any]:
        return asdict(
            SessionManifest(
                session_id=self._session_id,
                output_root=str(self._config.session.output_root),
                operator_id=self._config.session.operator_id,
                seed=self._config.session.seed,
                selection_mode=self._config.session.selection_mode,
                scenario_count=len(self._config.scenarios),
                enabled_scenario_count=len(self._config.enabled_scenarios()),
                status=status,
                started_at=self._session_started_at or _utc_now_iso(),
                ended_at=ended_at,
                config=dump_manual_collection_config(self._config),
            )
        )

    def _build_episode_manifest(
        self,
        *,
        context: EpisodeContext,
        status: str,
        ended_at: str | None,
        termination_reason: str | None,
        extra: Mapping[str, Any],
    ) -> EpisodeManifest:
        scenario = context.selection.scenario
        return EpisodeManifest(
            session_id=context.session_id,
            episode_id=context.episode_id,
            episode_index=context.episode_index,
            status=status,
            started_at=self._utc_for_episode(context),
            ended_at=ended_at,
            operator_id=self._config.session.operator_id,
            seed=self._config.session.seed,
            selection_mode=self._config.session.selection_mode,
            scenario={
                "id": scenario.id,
                "enabled": scenario.enabled,
                "weight": scenario.weight,
                "robot_start": list(scenario.robot_start),
                "robot_goal": list(scenario.robot_goal),
                "pedestrian_target_urinal_id": scenario.pedestrian_target_urinal_id,
                "selection_index": context.selection.selection_index,
                "enabled_index": context.selection.enabled_index,
                "selection_seed": context.selection.seed,
                "selection_mode": context.selection.selector_mode,
            },
            event_count=self._events_count,
            rosbag_started=self._bag_process is not None,
            rosbag_command=list(self._bag_command) if self._bag_command is not None else None,
            termination_reason=termination_reason,
            extra=extra,
        )

    def _utc_for_episode(self, context: EpisodeContext) -> str:
        if self._episode_manifest is not None:
            return self._episode_manifest.started_at
        return _utc_now_iso()

    def _next_episode_index(self) -> int:
        existing_indices = []
        if self._session_dir.exists():
            for path in self._session_dir.glob("episode_*"):
                if path.is_dir():
                    suffix = path.name.removeprefix("episode_")
                    if suffix.isdigit():
                        existing_indices.append(int(suffix))
        return (max(existing_indices) + 1) if existing_indices else 1

    def _stop_bag_process(self) -> None:
        process = self._bag_process
        if process is None:
            return
        timeout_expired = getattr(self._subprocess_module, "TimeoutExpired", subprocess.TimeoutExpired)
        try:
            if hasattr(process, "poll") and process.poll() is not None:
                return
        except Exception:
            pass
        try:
            if getattr(process, "pid", None) is not None:
                os.killpg(int(process.pid), signal.SIGINT)
            else:
                process.terminate()
        except Exception:
            try:
                process.terminate()
            except Exception:
                pass
        try:
            process.wait(timeout=5.0)
        except timeout_expired:
            try:
                process.kill()
            except Exception:
                pass
            try:
                process.wait(timeout=5.0)
            except Exception:
                pass
        except Exception:
            try:
                process.kill()
            except Exception:
                pass
