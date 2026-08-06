"""Best-effort structured diagnostics for authored scenario runs."""

from __future__ import annotations

from datetime import datetime
import json
import os
from pathlib import Path
import time
from typing import Any


DEFAULT_LOG_DIR = Path("/home/stardust/resources/arena_ws/log/toilet_benchmark")


def automatic_log_path(episode_path: str | Path) -> Path:
    directory = Path(
        os.environ.get("TOILET_BENCHMARK_LOG_DIR", str(DEFAULT_LOG_DIR))
    ).expanduser()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return directory / f"{Path(episode_path).stem}_{timestamp}.jsonl"


class JsonlDiagnosticLog:
    """Write flushed JSONL events without allowing diagnostics to stop a run."""

    def __init__(self, path: str | Path, *, update_latest: bool = True) -> None:
        self.path = Path(path).expanduser().resolve()
        self._handle = None
        self.error = ""
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = self.path.open("a", encoding="utf-8", buffering=1)
            if update_latest:
                latest = self.path.parent / "latest.jsonl"
                temporary = self.path.parent / ".latest.jsonl.tmp"
                temporary.unlink(missing_ok=True)
                temporary.symlink_to(self.path.name)
                temporary.replace(latest)
        except OSError as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.close()

    @property
    def enabled(self) -> bool:
        return self._handle is not None

    def emit(self, event: str, **fields: Any) -> None:
        if self._handle is None:
            return
        payload = {
            "event": str(event),
            "monotonic_sec": round(time.monotonic(), 6),
            "wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            **fields,
        }
        try:
            self._handle.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
            self._handle.flush()
        except OSError as exc:
            self.error = f"{type(exc).__name__}: {exc}"
            self.close()

    def close(self) -> None:
        handle, self._handle = self._handle, None
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass
