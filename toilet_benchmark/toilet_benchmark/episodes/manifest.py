"""Episode manifest serialization and deterministic content hashing."""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

from .schema import BENCHMARK_SCHEMA_VERSION, EpisodeSpec


EPISODE_MANIFEST_SCHEMA_VERSION = "toilet-benchmark-manifest-0.1"
_JSON_SEPARATORS = (",", ":")


def _canonical_json(payload: Mapping[str, Any]) -> str:
    return json.dumps(payload, sort_keys=True, ensure_ascii=False, separators=_JSON_SEPARATORS)


def compute_episode_hash(episode: EpisodeSpec) -> str:
    payload = episode.to_dict()
    payload["schema_version"] = payload.get("schema_version", BENCHMARK_SCHEMA_VERSION)
    return hashlib.sha256(_canonical_json(payload).encode("utf-8")).hexdigest()


def dump_episode_spec(episode: EpisodeSpec, path: str | Path) -> None:
    """Atomically serialize one canonical episode payload."""

    _write_json_atomic(Path(path), episode.to_dict())


def load_episode_spec(path: str | Path) -> EpisodeSpec:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise TypeError("episode payload must be a mapping")
    return EpisodeSpec.from_mapping(payload)


def _write_json_atomic(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    tmp_path = Path(tmp_name)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            handle.write(_canonical_json(payload))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp_path, path)
    except Exception:
        try:
            tmp_path.unlink()
        except FileNotFoundError:
            pass
        raise


@dataclass(frozen=True)
class ManifestEpisode:
    episode_id: str
    split: str
    episode_hash: str
    episode_path: str | None = None


@dataclass(frozen=True)
class EpisodeManifest:
    schema_version: str
    benchmark_version: str
    split_seed: int | None
    entries: tuple[ManifestEpisode, ...]
    content_hash: str = field(default="")

    def to_dict(self, *, include_hash: bool = True) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": self.schema_version,
            "benchmark_version": self.benchmark_version,
            "split_seed": self.split_seed,
            "episodes": [
                {
                    "episode_id": entry.episode_id,
                    "split": entry.split,
                    "episode_hash": entry.episode_hash,
                    **(
                        {"episode_path": entry.episode_path}
                        if entry.episode_path is not None
                        else {}
                    ),
                }
                for entry in self.entries
            ],
        }
        if include_hash:
            payload["content_hash"] = self.content_hash
        return payload

    def canonical_payload(self) -> str:
        payload = self.to_dict(include_hash=False)
        return _canonical_json(payload)

    def canonical_hash(self) -> str:
        return hashlib.sha256(self.canonical_payload().encode("utf-8")).hexdigest()

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "EpisodeManifest":
        payload = dict(value)
        return cls(
            schema_version=str(payload["schema_version"]),
            benchmark_version=str(payload.get("benchmark_version", "0.1.0")),
            split_seed=payload.get("split_seed"),
            entries=tuple(
                ManifestEpisode(
                    episode_id=str(item["episode_id"]),
                    split=str(item["split"]),
                    episode_hash=str(item["episode_hash"]),
                    episode_path=(
                        str(item["episode_path"])
                        if item.get("episode_path") is not None
                        else None
                    ),
                )
                for item in payload.get("episodes", ())
            ),
            content_hash=str(payload.get("content_hash", "")),
        )


def build_episode_manifest(
    assignments: Mapping[str, Sequence[str]],
    *,
    schema_version: str = EPISODE_MANIFEST_SCHEMA_VERSION,
    benchmark_version: str = "0.1.0",
    split_seed: int | None = None,
) -> EpisodeManifest:
    entries = []
    seen_episode_ids: set[str] = set()
    for split_name in sorted(assignments):
        for episode_id in assignments[split_name]:
            normalized_id = str(episode_id)
            if normalized_id in seen_episode_ids:
                raise ValueError(
                    f"episode_id assigned to more than one split: {normalized_id}"
                )
            seen_episode_ids.add(normalized_id)
            entries.append(
                ManifestEpisode(
                    episode_id=normalized_id,
                    split=split_name,
                    episode_hash="",
                )
            )
    return EpisodeManifest(
        schema_version=schema_version,
        benchmark_version=benchmark_version,
        split_seed=split_seed,
        entries=tuple(entries),
    )


def fill_episode_hashes(
    manifest: EpisodeManifest,
    episode_lookup: Mapping[str, EpisodeSpec],
    *,
    episode_paths: Mapping[str, str] | None = None,
    content_hash: bool = True,
) -> EpisodeManifest:
    entries = []
    for entry in manifest.entries:
        episode = episode_lookup[entry.episode_id]
        entries.append(
            ManifestEpisode(
                episode_id=entry.episode_id,
                split=entry.split,
                episode_hash=compute_episode_hash(episode),
                episode_path=(
                    episode_paths[entry.episode_id]
                    if episode_paths is not None
                    else entry.episode_path
                ),
            )
        )
    with_hash = EpisodeManifest(
        schema_version=manifest.schema_version,
        benchmark_version=manifest.benchmark_version,
        split_seed=manifest.split_seed,
        entries=tuple(entries),
    )
    manifest_hash = with_hash.canonical_hash() if content_hash else ""
    return EpisodeManifest(
        schema_version=with_hash.schema_version,
        benchmark_version=with_hash.benchmark_version,
        split_seed=with_hash.split_seed,
        entries=with_hash.entries,
        content_hash=manifest_hash,
    )


def dump_episode_manifest(
    manifest: EpisodeManifest,
    path: str | Path,
    *,
    ensure_hash: bool = True,
) -> None:
    payload_manifest = manifest
    if ensure_hash and not manifest.content_hash:
        payload_manifest = EpisodeManifest(
            schema_version=manifest.schema_version,
            benchmark_version=manifest.benchmark_version,
            split_seed=manifest.split_seed,
            entries=manifest.entries,
            content_hash=manifest.canonical_hash(),
        )
    _write_json_atomic(Path(path), payload_manifest.to_dict())


def load_episode_manifest(path: str | Path) -> EpisodeManifest:
    raw = Path(path).read_text(encoding="utf-8")
    payload = json.loads(raw)
    manifest = EpisodeManifest.from_mapping(payload)
    if manifest.content_hash:
        expected = manifest.canonical_hash()
        if manifest.content_hash != expected:
            raise ValueError("manifest content hash mismatch")
    return manifest
