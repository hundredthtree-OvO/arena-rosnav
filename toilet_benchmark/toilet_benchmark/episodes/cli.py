"""Generate and validate portable toilet benchmark episode manifests."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from toilet_benchmark.collection_scenarios import load_manual_collection_config

from .manifest import (
    EPISODE_MANIFEST_SCHEMA_VERSION,
    EpisodeManifest,
    build_episode_manifest,
    compute_episode_hash,
    dump_episode_manifest,
    dump_episode_spec,
    fill_episode_hashes,
    load_episode_spec,
)
from .schema import episodes_from_manual_config
from .splits import assign_episodes_to_splits
from .validator import require_valid_episode, validate_episode


DEFAULT_SPLIT_RATIOS = {"train": 0.8, "validation": 0.1, "test": 0.1}


@dataclass(frozen=True)
class ManifestValidationIssue:
    code: str
    path: str
    message: str


def parse_split_ratios(value: str) -> dict[str, float]:
    ratios: dict[str, float] = {}
    for item in value.split(","):
        name, separator, raw_ratio = item.strip().partition("=")
        if not separator or not name or not raw_ratio:
            raise argparse.ArgumentTypeError(
                "split ratios must use NAME=RATIO comma-separated syntax"
            )
        if name in ratios:
            raise argparse.ArgumentTypeError(f"duplicate split ratio: {name}")
        try:
            ratios[name] = float(raw_ratio)
        except ValueError as exc:
            raise argparse.ArgumentTypeError(
                f"invalid split ratio for {name}: {raw_ratio}"
            ) from exc
    return ratios


def generate_manifest_bundle(
    *,
    config_path: str | Path,
    scene_id: str,
    seed: int | None,
    split_ratios: Mapping[str, float],
    output_path: str | Path,
) -> tuple[EpisodeManifest, Path]:
    config = load_manual_collection_config(config_path)
    effective_seed = config.session.seed if seed is None else int(seed)
    episodes = episodes_from_manual_config(
        config,
        scene_id=scene_id,
        seed=effective_seed,
    )
    known_goals = {
        target
        for scenario in config.enabled_scenarios()
        for target in scenario.pedestrian_target_urinal_ids
    }
    for episode in episodes:
        require_valid_episode(episode, semantic_goal_ids=known_goals)

    assignment = assign_episodes_to_splits(
        [episode.episode_id for episode in episodes],
        split_ratios,
        seed=effective_seed,
    )
    episode_lookup = {episode.episode_id: episode for episode in episodes}
    output = Path(output_path)
    payload_dir = output.parent / f"{output.stem}.episodes"
    relative_paths = {
        episode.episode_id: (Path(payload_dir.name) / f"{episode.episode_id}.json").as_posix()
        for episode in episodes
    }
    manifest = build_episode_manifest(
        assignment.split_to_episodes,
        split_seed=effective_seed,
    )
    manifest = fill_episode_hashes(
        manifest,
        episode_lookup,
        episode_paths=relative_paths,
    )

    for episode in episodes:
        dump_episode_spec(episode, output.parent / relative_paths[episode.episode_id])
    dump_episode_manifest(manifest, output)
    return manifest, payload_dir


def _resolve_episode_path(manifest_path: Path, episode_id: str, stored_path: str | None) -> Path:
    if stored_path is None:
        return manifest_path.parent / f"{manifest_path.stem}.episodes" / f"{episode_id}.json"
    path = Path(stored_path)
    return path if path.is_absolute() else manifest_path.parent / path


def validate_manifest_bundle(path: str | Path) -> tuple[ManifestValidationIssue, ...]:
    manifest_path = Path(path)
    issues: list[ManifestValidationIssue] = []
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise TypeError("manifest payload must be a mapping")
        manifest = EpisodeManifest.from_mapping(payload)
    except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
        return (
            ManifestValidationIssue("manifest_parse_error", str(manifest_path), str(exc)),
        )

    if manifest.schema_version != EPISODE_MANIFEST_SCHEMA_VERSION:
        issues.append(
            ManifestValidationIssue(
                "manifest_schema_version_mismatch",
                "schema_version",
                f"expected {EPISODE_MANIFEST_SCHEMA_VERSION}, got {manifest.schema_version}",
            )
        )
    if not manifest.content_hash:
        issues.append(
            ManifestValidationIssue(
                "manifest_content_hash_missing",
                "content_hash",
                "manifest content_hash is required",
            )
        )
    elif manifest.content_hash != manifest.canonical_hash():
        issues.append(
            ManifestValidationIssue(
                "manifest_content_hash_mismatch",
                "content_hash",
                "manifest content hash does not match its canonical payload",
            )
        )

    seen: dict[str, str] = {}
    for index, entry in enumerate(manifest.entries):
        entry_path = f"episodes.{index}"
        previous_split = seen.get(entry.episode_id)
        if previous_split is not None:
            issues.append(
                ManifestValidationIssue(
                    "episode_split_overlap",
                    f"{entry_path}.episode_id",
                    (
                        f"episode {entry.episode_id} appears in both "
                        f"{previous_split} and {entry.split}"
                    ),
                )
            )
            continue
        seen[entry.episode_id] = entry.split
        if not entry.split:
            issues.append(
                ManifestValidationIssue(
                    "split_name_invalid",
                    f"{entry_path}.split",
                    "split name must be non-empty",
                )
            )

        episode_path = _resolve_episode_path(
            manifest_path,
            entry.episode_id,
            entry.episode_path,
        )
        try:
            episode = load_episode_spec(episode_path)
        except (OSError, TypeError, ValueError, KeyError, json.JSONDecodeError) as exc:
            issues.append(
                ManifestValidationIssue(
                    "episode_payload_error",
                    str(episode_path),
                    str(exc),
                )
            )
            continue

        if episode.episode_id != entry.episode_id:
            issues.append(
                ManifestValidationIssue(
                    "episode_id_mismatch",
                    f"{entry_path}.episode_id",
                    f"manifest id {entry.episode_id} does not match payload id {episode.episode_id}",
                )
            )
        for error in validate_episode(episode):
            issues.append(
                ManifestValidationIssue(
                    f"episode_{error.code}",
                    f"{entry.episode_id}.{'.'.join(error.path)}",
                    error.message,
                )
            )
        actual_hash = compute_episode_hash(episode)
        if not entry.episode_hash:
            issues.append(
                ManifestValidationIssue(
                    "episode_hash_missing",
                    f"{entry_path}.episode_hash",
                    f"episode hash is missing for {entry.episode_id}",
                )
            )
        elif actual_hash != entry.episode_hash:
            issues.append(
                ManifestValidationIssue(
                    "episode_hash_mismatch",
                    f"{entry_path}.episode_hash",
                    f"episode hash does not match payload {entry.episode_id}",
                )
            )

    return tuple(sorted(issues, key=lambda issue: (issue.path, issue.code, issue.message)))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_manifest", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    generate = commands.add_parser("generate", help="generate a manifest and episode payloads")
    generate.add_argument("--config", required=True, help="manual_collection.yaml path")
    generate.add_argument("--scene-id", required=True)
    generate.add_argument("--seed", type=int)
    generate.add_argument(
        "--split-ratios",
        type=parse_split_ratios,
        default=DEFAULT_SPLIT_RATIOS,
        metavar="NAME=RATIO,...",
    )
    generate.add_argument("--output", required=True, help="output manifest JSON path")

    validate = commands.add_parser("validate", help="validate a manifest bundle")
    validate.add_argument("manifest", help="manifest JSON path")
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _build_parser().parse_args(args)
    try:
        if namespace.command == "generate":
            manifest, payload_dir = generate_manifest_bundle(
                config_path=namespace.config,
                scene_id=namespace.scene_id,
                seed=namespace.seed,
                split_ratios=namespace.split_ratios,
                output_path=namespace.output,
            )
            split_count = len({entry.split for entry in manifest.entries})
            print(
                f"generated {len(manifest.entries)} episodes in {split_count} splits: "
                f"{namespace.output} (payloads: {payload_dir})"
            )
            return 0

        issues = validate_manifest_bundle(namespace.manifest)
        if issues:
            print(f"invalid manifest: {len(issues)} error(s)", file=sys.stderr)
            for issue in issues:
                print(
                    f"{issue.code} {issue.path}: {issue.message}",
                    file=sys.stderr,
                )
            return 1
        manifest = EpisodeManifest.from_mapping(
            json.loads(Path(namespace.manifest).read_text(encoding="utf-8"))
        )
        split_count = len({entry.split for entry in manifest.entries})
        print(f"valid manifest: {len(manifest.entries)} episodes in {split_count} splits")
        return 0
    except (OSError, TypeError, ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
