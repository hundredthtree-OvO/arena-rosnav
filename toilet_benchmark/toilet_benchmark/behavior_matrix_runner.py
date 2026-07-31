"""Sequentially run a matrix of HuNav interactive smoke configurations."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Sequence

from toilet_benchmark.interactive_smoke_runner import (
    DYNAMIC_CROSSING_PROFILES,
    discover_paths,
)


@dataclass(frozen=True)
class BehaviorMatrixSpec:
    behaviors: tuple[str, ...]
    agent_counts: tuple[int, ...]
    intervention_profiles: tuple[str, ...]
    seeds: tuple[int, ...]
    output_root: Path
    dry_run: bool
    stop_on_failure: bool
    reaction: str
    legacy_avoidance: str
    target_resource: str
    reuse_existing_bridge: bool = False
    skip_spawn: bool = False


@dataclass(frozen=True)
class BehaviorMatrixRunSpec:
    behavior: str
    initial_agents: int
    intervention_profile: str
    seed: int


def _csv_values(value: str) -> tuple[str, ...]:
    values = tuple(item.strip() for item in str(value).split(",") if item.strip())
    if not values:
        raise argparse.ArgumentTypeError("value list must not be empty")
    return values


def _csv_ints(value: str) -> tuple[int, ...]:
    return tuple(int(item) for item in _csv_values(value))


def _intervention_profile_values(value: str) -> tuple[str, ...]:
    values = _csv_values(value)
    allowed = set(DYNAMIC_CROSSING_PROFILES)
    invalid = [item for item in values if item not in allowed]
    if invalid:
        raise argparse.ArgumentTypeError(
            "intervention profiles must be one or more of: "
            + ", ".join(sorted(allowed))
        )
    return values


def _slug(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "run"


def build_hunav_interactive_smoke_command(
    paths,
    run_spec: BehaviorMatrixRunSpec,
    *,
    run_output_root: Path,
    reaction: str,
    legacy_avoidance: str,
    target_resource: str,
    reuse_existing_bridge: bool = False,
    skip_spawn: bool = False,
) -> list[str]:
    smoke_script = paths.workspace / "src" / "arena" / "arena-rosnav" / "toilet_benchmark" / "scripts" / "hunav_interactive_smoke"
    command = [
        sys.executable,
        str(smoke_script),
        "--profile",
        str(paths.profile),
        "--benchmark",
        str(paths.benchmark),
        "--output-root",
        str(run_output_root),
        "--behavior",
        str(run_spec.behavior),
        "--reaction",
        str(reaction),
        "--legacy-avoidance",
        str(legacy_avoidance),
        "--seed",
        str(run_spec.seed),
        "--target-resource",
        str(target_resource),
        "--initial-agents",
        str(run_spec.initial_agents),
        "--robot-intervention",
        "dynamic_crossing",
        "--dynamic-crossing-profile",
        str(run_spec.intervention_profile),
    ]
    if reuse_existing_bridge:
        command.append("--reuse-existing-bridge")
    if skip_spawn:
        command.append("--skip-spawn")
    return command


def expand_behavior_matrix(spec: BehaviorMatrixSpec) -> tuple[BehaviorMatrixRunSpec, ...]:
    return tuple(
        BehaviorMatrixRunSpec(
            behavior=behavior,
            initial_agents=agent_count,
            intervention_profile=intervention_profile,
            seed=seed,
        )
        for behavior in spec.behaviors
        for agent_count in spec.agent_counts
        for intervention_profile in spec.intervention_profiles
        for seed in spec.seeds
    )


def _read_subrun_summary(run_output_root: Path) -> dict[str, object] | None:
    summary_paths = sorted(run_output_root.rglob("summary.json"))
    if not summary_paths:
        return None
    return json.loads(summary_paths[0].read_text(encoding="utf-8"))


def _run_directory(base_output_root: Path) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    run_dir = base_output_root / f"{stamp}_behavior_matrix"
    run_dir.mkdir(parents=True, exist_ok=False)
    return run_dir


def run_behavior_matrix(
    spec: BehaviorMatrixSpec,
    *,
    paths=None,
) -> dict[str, object]:
    resolved_paths = discover_paths() if paths is None else paths
    matrix_run_dir = _run_directory(spec.output_root)
    runs_root = matrix_run_dir / "runs"
    runs_root.mkdir(parents=True, exist_ok=True)
    planned_runs = expand_behavior_matrix(spec)
    results: list[dict[str, object]] = []
    completed_runs = 0
    failed_runs = 0
    stopped_on_failure = False

    for index, run_spec in enumerate(planned_runs):
        run_slug = _slug(
            f"{index:03d}_{run_spec.behavior}_a{run_spec.initial_agents}_{run_spec.intervention_profile}_s{run_spec.seed}"
        )
        run_output_root = runs_root / run_slug
        command = build_hunav_interactive_smoke_command(
            resolved_paths,
            run_spec,
            run_output_root=run_output_root,
            reaction=spec.reaction,
            legacy_avoidance=spec.legacy_avoidance,
            target_resource=spec.target_resource,
            reuse_existing_bridge=spec.reuse_existing_bridge,
            skip_spawn=spec.skip_spawn,
        )
        run_result: dict[str, object] = {
            "index": index,
            "behavior": run_spec.behavior,
            "initial_agents": run_spec.initial_agents,
            "intervention_profile": run_spec.intervention_profile,
            "seed": run_spec.seed,
            "command": command,
            "output_root": str(run_output_root),
        }
        if spec.dry_run:
            run_result["status"] = "planned"
            results.append(run_result)
            continue

        run_output_root.mkdir(parents=True, exist_ok=True)
        completed = subprocess.run(
            command,
            cwd=str(resolved_paths.workspace),
            env=os.environ.copy(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            check=False,
        )
        launcher_log = run_output_root / "launcher.log"
        launcher_log.write_text(completed.stdout or "", encoding="utf-8")
        subrun_summary = _read_subrun_summary(run_output_root)
        run_result.update(
            {
                "status": "succeeded" if completed.returncode == 0 else "failed",
                "returncode": int(completed.returncode),
                "launcher_log": str(launcher_log),
                "subrun_summary": subrun_summary,
            }
        )
        summary_paths = sorted(run_output_root.rglob("summary.json"))
        if summary_paths:
            run_result["summary_path"] = str(summary_paths[0])
        if completed.returncode == 0:
            completed_runs += 1
        else:
            failed_runs += 1
            if spec.stop_on_failure:
                stopped_on_failure = True
                results.append(run_result)
                break
        results.append(run_result)

    matrix_summary = {
        "matrix_run_dir": str(matrix_run_dir),
        "dry_run": bool(spec.dry_run),
        "stop_on_failure": bool(spec.stop_on_failure),
        "planned_run_count": len(planned_runs),
        "completed_run_count": completed_runs,
        "failed_run_count": failed_runs,
        "stopped_on_failure": stopped_on_failure,
        "matrix": {
            "behaviors": list(spec.behaviors),
            "agent_counts": list(spec.agent_counts),
            "intervention_profiles": list(spec.intervention_profiles),
            "seeds": list(spec.seeds),
            "reaction": spec.reaction,
            "legacy_avoidance": spec.legacy_avoidance,
            "target_resource": spec.target_resource,
            "reuse_existing_bridge": spec.reuse_existing_bridge,
            "skip_spawn": spec.skip_spawn,
        },
        "runs": results,
    }
    (matrix_run_dir / "matrix_summary.json").write_text(
        json.dumps(matrix_summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return matrix_summary


def _build_parser() -> argparse.ArgumentParser:
    defaults = discover_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--behaviors",
        type=_csv_values,
        default=("surprised",),
        help="Comma-separated HuNav behaviors to sweep.",
    )
    parser.add_argument(
        "--agent-counts",
        type=_csv_ints,
        default=(1,),
        help="Comma-separated initial agent counts to sweep.",
    )
    parser.add_argument(
        "--intervention-profiles",
        type=_intervention_profile_values,
        default=("custom",),
        help="Comma-separated dynamic-crossing profiles to sweep.",
    )
    parser.add_argument(
        "--seeds",
        type=_csv_ints,
        default=(12345,),
        help="Comma-separated seeds to sweep.",
    )
    parser.add_argument("--output-root", type=Path, default=defaults.output_root)
    parser.add_argument("--reaction", choices=("source", "enabled", "disabled"), default="disabled")
    parser.add_argument(
        "--legacy-avoidance",
        choices=("source", "enabled", "disabled"),
        default="disabled",
    )
    parser.add_argument("--target-resource", default="urinal_1")
    parser.add_argument(
        "--reuse-existing-bridge",
        action="store_true",
        help="Reuse one externally managed bridge for every matrix run.",
    )
    parser.add_argument(
        "--skip-spawn",
        action="store_true",
        help="Assume the scene and robot have already been spawned.",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--stop-on-failure", action="store_true")
    return parser


def main(args: Sequence[str] | None = None) -> int:
    parsed = _build_parser().parse_args(args)
    spec = BehaviorMatrixSpec(
        behaviors=tuple(parsed.behaviors),
        agent_counts=tuple(parsed.agent_counts),
        intervention_profiles=tuple(parsed.intervention_profiles),
        seeds=tuple(parsed.seeds),
        output_root=parsed.output_root.expanduser().resolve(),
        dry_run=bool(parsed.dry_run),
        stop_on_failure=bool(parsed.stop_on_failure),
        reaction=parsed.reaction,
        legacy_avoidance=parsed.legacy_avoidance,
        target_resource=parsed.target_resource,
        reuse_existing_bridge=bool(parsed.reuse_existing_bridge),
        skip_spawn=bool(parsed.skip_spawn),
    )
    summary = run_behavior_matrix(spec)
    print(json.dumps(summary, indent=2, sort_keys=True), flush=True)
    return 0 if spec.dry_run or summary["failed_run_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
