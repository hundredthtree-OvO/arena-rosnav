"""Command-line preparation of reviewed behavior-cloning datasets."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Sequence

import yaml

from .audit import (
    annotation_template,
    audit_dataset,
    dataset_stats,
    load_annotations,
    write_jsonl,
)
from .schema import DATASET_SCHEMA_VERSION


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="toilet_dataset", description=__doc__)
    parser.add_argument("--data-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--session-glob",
        action="append",
        default=[],
        help="Glob relative to data-root; repeat for multiple groups.",
    )
    parser.add_argument("--split-seed", type=int, default=42)
    parser.add_argument(
        "--accept-unreviewed-success",
        action="store_true",
        help="Admit pending successful episodes for a smoke-test baseline.",
    )
    return parser


def main(args: Sequence[str] | None = None) -> int:
    namespace = _parser().parse_args(args)
    output = namespace.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    annotations_path = output / "annotations.yaml"
    try:
        annotations = load_annotations(annotations_path)
        records, split_by_session = audit_dataset(
            namespace.data_root,
            session_globs=namespace.session_glob or ["session_*"],
            annotations=annotations,
            split_seed=namespace.split_seed,
            accept_unreviewed_success=namespace.accept_unreviewed_success,
        )
        if not records:
            raise ValueError("no episode directories matched the requested sessions")
        write_jsonl(output / "index.jsonl", records)
        annotations_path.write_text(
            yaml.safe_dump(
                annotation_template(records), sort_keys=False, allow_unicode=True
            ),
            encoding="utf-8",
        )
        (output / "splits.yaml").write_text(
            yaml.safe_dump(
                {
                    "schema_version": DATASET_SCHEMA_VERSION,
                    "split_seed": namespace.split_seed,
                    "split_by_session": split_by_session,
                },
                sort_keys=False,
            ),
            encoding="utf-8",
        )
        stats = dataset_stats(records)
        (output / "stats.json").write_text(
            json.dumps(stats, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        print(json.dumps({"output": str(output), **stats}, sort_keys=True))
        return 0
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
