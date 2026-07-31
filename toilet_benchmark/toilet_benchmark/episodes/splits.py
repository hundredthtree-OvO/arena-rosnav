"""Episode split planning for S2-B."""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Any, Mapping, Sequence


class SplitValidationError(ValueError):
    """Raised when split config or assignment cannot be satisfied."""


@dataclass(frozen=True)
class SplitAssignment:
    split_to_episodes: dict[str, tuple[str, ...]]
    seed: int | None = None

    def all_assigned(self) -> set[str]:
        return {episode_id for values in self.split_to_episodes.values() for episode_id in values}


def _validate_ratio_map(ratios: Mapping[str, float]) -> None:
    if not ratios:
        raise SplitValidationError("splits must not be empty")
    if any(not isinstance(name, str) or not name for name in ratios):
        raise SplitValidationError("split names must be non-empty strings")
    for split_name, ratio in ratios.items():
        try:
            value = float(ratio)
        except (TypeError, ValueError) as exc:
            raise SplitValidationError(f"invalid ratio for {split_name}: {ratio}") from exc
        if value < 0:
            raise SplitValidationError(f"invalid ratio for {split_name}: {ratio}")
    total = float(sum(float(v) for v in ratios.values()))
    if total <= 0:
        raise SplitValidationError("ratios must sum to a positive value")


def assign_episodes_to_splits(
    episode_ids: Sequence[str],
    ratios: Mapping[str, float],
    *,
    seed: int | None = None,
) -> SplitAssignment:
    _validate_ratio_map(ratios)
    if not episode_ids:
        return SplitAssignment({split: () for split in ratios}, seed=seed)

    seen = set()
    normalized_ids: list[str] = []
    for episode_id in episode_ids:
        text = str(episode_id)
        if text in seen:
            raise SplitValidationError(f"duplicate episode_id: {text}")
        seen.add(text)
        normalized_ids.append(text)

    rng = random.Random(seed)
    ordered_ids = sorted(normalized_ids)
    if seed is not None:
        rng.shuffle(ordered_ids)

    total = len(ordered_ids)
    normalized_ratio = {name: float(value) for name, value in ratios.items()}
    total_ratio = sum(normalized_ratio.values())
    entries: list[tuple[str, int, float]] = []
    base_counts: dict[str, int] = {}
    used = 0
    for split_name in sorted(normalized_ratio):
        ratio = normalized_ratio[split_name]
        count = int((ratio / total_ratio) * total)
        base_counts[split_name] = count
        used += count
        entries.append((split_name, count, (ratio / total_ratio) * total - count))

    remaining = total - used
    entries.sort(key=lambda item: (-item[2], item[0]))
    for index in range(remaining):
        split_name = entries[index % len(entries)][0]
        base_counts[split_name] += 1

    cursor = 0
    result: dict[str, list[str]] = {split: [] for split in ratios}
    for split_name in sorted(base_counts):
        count = base_counts[split_name]
        selected = ordered_ids[cursor : cursor + count]
        result[split_name] = selected
        cursor += count

    if cursor != total:
        raise SplitValidationError("split assignment exhausted with mismatch")

    return SplitAssignment(split_to_episodes={name: tuple(result[name]) for name in sorted(result)}, seed=seed)
