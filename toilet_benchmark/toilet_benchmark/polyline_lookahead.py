"""Monotonic lookahead tracking over a planar global route."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class LookaheadTarget:
    x: float
    y: float
    progress_m: float
    remaining_m: float
    target_progress_m: float
    is_final: bool


class PolylineLookaheadTracker:
    """Project noisy poses onto a route without allowing progress to regress."""

    def __init__(
        self,
        points: Sequence[Sequence[float]],
        *,
        lookahead_m: float,
        dedupe_distance_m: float = 0.05,
    ):
        self.lookahead_m = max(0.05, float(lookahead_m))
        self._points = self._dedupe(points, max(0.0, float(dedupe_distance_m)))
        if not self._points:
            raise ValueError("lookahead tracking requires at least one route point")
        self._cumulative = [0.0]
        for start, end in zip(self._points, self._points[1:]):
            self._cumulative.append(
                self._cumulative[-1]
                + math.hypot(end[0] - start[0], end[1] - start[1])
            )
        self.total_length_m = self._cumulative[-1]
        self.progress_m = 0.0

    @staticmethod
    def _dedupe(
        points: Sequence[Sequence[float]],
        threshold_m: float,
    ) -> list[tuple[float, float]]:
        result: list[tuple[float, float]] = []
        for point in points:
            parsed = (float(point[0]), float(point[1]))
            if result and math.hypot(
                parsed[0] - result[-1][0],
                parsed[1] - result[-1][1],
            ) <= threshold_m:
                result[-1] = parsed
            else:
                result.append(parsed)
        return result

    def update(self, x: float, y: float) -> LookaheadTarget:
        projected = self._nearest_progress(float(x), float(y))
        self.progress_m = max(self.progress_m, projected)
        remaining = max(0.0, self.total_length_m - self.progress_m)
        target_progress = min(
            self.total_length_m,
            self.progress_m + self.lookahead_m,
        )
        target_x, target_y = self._point_at(target_progress)
        return LookaheadTarget(
            x=target_x,
            y=target_y,
            progress_m=self.progress_m,
            remaining_m=remaining,
            target_progress_m=target_progress,
            is_final=target_progress >= self.total_length_m - 1e-6,
        )

    def _nearest_progress(self, x: float, y: float) -> float:
        best_distance_sq = math.inf
        best_progress = self.progress_m
        for index, (start, end) in enumerate(zip(self._points, self._points[1:])):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length_sq = dx * dx + dy * dy
            if length_sq <= 1e-12:
                continue
            ratio = max(
                0.0,
                min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy) / length_sq),
            )
            projected_x = start[0] + ratio * dx
            projected_y = start[1] + ratio * dy
            distance_sq = (x - projected_x) ** 2 + (y - projected_y) ** 2
            progress = self._cumulative[index] + ratio * math.sqrt(length_sq)
            if progress + 1e-6 < self.progress_m:
                continue
            if distance_sq < best_distance_sq:
                best_distance_sq = distance_sq
                best_progress = progress
        return best_progress

    def _point_at(self, progress_m: float) -> tuple[float, float]:
        if len(self._points) == 1:
            return self._points[0]
        progress = max(0.0, min(float(progress_m), self.total_length_m))
        for index in range(len(self._points) - 1):
            start_progress = self._cumulative[index]
            end_progress = self._cumulative[index + 1]
            if progress > end_progress and index < len(self._points) - 2:
                continue
            segment_length = max(end_progress - start_progress, 1e-12)
            ratio = (progress - start_progress) / segment_length
            start = self._points[index]
            end = self._points[index + 1]
            return (
                start[0] + ratio * (end[0] - start[0]),
                start[1] + ratio * (end[1] - start[1]),
            )
        return self._points[-1]
