"""Yaw-aware pedestrian envelope checks against the static walkable map."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


Point2D = tuple[float, float]


def _wrapped_delta(start: float, end: float) -> float:
    return math.atan2(math.sin(end - start), math.cos(end - start))


@dataclass(frozen=True)
class SweptEnvelopeConfig:
    """A capsule approximated by closely spaced discs along the body axis."""

    disc_radius_m: float = 0.20
    half_length_m: float = 0.125
    clearance_m: float = 0.01
    axis_sample_spacing_m: float = 0.05
    sweep_sample_spacing_m: float = 0.02
    angular_sample_step_rad: float = math.radians(5.0)
    clip_iterations: int = 12

    @classmethod
    def from_mapping(cls, values: Mapping[str, object] | None):
        values = dict(values or {})
        return cls(
            disc_radius_m=max(
                0.01, float(values.get("disc_radius_m", cls.disc_radius_m))
            ),
            half_length_m=max(
                0.0, float(values.get("half_length_m", cls.half_length_m))
            ),
            clearance_m=max(
                0.0, float(values.get("clearance_m", cls.clearance_m))
            ),
            axis_sample_spacing_m=max(
                0.01,
                float(
                    values.get(
                        "axis_sample_spacing_m",
                        cls.axis_sample_spacing_m,
                    )
                ),
            ),
            sweep_sample_spacing_m=max(
                0.005,
                float(
                    values.get(
                        "sweep_sample_spacing_m",
                        cls.sweep_sample_spacing_m,
                    )
                ),
            ),
            angular_sample_step_rad=max(
                math.radians(1.0),
                float(
                    values.get(
                        "angular_sample_step_rad",
                        cls.angular_sample_step_rad,
                    )
                ),
            ),
            clip_iterations=max(
                4, int(values.get("clip_iterations", cls.clip_iterations))
            ),
        )


@dataclass(frozen=True)
class SweptStepResult:
    position: Point2D
    fraction: float
    clipped: bool
    minimum_clearance_m: float
    recovering_overlap: bool = False


class SweptEnvelope:
    """Check a calibrated character capsule over translation and yaw changes."""

    def __init__(self, config: SweptEnvelopeConfig):
        self.config = config

    def disc_centers(self, position: Point2D, yaw: float) -> tuple[Point2D, ...]:
        half_length = self.config.half_length_m
        if half_length <= 1e-9:
            offsets = (0.0,)
        else:
            count = max(
                2,
                int(
                    math.ceil(
                        2.0 * half_length
                        / self.config.axis_sample_spacing_m
                    )
                ),
            )
            offsets = tuple(
                -half_length + 2.0 * half_length * index / count
                for index in range(count + 1)
            )
        forward = (math.cos(yaw), math.sin(yaw))
        return tuple(
            (
                float(position[0]) + offset * forward[0],
                float(position[1]) + offset * forward[1],
            )
            for offset in offsets
        )

    def pose_clearance(self, planner, position: Point2D, yaw: float) -> float:
        required = self.config.disc_radius_m + self.config.clearance_m
        return min(
            float(planner.clearance_at(center)) - required
            for center in self.disc_centers(position, yaw)
        )

    def pose_is_free(self, planner, position: Point2D, yaw: float) -> bool:
        return self.pose_clearance(planner, position, yaw) >= -1e-9

    def sweep_minimum_clearance(
        self,
        planner,
        start: Point2D,
        proposed: Point2D,
        start_yaw: float,
        proposed_yaw: float,
    ) -> float:
        return min(
            self.pose_clearance(planner, position, yaw)
            for _, position, yaw in self._sweep_poses(
                start,
                proposed,
                start_yaw,
                proposed_yaw,
            )
        )

    def clip_step(
        self,
        planner,
        start: Point2D,
        proposed: Point2D,
        start_yaw: float,
        proposed_yaw: float,
    ) -> SweptStepResult:
        poses = self._sweep_poses(start, proposed, start_yaw, proposed_yaw)
        first_clearance = self.pose_clearance(
            planner,
            poses[0][1],
            poses[0][2],
        )
        if first_clearance < -1e-9:
            return self._clip_overlap_recovery(
                planner,
                poses,
                first_clearance,
            )

        minimum_clearance = first_clearance
        previous_fraction = 0.0
        for fraction, position, yaw in poses[1:]:
            clearance = self.pose_clearance(planner, position, yaw)
            minimum_clearance = min(minimum_clearance, clearance)
            if clearance >= -1e-9:
                previous_fraction = fraction
                continue

            low = previous_fraction
            high = fraction
            yaw_delta = _wrapped_delta(start_yaw, proposed_yaw)
            for _ in range(self.config.clip_iterations):
                candidate_fraction = 0.5 * (low + high)
                candidate = self._interpolate_position(
                    start,
                    proposed,
                    candidate_fraction,
                )
                candidate_yaw = start_yaw + yaw_delta * candidate_fraction
                if self.pose_is_free(planner, candidate, candidate_yaw):
                    low = candidate_fraction
                else:
                    high = candidate_fraction
            safe = self._interpolate_position(start, proposed, low)
            return SweptStepResult(
                safe,
                low,
                True,
                minimum_clearance,
            )

        return SweptStepResult(
            (float(proposed[0]), float(proposed[1])),
            1.0,
            False,
            minimum_clearance,
        )

    def _clip_overlap_recovery(
        self,
        planner,
        poses: Sequence[tuple[float, Point2D, float]],
        first_clearance: float,
    ) -> SweptStepResult:
        """Allow only the requested motion that monotonically reduces overlap."""
        best_fraction = 0.0
        best_position = poses[0][1]
        best_clearance = first_clearance
        for fraction, position, yaw in poses[1:]:
            clearance = self.pose_clearance(planner, position, yaw)
            if clearance + 1e-9 < best_clearance:
                break
            best_fraction = fraction
            best_position = position
            best_clearance = clearance
        if best_fraction <= 1e-9:
            return SweptStepResult(
                poses[0][1],
                0.0,
                True,
                first_clearance,
                recovering_overlap=False,
            )
        return SweptStepResult(
            best_position,
            best_fraction,
            best_fraction < 1.0 - 1e-9,
            first_clearance,
            recovering_overlap=True,
        )

    def polyline_minimum_clearance(
        self,
        planner,
        points: Sequence[Point2D],
        *,
        start_yaw: float,
    ) -> float:
        if not points:
            return -math.inf
        if len(points) == 1:
            return self.pose_clearance(planner, points[0], start_yaw)

        minimum = math.inf
        current_yaw = float(start_yaw)
        for start, end in zip(points, points[1:]):
            dx = float(end[0]) - float(start[0])
            dy = float(end[1]) - float(start[1])
            segment_yaw = (
                current_yaw
                if math.hypot(dx, dy) <= 1e-9
                else math.atan2(dy, dx)
            )
            minimum = min(
                minimum,
                self.sweep_minimum_clearance(
                    planner,
                    start,
                    end,
                    current_yaw,
                    segment_yaw,
                ),
            )
            current_yaw = segment_yaw
        return minimum

    def _sweep_poses(
        self,
        start: Point2D,
        proposed: Point2D,
        start_yaw: float,
        proposed_yaw: float,
    ) -> tuple[tuple[float, Point2D, float], ...]:
        translation = math.dist(start, proposed)
        yaw_delta = _wrapped_delta(start_yaw, proposed_yaw)
        translation_steps = math.ceil(
            translation / self.config.sweep_sample_spacing_m
        )
        angular_steps = math.ceil(
            abs(yaw_delta) / self.config.angular_sample_step_rad
        )
        count = max(1, translation_steps, angular_steps)
        return tuple(
            (
                fraction,
                self._interpolate_position(start, proposed, fraction),
                float(start_yaw) + yaw_delta * fraction,
            )
            for fraction in (index / count for index in range(count + 1))
        )

    @staticmethod
    def _interpolate_position(
        start: Point2D,
        proposed: Point2D,
        fraction: float,
    ) -> Point2D:
        return (
            float(start[0])
            + float(fraction) * (float(proposed[0]) - float(start[0])),
            float(start[1])
            + float(fraction) * (float(proposed[1]) - float(start[1])),
        )
