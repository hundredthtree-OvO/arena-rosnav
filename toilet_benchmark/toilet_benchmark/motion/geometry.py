"""GeometrySafetyPort adapter for the calibrated pedestrian envelope."""

from __future__ import annotations

from typing import Sequence

from .contracts import GeometrySafetyRequest, GeometrySafetyResult, Point2D
from .swept_envelope import SweptEnvelope


class SweptEnvelopeGeometrySafety:
    """Clip illegal motion without selecting a social avoidance direction."""

    def __init__(self, envelope: SweptEnvelope):
        self.envelope = envelope
        self._planner = None

    def bind(self, planner) -> None:
        self._planner = planner

    def project(self, request: GeometrySafetyRequest) -> GeometrySafetyResult:
        planner = self._require_planner()
        result = self.envelope.clip_step(
            planner,
            request.start_xy,
            request.proposed_xy,
            request.start_yaw,
            request.proposed_yaw,
        )
        return GeometrySafetyResult(
            position_xy=result.position,
            applied_fraction=result.fraction,
            clipped=result.clipped,
            minimum_clearance_m=result.minimum_clearance_m,
            recovering_overlap=result.recovering_overlap,
        )

    def pose_clearance(self, position: Point2D, yaw: float) -> float:
        return self.envelope.pose_clearance(self._require_planner(), position, yaw)

    def polyline_minimum_clearance(
        self,
        points: Sequence[Point2D],
        *,
        start_yaw: float,
    ) -> float:
        return self.envelope.polyline_minimum_clearance(
            self._require_planner(),
            points,
            start_yaw=start_yaw,
        )

    def _require_planner(self):
        if self._planner is None:
            raise RuntimeError("geometry safety requires a bound static planner")
        return self._planner
