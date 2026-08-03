"""Dependency-free deterministic sampled reciprocal velocity-obstacle baseline."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from ..domain.agent import AgentSnapshot
from .contracts import (
    BehaviorMode,
    GeometrySafetyRequest,
    LocalMotionRequest,
    LocalMotionResult,
    Point2D,
)
from .ports import GeometrySafetyPort


@dataclass(frozen=True)
class SampledRvoConfig:
    time_horizon_sec: float = 2.0
    neighbor_distance_m: float = 3.0
    clearance_m: float = 0.05
    angular_samples: int = 36
    speed_samples: tuple[float, ...] = (
        1.0,
        0.875,
        0.75,
        0.625,
        0.5,
        0.375,
        0.25,
    )
    side_velocity_fraction: float = 0.35
    boundary_samples: int = 3
    boundary_padding_rad: float = math.radians(3.0)
    max_acceleration_mps2: float = 2.0
    max_heading_rate_rps: float = math.radians(240.0)
    stationary_heading_speed_mps: float = 0.03
    geometry_min_fraction: float = 0.999
    minimum_forward_speed_fraction: float = 0.05
    dynamic_clearance_target_m: float = 0.15
    dynamic_clearance_weight: float = 8.0
    overlap_recovery_horizon_sec: float = 0.50
    overlap_recovery_min_gain_m: float = 0.005

    def __post_init__(self) -> None:
        if self.time_horizon_sec <= 0.0:
            raise ValueError("time_horizon_sec must be positive")
        if self.neighbor_distance_m <= 0.0:
            raise ValueError("neighbor_distance_m must be positive")
        if self.clearance_m < 0.0:
            raise ValueError("clearance_m must be non-negative")
        if self.angular_samples < 8:
            raise ValueError("angular_samples must be at least 8")
        if not self.speed_samples or any(
            not 0.0 <= float(value) <= 1.0 for value in self.speed_samples
        ):
            raise ValueError("speed_samples must contain values between 0 and 1")
        if self.side_velocity_fraction < 0.0:
            raise ValueError("side_velocity_fraction must be non-negative")
        if self.boundary_samples < 1:
            raise ValueError("boundary_samples must be positive")
        if self.boundary_padding_rad < 0.0:
            raise ValueError("boundary_padding_rad must be non-negative")
        if self.max_acceleration_mps2 <= 0.0:
            raise ValueError("max_acceleration_mps2 must be positive")
        if self.max_heading_rate_rps <= 0.0:
            raise ValueError("max_heading_rate_rps must be positive")
        if self.stationary_heading_speed_mps < 0.0:
            raise ValueError("stationary_heading_speed_mps must be non-negative")
        if not 0.0 <= self.geometry_min_fraction <= 1.0:
            raise ValueError("geometry_min_fraction must be between 0 and 1")
        if not 0.0 <= self.minimum_forward_speed_fraction <= 1.0:
            raise ValueError(
                "minimum_forward_speed_fraction must be between 0 and 1"
            )
        if self.dynamic_clearance_target_m < 0.0:
            raise ValueError("dynamic_clearance_target_m must be non-negative")
        if self.dynamic_clearance_weight < 0.0:
            raise ValueError("dynamic_clearance_weight must be non-negative")
        if self.overlap_recovery_horizon_sec <= 0.0:
            raise ValueError("overlap_recovery_horizon_sec must be positive")
        if self.overlap_recovery_min_gain_m < 0.0:
            raise ValueError("overlap_recovery_min_gain_m must be non-negative")


class SampledRvoLocalMotion:
    """Choose the closest collision-free velocity to the requested route motion."""

    def __init__(
        self,
        config: SampledRvoConfig = SampledRvoConfig(),
        *,
        geometry_safety: GeometrySafetyPort | None = None,
    ):
        self.config = config
        self.geometry_safety = geometry_safety

    def step(self, request: LocalMotionRequest) -> LocalMotionResult:
        if request.behavior.hold_position:
            return LocalMotionResult(
                velocity_xy=(0.0, 0.0),
                heading_rad=request.agent.yaw,
                feasible=True,
                reason=request.behavior.reason or "behavior hold",
                diagnostics={"candidate_count": 1, "safe_candidate_count": 1},
            )

        target = request.route_target_xy or self._route_target(request)
        direction = (target[0] - request.agent.x, target[1] - request.agent.y)
        distance = math.hypot(*direction)
        if distance <= 1e-9:
            return LocalMotionResult(
                velocity_xy=(0.0, 0.0),
                heading_rad=request.agent.yaw,
                feasible=True,
                reason="route target reached",
            )
        heading = math.atan2(direction[1], direction[0])
        speed = max(
            0.0,
            float(request.preferred_speed_mps) * float(request.behavior.speed_scale),
        )
        preferred = (speed * math.cos(heading), speed * math.sin(heading))
        neighbors = tuple(self._neighbors(request))
        candidates, boundary_candidate_count = self._candidate_velocities(
            preferred,
            heading,
            speed,
            request.agent,
            neighbors,
        )
        dynamic_status_by_velocity = {
            velocity: self._dynamic_candidate_status(
                request.agent,
                velocity,
                neighbors,
                request.dt_sec,
                preferred,
            )
            for velocity in candidates
        }
        dynamic_clearance_by_velocity = {
            velocity: status[1]
            for velocity, status in dynamic_status_by_velocity.items()
        }
        dynamic_safe = tuple(
            velocity
            for velocity, status in dynamic_status_by_velocity.items()
            if status[0]
        )
        geometry_by_velocity = {
            velocity: self._geometry_candidate(request, velocity)
            for velocity in dynamic_safe
        }
        safe = tuple(
            velocity
            for velocity, geometry in geometry_by_velocity.items()
            if geometry[0]
        )
        geometry_safe_count = len(safe)
        forward_safe = tuple(
            velocity
            for velocity in safe
            if self._forward_speed(velocity, heading)
            >= speed * float(self.config.minimum_forward_speed_fraction) - 1e-9
        )
        backward_rejected_count = len(safe) - len(forward_safe)
        if forward_safe:
            safe = forward_safe
        elif not self._has_current_overlap(request.agent, neighbors):
            return LocalMotionResult(
                velocity_xy=(0.0, 0.0),
                heading_rad=request.agent.yaw,
                feasible=False,
                reason="yielding: no forward collision-free velocity",
                diagnostics={
                    "candidate_count": len(candidates),
                    "safe_candidate_count": geometry_safe_count,
                    "neighbor_count": len(neighbors),
                    "boundary_candidate_count": boundary_candidate_count,
                    "dynamic_rejected_count": len(candidates) - len(dynamic_safe),
                    "static_rejected_count": len(dynamic_safe) - geometry_safe_count,
                    "backward_rejected_count": backward_rejected_count,
                    "yielding_without_reverse": True,
                },
            )
        if not safe:
            return LocalMotionResult(
                velocity_xy=(0.0, 0.0),
                heading_rad=request.agent.yaw,
                feasible=False,
                reason="no collision-free sampled velocity",
                diagnostics={
                    "candidate_count": len(candidates),
                    "safe_candidate_count": 0,
                    "neighbor_count": len(neighbors),
                    "boundary_candidate_count": boundary_candidate_count,
                    "dynamic_rejected_count": len(candidates) - len(dynamic_safe),
                    "static_rejected_count": len(dynamic_safe),
                    "backward_rejected_count": 0,
                },
            )
        raw_selected = min(
            safe,
            key=lambda velocity: self._score_velocity(
                velocity,
                preferred,
                heading,
                request.behavior.preferred_side,
                speed,
                (request.agent.vx, request.agent.vy),
                geometry_by_velocity[velocity][1],
                dynamic_clearance_by_velocity[velocity],
            ),
        )
        selected, acceleration_limited = self._limit_acceleration(
            request,
            raw_selected,
            neighbors,
        )
        _, selected_static_clearance = self._geometry_candidate(request, selected)
        selected_speed = math.hypot(*selected)
        velocity_deviation = math.dist(selected, preferred)
        selected_heading = self._motion_heading(request, selected)
        raw_selected_static_clearance = geometry_by_velocity[raw_selected][1]
        effective_dynamic_target = self._effective_dynamic_clearance_target(
            raw_selected_static_clearance
        )
        return LocalMotionResult(
            velocity_xy=selected,
            heading_rad=selected_heading,
            feasible=selected_speed > 1e-9 or not neighbors,
            reason="sampled reciprocal velocity",
            diagnostics={
                "candidate_count": len(candidates),
                "safe_candidate_count": len(safe),
                "neighbor_count": len(neighbors),
                "boundary_candidate_count": boundary_candidate_count,
                "dynamic_rejected_count": len(candidates) - len(dynamic_safe),
                "static_rejected_count": len(dynamic_safe) - geometry_safe_count,
                "backward_rejected_count": backward_rejected_count,
                "preferred_speed_mps": speed,
                "selected_speed_mps": selected_speed,
                "velocity_deviation_mps": velocity_deviation,
                "acceleration_mps2": math.dist(
                    selected,
                    (request.agent.vx, request.agent.vy),
                )
                / request.dt_sec,
                "acceleration_limited": acceleration_limited,
                "minimum_static_clearance_m": selected_static_clearance,
                "minimum_dynamic_clearance_m": dynamic_clearance_by_velocity[raw_selected],
                "minimum_peer_dynamic_clearance_m": self._dynamic_clearance(
                    request.agent,
                    raw_selected,
                    request.peers,
                ),
                "minimum_robot_dynamic_clearance_m": self._dynamic_clearance(
                    request.agent,
                    raw_selected,
                    (request.robot,) if request.robot is not None else (),
                ),
                "effective_dynamic_clearance_target_m": effective_dynamic_target,
                "overlap_recovery_neighbor_count": dynamic_status_by_velocity[
                    raw_selected
                ][2],
                "avoidance_active": velocity_deviation > 0.05,
            },
        )

    def _route_target(self, request: LocalMotionRequest) -> Point2D:
        position = (request.agent.x, request.agent.y)
        for point in request.route.points:
            if math.dist(position, point[:2]) > 0.25:
                return (point[0], point[1])
        point = request.route.points[-1]
        return (point[0], point[1])

    def _neighbors(self, request: LocalMotionRequest) -> Iterable[AgentSnapshot]:
        candidates = request.peers + ((request.robot,) if request.robot is not None else ())
        for neighbor in candidates:
            if neighbor is None:
                continue
            if math.hypot(
                neighbor.x - request.agent.x,
                neighbor.y - request.agent.y,
            ) <= self.config.neighbor_distance_m:
                yield neighbor

    def _candidate_velocities(
        self,
        preferred: Point2D,
        heading: float,
        speed: float,
        agent: AgentSnapshot,
        neighbors: tuple[AgentSnapshot, ...],
    ) -> tuple[tuple[Point2D, ...], int]:
        if speed <= 1e-9:
            return ((0.0, 0.0),), 0
        candidates = [preferred]
        count = max(8, int(self.config.angular_samples))
        for speed_scale in self.config.speed_samples:
            candidate_speed = speed * max(0.0, min(1.0, float(speed_scale)))
            for index in range(count):
                angle = heading - math.pi + 2.0 * math.pi * index / count
                candidates.append(
                    (candidate_speed * math.cos(angle), candidate_speed * math.sin(angle))
                )
        before_boundaries = len(candidates)
        candidates.extend(
            self._velocity_obstacle_boundary_candidates(
                agent,
                neighbors,
                speed,
            )
        )
        boundary_candidate_count = len(candidates) - before_boundaries
        candidates.append((0.0, 0.0))
        # Stable de-duplication also makes diagnostic counts reproducible.
        return (
            tuple(
                dict.fromkeys(
                    (round(x, 12), round(y, 12)) for x, y in candidates
                )
            ),
            boundary_candidate_count,
        )

    def _velocity_obstacle_boundary_candidates(
        self,
        agent: AgentSnapshot,
        neighbors: tuple[AgentSnapshot, ...],
        preferred_speed: float,
    ) -> tuple[Point2D, ...]:
        candidates: list[Point2D] = []
        padding = float(self.config.boundary_padding_rad)
        sample_count = max(1, int(self.config.boundary_samples))
        for neighbor in neighbors:
            relative = (neighbor.x - agent.x, neighbor.y - agent.y)
            distance = math.hypot(*relative)
            required = (
                agent.radius_m
                + neighbor.radius_m
                + max(0.0, float(self.config.clearance_m))
            )
            if distance <= required + 1e-9:
                continue
            axis = math.atan2(relative[1], relative[0])
            half_angle = math.asin(min(1.0, required / distance))
            for side in (-1.0, 1.0):
                for index in range(sample_count):
                    offset = padding * (index + 1) / sample_count
                    angle = axis + side * (half_angle + offset)
                    for speed_scale in self.config.speed_samples:
                        relative_speed = preferred_speed * float(speed_scale)
                        candidate = (
                            neighbor.vx + relative_speed * math.cos(angle),
                            neighbor.vy + relative_speed * math.sin(angle),
                        )
                        magnitude = math.hypot(*candidate)
                        if magnitude > preferred_speed + 1e-9:
                            scale = preferred_speed / magnitude
                            candidate = (
                                candidate[0] * scale,
                                candidate[1] * scale,
                            )
                        candidates.append(candidate)
        return tuple(candidates)

    def _limit_acceleration(
        self,
        request: LocalMotionRequest,
        selected: Point2D,
        neighbors: tuple[AgentSnapshot, ...],
    ) -> tuple[Point2D, bool]:
        current = (request.agent.vx, request.agent.vy)
        maximum_delta = (
            float(self.config.max_acceleration_mps2) * request.dt_sec
        )
        delta = (selected[0] - current[0], selected[1] - current[1])
        magnitude = math.hypot(*delta)
        if magnitude <= maximum_delta + 1e-9:
            return selected, False
        limited = (
            current[0] + delta[0] * maximum_delta / magnitude,
            current[1] + delta[1] * maximum_delta / magnitude,
        )
        geometry_ok, _ = self._geometry_candidate(request, limited)
        if geometry_ok and self._collision_free(request.agent, limited, neighbors):
            return limited, True
        # Collision avoidance outranks smooth acceleration during emergencies.
        return selected, False

    def _motion_heading(
        self,
        request: LocalMotionRequest,
        velocity: Point2D,
    ) -> float:
        speed = math.hypot(*velocity)
        if speed <= float(self.config.stationary_heading_speed_mps):
            return float(request.agent.yaw)
        desired = math.atan2(velocity[1], velocity[0])
        delta = math.atan2(
            math.sin(desired - request.agent.yaw),
            math.cos(desired - request.agent.yaw),
        )
        maximum_delta = float(self.config.max_heading_rate_rps) * request.dt_sec
        delta = max(-maximum_delta, min(maximum_delta, delta))
        return math.atan2(
            math.sin(request.agent.yaw + delta),
            math.cos(request.agent.yaw + delta),
        )

    def _geometry_candidate(
        self,
        request: LocalMotionRequest,
        velocity: Point2D,
    ) -> tuple[bool, float]:
        if self.geometry_safety is None:
            return True, math.inf
        proposed = (
            request.agent.x + velocity[0] * request.dt_sec,
            request.agent.y + velocity[1] * request.dt_sec,
        )
        heading = self._motion_heading(request, velocity)
        try:
            result = self.geometry_safety.project(
                GeometrySafetyRequest(
                    start_xy=(request.agent.x, request.agent.y),
                    proposed_xy=proposed,
                    start_yaw=request.agent.yaw,
                    proposed_yaw=heading,
                )
            )
        except RuntimeError:
            return False, -math.inf
        # The external-motion adapter executes the sampled velocity itself; it
        # does not consume GeometrySafetyResult.position_xy.  Accepting a
        # partially clipped recovery would therefore apply the *unclipped*
        # velocity and can deepen a static overlap.  A recovery candidate is
        # safe here only when its complete step is monotonic and executable.
        accepted = bool(
            not result.clipped
            and result.applied_fraction
            >= float(self.config.geometry_min_fraction)
        )
        return accepted, float(result.minimum_clearance_m)

    def _collision_free(
        self,
        agent: AgentSnapshot,
        velocity: Point2D,
        neighbors: tuple[AgentSnapshot, ...],
    ) -> bool:
        return self._dynamic_clearance(agent, velocity, neighbors) >= -1e-9

    def _dynamic_candidate_status(
        self,
        agent: AgentSnapshot,
        velocity: Point2D,
        neighbors: tuple[AgentSnapshot, ...],
        dt_sec: float,
        preferred_velocity: Point2D,
    ) -> tuple[bool, float, int]:
        """Accept overlap recovery only when the candidate increases separation.

        A conventional velocity-obstacle test includes t=0, so once two actors
        overlap every candidate is rejected forever.  Existing overlap is
        handled separately: non-overlapping neighbors retain the full horizon
        check, while overlapping neighbors require monotonic separation over a
        short horizon.  The normal forward-route filter then gives way to the
        actor whose route actually leaves the conflict instead of making both
        actors back away or oscillate.
        """

        minimum_clearance = math.inf
        recovering_neighbors = 0
        recovery_horizon = min(
            float(self.config.time_horizon_sec),
            max(float(dt_sec), float(self.config.overlap_recovery_horizon_sec)),
        )
        for neighbor in neighbors:
            relative_position = (neighbor.x - agent.x, neighbor.y - agent.y)
            current_clearance = self._pair_clearance(
                agent,
                (agent.x, agent.y),
                agent.yaw,
                neighbor,
                (neighbor.x, neighbor.y),
                neighbor.yaw,
            )
            if current_clearance >= -1e-9:
                predicted_clearance = self._dynamic_clearance(
                    agent,
                    velocity,
                    (neighbor,),
                )
                minimum_clearance = min(minimum_clearance, predicted_clearance)
                if predicted_clearance < -1e-9:
                    return False, minimum_clearance, recovering_neighbors
                continue

            relative_velocity = (
                neighbor.vx - velocity[0],
                neighbor.vy - velocity[1],
            )
            preferred_relative_velocity = (
                neighbor.vx - preferred_velocity[0],
                neighbor.vy - preferred_velocity[1],
            )
            preferred_separation_rate = (
                relative_position[0] * preferred_relative_velocity[0]
                + relative_position[1] * preferred_relative_velocity[1]
            )
            if (
                preferred_separation_rate <= 0.0
                and str(agent.agent_id) < str(neighbor.agent_id)
            ):
                return False, min(minimum_clearance, current_clearance), recovering_neighbors
            future_clearance = self._pair_clearance(
                agent,
                (
                    agent.x + velocity[0] * recovery_horizon,
                    agent.y + velocity[1] * recovery_horizon,
                ),
                self._velocity_heading(velocity, agent.yaw),
                neighbor,
                (
                    neighbor.x + neighbor.vx * recovery_horizon,
                    neighbor.y + neighbor.vy * recovery_horizon,
                ),
                self._velocity_heading((neighbor.vx, neighbor.vy), neighbor.yaw),
            )
            separation_gain = future_clearance - current_clearance
            separation_rate = (
                relative_position[0] * relative_velocity[0]
                + relative_position[1] * relative_velocity[1]
            )
            minimum_clearance = min(minimum_clearance, current_clearance)
            if (
                separation_rate <= 0.0
                or separation_gain
                < float(self.config.overlap_recovery_min_gain_m)
            ):
                return False, minimum_clearance, recovering_neighbors
            recovering_neighbors += 1

        return True, minimum_clearance, recovering_neighbors

    def _dynamic_clearance(
        self,
        agent: AgentSnapshot,
        velocity: Point2D,
        neighbors: tuple[AgentSnapshot, ...],
    ) -> float:
        horizon = max(0.01, float(self.config.time_horizon_sec))
        minimum = math.inf
        for neighbor in neighbors:
            relative_position = (neighbor.x - agent.x, neighbor.y - agent.y)
            relative_velocity = (
                neighbor.vx - velocity[0],
                neighbor.vy - velocity[1],
            )
            velocity_sq = (
                relative_velocity[0] ** 2 + relative_velocity[1] ** 2
            )
            closest_time = (
                max(
                    0.0,
                    min(
                        horizon,
                        -(
                            relative_position[0] * relative_velocity[0]
                            + relative_position[1] * relative_velocity[1]
                        )
                        / velocity_sq,
                    ),
                )
                if velocity_sq > 1e-12
                else 0.0
            )
            closest = (
                agent.x + velocity[0] * closest_time,
                agent.y + velocity[1] * closest_time,
            )
            neighbor_closest = (
                neighbor.x + neighbor.vx * closest_time,
                neighbor.y + neighbor.vy * closest_time,
            )
            minimum = min(
                minimum,
                self._pair_clearance(
                    agent,
                    closest,
                    self._velocity_heading(velocity, agent.yaw),
                    neighbor,
                    neighbor_closest,
                    self._velocity_heading((neighbor.vx, neighbor.vy), neighbor.yaw),
                ),
            )
        return minimum

    def _has_current_overlap(
        self,
        agent: AgentSnapshot,
        neighbors: tuple[AgentSnapshot, ...],
    ) -> bool:
        return any(
            self._pair_clearance(
                agent,
                (agent.x, agent.y),
                agent.yaw,
                neighbor,
                (neighbor.x, neighbor.y),
                neighbor.yaw,
            )
            < -1e-9
            for neighbor in neighbors
        )

    def _pair_clearance(
        self,
        agent: AgentSnapshot,
        agent_xy: Point2D,
        agent_yaw: float,
        neighbor: AgentSnapshot,
        neighbor_xy: Point2D,
        neighbor_yaw: float,
    ) -> float:
        margin = max(0.0, float(self.config.clearance_m))
        if neighbor.box_half_length_m > 0.0 and neighbor.box_half_width_m > 0.0:
            return self._capsule_box_clearance(
                capsule=agent,
                capsule_xy=agent_xy,
                capsule_yaw=agent_yaw,
                box=neighbor,
                box_xy=neighbor_xy,
                box_yaw=neighbor_yaw,
            ) - margin
        first = self._capsule_segment(agent, agent_xy, agent_yaw)
        second = self._capsule_segment(neighbor, neighbor_xy, neighbor_yaw)
        return (
            self._segment_distance(first[0], first[1], second[0], second[1])
            - float(agent.radius_m)
            - float(neighbor.radius_m)
            - margin
        )

    @staticmethod
    def _velocity_heading(velocity: Point2D, fallback: float) -> float:
        return (
            float(fallback)
            if math.hypot(*velocity) <= 0.03
            else math.atan2(float(velocity[1]), float(velocity[0]))
        )

    @staticmethod
    def _capsule_segment(
        actor: AgentSnapshot,
        position: Point2D,
        yaw: float,
    ) -> tuple[Point2D, Point2D]:
        offset_x = float(actor.body_half_length_m) * math.cos(float(yaw))
        offset_y = float(actor.body_half_length_m) * math.sin(float(yaw))
        return (
            (float(position[0]) - offset_x, float(position[1]) - offset_y),
            (float(position[0]) + offset_x, float(position[1]) + offset_y),
        )

    def _capsule_box_clearance(
        self,
        *,
        capsule: AgentSnapshot,
        capsule_xy: Point2D,
        capsule_yaw: float,
        box: AgentSnapshot,
        box_xy: Point2D,
        box_yaw: float,
    ) -> float:
        start, end = self._capsule_segment(capsule, capsule_xy, capsule_yaw)
        half_length = max(0.01, float(capsule.body_half_length_m))
        spacing = 0.05
        samples = max(2, int(math.ceil((2.0 * half_length) / spacing)))
        minimum = math.inf
        c = math.cos(float(box_yaw))
        s = math.sin(float(box_yaw))
        for index in range(samples + 1):
            fraction = index / samples
            point = (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
            dx = point[0] - float(box_xy[0])
            dy = point[1] - float(box_xy[1])
            local_x = c * dx + s * dy
            local_y = -s * dx + c * dy
            outside_x = max(abs(local_x) - float(box.box_half_length_m), 0.0)
            outside_y = max(abs(local_y) - float(box.box_half_width_m), 0.0)
            if outside_x > 0.0 or outside_y > 0.0:
                signed_distance = math.hypot(outside_x, outside_y)
            else:
                signed_distance = -min(
                    float(box.box_half_length_m) - abs(local_x),
                    float(box.box_half_width_m) - abs(local_y),
                )
            minimum = min(minimum, signed_distance)
        return minimum - float(capsule.radius_m)

    @classmethod
    def _segment_distance(
        cls,
        a0: Point2D,
        a1: Point2D,
        b0: Point2D,
        b1: Point2D,
    ) -> float:
        if cls._segments_intersect(a0, a1, b0, b1):
            return 0.0
        return min(
            cls._point_segment_distance(a0, b0, b1),
            cls._point_segment_distance(a1, b0, b1),
            cls._point_segment_distance(b0, a0, a1),
            cls._point_segment_distance(b1, a0, a1),
        )

    @staticmethod
    def _point_segment_distance(point: Point2D, start: Point2D, end: Point2D) -> float:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_sq = dx * dx + dy * dy
        if length_sq <= 1e-12:
            return math.dist(point, start)
        fraction = max(
            0.0,
            min(1.0, ((point[0] - start[0]) * dx + (point[1] - start[1]) * dy) / length_sq),
        )
        return math.hypot(
            point[0] - (start[0] + fraction * dx),
            point[1] - (start[1] + fraction * dy),
        )

    @staticmethod
    def _segments_intersect(a0: Point2D, a1: Point2D, b0: Point2D, b1: Point2D) -> bool:
        def cross(origin, first, second):
            return (
                (first[0] - origin[0]) * (second[1] - origin[1])
                - (first[1] - origin[1]) * (second[0] - origin[0])
            )

        c1 = cross(a0, a1, b0)
        c2 = cross(a0, a1, b1)
        c3 = cross(b0, b1, a0)
        c4 = cross(b0, b1, a1)
        if max(abs(c1), abs(c2), abs(c3), abs(c4)) <= 1e-9:
            return not (
                max(a0[0], a1[0]) < min(b0[0], b1[0]) - 1e-9
                or max(b0[0], b1[0]) < min(a0[0], a1[0]) - 1e-9
                or max(a0[1], a1[1]) < min(b0[1], b1[1]) - 1e-9
                or max(b0[1], b1[1]) < min(a0[1], a1[1]) - 1e-9
            )
        return c1 * c2 <= 0.0 and c3 * c4 <= 0.0

    def _score_velocity(
        self,
        velocity: Point2D,
        preferred: Point2D,
        heading: float,
        preferred_side: int,
        preferred_speed: float,
        current_velocity: Point2D,
        static_clearance_m: float,
        dynamic_clearance_m: float,
    ) -> tuple[float, float, float]:
        deviation = (velocity[0] - preferred[0]) ** 2 + (
            velocity[1] - preferred[1]
        ) ** 2
        speed_loss = max(0.0, preferred_speed - math.hypot(*velocity))
        lateral = -math.sin(heading) * velocity[0] + math.cos(heading) * velocity[1]
        side_target = (
            preferred_side
            * preferred_speed
            * max(0.0, float(self.config.side_velocity_fraction))
        )
        side_cost = (lateral - side_target) ** 2 if preferred_side else lateral**2 * 0.05
        acceleration_cost = math.dist(velocity, current_velocity) ** 2
        clearance_bonus = (
            min(1.0, max(0.0, static_clearance_m))
            if math.isfinite(static_clearance_m)
            else 0.0
        )
        dynamic_margin_error = max(
            0.0,
            self._effective_dynamic_clearance_target(static_clearance_m)
            - dynamic_clearance_m,
        ) if math.isfinite(dynamic_clearance_m) else 0.0
        return (
            deviation
            + 0.35 * side_cost
            + 0.1 * speed_loss**2
            + 0.15 * acceleration_cost
            + float(self.config.dynamic_clearance_weight) * dynamic_margin_error**2
            - 0.02 * clearance_bonus,
            -math.hypot(*velocity),
            math.atan2(velocity[1], velocity[0]) if math.hypot(*velocity) > 1e-9 else 0.0,
        )

    def _effective_dynamic_clearance_target(
        self,
        static_clearance_m: float,
    ) -> float:
        target = float(self.config.dynamic_clearance_target_m)
        if not math.isfinite(static_clearance_m):
            return target
        # Social spacing is soft. In a narrow passage it must shrink before
        # the hard actor/geometry envelopes do, otherwise avoidance steers the
        # character into a wall while trying to preserve open-space comfort.
        return min(target, max(0.0, float(static_clearance_m)))

    @staticmethod
    def _forward_speed(velocity: Point2D, heading: float) -> float:
        return (
            velocity[0] * math.cos(heading)
            + velocity[1] * math.sin(heading)
        )
