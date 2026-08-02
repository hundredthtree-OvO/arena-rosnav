"""Deterministic geometric hard safety for HuNav motion proposals.

HuNav remains responsible for social motion. This module only projects a single
simulation step back into the legal disc-geometry set while preserving as much
tangential motion as possible.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Mapping, Sequence, Set, Tuple


Point2D = Tuple[float, float]
RobotComponent = Tuple[Point2D, Point2D, float]


@dataclass(frozen=True)
class SafetyConfig:
    clearance_m: float = 0.01
    contact_epsilon_m: float = 1e-4
    static_obstacle_radius_m: float = 0.03
    max_iterations: int = 8


@dataclass(frozen=True)
class SafetyDiagnostics:
    intervened: bool
    correction_count: int
    pedestrian_pair_contacts: int
    robot_contacts: int
    static_contacts: int
    max_correction_m: float
    constrained_agents: Tuple[int, ...]
    residual_violation_count: int

    def to_dict(self) -> Dict[str, object]:
        return {
            "intervened": self.intervened,
            "correction_count": self.correction_count,
            "pedestrian_pair_contacts": self.pedestrian_pair_contacts,
            "robot_contacts": self.robot_contacts,
            "static_contacts": self.static_contacts,
            "max_correction_m": self.max_correction_m,
            "constrained_agents": list(self.constrained_agents),
            "residual_violation_count": self.residual_violation_count,
        }


@dataclass(frozen=True)
class SafetyStepResult:
    positions: Mapping[int, Point2D]
    diagnostics: SafetyDiagnostics


def _add(left: Point2D, right: Point2D) -> Point2D:
    return left[0] + right[0], left[1] + right[1]


def _sub(left: Point2D, right: Point2D) -> Point2D:
    return left[0] - right[0], left[1] - right[1]


def _scale(vector: Point2D, factor: float) -> Point2D:
    return vector[0] * factor, vector[1] * factor


def _dot(left: Point2D, right: Point2D) -> float:
    return left[0] * right[0] + left[1] * right[1]


def _norm(vector: Point2D) -> float:
    return math.hypot(vector[0], vector[1])


def _normal(vector: Point2D, fallback_key: int = 0) -> Point2D:
    length = _norm(vector)
    if length > 1e-12:
        return vector[0] / length, vector[1] / length
    # Stable fallback prevents coincident discs from choosing a random side.
    angle = (fallback_key % 16) * (math.tau / 16.0)
    return math.cos(angle), math.sin(angle)


def _safe_relative_endpoint(
    start: Point2D,
    proposed: Point2D,
    minimum_distance: float,
    epsilon: float,
    fallback_key: int,
) -> Tuple[Point2D, bool]:
    """Clip relative disc motion at first contact and retain tangent motion."""
    minimum_distance = max(0.0, float(minimum_distance))
    epsilon = max(0.0, float(epsilon))
    start_distance = _norm(start)
    proposed_distance = _norm(proposed)
    target_distance = minimum_distance + epsilon

    if start_distance < minimum_distance:
        normal = _normal(start if start_distance > 1e-12 else proposed, fallback_key)
        outward = _dot(_sub(proposed, start), normal)
        if outward > 0.0 and proposed_distance >= target_distance:
            return proposed, False
        return _scale(normal, target_distance), True

    delta = _sub(proposed, start)
    a = _dot(delta, delta)
    if a <= 1e-18:
        if proposed_distance + 1e-12 >= target_distance:
            return proposed, False
        return _scale(_normal(proposed, fallback_key), target_distance), True

    b = 2.0 * _dot(start, delta)
    c = _dot(start, start) - minimum_distance * minimum_distance
    discriminant = b * b - 4.0 * a * c
    contact_time = None
    if discriminant >= 0.0:
        root = math.sqrt(max(0.0, discriminant))
        for candidate in sorted(((-b - root) / (2.0 * a), (-b + root) / (2.0 * a))):
            if -1e-12 <= candidate <= 1.0 + 1e-12:
                contact = _add(start, _scale(delta, min(1.0, max(0.0, candidate))))
                if _dot(delta, _normal(contact, fallback_key)) < 0.0:
                    contact_time = min(1.0, max(0.0, candidate))
                    break

    if contact_time is None:
        if proposed_distance + 1e-12 >= target_distance:
            return proposed, False
        return _scale(_normal(proposed, fallback_key), target_distance), True

    contact = _add(start, _scale(delta, contact_time))
    normal = _normal(contact, fallback_key)
    remaining = _scale(delta, 1.0 - contact_time)
    inward = _dot(remaining, normal)
    if inward < 0.0:
        remaining = _sub(remaining, _scale(normal, inward))
    safe = _add(_scale(normal, target_distance), remaining)
    if _norm(safe) < target_distance:
        safe = _scale(_normal(safe, fallback_key), target_distance)
    return safe, True


def _resolve_pair(
    start_left: Point2D,
    end_left: Point2D,
    start_right: Point2D,
    end_right: Point2D,
    minimum_distance: float,
    epsilon: float,
    *,
    left_fixed: bool,
    right_fixed: bool,
    fallback_key: int,
) -> Tuple[Point2D, Point2D, bool]:
    relative_start = _sub(start_left, start_right)
    relative_end = _sub(end_left, end_right)
    safe_relative, corrected = _safe_relative_endpoint(
        relative_start,
        relative_end,
        minimum_distance,
        epsilon,
        fallback_key,
    )
    if not corrected:
        return end_left, end_right, False
    if left_fixed and right_fixed:
        return end_left, end_right, False
    if left_fixed:
        return end_left, _sub(end_left, safe_relative), True
    if right_fixed:
        return _add(end_right, safe_relative), end_right, True

    center = _scale(_add(end_left, end_right), 0.5)
    half_relative = _scale(safe_relative, 0.5)
    return _add(center, half_relative), _sub(center, half_relative), True


def _distance(left: Point2D, right: Point2D) -> float:
    return _norm(_sub(left, right))


def project_safe_step(
    *,
    previous: Mapping[int, Point2D],
    proposed: Mapping[int, Point2D],
    radii: Mapping[int, float],
    robot_previous: Point2D,
    robot_proposed: Point2D,
    robot_radius: float,
    robot_components: Sequence[RobotComponent] = (),
    static_obstacles: Mapping[int, Sequence[Point2D]],
    fixed_ids: Iterable[int] = (),
    config: SafetyConfig = SafetyConfig(),
) -> SafetyStepResult:
    """Project one multi-agent step into a non-overlapping disc configuration."""
    agent_ids = sorted(previous)
    if set(agent_ids) != set(proposed) or set(agent_ids) != set(radii):
        raise ValueError("previous, proposed and radii must contain identical agent ids")

    fixed: Set[int] = set(fixed_ids)
    positions = {
        agent_id: (
            previous[agent_id] if agent_id in fixed else proposed[agent_id]
        )
        for agent_id in agent_ids
    }
    contact_keys: Set[str] = set()
    constrained_agents: Set[int] = set()
    correction_count = 0
    max_correction = 0.0
    components = tuple(robot_components)
    if not components and robot_radius > 0.0:
        components = ((robot_previous, robot_proposed, robot_radius),)

    def record(
        key: str,
        affected: Iterable[int],
        before: Mapping[int, Point2D],
    ) -> None:
        nonlocal correction_count, max_correction
        correction_count += 1
        contact_keys.add(key)
        for agent_id in affected:
            constrained_agents.add(agent_id)
            max_correction = max(
                max_correction,
                _distance(before[agent_id], positions[agent_id]),
            )

    for _ in range(max(1, int(config.max_iterations))):
        changed = False

        for agent_id in agent_ids:
            if agent_id in fixed:
                continue
            for obstacle_index, obstacle in enumerate(static_obstacles.get(agent_id, ())):
                before_position = positions[agent_id]
                relative_start = _sub(previous[agent_id], obstacle)
                relative_end = _sub(before_position, obstacle)
                safe_relative, corrected = _safe_relative_endpoint(
                    relative_start,
                    relative_end,
                    float(radii[agent_id])
                    + float(config.static_obstacle_radius_m)
                    + float(config.clearance_m),
                    float(config.contact_epsilon_m),
                    fallback_key=agent_id * 1009 + obstacle_index,
                )
                if corrected:
                    before = {agent_id: before_position}
                    positions[agent_id] = _add(obstacle, safe_relative)
                    record(f"static:{agent_id}:{obstacle_index}", (agent_id,), before)
                    changed = True

        for component_index, (
            component_previous,
            component_proposed,
            component_radius,
        ) in enumerate(components):
            if component_radius <= 0.0:
                continue
            for agent_id in agent_ids:
                if agent_id in fixed:
                    continue
                before_position = positions[agent_id]
                relative_start = _sub(previous[agent_id], component_previous)
                relative_end = _sub(before_position, component_proposed)
                safe_relative, corrected = _safe_relative_endpoint(
                    relative_start,
                    relative_end,
                    float(radii[agent_id])
                    + float(component_radius)
                    + float(config.clearance_m),
                    float(config.contact_epsilon_m),
                    fallback_key=agent_id * 7919 + component_index,
                )
                if corrected:
                    before = {agent_id: before_position}
                    positions[agent_id] = _add(
                        component_proposed,
                        safe_relative,
                    )
                    record(
                        f"robot:{agent_id}:{component_index}",
                        (agent_id,),
                        before,
                    )
                    changed = True

        for index, left_id in enumerate(agent_ids):
            for right_id in agent_ids[index + 1 :]:
                before_left = positions[left_id]
                before_right = positions[right_id]
                safe_left, safe_right, corrected = _resolve_pair(
                    previous[left_id],
                    before_left,
                    previous[right_id],
                    before_right,
                    float(radii[left_id])
                    + float(radii[right_id])
                    + float(config.clearance_m),
                    float(config.contact_epsilon_m),
                    left_fixed=left_id in fixed,
                    right_fixed=right_id in fixed,
                    fallback_key=left_id * 1009 + right_id,
                )
                if corrected:
                    before = {left_id: before_left, right_id: before_right}
                    positions[left_id] = safe_left
                    positions[right_id] = safe_right
                    affected = [
                        agent_id
                        for agent_id in (left_id, right_id)
                        if agent_id not in fixed
                    ]
                    record(f"pair:{left_id}:{right_id}", affected, before)
                    changed = True

        if not changed:
            break

    # Final projection closes small residual penetrations left by coupled constraints.
    for index, left_id in enumerate(agent_ids):
        for right_id in agent_ids[index + 1 :]:
            minimum = (
                float(radii[left_id])
                + float(radii[right_id])
                + float(config.clearance_m)
                + float(config.contact_epsilon_m)
            )
            relative = _sub(positions[left_id], positions[right_id])
            if _norm(relative) + 1e-12 >= minimum:
                continue
            normal = _normal(relative, left_id * 1009 + right_id)
            correction = minimum - _norm(relative)
            before = {
                left_id: positions[left_id],
                right_id: positions[right_id],
            }
            if left_id in fixed and right_id not in fixed:
                positions[right_id] = _sub(
                    positions[right_id], _scale(normal, correction)
                )
            elif right_id in fixed and left_id not in fixed:
                positions[left_id] = _add(
                    positions[left_id], _scale(normal, correction)
                )
            elif left_id not in fixed and right_id not in fixed:
                half = _scale(normal, correction * 0.5)
                positions[left_id] = _add(positions[left_id], half)
                positions[right_id] = _sub(positions[right_id], half)
            else:
                continue
            record(
                f"pair:{left_id}:{right_id}",
                (
                    agent_id
                    for agent_id in (left_id, right_id)
                    if agent_id not in fixed
                ),
                before,
            )

    pair_contacts = sum(key.startswith("pair:") for key in contact_keys)
    robot_contacts = sum(key.startswith("robot:") for key in contact_keys)
    static_contacts = sum(key.startswith("static:") for key in contact_keys)
    residual_violations = 0
    for index, left_id in enumerate(agent_ids):
        for right_id in agent_ids[index + 1 :]:
            minimum = (
                float(radii[left_id])
                + float(radii[right_id])
                + float(config.clearance_m)
            )
            residual_violations += int(
                _distance(positions[left_id], positions[right_id])
                < minimum - 1e-8
            )
    for _, component_proposed, component_radius in components:
        for agent_id in agent_ids:
            minimum = (
                float(radii[agent_id])
                + float(component_radius)
                + float(config.clearance_m)
            )
            residual_violations += int(
                _distance(positions[agent_id], component_proposed)
                < minimum - 1e-8
            )
    for agent_id in agent_ids:
        minimum = (
            float(radii[agent_id])
            + float(config.static_obstacle_radius_m)
            + float(config.clearance_m)
        )
        residual_violations += sum(
            _distance(positions[agent_id], obstacle) < minimum - 1e-8
            for obstacle in static_obstacles.get(agent_id, ())
        )
    diagnostics = SafetyDiagnostics(
        intervened=bool(contact_keys),
        correction_count=correction_count,
        pedestrian_pair_contacts=pair_contacts,
        robot_contacts=robot_contacts,
        static_contacts=static_contacts,
        max_correction_m=max_correction,
        constrained_agents=tuple(sorted(constrained_agents)),
        residual_violation_count=residual_violations,
    )
    return SafetyStepResult(positions=positions, diagnostics=diagnostics)
