from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SemanticPose:
    position: tuple[float, float, float]
    yaw: float

    def as_list(self) -> list[float]:
        return [float(v) for v in self.position]

    def with_z(self, z: float) -> "SemanticPose":
        return SemanticPose(
            position=(float(self.position[0]), float(self.position[1]), float(z)),
            yaw=float(self.yaw),
        )


def parse_semantic_pose(
    value,
    *,
    default_z: float = 0.0,
    default_yaw: float = 0.0,
) -> SemanticPose:
    try:
        vals = [float(v) for v in value]
    except Exception as exc:
        raise ValueError(f"Invalid semantic pose: {value!r}") from exc

    if len(vals) >= 4:
        x, y, z, yaw = vals[:4]
        return SemanticPose(position=(x, y, z), yaw=yaw)
    if len(vals) >= 3:
        x, y, yaw = vals[:3]
        return SemanticPose(position=(x, y, float(default_z)), yaw=yaw)
    if len(vals) >= 2:
        x, y = vals[:2]
        return SemanticPose(position=(x, y, float(default_z)), yaw=float(default_yaw))
    raise ValueError(f"Semantic pose must provide at least x and y: {value!r}")
