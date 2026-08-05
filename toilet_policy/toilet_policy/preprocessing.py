"""Shared online/offline observation preprocessing."""

from __future__ import annotations

import math

import numpy as np


def quaternion_yaw(quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def normalize_scan(message, beam_count: int) -> np.ndarray:
    values = np.asarray(message.ranges, dtype=np.float32)
    minimum = max(0.0, float(message.range_min))
    maximum = max(minimum + 1e-3, float(message.range_max))
    values = np.nan_to_num(values, nan=maximum, posinf=maximum, neginf=minimum)
    values = np.clip(values, minimum, maximum)
    if values.size != beam_count:
        source = np.linspace(0.0, 1.0, values.size, dtype=np.float32)
        target = np.linspace(0.0, 1.0, beam_count, dtype=np.float32)
        values = np.interp(target, source, values).astype(np.float32)
    return ((values - minimum) / (maximum - minimum)).astype(np.float32)
