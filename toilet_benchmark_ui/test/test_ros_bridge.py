import math
from types import SimpleNamespace

import pytest

from toilet_benchmark_ui.ros_bridge import _quaternion_yaw


def test_quaternion_yaw_extracts_robot_odom_heading() -> None:
    yaw = 0.7
    orientation = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=math.sin(0.5 * yaw),
        w=math.cos(0.5 * yaw),
    )

    assert _quaternion_yaw(orientation) == pytest.approx(yaw)


def test_quaternion_yaw_falls_back_for_missing_orientation() -> None:
    assert _quaternion_yaw(None) == 0.0
