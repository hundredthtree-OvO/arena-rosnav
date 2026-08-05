import math
from types import SimpleNamespace

import pytest

from toilet_benchmark.geometry import person_yaw, point_to_polyline_distance


def test_person_yaw_is_read_by_tag_name() -> None:
    person = SimpleNamespace(
        tagnames=("motion_state", "yaw_valid", "yaw_rad"),
        tags=("executing", "true", "1.25"),
    )

    assert person_yaw(person) == pytest.approx(1.25)


def test_person_yaw_rejects_invalid_or_non_finite_values() -> None:
    invalid = SimpleNamespace(tagnames=("yaw_valid",), tags=("false",))
    non_finite = SimpleNamespace(
        tagnames=("yaw_valid", "yaw_rad"), tags=("true", str(math.inf))
    )

    assert person_yaw(invalid) is None
    assert person_yaw(non_finite) is None


def test_point_to_polyline_distance_uses_clamped_segments() -> None:
    route = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]

    assert point_to_polyline_distance(0.5, 0.2, route) == pytest.approx(0.2)
    assert point_to_polyline_distance(1.2, 0.5, route) == pytest.approx(0.2)
    assert point_to_polyline_distance(-0.3, 0.0, route) == pytest.approx(0.3)
