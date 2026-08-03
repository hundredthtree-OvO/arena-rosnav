import pytest

from toilet_benchmark.toilet_director_node import _polyline_prefix_before_goal


def test_route_holding_is_interpolated_before_goal():
    route = [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 2.0, 0.0]]

    prefix = _polyline_prefix_before_goal(route, 1.25)

    assert prefix[:-1] == route[:2]
    assert prefix[-1] == pytest.approx([2.0, 0.75, 0.0])


def test_route_holding_clamps_to_route_start():
    route = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]

    assert _polyline_prefix_before_goal(route, 5.0) == [[0.0, 0.0, 0.0]]
