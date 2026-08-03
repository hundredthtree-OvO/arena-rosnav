from unittest.mock import patch

import pytest

from toilet_benchmark.motion.backend_factory import build_motion_backend


def test_backend_factory_keeps_isaac_people_selection_boundary():
    pytest.importorskip("isaacsim_msgs")
    from toilet_benchmark.motion import backend_factory

    sentinel = object()
    with patch(
        "toilet_benchmark.motion_backend.IsaacPeopleBackend",
        return_value=sentinel,
    ) as backend:
        result = backend_factory.build_motion_backend(
            object(),
            backend_name="isaac_people",
            move_service_name="/isaac/move_pedestrians",
            hunav_namespace="/hunav",
            hunav_config={},
            motion_config={},
            arrival_seed=12345,
        )

    assert result is sentinel
    backend.assert_called_once()


def test_backend_factory_rejects_unknown_backend_before_runtime_start():
    with pytest.raises(ValueError, match="Unsupported pedestrian motion backend"):
        build_motion_backend(
            object(),
            backend_name="unknown",
            move_service_name="/isaac/move_pedestrians",
            hunav_namespace="/hunav",
            hunav_config={},
            motion_config={},
            arrival_seed=12345,
        )
