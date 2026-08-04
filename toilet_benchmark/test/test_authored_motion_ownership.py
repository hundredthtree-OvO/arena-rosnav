from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_authored_runner_uses_people_only_as_segment_executor() -> None:
    source = (ROOT / "toilet_benchmark" / "tracks" / "authored_scenario.py").read_text(
        encoding="utf-8"
    )

    assert "IsaacPeopleBackend" in source
    assert '"/isaac/move_pedestrians"' in source
    assert "ExternalMotionStreamPublisher" not in source
    assert "runtime.execution_segment()" in source
    assert "runtime.arrive(now)" in source
    assert "self._poses.get(agent_id)" in source
    assert 'metadata.get("embodiment_generation"' in source
    assert 'metadata.get("command_generation"' in source
    assert 'metadata.get("motion_state"' in source
    assert "runtime.observe_activation(" in source
    assert 'runtime.state = "MOVING"' in source
    assert "self.create_service(Trigger, cancel_service" in source
    assert "AUTHORED_CANCEL" in source
    assert "pedestrian_departure_diagnostic" in source
