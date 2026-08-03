from concurrent.futures import Future
from dataclasses import replace
import json
import time
from types import SimpleNamespace

import pytest

from toilet_benchmark.domain import AgentSnapshot
from toilet_benchmark.domain.task import (
    EXTERNAL_MOTION_LOCOMOTION,
    EXTERNAL_MOTION_TERMINAL_ALIGN,
    MotionCommand,
)
import toilet_benchmark.local_motion_backend as backend_module


class _Logger:
    def info(self, _message):
        pass


class _Node:
    def __init__(self):
        self.timer_callback = None

    def get_logger(self):
        return _Logger()

    def create_timer(self, _period, callback):
        self.timer_callback = callback
        return object()

    def create_publisher(self, _message_type, _topic, _qos):
        return _Publisher()


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _IsaacBackend:
    def __init__(self, _node, _service_name):
        self.commands = []

    def wait_for_service(self, _timeout_sec):
        return True

    def send(self, command, done_callback=None):
        self.commands.append(command)
        future = Future()
        if done_callback is not None:
            future.add_done_callback(done_callback)
        future.set_result(type("MoveResult", (), {"ret": True})())
        return future


class _OpenPlanner:
    resolution = 0.05

    def clearance_at(self, _point):
        return 10.0


@pytest.fixture
def backend(monkeypatch):
    monkeypatch.setattr(backend_module, "IsaacPeopleBackend", _IsaacBackend)
    node = _Node()
    result = backend_module.LocalMotionBackend(
        node,
        move_service_name="/isaac/move_pedestrians",
        config={"compute_hz": 10.0},
    )
    result.set_walkable_planner(_OpenPlanner())
    return result


def _snapshot(*, agent_id="agent_01", x=0.0, yaw=0.0, vx=0.0):
    return AgentSnapshot(
        agent_id=agent_id,
        x=x,
        y=0.0,
        z=0.0,
        yaw=yaw,
        vx=vx,
        vy=0.0,
        radius_m=0.26,
        timestamp_sec=time.monotonic(),
        source="test",
    )


def _command():
    return MotionCommand(
        agent_id="agent_01",
        goal_pose=(2.0, 0.0, 0.0),
        path_points=((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        velocity=0.8,
        orientation=0.0,
        phase="WALK_TO_URINAL",
    )


def _stream_commands(backend):
    return [
        command
        for message in backend._external_stream._publisher.messages
        for command in json.loads(message.data)["commands"]
    ]


def test_independent_backend_dispatches_external_motion_without_hunav(backend):
    backend.send(_command())
    backend.update_agent_snapshot(_snapshot(vx=0.2))

    backend._tick()

    external = _stream_commands(backend)[-1]
    assert external["motion_mode"] == EXTERNAL_MOTION_LOCOMOTION
    assert external["velocity"][0] > 0.0
    assert external["position"][0] > 0.0
    assert backend._isaac.commands == []


def test_command_acceleration_respects_calibrated_embodiment_speed(backend):
    backend.send(_command())

    for _ in range(4):
        backend.update_agent_snapshot(_snapshot(vx=0.0))
        backend._last_tick_at = time.monotonic() - 0.1
        backend._tick()

    speeds = [
        command["velocity"][0]
        for command in _stream_commands(backend)
        if command["motion_mode"] == EXTERNAL_MOTION_LOCOMOTION
    ]
    assert speeds[0] == pytest.approx(0.2, abs=0.03)
    assert speeds[-1] == pytest.approx(0.31, abs=0.02)


def test_each_tick_publishes_latest_external_command_without_service_inflight(backend):
    backend.send(_command())
    backend.update_agent_snapshot(_snapshot(x=0.0, vx=0.0))
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()
    first = _stream_commands(backend)[-1]

    backend.update_agent_snapshot(_snapshot(x=0.25, vx=0.0))
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()
    commands = _stream_commands(backend)
    assert len(commands) == 2
    assert commands[-1]["position"][0] > first["position"][0]
    assert backend._isaac.commands == []


def test_stop_after_streamed_motion_clears_batch_without_legacy_state(backend):
    backend.send(_command())
    backend.update_agent_snapshot(_snapshot(vx=0.0))
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()

    stop = MotionCommand(
        agent_id="agent_01",
        goal_pose=(0.0, 0.0, 0.0),
        path_points=(),
        velocity=0.0,
        stop=True,
        phase="PRESPAWN_HOLD",
    )
    backend.send(stop)

    assert "agent_01" not in backend._commands
    assert "agent_01" not in backend._stream_batch
    assert backend._isaac.commands[-1] == stop


def test_external_timeout_and_envelope_cover_service_jitter_and_agent_radius(backend):
    assert backend._external_timeout_sec >= 1.0
    assert (
        backend._geometry.envelope.config.disc_radius_m
        >= backend.agent_radius_m
    )


def test_peer_prediction_uses_last_commanded_velocity_not_animgraph_pause(backend):
    captured = []

    class _Pipeline:
        def step(self, request, *, dt_sec, route_target_xy):
            del dt_sec, route_target_xy
            captured.append(request)
            return SimpleNamespace(
                motion=SimpleNamespace(
                    velocity_xy=(0.1, 0.0),
                    heading_rad=0.0,
                    feasible=True,
                    diagnostics={},
                )
            )

    backend._pipeline = _Pipeline()
    first = _command()
    second = replace(_command(), agent_id="agent_02")
    backend.send(first)
    backend.send(second)
    backend.update_agent_snapshot(_snapshot(agent_id="agent_01", x=0.0, vx=0.0))
    backend.update_agent_snapshot(_snapshot(agent_id="agent_02", x=1.0, vx=0.0))

    backend._tick()
    backend._tick()

    first_request = next(
        request
        for request in reversed(captured)
        if request.agent.agent_id == "agent_01"
    )
    peer = next(item for item in first_request.peers if item.agent_id == "agent_02")
    assert peer.vx == pytest.approx(0.1)


def test_prespawn_roster_does_not_activate_parked_agent(backend):
    parked = MotionCommand(
        agent_id="agent_01",
        goal_pose=(1000.0, 1000.0, 0.0),
        path_points=(),
        velocity=0.8,
        stop=True,
        phase="PRESPAWN_HOLD",
    )

    backend.register_agents((parked,))
    backend.update_agent_snapshot(_snapshot(x=1000.0))
    backend._tick()

    assert backend._isaac.commands == []


def test_independent_backend_reports_settled_after_terminal_alignment(backend):
    command = _command()
    backend.send(command)
    backend.update_agent_snapshot(_snapshot(x=1.9, yaw=0.0, vx=0.0))

    backend._tick()

    external = _stream_commands(backend)[-1]
    assert external["motion_mode"] == EXTERNAL_MOTION_TERMINAL_ALIGN
    assert backend.is_goal_settled(
        "agent_01",
        (1.9, 0.0, 0.0),
        command.goal_pose,
    )
