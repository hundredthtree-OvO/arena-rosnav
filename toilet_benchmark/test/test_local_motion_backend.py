from concurrent.futures import Future
import time

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


class _BlockingIsaacBackend(_IsaacBackend):
    def send(self, command, done_callback=None):
        self.commands.append(command)
        future = Future()
        if done_callback is not None:
            future.add_done_callback(done_callback)
        self.pending = future
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


def _snapshot(*, x=0.0, yaw=0.0, vx=0.0):
    return AgentSnapshot(
        agent_id="agent_01",
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


def test_independent_backend_dispatches_external_motion_without_hunav(backend):
    backend.send(_command())
    backend.update_agent_snapshot(_snapshot(vx=0.2))

    backend._tick()

    external = backend._isaac.commands[-1]
    assert external.use_external_motion is True
    assert external.external_motion_mode == EXTERNAL_MOTION_LOCOMOTION
    assert external.external_velocity[0] > 0.0
    assert external.direct_pose[0] > 0.0


def test_command_acceleration_respects_calibrated_embodiment_speed(backend):
    backend.send(_command())

    for _ in range(4):
        backend.update_agent_snapshot(_snapshot(vx=0.0))
        backend._last_tick_at = time.monotonic() - 0.1
        backend._tick()

    speeds = [
        command.external_velocity[0]
        for command in backend._isaac.commands
        if command.external_motion_mode == EXTERNAL_MOTION_LOCOMOTION
    ]
    assert speeds[0] == pytest.approx(0.2, abs=0.03)
    assert speeds[-1] == pytest.approx(0.31, abs=0.02)


def test_latest_external_command_is_dispatched_after_inflight_request(backend):
    blocking = _BlockingIsaacBackend(None, "unused")
    backend._isaac = blocking
    backend.send(_command())
    backend.update_agent_snapshot(_snapshot(x=0.0, vx=0.0))
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()
    first = blocking.commands[-1]

    backend.update_agent_snapshot(_snapshot(x=0.25, vx=0.0))
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()
    assert len(blocking.commands) == 1

    blocking.pending.set_result(type("MoveResult", (), {"ret": True})())
    backend._last_tick_at = time.monotonic() - 0.1
    backend._tick()

    assert len(blocking.commands) == 2
    assert blocking.commands[-1].direct_pose[0] > first.direct_pose[0]


def test_external_timeout_and_envelope_cover_service_jitter_and_agent_radius(backend):
    assert backend._external_timeout_sec >= 1.0
    assert (
        backend._geometry.envelope.config.disc_radius_m
        >= backend.agent_radius_m
    )


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

    external = backend._isaac.commands[-1]
    assert external.external_motion_mode == EXTERNAL_MOTION_TERMINAL_ALIGN
    assert backend.is_goal_settled(
        "agent_01",
        (1.9, 0.0, 0.0),
        command.goal_pose,
    )
