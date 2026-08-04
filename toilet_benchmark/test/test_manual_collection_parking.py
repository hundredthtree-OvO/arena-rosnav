import json
from types import SimpleNamespace

from toilet_benchmark.manual_collection_node import (
    ManualCollectionNode,
    _authored_runtime_nodes,
)


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _Future:
    def __init__(self, events, agent_id, result=True):
        self.events = events
        self.agent_id = agent_id
        self._result = result
        self.callback = None

    def add_done_callback(self, callback):
        self.callback = callback

    def result(self):
        self.events.append(f"park_done:{self.agent_id}")
        return SimpleNamespace(ret=self._result)


class _ParkingClient:
    def __init__(self, events):
        self.events = events
        self.futures = []
        self.results = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        agent_id = request.name.rsplit("/", 1)[-1]
        self.events.append(f"park_requested:{agent_id}")
        result = self.results.pop(0) if self.results else True
        future = _Future(self.events, agent_id, result=result)
        self.futures.append(future)
        return future


class _Recorder:
    has_active_episode = True

    def __init__(self, events):
        self.events = events

    def record_event(self, _name, _payload):
        self.events.append("finish_event")

    def finalize_episode(self, **_kwargs):
        self.events.append("bag_finalized")


class _Process:
    def __init__(self, return_code=None):
        self.return_code = return_code

    def poll(self):
        return self.return_code


class _Harness:
    _active_agent_ids = ManualCollectionNode._active_agent_ids
    _start_pedestrian_parking = ManualCollectionNode._start_pedestrian_parking
    _request_pending_pedestrian_parks = ManualCollectionNode._request_pending_pedestrian_parks
    _pedestrian_park_done = ManualCollectionNode._pedestrian_park_done
    _complete_episode_cleanup = ManualCollectionNode._complete_episode_cleanup
    _advance_authored_runtime_exit = ManualCollectionNode._advance_authored_runtime_exit

    def __init__(self):
        self.events = []
        scenario = SimpleNamespace(
            pedestrian_agent_ids=("toilet_agent_01", "toilet_agent_02")
        )
        self._selection = SimpleNamespace(scenario=scenario)
        self._finishing = False
        self._pending_parking = set()
        self._parking_inflight = set()
        self._parking_failures = []
        self._next_parking_retry_at = 0.0
        self._parking_client = _ParkingClient(self.events)
        self.recorder = _Recorder(self.events)
        self._episodes_finished = 0
        self.inter_episode_delay_sec = 1.0
        self._state = "RUNNING"
        self._state_started_at = 0.0
        self.authored_runtime_exit_timeout_sec = 60.0
        self._pedestrian_process = _Process()

    def _engage_control_hold_best_effort(self):
        self.events.append("control_held")

    def _stop_pedestrian_runtime(self):
        self.events.append("runtime_stopped")

    def _set_state(self, state):
        self._state = state

    def get_logger(self):
        return _Logger()

    def request_shutdown(self, reason):
        raise AssertionError(f"unexpected shutdown: {reason}")


def test_episode_waits_for_all_parks_after_bag_finalization():
    node = _Harness()

    ManualCollectionNode._finish_episode(node, "failed", "robot_human_collision")

    assert node.events[:3] == ["control_held", "finish_event", "bag_finalized"]
    assert node._state == "WAIT_AUTHORED_RUNTIME_EXIT"
    assert node._selection is not None
    assert node._finishing
    assert not node._parking_client.futures

    node._pedestrian_process.return_code = 0
    node._advance_authored_runtime_exit(node._state_started_at + 1.0)

    assert node._state == "BETWEEN_EPISODES"
    assert node._selection is None
    assert not node._finishing
    assert node._episodes_finished == 1


def test_authored_runtime_timeout_uses_forced_parking_fallback():
    node = _Harness()

    ManualCollectionNode._finish_episode(node, "failed", "robot_human_collision")
    node._advance_authored_runtime_exit(
        node._state_started_at + node.authored_runtime_exit_timeout_sec + 0.1
    )

    assert "runtime_stopped" in node.events
    assert node._state == "WAIT_PEDESTRIAN_PARK"
    assert len(node._parking_client.futures) == 2


def test_pending_park_response_is_retried_before_episode_cleanup():
    node = _Harness()
    node._parking_client.results = [False, True, True]

    node._start_pedestrian_parking(("toilet_agent_01", "toilet_agent_02"))
    first_batch = list(node._parking_client.futures)
    for future in first_batch:
        future.callback(future)

    assert node._state == "WAIT_PEDESTRIAN_PARK"
    assert node._pending_parking == {"toilet_agent_01"}
    assert node._selection is not None

    node._request_pending_pedestrian_parks()
    retry = node._parking_client.futures[-1]
    retry.callback(retry)

    assert node._state == "BETWEEN_EPISODES"
    assert node._selection is None


class _ActivationHarness:
    _event_payload = ManualCollectionNode._event_payload
    _active_agent_ids = ManualCollectionNode._active_agent_ids
    _pedestrian_status_cb = ManualCollectionNode._pedestrian_status_cb

    def __init__(self):
        scenario = SimpleNamespace(
            pedestrian_agent_ids=("toilet_agent_01", "toilet_agent_02"),
            source_mode="authored_route",
        )
        self._selection = SimpleNamespace(scenario=scenario)
        self._state = "WAIT_PEDESTRIAN"
        self._active_pedestrian_generations = {}
        self.release_count = 0

    def _request_control_release(self):
        self.release_count += 1

    def get_logger(self):
        return _Logger()


def test_operator_control_waits_for_every_pedestrian_generation():
    node = _ActivationHarness()

    node._pedestrian_status_cb(
        SimpleNamespace(
            data=json.dumps(
                {
                    "event": "pedestrian_active",
                    "agent_id": "toilet_agent_01",
                    "generation": 2,
                }
            )
        )
    )
    assert node.release_count == 0

    node._pedestrian_status_cb(
        SimpleNamespace(
            data=json.dumps(
                {
                    "event": "pedestrian_active",
                    "agent_id": "toilet_agent_02",
                    "generation": 5,
                }
            )
        )
    )
    assert node.release_count == 1
    assert node._active_pedestrian_generations == {
        "toilet_agent_01": 2,
        "toilet_agent_02": 5,
    }


def test_authored_runtime_detection_ignores_unrelated_nodes():
    assert _authored_runtime_nodes(
        [
            ("manual_collection_node", "/"),
            ("toilet_authored_scenario", "/"),
            ("toilet_authored_scenario", "/shadow"),
        ]
    ) == ("/shadow/toilet_authored_scenario", "/toilet_authored_scenario")
