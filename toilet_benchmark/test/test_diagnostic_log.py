import json

from toilet_benchmark.diagnostic_log import JsonlDiagnosticLog


def test_jsonl_diagnostic_log_flushes_events_and_updates_latest(tmp_path) -> None:
    path = tmp_path / "run.jsonl"
    log = JsonlDiagnosticLog(path)

    log.emit("activation_wait", agent_id="agent_01", reasons=["reactivation_not_ready"])
    payload = json.loads(path.read_text(encoding="utf-8"))

    assert payload["event"] == "activation_wait"
    assert payload["agent_id"] == "agent_01"
    assert payload["reasons"] == ["reactivation_not_ready"]
    assert (tmp_path / "latest.jsonl").resolve() == path.resolve()
    log.close()


def test_jsonl_diagnostic_log_open_failure_is_nonfatal(tmp_path) -> None:
    blocker = tmp_path / "not-a-directory"
    blocker.write_text("occupied", encoding="utf-8")

    log = JsonlDiagnosticLog(blocker / "run.jsonl")

    assert not log.enabled
    assert log.error
    log.emit("ignored")
