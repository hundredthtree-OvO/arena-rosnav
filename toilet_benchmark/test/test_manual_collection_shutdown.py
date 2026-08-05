import signal
import unittest

from toilet_benchmark.manual_collection_node import (
    _ShutdownSignalLatch,
    _build_authored_scenario_command,
)


class TestShutdownSignalLatch(unittest.TestCase):
    def test_authored_command_does_not_depend_on_director(self):
        command = _build_authored_scenario_command(episode_path="/tmp/episode.json")

        self.assertIn("toilet_benchmark.tracks.authored_scenario", command)
        self.assertIn("--skip-robot-reset", command)
        self.assertIn("--pedestrian-robot-policy", command)
        self.assertIn("detect_and_fail", command)
        self.assertNotIn("toilet_benchmark.toilet_director_node", command)

    def test_authored_command_can_request_a_fresh_incarnation(self):
        command = _build_authored_scenario_command(
            episode_path="/tmp/episode.json",
            agent_id_suffix="__inc003",
        )

        self.assertEqual(command[-2:], ["--agent-id-suffix", "__inc003"])

    def test_sigint_requests_operator_interrupt_without_raising(self):
        latch = _ShutdownSignalLatch()

        latch.handle(signal.SIGINT, None)

        self.assertEqual(latch.reason, "operator_interrupt")

    def test_first_shutdown_signal_wins(self):
        latch = _ShutdownSignalLatch()

        latch.handle(signal.SIGTERM, None)
        latch.handle(signal.SIGINT, None)

        self.assertEqual(latch.reason, "termination_signal")


if __name__ == "__main__":
    unittest.main()
