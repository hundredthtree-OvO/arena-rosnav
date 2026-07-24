import signal
import unittest

from toilet_benchmark.manual_collection_node import _ShutdownSignalLatch, _build_director_command


class TestShutdownSignalLatch(unittest.TestCase):
    def test_director_command_carries_count_character_pool_and_all_targets(self):
        command = _build_director_command(
            semantics_path="/tmp/semantics.yaml",
            benchmark_path="/tmp/benchmark.yaml",
            pedestrian_count=3,
            character_name="fallback_character",
            character_pool=("character_a", "character_b"),
            target_resource_ids=("urinal_3", "urinal_5"),
        )

        self.assertEqual(command[command.index("--initial-agents") + 1], "3")
        self.assertEqual(command[command.index("--character-pool") + 1], "character_a,character_b")
        self.assertEqual(
            [command[index + 1] for index, value in enumerate(command) if value == "--target-resource"],
            ["urinal_3", "urinal_5"],
        )

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
