import signal
import unittest

from toilet_benchmark.manual_collection_node import _ShutdownSignalLatch


class TestShutdownSignalLatch(unittest.TestCase):
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
