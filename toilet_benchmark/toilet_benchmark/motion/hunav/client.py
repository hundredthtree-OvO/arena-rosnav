"""Thin owner for HuNav service endpoints."""

from __future__ import annotations

from hunav_msgs.srv import ComputeAgents, ResetAgents


class HuNavServiceClient:
    """Keep service naming and readiness checks out of backend state logic."""

    def __init__(self, node, namespace: str):
        normalized = "/" + str(namespace).strip().strip("/")
        self.compute = node.create_client(
            ComputeAgents,
            f"{normalized}/compute_agents",
        )
        self.reset = node.create_client(
            ResetAgents,
            f"{normalized}/reset_agents",
        )

    def wait_for_service(self, timeout_sec: float) -> bool:
        timeout = float(timeout_sec)
        return bool(
            self.compute.wait_for_service(timeout_sec=timeout)
            and self.reset.wait_for_service(timeout_sec=timeout)
        )
