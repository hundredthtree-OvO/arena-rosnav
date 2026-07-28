import json
from pathlib import Path
import tempfile
import unittest

from toilet_benchmark.walkable_map import (
    cell_for_world,
    load_validation_anchors,
    load_walkable_map,
    value_at_world,
)


class TestWalkableMap(unittest.TestCase):
    def _payload(self):
        return {
            "schema": "arena.walkable_map.v1",
            "resolution": 0.5,
            "origin": [-1.0, -1.0, 0.0],
            "width": 3,
            "height": 2,
            "data": [0, 100, -1, 0, 0, 100],
        }

    def test_world_lookup_uses_ros_bottom_left_row_order(self):
        payload = self._payload()
        self.assertEqual(cell_for_world(payload, -0.75, -0.75), (0, 0))
        self.assertEqual(value_at_world(payload, -0.25, -0.75), 100)
        self.assertEqual(value_at_world(payload, -0.75, -0.25), 0)
        self.assertIsNone(value_at_world(payload, 2.0, 2.0))

    def test_loader_rejects_wrong_data_size(self):
        payload = self._payload()
        payload["data"] = [0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "map.json"
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(ValueError):
                load_walkable_map(path)

    def test_validation_anchor_loader(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "anchors.yaml"
            path.write_text(
                "anchors:\n  - id: entrance\n    pose: [1.0, 2.0]\n",
                encoding="utf-8",
            )
            anchors = load_validation_anchors(path)

        self.assertEqual(
            anchors,
            [{"id": "entrance", "pose": [1.0, 2.0], "expected": "free"}],
        )


if __name__ == "__main__":
    unittest.main()
