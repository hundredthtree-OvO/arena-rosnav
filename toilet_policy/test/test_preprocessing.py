import math
from types import SimpleNamespace

import numpy as np

from toilet_policy.preprocessing import normalize_scan, quaternion_yaw


def test_normalize_scan_matches_training_contract():
    scan = SimpleNamespace(ranges=[0.0, 5.0, float("inf")], range_min=0.0, range_max=10.0)
    result = normalize_scan(scan, 3)
    np.testing.assert_allclose(result, [0.0, 0.5, 1.0])


def test_normalize_scan_resamples_beam_count():
    scan = SimpleNamespace(ranges=[0.0, 10.0], range_min=0.0, range_max=10.0)
    np.testing.assert_allclose(normalize_scan(scan, 3), [0.0, 0.5, 1.0])


def test_quaternion_yaw():
    half = math.pi / 4.0
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))
    assert math.isclose(quaternion_yaw(quaternion), math.pi / 2.0)
