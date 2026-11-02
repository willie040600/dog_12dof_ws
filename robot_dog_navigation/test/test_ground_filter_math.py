import math

import numpy as np

from robot_dog_navigation.footprint_math import rpy_to_quat
from robot_dog_navigation.ground_filter_math import floor_mask

LASER_Z = 0.34
N = 360
ANGLE_MIN, INC = -math.pi, 2 * math.pi / N
FRONT, BACK = N // 2, 0  # beam 180 points along +x, beam 0 along -x


def _mask(ranges, pitch=0.0):
    return floor_mask(np.asarray(ranges, dtype=float), ANGLE_MIN, INC, 0.05, rpy_to_quat(0.0, pitch, 0.0), LASER_Z)


def test_level_scan_drops_nothing():
    assert not _mask([5.0] * N).any()


def test_pitched_scan_drops_floor_hits():
    ranges = [10.0] * N
    m = _mask(ranges, pitch=math.radians(5))  # nose down
    assert m[FRONT] and not m[BACK]


def test_wall_is_kept_when_pitched():
    ranges = [1.0] * N  # 1 m ahead is ~0.25 m above the floor even with the nose 5 deg down
    assert not _mask(ranges, pitch=math.radians(5))[FRONT]


def test_inf_and_short_ranges_ignored():
    ranges = [math.inf] * N
    ranges[FRONT] = 0.01  # below range_min
    assert not _mask(ranges, pitch=math.radians(10)).any()
