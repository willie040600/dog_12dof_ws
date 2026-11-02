import math
import random

from robot_dog_navigation.footprint_math import quat_to_rpy, rpy_to_quat, split_footprint


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def test_split_recomposes_original():
    rng = random.Random(0)
    for _ in range(500):
        roll, pitch, yaw = (rng.uniform(-0.5, 0.5) for _ in range(3))
        q = rpy_to_quat(roll, pitch, yaw)
        (x, y, yaw2), (z, roll2, pitch2) = split_footprint((1.0, 2.0, 0.3), q)
        recomposed = _mul(rpy_to_quat(0.0, 0.0, yaw2), rpy_to_quat(roll2, pitch2, 0.0))
        dot = abs(sum(a * b for a, b in zip(q, recomposed)))
        assert abs(dot - 1.0) < 1e-9
        assert (x, y, z) == (1.0, 2.0, 0.3)


def test_level_body_has_no_tilt():
    (_, _, yaw), (_, roll, pitch) = split_footprint((0.0, 0.0, 0.24), rpy_to_quat(0.0, 0.0, 1.2))
    assert abs(yaw - 1.2) < 1e-12 and abs(roll) < 1e-12 and abs(pitch) < 1e-12


def test_quat_to_rpy_roundtrip():
    r, p, y = quat_to_rpy(*rpy_to_quat(0.1, -0.2, 2.5))
    assert max(abs(r - 0.1), abs(p + 0.2), abs(y - 2.5)) < 1e-12
    assert abs(quat_to_rpy(*rpy_to_quat(0.0, 0.0, math.pi - 1e-9))[2]) > 3.0
