"""Which 2D-lidar returns hit the floor because the body is tilted.

Each return is moved into the fixed (level) frame with the laser's pose there; returns lower than
min_height are floor hits. Walls and obstacles stay, however the body pitches.
"""
import numpy as np


def rotate(q, v):
    """Rotate vectors v (N, 3) by quaternion q = (x, y, z, w)."""
    u = np.array(q[:3], dtype=float)
    t = 2.0 * np.cross(u, v)
    return v + q[3] * t + np.cross(u, t)


def floor_mask(ranges, angle_min, angle_increment, range_min, laser_quat_xyzw, laser_z, min_height=0.05):
    r = np.asarray(ranges, dtype=float)
    a = angle_min + angle_increment * np.arange(len(r))
    finite = np.isfinite(r) & (r > range_min)
    rr = np.where(finite, r, 0.0)
    pts = np.stack([rr * np.cos(a), rr * np.sin(a), np.zeros_like(rr)], axis=1)
    z = rotate(laser_quat_xyzw, pts)[:, 2] + laser_z
    return finite & (z < min_height)
