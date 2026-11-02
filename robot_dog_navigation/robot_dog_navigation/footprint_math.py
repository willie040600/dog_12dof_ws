"""Split a 3D base pose into a planar base_footprint and the body's height and tilt (REP-105/120).

Quaternions are (x, y, z, w) like ROS messages; Euler angles are ZYX (yaw, pitch, roll).
"""
import math


def quat_to_rpy(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def split_footprint(position, quat_xyzw):
    """((x, y, yaw) of odom -> base_footprint, (z, roll, pitch) of base_footprint -> base_link)."""
    roll, pitch, yaw = quat_to_rpy(*quat_xyzw)
    return (position[0], position[1], yaw), (position[2], roll, pitch)
