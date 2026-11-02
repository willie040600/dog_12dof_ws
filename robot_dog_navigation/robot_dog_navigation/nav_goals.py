"""The five nav_eval goals and how a goal attempt is judged (spec M5). Plain Python."""
import math
import os
import sys

# (name, x, y, yaw) in the map frame = world frame (the dog spawns at the origin facing +x)
GOALS = (
    ('centre north-east', 1.0, 2.0, 0.0),
    ('east zone via the south doorway', 3.0, -3.0, 0.0),
    ('east zone north', 3.5, 3.8, math.pi / 2),
    ('west zone via the north doorway', -3.0, 3.2, math.pi),
    ('west zone south', -2.6, -4.0, -math.pi / 2),
)
XY_TOLERANCE, YAW_TOLERANCE = 0.25, 0.3   # m, rad (Nav2's goal checker is a bit tighter)


def room_layout():
    """robot_dog_sim.room_layout from the repo checkout, or from robot_dog_description's installed share."""
    candidates = [os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'robot_dog_description', 'isaac')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('robot_dog_description'), 'isaac'))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(os.path.join(path, 'robot_dog_sim', 'room_layout.py')):
            if path not in sys.path:
                sys.path.insert(0, path)
            from robot_dog_sim import room_layout as layout
            return layout
    raise ImportError('robot_dog_sim/room_layout.py not found in %s' % candidates)


def pose_error(goal, x, y, yaw):
    _, gx, gy, gyaw = goal
    d = math.atan2(math.sin(yaw - gyaw), math.cos(yaw - gyaw))
    return math.hypot(x - gx, y - gy), abs(d)


def judge(goal, final_pose, status, fell, elapsed, timeout=120.0):
    """(passed, reason); final_pose = (x, y, yaw) from ground truth."""
    if fell:
        return False, 'fall during the attempt'
    if elapsed > timeout:
        return False, 'timeout after %.0f s' % elapsed
    if status != 'SUCCEEDED':
        return False, 'Nav2 status %s' % status
    dist, yaw_err = pose_error(goal, *final_pose)
    if dist > XY_TOLERANCE:
        return False, 'distance %.2f m > %.2f' % (dist, XY_TOLERANCE)
    if yaw_err > YAW_TOLERANCE:
        return False, 'yaw error %.2f rad > %.2f' % (yaw_err, YAW_TOLERANCE)
    return True, 'distance %.2f m, yaw error %.2f rad' % (dist, yaw_err)
