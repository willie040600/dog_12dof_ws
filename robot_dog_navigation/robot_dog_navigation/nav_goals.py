"""The five nav_eval goals in the apartment and how a goal attempt is judged (spec 2026-11-03, 5). Plain Python."""
import math
import os

from scipy import ndimage

from robot_dog_navigation.map_check import load_map, reachable_free, to_cell

# (name, x, y, yaw) in the map frame = world frame = apartment - (3.5, 1.8); the dog spawns at the origin facing +x
GOALS = (
    ('living room west', -2.5, -0.3, math.pi),
    ('master bedroom', -1.5, 3.0, math.pi / 2),
    ('second bedroom', 3.0, 2.2, 0.0),
    ('dining and kitchen east', 3.5, -0.3, -math.pi / 2),
    ('back to the start', 0.0, 0.0, 0.0),
)
XY_TOLERANCE, YAW_TOLERANCE = 0.25, 0.3   # m, rad (Nav2's goal checker is a bit tighter)


def load_truth(name):
    """(grid, resolution, origin) of maps/truth/<name>.yaml, from the repo checkout or the installed share."""
    candidates = [os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps', 'truth')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('robot_dog_navigation'), 'maps', 'truth'))
    except Exception:
        pass
    for path in candidates:
        yaml_path = os.path.join(path, name + '.yaml')
        if os.path.isfile(yaml_path):
            return load_map(yaml_path)
    raise FileNotFoundError('%s.yaml not found in %s' % (name, candidates))


def clearance(truth, x, y):
    """Distance (m) from (x, y) to the nearest occupied truth cell; 0 on one or off the map."""
    grid, res, origin = truth
    r, c = to_cell(res, origin, x, y)
    if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
        return 0.0
    return float(ndimage.distance_transform_edt(grid != 100)[r, c] * res)


def reachable(truth, x, y):
    """True when (x, y) is in the free space connected to the world origin."""
    grid, res, origin = truth
    r, c = to_cell(res, origin, x, y)
    if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
        return False
    return bool(reachable_free(truth)[r, c])


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


def sideways_share(velocities, window=10, min_speed=0.1, max_angle=math.radians(20)):
    """Share of the moving time the dog travels more than max_angle off its heading (crab walk or backwards).

    velocities: (vx, vy) in base_link at 50 Hz; averaged over `window` samples first so the trot's side-to-side
    sway does not count. Windows slower than min_speed (standing, turning in place) are ignored.
    """
    means = []
    for i in range(0, len(velocities) - window + 1, window):
        chunk = velocities[i:i + window]
        means.append((sum(v[0] for v in chunk) / window, sum(v[1] for v in chunk) / window))
    moving = [(vx, vy) for vx, vy in means if math.hypot(vx, vy) >= min_speed]
    if not moving:
        return 0.0
    return sum(abs(math.atan2(vy, vx)) > max_angle for vx, vy in moving) / len(moving)
