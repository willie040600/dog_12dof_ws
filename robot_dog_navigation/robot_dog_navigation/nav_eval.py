"""Send the five nav_goals to Nav2 one after another and judge each with ground truth (spec M5).

    ros2 run robot_dog_navigation nav_eval [--slam] [--timeout 120] [--extra-goal x,y,yaw]
Prints one [NAV] line per goal and [NAV] PASS k/5; exit code 0 only when all five pass. --extra-goal adds a
sixth goal after them whose result is printed but does not change the exit code (to check that an
unreachable goal is given up on instead of hanging).
"""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_dog_navigation.nav_goals import GOALS, judge, sideways_share

FALL_HEIGHT = 0.12  # m, base height below which the dog has fallen


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def make_pose(nav, x, y, yaw, cls=PoseStamped):
    msg = cls()
    msg.header.frame_id = 'map'
    msg.header.stamp = nav.get_clock().now().to_msg()
    pose = msg.pose.pose if cls is PoseWithCovarianceStamped else msg.pose
    pose.position.x, pose.position.y = float(x), float(y)
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return msg


class Truth:
    """Latest ground-truth pose from Isaac Sim's /odom_gt (x, y, z, yaw), and every (vx, vy) in base_link."""

    def __init__(self, node):
        self.pose = None
        self.velocities = []
        node.create_subscription(Odometry, '/odom_gt', self._on, 10)

    def _on(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
        self.velocities.append((m.twist.twist.linear.x, m.twist.twist.linear.y))


def sim_seconds(nav):
    return nav.get_clock().now().nanoseconds * 1e-9


def wait_sim(nav, seconds):
    end = sim_seconds(nav) + seconds
    while sim_seconds(nav) < end:
        rclpy.spin_once(nav, timeout_sec=0.05)


def check_localization(nav, truth, buf):
    """AMCL mode: map->base_footprint must start at the true pose; re-seed it at the origin when it does not."""
    for attempt in range(2):
        tf = None
        deadline = time.monotonic() + 5.0  # wall clock: spin_once returns after any callback (/clock, /odom_gt)
        while time.monotonic() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.05)
            try:
                tf = buf.lookup_transform('map', 'base_footprint', Time())
                break
            except TransformException:
                pass
        if tf is None or truth.pose is None:
            print('[NAV] no map->base_footprint transform or /odom_gt yet', flush=True)
            return False
        t = tf.transform.translation
        err = math.hypot(t.x - truth.pose[0], t.y - truth.pose[1])
        print('[NAV] initial localization error %.3f m' % err, flush=True)
        if err <= 0.1:
            return True
        if attempt == 0:
            nav.setInitialPose(make_pose(nav, truth.pose[0], truth.pose[1], truth.pose[3]))
            wait_sim(nav, 2.0)
    return False


def localization_error(buf, truth):
    """Distance between map->base_footprint (SLAM/AMCL estimate) and ground truth, or nan."""
    try:
        t = buf.lookup_transform('map', 'base_footprint', Time()).transform.translation
    except TransformException:
        return math.nan
    return math.hypot(t.x - truth.pose[0], t.y - truth.pose[1])


def run_goal(nav, truth, goal, timeout, buf):
    name, x, y, yaw = goal
    truth.velocities = []
    if not nav.goToPose(make_pose(nav, x, y, yaw)):
        # a rejected goal leaves BasicNavigator's result future on the previous goal, so do not poll it
        print('[NAV] %-34s FAIL goal rejected by Nav2' % name, flush=True)
        return False, 0.0
    t0, fell, timed_out = sim_seconds(nav), False, False
    while not nav.isTaskComplete():
        if truth.pose is not None and truth.pose[2] < FALL_HEIGHT and not fell:
            fell = True
            nav.cancelTask()
        if sim_seconds(nav) - t0 > timeout and not timed_out:
            timed_out = True
            nav.cancelTask()
    elapsed = sim_seconds(nav) - t0
    status = nav.getResult().name  # SUCCEEDED / CANCELED / FAILED / UNKNOWN
    final = (truth.pose[0], truth.pose[1], truth.pose[3]) if truth.pose else (math.nan, math.nan, math.nan)
    ok, why = judge(goal, final, status, fell=fell, elapsed=elapsed, timeout=timeout)
    side = sideways_share(truth.velocities)
    print('[NAV] %-34s %s t=%5.1f s  %s  (localization error %.3f m, sideways %d%%)' % (
        name, 'PASS' if ok else 'FAIL', elapsed, why, localization_error(buf, truth), round(100 * side)), flush=True)
    return ok, side


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--slam', action='store_true', help='slam_toolbox instead of AMCL is running')
    parser.add_argument('--timeout', type=float, default=120.0, help='sim seconds per goal')
    parser.add_argument('--extra-goal', help='x,y,yaw of an additional goal after the five')
    args, _ = parser.parse_known_args()
    rclpy.init()
    nav = BasicNavigator()
    nav.set_parameters([Parameter('use_sim_time', value=True)])
    truth = Truth(nav)
    buf = Buffer()
    # own node for TF: BasicNavigator spins `nav` itself, so a listener thread on `nav` would put it in two executors
    tf_node = rclpy.create_node('nav_eval_tf', parameter_overrides=[Parameter('use_sim_time', value=True)])
    listener = TransformListener(buf, tf_node, spin_thread=True)
    nav.waitUntilNav2Active(localizer='robot_localization' if args.slam else 'amcl')
    while truth.pose is None:
        rclpy.spin_once(nav, timeout_sec=0.1)
    try:
        if not args.slam and not check_localization(nav, truth, buf):
            print('[NAV] FAIL: AMCL is not localized at the start', flush=True)
            return 1
        results = [run_goal(nav, truth, g, args.timeout, buf) for g in GOALS]
        passed = sum(ok for ok, _ in results)
        print('[NAV] PASS %d/%d  (sideways %d%% of the moving time on average)' % (
            passed, len(GOALS), round(100 * sum(side for _, side in results) / len(results))), flush=True)
        if args.extra_goal:
            x, y, yaw = (float(v) for v in args.extra_goal.split(','))
            run_goal(nav, truth, ('extra goal (not scored)', x, y, yaw), args.timeout, buf)
        return 0 if passed == len(GOALS) else 1
    finally:
        listener.unregister()  # stop its spin thread before rclpy shuts down (else a traceback at exit)
        tf_node.destroy_node()
        nav.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
