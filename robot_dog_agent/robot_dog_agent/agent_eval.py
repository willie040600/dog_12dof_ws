"""Six spoken commands through Gemini and Nav2, judged with ground truth (spec 2026-11-10, 7).

    ros2 run robot_dog_agent agent_eval [--model gemini-3.5-flash-lite]
Needs GEMINI_API_KEY and a freshly started Isaac Sim + nav.launch.py in AMCL mode (the dog at the origin).
Prints one [AGENT] line per sentence and [AGENT] PASS k/6; exit code 0 only when all six pass.
"""
import argparse
import functools
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry

from robot_dog_agent import chat
from robot_dog_agent.eval_cases import CASES, WatchedRobot, run_cases
from robot_dog_agent.nav2_robot import Nav2Robot
from robot_dog_navigation.nav_eval import FALL_HEIGHT, check_localization, yaw_of

SETTLE = 0.5  # s of wall time spun before a pose is read, so the newest /odom_gt message is in


class GroundTruth:
    """The pose from Isaac Sim's /odom_gt and the lowest base height seen since the last fell()."""

    def __init__(self, node):
        self.node, self.pose, self.lowest = node, None, math.inf  # pose: (x, y, z, yaw), as nav_eval.Truth
        node.create_subscription(Odometry, '/odom_gt', self._on, 10)

    def _on(self, m):
        p = m.pose.pose
        self.pose = (p.position.x, p.position.y, p.position.z, yaw_of(p.orientation))
        self.lowest = min(self.lowest, p.position.z)

    def xy_yaw(self):
        end = time.monotonic() + SETTLE
        while time.monotonic() < end or self.pose is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.pose[0], self.pose[1], self.pose[3]

    def fell(self):
        self.xy_yaw()
        lowest, self.lowest = self.lowest, math.inf
        return lowest < FALL_HEIGHT


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default=chat.DEFAULT_MODEL, help='Gemini model id')
    args, _ = parser.parse_known_args(argv)
    api_key = chat.usable_key(os.environ.get('GEMINI_API_KEY'))
    if not api_key:
        print(chat.BAD_KEY if os.environ.get('GEMINI_API_KEY', '').strip() else chat.NO_KEY)
        return 2
    say = functools.partial(print, flush=True)
    try:
        robot = Nav2Robot()
    except RuntimeError as e:
        say('[AGENT] FAIL: the robot is not localized (%s)' % e)
        return 1
    except KeyboardInterrupt:
        return 130
    try:
        truth = GroundTruth(robot.node)
        truth.xy_yaw()
        if not check_localization(robot.node, truth, robot.tf_buffer):
            say('[AGENT] FAIL: AMCL is not localized at the start')
            return 1
        watched = WatchedRobot(robot, truth.xy_yaw)
        agent = chat.build_agent(watched, api_key, args.model, say=say, on_call=watched.on_call)
        passed = run_cases(agent.ask, watched, truth.xy_yaw, truth.fell, say=say)
        return 0 if passed == len(CASES) else 1
    finally:
        robot.close()


if __name__ == '__main__':
    sys.exit(main())
