"""What the tools need from a robot, and a stand-in that needs no ROS (spec 2026-11-10, 3). Plain Python.

A robot has
    pose() -> (x, y, yaw) in the map frame; raises RuntimeError when it is not known
    go_to(x, y, yaw, on_progress=None) -> NavResult; blocks until the navigation ends and calls
        on_progress(distance_remaining) now and then on the way
    cancel(); from any thread: ends the navigation under way as 'canceled', or, when there is none, makes the
        next go_to return 'canceled' without moving. One cancel ends one navigation.
    clear_cancel(); forgets a cancel that has not been used
    close()
nav2_robot.Nav2Robot is the real one.
"""
import math
from collections import namedtuple

NavResult = namedtuple('NavResult', 'status reason seconds')  # status: reached / failed / timeout / canceled
SAME_SPOT = 0.1  # m; a target this close to the robot is where it already stands
CANCELED = 'canceled by the user'


def turn_in_place(pose, x, y, yaw):
    """The signed angle (rad, the short way round) to turn when (x, y) is where the robot stands, else None."""
    if math.hypot(x - pose[0], y - pose[1]) >= SAME_SPOT:
        return None
    return math.pi - (math.pi - (yaw - pose[2])) % (2.0 * math.pi)


class FakeRobot:
    """Arrives at once. `results` are returned by go_to in turn before that; only 'reached' moves the robot."""

    def __init__(self, pose=(0.0, 0.0, 0.0), results=()):
        self._pose = tuple(pose)
        self.results = list(results)
        self.goals = []
        self.progress_callbacks = []
        self.pose_error = None  # set to a message to make pose() fail
        self.closed = False
        self._cancel = False

    def pose(self, wait=None):
        if self.pose_error:
            raise RuntimeError(self.pose_error)
        return self._pose

    def cancel(self):
        self._cancel = True

    def clear_cancel(self):
        self._cancel = False

    def close(self):
        self.closed = True

    def go_to(self, x, y, yaw, on_progress=None):
        if self._cancel:
            self._cancel = False
            return NavResult('canceled', CANCELED, 0.0)
        self.goals.append((x, y, yaw))
        self.progress_callbacks.append(on_progress)
        result = self.results.pop(0) if self.results else NavResult('reached', '', 1.0)
        if result.status == 'reached':
            self._pose = (x, y, yaw)
        return result
