"""What the tools need from a robot, and a stand-in that needs no ROS (spec 2026-11-10, 3). Plain Python.

A robot has
    pose() -> (x, y, yaw) in the map frame; raises RuntimeError when it is not known
    go_to(x, y, yaw, on_progress=None) -> NavResult; blocks until the navigation ends and calls
        on_progress(distance_remaining) now and then on the way
nav2_robot.Nav2Robot is the real one.
"""
from collections import namedtuple

NavResult = namedtuple('NavResult', 'status reason seconds')  # status: reached / failed / timeout / canceled


class FakeRobot:
    """Arrives at once. `results` are returned by go_to in turn before that; only 'reached' moves the robot."""

    def __init__(self, pose=(0.0, 0.0, 0.0), results=()):
        self._pose = tuple(pose)
        self.results = list(results)
        self.goals = []
        self.progress_callbacks = []
        self.pose_error = None  # set to a message to make pose() fail

    def pose(self):
        if self.pose_error:
            raise RuntimeError(self.pose_error)
        return self._pose

    def go_to(self, x, y, yaw, on_progress=None):
        self.goals.append((x, y, yaw))
        self.progress_callbacks.append(on_progress)
        result = self.results.pop(0) if self.results else NavResult('reached', '', 1.0)
        if result.status == 'reached':
            self._pose = (x, y, yaw)
        return result
