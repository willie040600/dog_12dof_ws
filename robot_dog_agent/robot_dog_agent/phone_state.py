"""What the phone app is shown: numbered events, the path and target on the map, and /api/state (spec 2026-11-19, 4).

Plain Python (numpy and OpenCV for the map picture): no network, no ROS, no Qt.
"""
import threading
import time

import cv2
import numpy as np

from robot_dog_agent.map_view import grid_to_gray
from robot_dog_agent.session import pose_text

MAX_EVENTS = 300       # events kept for a phone that connects later
MAX_PATH_POINTS = 300  # points of a path sent to the phone
TARGET_HOLD = 2.0      # s the clicked target stays after its command ended (as in the window)
DECIMALS = 3           # mm: enough for a map with 5 cm cells, and short JSON


class EventLog:
    """Session events, numbered from 1, that phones read with a long poll."""

    def __init__(self, max_events=MAX_EVENTS):
        self.max_events = max_events
        self._events, self._last, self._closed = [], 0, False
        self._changed = threading.Condition()

    def add(self, kind, text):
        with self._changed:
            self._last += 1
            self._events.append({'id': self._last, 'kind': kind, 'text': text})
            del self._events[:-self.max_events]
            self._changed.notify_all()
            return self._last

    def after(self, n, timeout):
        """(events newer than n, the number to ask after next), waiting up to timeout s for the first one.
        An n beyond the last event (counted before the server restarted) is answered at once with the last number."""
        with self._changed:
            self._changed.wait_for(lambda: self._closed or self._last != n, timeout)
            if self._closed:
                return [], n
            return [dict(e) for e in self._events if e['id'] > n], self._last

    def close(self):
        """Wake every waiter; from now on after() returns at once."""
        with self._changed:
            self._closed = True
            self._changed.notify_all()


def _point(x, y):
    return [round(float(x), DECIMALS), round(float(y), DECIMALS)]


def sample(points, most=MAX_PATH_POINTS):
    """At most `most` points spread evenly over points, the first and the last kept."""
    if len(points) <= most:
        return list(points)
    return [points[i] for i in np.linspace(0, len(points) - 1, most).round().astype(int)]


class MapTracker:
    """The path and the clicked target to draw, by the same rules as the window: only the running command's path;
    the target stays TARGET_HOLD s after its command and goes when the next one starts. Thread-safe."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self._lock = threading.Lock()
        self._busy_since, self._target, self._target_until = None, None, None
        self._path, self._version = [], 0

    def on_state(self, state):
        with self._lock:
            if state == 'busy':
                self._busy_since = self.clock()
                if self._target_until is not None:  # the last command's target: not where this one goes
                    self._target, self._target_until = None, None
            elif self._busy_since is not None:
                self._busy_since = None
                if self._target is not None:
                    self._target_until = self.clock() + TARGET_HOLD

    def set_target(self, x, y):
        with self._lock:
            self._target, self._target_until = _point(x, y), None

    def clear_target(self):
        with self._lock:
            self._target, self._target_until = None, None

    def snapshot(self, latest_path):
        """(path version, path [[x, y], ...], target [x, y] or None); latest_path is MapFeed.latest_path()."""
        with self._lock:
            path = []
            fresh = self._busy_since is not None and latest_path is not None and latest_path[1] >= self._busy_since
            if fresh:
                path = [_point(x, y) for x, y in sample(latest_path[0])]
            if path != self._path:
                self._path, self._version = path, self._version + 1
            if self._target_until is not None and self.clock() > self._target_until:
                self._target, self._target_until = None, None
            return self._version, self._path, self._target


def state_json(session, tracker, latest_path, map_info, objects, path_version_seen):
    """The body of /api/state (spec 2026-11-19, 4); the path only when its version is not path_version_seen."""
    pose = session.pose()
    version, path, target = tracker.snapshot(latest_path)
    return {
        'state': session.state,
        'status': session.status_text(),
        'pose': None if pose is None else [round(float(v), DECIMALS) for v in pose],
        'pose_text': pose_text(pose),
        'path_version': version,
        'path': path if version != path_version_seen else None,
        'target': target,
        'objects': objects,
        'map': map_info,
    }


def grid_png(grid):
    """The map as a grey PNG, one pixel per cell, the top line at the largest y."""
    height, width = np.asarray(grid).shape
    gray = np.frombuffer(grid_to_gray(grid), dtype=np.uint8).reshape(height, width)
    ok, data = cv2.imencode('.png', gray)
    if not ok:
        raise ValueError('could not encode the map')
    return data.tobytes()


def read_objects(objects, last):
    """objects(), tried twice: the survey thread may change the memory while it is read. Else last."""
    for _ in range(2):
        try:
            return objects()
        except RuntimeError:
            pass
    return last
