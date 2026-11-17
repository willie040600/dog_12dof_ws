"""The newest map and Nav2 path, for the window to pick up (spec 2026-11-17, 5). Needs rclpy, already initialised."""
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class MapFeed:
    """Subscribes to /map and /plan on its own node and thread; keeps only the latest of each."""

    def __init__(self, map_topic='/map', path_topic='/plan'):
        self._lock = threading.Lock()
        self._map, self._path, self._version = None, None, 0
        self._node = rclpy.create_node('robot_dog_agent_map',
                                       parameter_overrides=[Parameter('use_sim_time', value=True)])
        # transient local: the map was published once, before this window started
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._node.create_subscription(OccupancyGrid, map_topic, self._on_map, latched)
        self._node.create_subscription(Path, path_topic, self._on_path, 1)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # the context was shut down under us while closing
            pass

    def _on_map(self, msg):
        grid = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        with self._lock:
            self._version += 1
            self._map = (grid, msg.info.resolution, origin, self._version)

    def _on_path(self, msg):
        points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        with self._lock:
            self._path = (points, time.monotonic())

    def latest_map(self):
        """(grid [rows, cols] int8, resolution, (origin_x, origin_y), version) or None; version counts maps from 1."""
        with self._lock:
            return self._map

    def latest_path(self):
        """([(x, y), ...], time.monotonic() when it arrived) or None."""
        with self._lock:
            return self._path

    def close(self):
        """Safe to call after rclpy has been shut down (the robot's close does that)."""
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self._thread.join(2.0)
            self._node.destroy_node()
        except Exception:
            pass
