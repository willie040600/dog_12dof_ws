"""The newest camera image, for the window to pick up (spec 2026-11-11, 5). Needs rclpy, already initialised."""
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from robot_dog_agent.frames import to_rgb


class CameraFeed:
    """Subscribes on its own node and thread; keeps only the last frame it could convert."""

    def __init__(self, topic='/camera/image_raw'):
        self._lock, self._frame = threading.Lock(), None
        self._node = rclpy.create_node('robot_dog_agent_camera',
                                       parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # the context was shut down under us while closing
            pass

    def _on_image(self, msg):
        rgb = to_rgb(msg.encoding, msg.width, msg.height, msg.step, msg.data)
        if rgb is not None:
            with self._lock:
                self._frame = (msg.width, msg.height, rgb, time.monotonic())

    def latest(self):
        """(width, height, RGB bytes, time.monotonic() when it arrived), or None before the first frame."""
        with self._lock:
            return self._frame

    def close(self):
        self._executor.shutdown(timeout_sec=1.0)
        self._thread.join(2.0)
        self._node.destroy_node()
