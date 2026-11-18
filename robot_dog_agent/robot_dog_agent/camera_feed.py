"""The newest camera image, for the window to pick up (spec 2026-11-11, 5). Needs rclpy, already initialised."""
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from robot_dog_agent.frames import to_rgb
from robot_dog_agent.vision import Snapshot, depth_from_msg, transform_matrix

DEPTHS_KEPT = 5       # recent depth images, to pair with a colour image of the same time
PAIR_TOLERANCE = 0.05  # s between the stamps of a colour and a depth image that still make a pair


class CameraFeed:
    """Subscribes on its own node and thread; keeps only the last frame it could convert."""

    def __init__(self, topic='/camera/image_raw', depth_topic='/camera/depth/image_raw',
                 info_topic='/camera/camera_info'):
        self._lock, self._frame = threading.Lock(), None
        self._stamp, self._depths, self._K = None, [], None  # stamp of _frame (sim seconds); [(stamp, depth)]
        self._node = rclpy.create_node('robot_dog_agent_camera',
                                       parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._node.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self._node.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self._node.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
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
                self._stamp = msg.header.stamp

    def _on_depth(self, msg):
        depth = depth_from_msg(msg.encoding, msg.width, msg.height, msg.step, msg.data)
        if depth is not None:
            with self._lock:
                self._depths = (self._depths + [(msg.header.stamp, depth)])[-DEPTHS_KEPT:]

    def _on_info(self, msg):
        with self._lock:
            self._K = np.array(msg.k, dtype=float).reshape(3, 3)

    def snapshot(self, tf_buffer):
        """vision.Snapshot of the newest colour image with its depth image and camera pose, or None."""
        with self._lock:
            frame, stamp, depths, K = self._frame, self._stamp, list(self._depths), self._K
        if frame is None or K is None or not depths:
            return None
        seconds = lambda s: s.sec + s.nanosec * 1e-9  # noqa: E731
        gap, depth = min(((abs(seconds(s) - seconds(stamp)), d) for s, d in depths), key=lambda p: p[0])
        if gap > PAIR_TOLERANCE:
            return None
        width, height, rgb, _ = frame
        if depth.shape != (height, width):
            return None
        try:  # the camera pose when the picture was taken, else the newest
            tf = tf_buffer.lookup_transform('map', 'camera_optical_link', Time.from_msg(stamp)).transform
        except TransformException:
            try:
                tf = tf_buffer.lookup_transform('map', 'camera_optical_link', Time()).transform
            except TransformException:
                return None
        t, q = tf.translation, tf.rotation
        return Snapshot(rgb, width, height, depth, K, transform_matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w)))

    def latest(self):
        """(width, height, RGB bytes, time.monotonic() when it arrived), or None before the first frame."""
        with self._lock:
            return self._frame

    def close(self):
        """Safe to call after rclpy has been shut down (the robot's close does that)."""
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self._thread.join(2.0)
            self._node.destroy_node()
        except Exception:
            pass
