"""Drop lidar returns that hit the floor because the walking body is tilted; republish on scan_filtered.

The 2D lidar sits ~0.34 m up on a body that pitches a few degrees while walking, so part of a scan lands
on the floor a few metres ahead; slam_toolbox maps those points and the costmaps mark them as obstacles.
Each return is moved into the fixed frame with TF (odom -> laser_link, which includes the tilt); returns
lower than min_height become +inf. Ported from the 8-DOF dog (scan_ground_filter_node.py).
"""
import math

import numpy as np
import rclpy
import rclpy.parameter
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from robot_dog_navigation.ground_filter_math import floor_mask


class ScanGroundFilter(Node):
    def __init__(self):
        super().__init__('scan_ground_filter')
        self.fixed_frame = self.declare_parameter('fixed_frame', 'odom').value
        self.min_height = self.declare_parameter('min_height', 0.05).value
        self.buf = Buffer()
        # TF gets its own node and thread (TF arrives while a scan is handled). Giving TransformListener this
        # node with spin_thread=True put one node in two executors and crashed (segfault) intermittently.
        self.tf_node = rclpy.create_node('scan_ground_filter_tf', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=self.get_parameter('use_sim_time').value)])
        self.tl = TransformListener(self.buf, self.tf_node, spin_thread=True)
        self.pub = self.create_publisher(LaserScan, 'scan_filtered', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.dropped = 0
        self.create_timer(10.0, self.report)

    def report(self):
        if self.dropped:
            self.get_logger().info(f'removed {self.dropped} floor returns in the last 10 s')
        self.dropped = 0

    def on_scan(self, msg):
        try:
            tf = self.buf.lookup_transform(self.fixed_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp))
        except TransformException:
            try:
                tf = self.buf.lookup_transform(self.fixed_frame, msg.header.frame_id, Time())
            except TransformException:
                return
        q = tf.transform.rotation
        floor = floor_mask(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min,
                           (q.x, q.y, q.z, q.w), tf.transform.translation.z, self.min_height)
        self.dropped += int(floor.sum())
        out = LaserScan()
        out.header = msg.header
        out.angle_min, out.angle_max, out.angle_increment = msg.angle_min, msg.angle_max, msg.angle_increment
        out.time_increment, out.scan_time = msg.time_increment, msg.scan_time
        out.range_min, out.range_max = msg.range_min, msg.range_max
        out.ranges = np.where(floor, math.inf, np.asarray(msg.ranges, dtype=float)).astype(np.float32).tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanGroundFilter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.tl.unregister()  # stop the TF spin thread before rclpy shuts down (else a traceback at exit)
        node.tf_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
