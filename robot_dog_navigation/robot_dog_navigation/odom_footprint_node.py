"""Split the simulator's 3D base odometry into a planar base_footprint and the body's tilt (REP-105/120).

Isaac Sim publishes odom_gt (odom -> base_link, full 3D, including the body bobbing and rolling while the
dog walks). slam_toolbox / AMCL / Nav2 assume a planar base frame, so this node publishes instead:
    odom -> base_footprint        x, y, yaw   (TF + odom, planar twist)
    base_footprint -> base_link   z, roll, pitch   (TF)
Ported from the 8-DOF dog (~/robotdog_ws/dog_gait/dog_gait/odom_footprint_node.py).
"""
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from robot_dog_navigation.footprint_math import rpy_to_quat, split_footprint


def _transform(stamp, parent, child, xyz, quat):
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = quat
    return t


class OdomFootprintNode(Node):
    def __init__(self):
        super().__init__('odom_footprint_node')
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.footprint_frame = self.declare_parameter('footprint_frame', 'base_footprint').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.tf = TransformBroadcaster(self)
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        self.create_subscription(Odometry, 'odom_gt', self.on_odom, 10)

    def on_odom(self, msg):
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        (x, y, yaw), (z, roll, pitch) = split_footprint((p.x, p.y, p.z), (o.x, o.y, o.z, o.w))
        stamp = msg.header.stamp
        q_yaw = rpy_to_quat(0.0, 0.0, yaw)
        self.tf.sendTransform([
            _transform(stamp, self.odom_frame, self.footprint_frame, (x, y, 0.0), q_yaw),
            _transform(stamp, self.footprint_frame, self.base_frame, (0.0, 0.0, z), rpy_to_quat(roll, pitch, 0.0)),
        ])
        out = Odometry()
        out.header.stamp = stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.footprint_frame
        out.pose.pose.position.x, out.pose.pose.position.y = x, y
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = q_yaw[0], q_yaw[1]
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = q_yaw[2], q_yaw[3]
        out.pose.covariance = msg.pose.covariance
        out.twist.twist.linear.x = msg.twist.twist.linear.x
        out.twist.twist.linear.y = msg.twist.twist.linear.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z
        out.twist.covariance = msg.twist.covariance
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomFootprintNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
