#!/usr/bin/env python3
"""Move robot_dog smoothly from its current joint positions to the standing pose.

ros2 run robot_dog_description stand_up.py --ros-args -p use_sim_time:=true
Parameters: duration (s), thigh / calf (rad) of the target pose.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# same order as joint_group_position_controller in config/ros2_controllers.yaml
JOINTS = ['%s_%s_joint' % (leg, part) for leg in ('FL', 'FR', 'RL', 'RR') for part in ('hip', 'thigh', 'calf')]


class StandUp(Node):
    def __init__(self):
        super().__init__('stand_up')
        self.declare_parameter('duration', 2.0)
        self.declare_parameter('thigh', 0.8)
        self.declare_parameter('calf', -1.5)
        self.pub = self.create_publisher(Float64MultiArray, '/joint_group_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)
        self.start = None
        self.t0 = None
        self.create_timer(0.01, self.tick)

    def on_joint_states(self, msg):
        if self.start is not None:
            return
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in JOINTS):
            self.start = [pos[j] for j in JOINTS]
            self.t0 = self.get_clock().now()
            self.get_logger().info('standing up')

    def tick(self):
        if self.start is None:
            return
        pose = {'hip': 0.0, 'thigh': self.get_parameter('thigh').value, 'calf': self.get_parameter('calf').value}
        target = [pose[j.split('_')[1]] for j in JOINTS]
        duration = max(self.get_parameter('duration').value, 1e-3)
        s = min(1.0, (self.get_clock().now() - self.t0).nanoseconds * 1e-9 / duration)
        s = s * s * (3 - 2 * s)  # smoothstep
        self.pub.publish(Float64MultiArray(data=[a + (b - a) * s for a, b in zip(self.start, target)]))


def main():
    rclpy.init()
    node = StandUp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
