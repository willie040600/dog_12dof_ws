"""ROS side of the Isaac Sim navigation mode: robot_state_publisher, odom_footprint, scan_ground_filter.

Start robot_dog_description/isaac/robot_dog_nav_sim.py first (it publishes /clock, /joint_states, /odom_gt,
/scan, /imu, /camera/*), then:  ros2 launch robot_dog_navigation bridge.launch.py
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('robot_dog_description'), 'urdf', 'robot_dog.urdf.xacro')
    robot_description = xacro.process_file(urdf, mappings={'sim_mode': 'none'}).toxml()
    sim_time = {'use_sim_time': True}
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim_time]),
        Node(package='robot_dog_navigation', executable='odom_footprint_node', parameters=[sim_time]),
        Node(package='robot_dog_navigation', executable='scan_ground_filter_node', parameters=[sim_time]),
    ])
