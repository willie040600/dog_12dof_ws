"""ROS 2 side of the Isaac Sim setup: ros2_control over joint state topics.

Start Isaac Sim with isaac/robot_dog_isaac.py first (it publishes /clock,
/isaac_joint_states, /imu, /scan, /camera/* and subscribes /isaac_joint_commands).
  ros2 launch robot_dog_description isaac.launch.py [command:=effort] [rviz:=true]
command must match the --effort flag given to robot_dog_isaac.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    pkg_share = get_package_share_directory('robot_dog_description')
    command = LaunchConfiguration('command').perform(context)
    if command not in ('position', 'effort'):
        raise RuntimeError("command must be 'position' or 'effort', got %r" % command)
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_dog.urdf.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim_mode:=isaac isaac_command:=', command]), value_type=str)
    controller = 'joint_group_%s_controller' % command

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[os.path.join(pkg_share, 'config', 'ros2_controllers.yaml'), {'use_sim_time': True}],
             remappings=[('~/robot_description', '/robot_description')]),
        Node(package='controller_manager', executable='spawner', output='screen',
             arguments=['joint_state_broadcaster', controller]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_dog.rviz')],
             parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('command', default_value='position',
                              description="'position' (Isaac PD drives) or 'effort' (torques, your own PD)"),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
