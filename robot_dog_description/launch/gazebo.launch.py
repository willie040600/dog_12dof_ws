"""Gazebo Harmonic + gz_ros2_control + sensor bridge for robot_dog (ROS 2 Jazzy)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_dog_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot_dog.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' sim_mode:=gazebo']), value_type=str)

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -v 3 ', LaunchConfiguration('world')],
                          'on_exit_shutdown': 'true'}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'robot_dog', '-z', LaunchConfiguration('z')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'gz_bridge.yaml'),
                     'use_sim_time': True}])

    controllers = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', 'imu_sensor_broadcaster', 'joint_group_position_controller'])
    effort_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_group_effort_controller', '--inactive'])

    rviz = Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'robot_dog.rviz')],
        parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        # lets Gazebo resolve package://robot_dog_description/meshes/...
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_share)),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg_share, 'worlds', 'empty.sdf')),
        DeclareLaunchArgument('z', default_value='0.32', description='Spawn height of base_link (m)'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gz_sim,
        robot_state_publisher,
        spawn,
        bridge,
        rviz,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers, effort_controller])),
    ])
