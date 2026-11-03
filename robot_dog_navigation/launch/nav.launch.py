"""robot_dog navigation (ROS side): bridge (TF, scan filter) + Nav2 with slam_toolbox or AMCL + RViz.

Start robot_dog_description/isaac/robot_dog_nav_sim.py first, then
    ros2 launch robot_dog_navigation nav.launch.py [slam:=True|False] [map:=<yaml>] [rviz:=true]
slam:=True builds the map while navigating; slam:=False localizes with AMCL on `map` (default maps/apartment.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('robot_dog_navigation')
    nav2 = get_package_share_directory('nav2_bringup')
    slam, map_yaml = LaunchConfiguration('slam'), LaunchConfiguration('map')
    params, rviz = LaunchConfiguration('params_file'), LaunchConfiguration('rviz')
    return LaunchDescription([
        DeclareLaunchArgument('slam', default_value='True', description='True: slam_toolbox, False: AMCL on map'),
        DeclareLaunchArgument('map', default_value=os.path.join(share, 'maps', 'apartment.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'bridge.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={'slam': slam, 'map': map_yaml, 'params_file': params,
                              'use_sim_time': 'true', 'autostart': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'nav.rviz')],
             parameters=[{'use_sim_time': True}], condition=IfCondition(rviz), output='screen'),
    ])
