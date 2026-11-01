"""RViz view of robot_dog with joint sliders (no simulation)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}


def generate_launch_description():
    pkg = FindPackageShare('robot_dog_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([pkg, 'urdf', 'robot_dog.urdf.xacro'])]), value_type=str)
    # start the sliders at the standing pose
    zeros = {'%s_%s_joint' % (leg, part): q
             for leg in ('FL', 'FR', 'RL', 'RR') for part, q in STAND.items()}
    gui = LaunchConfiguration('gui')

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show joint sliders'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             parameters=[{'zeros': zeros}], condition=IfCondition(gui)),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[{'zeros': zeros}], condition=UnlessCondition(gui)),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([pkg, 'rviz', 'robot_dog.rviz'])]),
    ])
