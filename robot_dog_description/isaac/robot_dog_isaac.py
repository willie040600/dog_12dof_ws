"""Load robot_dog into Isaac Sim 6.2 and expose it to ROS 2 the same way as Gazebo.

Run with the Isaac Sim 6.2 pip install in ~/IsaacLab's uv environment, ROS 2 Jazzy sourced:
  source /opt/ros/jazzy/setup.bash
  cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
  uv run --extra isaacsim python <path>/robot_dog_isaac.py            # Isaac PD drives follow position commands
  uv run --extra isaacsim python <path>/robot_dog_isaac.py --effort   # drives off, joints take torques
  ... --headless --max-frames 600                                     # smoke test: run 600 frames, then exit 0
Then on the ROS 2 side:
  ros2 launch robot_dog_description isaac.launch.py [command:=effort]

Publishes  /clock  /isaac_joint_states  /imu  /scan  /camera/image_raw  /camera/camera_info
Subscribes /isaac_joint_commands  (sensor_msgs/JointState, from ros2_control)

Written against the Isaac Sim 6.x APIs (URDFImporter, isaacsim.sensors.experimental.rtx,
ROS 2 OmniGraph nodes from isaacsim.ros2.nodes). Each sensor is set up on its own: if a
call differs in your build, a warning is printed and the rest keeps running.
The building blocks live in robot_dog_sim/ (shared with the navigation mode).
"""
import argparse
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robot_dog_sim.robot_spec import (CAMERA_HFOV, CAMERA_SIZE, CAMERA_XYZ, DEFAULT_URDF, EFFORT,  # noqa: E402
                                      IMU_XYZ, LASER_XYZ, SPAWN_Z, STAND, VELOCITY)
from robot_dog_sim.util import log, require_ros_jazzy, try_step  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('--urdf', default=DEFAULT_URDF)
parser.add_argument('--usd', help='reference an already converted USD instead of importing the URDF')
parser.add_argument('--usd-dir', default=os.path.join(tempfile.gettempdir(), 'robot_dog_isaac_usd'),
                    help='where the URDF importer writes the USD')
parser.add_argument('--effort', action='store_true', help='torque control: drive stiffness/damping = 0')
parser.add_argument('--kp', type=float, default=20.0, help='drive stiffness, N*m/rad')
parser.add_argument('--kd', type=float, default=0.5, help='drive damping, N*m*s/rad')
parser.add_argument('--physics-hz', type=int, default=500)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--max-frames', type=int, default=0, help='exit after this many app updates (0: run until closed)')
args, _ = parser.parse_known_args()
require_ros_jazzy()

from isaacsim import SimulationApp  # noqa: E402  must start before any other omni import

simulation_app = SimulationApp({'headless': args.headless})

import omni.timeline  # noqa: E402

from robot_dog_sim import kit, ros_graph, sensors, world  # noqa: E402
from robot_dog_sim.robot import configure_drives, convert_urdf, load_robot, robot_parts  # noqa: E402

kit.enable_extensions()
stage = kit.new_stage(simulation_app)


def joint_graphs(art_path):
    ros_graph.clock_graph(stage)
    ros_graph.joint_state_graph(stage, art_path, '/isaac_joint_states')
    ros_graph.joint_command_graph(stage, art_path, '/isaac_joint_commands')


def main():
    world.demo_world(stage, args.physics_hz)
    usd = os.path.abspath(args.usd) if args.usd else convert_urdf(args.urdf, args.usd_dir)
    root = load_robot(stage, simulation_app, usd, SPAWN_Z)
    art, base, joints = robot_parts(root)
    if base is None or len(joints) != 12:
        raise RuntimeError('expected base_link and 12 revolute joints, found %s / %d' % (base, len(joints)))
    kp, kd = (0.0, 0.0) if args.effort else (args.kp, args.kd)
    configure_drives(joints, kp, kd, EFFORT, VELOCITY, STAND)
    log('articulation root %s, base %s, %s control' % (
        art.GetPath(), base.GetPath(), 'effort' if args.effort else 'position (kp=%g kd=%g)' % (args.kp, args.kd)))

    base_path = base.GetPath().pathString
    if not try_step('joint states / commands / clock', joint_graphs, art.GetPath().pathString):
        log('ros2_control cannot reach the robot without the joint graph')
    try_step('IMU -> /imu', lambda: ros_graph.imu_graph(stage, sensors.create_imu(stage, base_path, IMU_XYZ)))
    try_step('camera -> /camera/image_raw', lambda: ros_graph.camera_graph(
        stage, sensors.create_camera(stage, base_path, CAMERA_XYZ, CAMERA_HFOV, CAMERA_SIZE), CAMERA_SIZE))
    try_step('LiDAR', lambda: ros_graph.lidar_graph(stage, *sensors.create_lidar(stage, base_path, LASER_XYZ)))

    omni.timeline.get_timeline_interface().play()
    frames = 0
    while simulation_app.is_running() and (args.max_frames <= 0 or frames < args.max_frames):
        simulation_app.update()
        frames += 1


exit_code = 0
try:
    main()
except Exception:
    traceback.print_exc()  # close() ends the process with _exit(), so print before it
    exit_code = 1
finally:
    simulation_app.close(exit_code=exit_code)
