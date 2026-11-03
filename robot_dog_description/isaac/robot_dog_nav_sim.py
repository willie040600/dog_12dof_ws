"""Navigation mode: the trained policy walks robot_dog in Isaac Sim, driven by /cmd_vel (spec 5).

Runs the policy in lockstep with physics (stepping.Lockstep + policy_runner.PolicyRunner) and exposes
the robot to ROS 2 like a wheeled base:
  subscribes /cmd_vel (geometry_msgs/Twist; zero after 0.5 s without messages, clipped to training ranges)
  publishes  /clock /joint_states /imu /scan /camera/image_raw /camera/camera_info
             /odom_gt (nav_msgs/Odometry odom -> base_link, ground truth; twist in base_link)
No TF: robot_dog_navigation's bridge.launch.py publishes it from /odom_gt and /joint_states.

  source /opt/ros/jazzy/setup.bash
  cd ~/IsaacLab && uv run --extra isaacsim python <path>/robot_dog_nav_sim.py [--headless] [--no-camera] [--hold]
"""
import argparse
import glob
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robot_dog_sim.robot_spec import (CAMERA_HFOV, CAMERA_SIZE, CAMERA_XYZ, DEFAULT_URDF, IMU_XYZ,  # noqa: E402
                                      LASER_XYZ, LIDAR_RANGE)
from robot_dog_sim.util import log, require_ros_jazzy, sim_time_to_stamp, try_step  # noqa: E402

policies = sorted(glob.glob(os.path.join(HERE, 'policies', '*', 'policy.json')))
parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('--policy', default=os.path.dirname(policies[-1]) if policies else None,
                    help='folder with policy.pt + policy.json (default: last one under isaac/policies/)')
parser.add_argument('--world', help='USD to load instead of the apartment (gets the same collision and friction, not shifted)')
parser.add_argument('--usd-dir', default=os.path.join(tempfile.gettempdir(), 'robot_dog_isaac_usd'))
parser.add_argument('--headless', action='store_true')
parser.add_argument('--no-camera', action='store_true')
parser.add_argument('--hold', action='store_true',
                    help='freeze the standing pose after 0.3 s of zero command (default: the policy keeps standing;\n'
                         'freezing it made the dog lurch when it had just stopped)')
parser.add_argument('--max-frames', type=int, default=0, help='exit after this many cycles (0: run until closed)')
args, _ = parser.parse_known_args()
require_ros_jazzy()
if not args.policy:
    log('no policy found under %s/policies; pass --policy' % HERE)
    raise SystemExit(1)

from isaacsim import SimulationApp  # noqa: E402  must start before any other omni import

simulation_app = SimulationApp({'headless': args.headless})

from isaacsim.core.experimental.prims import Articulation  # noqa: E402

from robot_dog_sim import kit, ros_graph, sensors, world  # noqa: E402
from robot_dog_sim.command_buffer import CommandBuffer  # noqa: E402
from robot_dog_sim.observations import quat_rotate_inverse_wxyz  # noqa: E402
from robot_dog_sim.policy_meta import load_policy_meta  # noqa: E402
from robot_dog_sim.policy_runner import PolicyRunner  # noqa: E402
from robot_dog_sim.robot import (apply_policy_dof_settings, author_policy_physics, convert_urdf,  # noqa: E402
                                 load_robot, robot_parts)
from robot_dog_sim.stepping import Lockstep  # noqa: E402

kit.enable_extensions()
stage = kit.new_stage(simulation_app)


def make_ros_node(buffer, lock):
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry

    from rclpy.executors import SingleThreadedExecutor

    rclpy.init()
    node = rclpy.create_node('robot_dog_nav_sim')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    node.create_subscription(Twist, '/cmd_vel', lambda m: buffer.update(
        (m.linear.x, m.linear.y, m.angular.z), lock.sim_time), 10)
    return rclpy, node, executor, node.create_publisher(Odometry, '/odom_gt', 10), Odometry


def odom_message(Odometry, runner, t):
    msg = Odometry()
    msg.header.stamp.sec, msg.header.stamp.nanosec = sim_time_to_stamp(t)
    msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
    p, q = runner.base_pos.tolist(), runner.base_quat.tolist()  # q is wxyz
    msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = p
    o = msg.pose.pose.orientation
    o.w, o.x, o.y, o.z = q
    v = quat_rotate_inverse_wxyz(runner.base_quat, runner.lin_vel_w).tolist()
    w = quat_rotate_inverse_wxyz(runner.base_quat, runner.ang_vel_w).tolist()
    msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v
    msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z = w
    return msg


def main():
    policy_dir = os.path.abspath(os.path.expanduser(args.policy))
    meta = load_policy_meta(os.path.join(policy_dir, 'policy.json'))
    log('policy %s (%s)' % (os.path.basename(policy_dir), meta['checkpoint']))
    if args.world:
        log('world %s: %d collision meshes' % (args.world, world.usd_world(stage, meta, args.world)))
    else:
        log('apartment: %d collision meshes' % world.apartment_world(stage, meta))
    root = load_robot(stage, simulation_app, convert_urdf(DEFAULT_URDF, args.usd_dir), meta['spawn_height'])
    art_prim, base, joints = robot_parts(root)
    author_policy_physics(root, art_prim, meta)
    art_path, base_path = art_prim.GetPath().pathString, base.GetPath().pathString

    try_step('clock / joint states', lambda: (ros_graph.clock_graph(stage),
                                              ros_graph.joint_state_graph(stage, art_path, '/joint_states')))
    try_step('IMU -> /imu', lambda: ros_graph.imu_graph(stage, sensors.create_imu(stage, base_path, IMU_XYZ)))
    lidar = []
    if not try_step('LiDAR -> /scan', lambda: lidar.extend(sensors.create_lidar(stage, base_path, LASER_XYZ, LIDAR_RANGE))) \
            or lidar[1] != 'laser_scan':
        log('navigation needs a 2D LiDAR on /scan (got %s); exiting' % (lidar[2] if lidar else 'none'))
        return 1
    try_step('LiDAR graph', lambda: ros_graph.lidar_graph(stage, *lidar))
    if args.no_camera:
        log('camera disabled (--no-camera)')
    else:
        try_step('camera -> /camera/image_raw', lambda: ros_graph.camera_graph(
            stage, sensors.create_camera(stage, base_path, CAMERA_XYZ, CAMERA_HFOV, CAMERA_SIZE), CAMERA_SIZE,
            frame_skip=2))

    lock = Lockstep(simulation_app, meta['sim_dt'], meta['decimation'])
    articulation = Articulation(art_path)
    runner = PolicyRunner(meta, os.path.join(policy_dir, 'policy.pt'), articulation, hold=args.hold)
    apply_policy_dof_settings(articulation, meta, runner.dof_indices)
    buffer = CommandBuffer(meta['command_ranges'])
    rclpy, node, executor, odom_pub, Odometry = make_ros_node(buffer, lock)
    log('running: /cmd_vel in, /odom_gt out')

    frames, reported_fall = 0, False
    while simulation_app.is_running() and (args.max_frames <= 0 or frames < args.max_frames):
        for _ in range(20):  # handle every /cmd_vel received since the last cycle; the buffer keeps the newest
            executor.spin_once(timeout_sec=0.0)
        t = lock.sim_time
        runner.policy_step(buffer.get(t), t)
        odom_pub.publish(odom_message(Odometry, runner, t))
        if runner.fallen and not reported_fall:
            log('FALLEN at t=%.2f s' % t)
        reported_fall = runner.fallen
        lock.step(runner.pre_physics)
        frames += 1
    node.destroy_node()
    rclpy.shutdown()
    return 0


exit_code = 1
try:
    exit_code = main()
except Exception:
    traceback.print_exc()  # close() ends the process with _exit(), so print before it
finally:
    simulation_app.close(exit_code=exit_code)
