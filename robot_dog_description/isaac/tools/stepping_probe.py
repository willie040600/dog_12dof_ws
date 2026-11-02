"""Check lockstep stepping: every cycle runs exactly 4 x 5 ms physics steps and /clock follows sim time.

Builds the demo world with the robot, publishes /clock from OmniGraph, receives it with rclpy in the same
process and runs 200 Lockstep cycles. Exit code 0 when all checks pass, 1 otherwise, 2 without ROS.

  source /opt/ros/jazzy/setup.bash
  cd ~/IsaacLab && uv run --extra isaacsim python <this file>
"""
import os
import sys
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from robot_dog_sim.robot_spec import DEFAULT_URDF, SPAWN_Z  # noqa: E402
from robot_dog_sim.util import require_ros_jazzy  # noqa: E402

require_ros_jazzy()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({'headless': True})

from robot_dog_sim import kit, ros_graph, world  # noqa: E402
from robot_dog_sim.robot import convert_urdf, load_robot  # noqa: E402

kit.enable_extensions()
stage = kit.new_stage(simulation_app)
CYCLES, DT, DECIMATION = 200, 0.005, 4


def main():
    import rclpy
    from rosgraph_msgs.msg import Clock

    from robot_dog_sim.stepping import Lockstep

    world.demo_world(stage, int(round(1.0 / DT)))
    load_robot(stage, simulation_app, convert_urdf(DEFAULT_URDF, '/tmp/robot_dog_isaac_usd'), SPAWN_Z)
    ros_graph.clock_graph(stage)
    rclpy.init()
    node = rclpy.create_node('stepping_probe')
    clock = []
    node.create_subscription(Clock, '/clock', lambda m: clock.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)

    lock = Lockstep(simulation_app, DT, DECIMATION)
    calls = [0]
    bad, lag = [], 0.0
    for i in range(CYCLES):
        s0, t0, c0 = lock.physics_steps, lock.sim_time, calls[0]
        lock.step(lambda: calls.__setitem__(0, calls[0] + 1))
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.0)
        ds, dt, dc = lock.physics_steps - s0, lock.sim_time - t0, calls[0] - c0
        if ds != DECIMATION or abs(dt - DT * DECIMATION) > 1e-6 or dc != DECIMATION:
            bad.append((i, ds, round(dt, 6), dc))
        if clock:
            lag = lock.sim_time - clock[-1]
    print('[stepping_probe] %d cycles, sim time %.3f s, %d physics steps, %d bad cycles %s' % (
        CYCLES, lock.sim_time, lock.physics_steps, len(bad), bad[:5]), flush=True)
    print('[stepping_probe] %d /clock messages, last /clock lag %.4f s' % (len(clock), lag), flush=True)
    rclpy.shutdown()
    return 0 if not bad and clock and abs(lag) < DT * DECIMATION + 1e-6 else 1


code = 1
try:
    code = main()
except Exception:
    traceback.print_exc()
finally:
    simulation_app.close(exit_code=code)
