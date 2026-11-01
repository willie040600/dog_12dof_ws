"""Check that rclpy works inside the Isaac Sim process next to the OmniGraph ROS 2 bridge.

Publishes /clock from an OmniGraph graph and receives it with an rclpy subscription in the same
process. Exit code 0 when at least 10 messages with a positive stamp arrived, 1 otherwise,
2 when ROS 2 Jazzy is not sourced.

  source /opt/ros/jazzy/setup.bash
  cd ~/IsaacLab && uv run --extra isaacsim python <this file> [--frames 300]
"""
import argparse
import os
import sys
import traceback

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from robot_dog_sim.util import require_ros_jazzy  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('--frames', type=int, default=300)
args, _ = parser.parse_known_args()
require_ros_jazzy()

from isaacsim import SimulationApp  # noqa: E402  must start before any other omni import

simulation_app = SimulationApp({'headless': True})

import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402

ext = omni.kit.app.get_app().get_extension_manager()
for name in ('isaacsim.core.nodes', 'isaacsim.ros2.bridge', 'isaacsim.ros2.nodes'):
    ext.set_extension_enabled_immediate(name, True)
omni.usd.get_context().new_stage()
simulation_app.update()


def main():
    og.Controller.edit({'graph_path': '/ROS/clock', 'evaluator_name': 'execution'}, {
        og.Controller.Keys.CREATE_NODES: [
            ('Tick', 'omni.graph.action.OnPlaybackTick'),
            ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
            ('Context', 'isaacsim.ros2.bridge.ROS2Context'),
            ('PubClock', 'isaacsim.ros2.bridge.ROS2PublishClock')],
        og.Controller.Keys.SET_VALUES: [('PubClock.inputs:topicName', '/clock')],
        og.Controller.Keys.CONNECT: [
            ('Tick.outputs:tick', 'PubClock.inputs:execIn'),
            ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),
            ('Context.outputs:context', 'PubClock.inputs:context')],
    })

    import rclpy
    from rosgraph_msgs.msg import Clock

    rclpy.init()
    node = rclpy.create_node('rclpy_probe')
    received = []
    node.create_subscription(Clock, '/clock', lambda m: received.append(m.clock.sec + m.clock.nanosec * 1e-9), 10)

    omni.timeline.get_timeline_interface().play()
    for _ in range(args.frames):
        simulation_app.update()
        rclpy.spin_once(node, timeout_sec=0.0)

    last = received[-1] if received else 0.0
    print('[rclpy_probe] received %d /clock messages, last stamp %.3f s' % (len(received), last), flush=True)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if len(received) >= 10 and last > 0.0 else 1


code = 1
try:
    code = main()
except Exception:
    traceback.print_exc()  # close() ends the process with _exit(), so print before it
finally:
    simulation_app.close(exit_code=code)
