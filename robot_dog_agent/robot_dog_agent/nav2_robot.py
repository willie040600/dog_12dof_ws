"""The robot behind the tools: goals to Nav2, the pose from TF (spec 2026-11-10, 5.3). Needs rclpy."""
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_dog_agent.robot import NavResult
from robot_dog_navigation.nav_eval import make_pose, sim_seconds, yaw_of

PROGRESS_PERIOD = 2.0  # s of wall time between on_progress calls
POSE_WAIT = 5.0        # s of wall time to wait for map->base_footprint
LOCALIZATION_WAIT = 60.0  # the same at start-up, when AMCL / slam_toolbox may still be coming up


class Nav2Robot:
    """Blocks until Nav2 is active and the robot is localized (AMCL or slam_toolbox, whichever is running).

    timeout: sim seconds a navigation may take before it is canceled.
    """

    def __init__(self, timeout=120.0):
        # no rclpy SIGINT handler: Ctrl+C must reach go_to as KeyboardInterrupt instead of shutting rclpy down
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.timeout = timeout
        self.node = BasicNavigator()
        self.node.get_logger().set_level(LoggingSeverity.WARN)  # its INFO lines would clutter the chat
        self.node.set_parameters([Parameter('use_sim_time', value=True)])
        self.tf_buffer = Buffer()
        # own node for TF: BasicNavigator spins itself, so a listener thread on it would put it in two executors
        self._tf_node = rclpy.create_node('robot_dog_agent_tf',
                                          parameter_overrides=[Parameter('use_sim_time', value=True)])
        self._listener = TransformListener(self.tf_buffer, self._tf_node, spin_thread=True)
        # 'robot_localization' = do not wait for AMCL the BasicNavigator way: that publishes an initial pose at the
        # origin, which throws AMCL off whenever the chat starts with the dog somewhere else
        self.node.waitUntilNav2Active(localizer='robot_localization')
        self.pose(wait=LOCALIZATION_WAIT)

    def pose(self, wait=POSE_WAIT):
        deadline = time.monotonic() + wait
        while True:
            try:
                tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time()).transform
                return tf.translation.x, tf.translation.y, yaw_of(tf.rotation)
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError('no transform from map to base_footprint yet')
                time.sleep(0.05)

    def go_to(self, x, y, yaw, on_progress=None):
        nav = self.node
        if not nav.goToPose(make_pose(nav, x, y, yaw)):
            return NavResult('failed', 'Nav2 rejected the goal', 0.0)
        t0, last, ended = sim_seconds(nav), time.monotonic(), None
        try:
            while not nav.isTaskComplete():
                if ended is None and sim_seconds(nav) - t0 > self.timeout:
                    ended = ('timeout', 'no arrival within %.0f s' % self.timeout)
                    nav.cancelTask()
                if on_progress and ended is None and time.monotonic() - last >= PROGRESS_PERIOD:
                    last = time.monotonic()
                    feedback = nav.getFeedback()
                    if feedback is not None:
                        on_progress(feedback.distance_remaining)
        except KeyboardInterrupt:
            ended = ('canceled', 'canceled by the user')
            nav.cancelTask()
            while not nav.isTaskComplete():
                pass
        elapsed = sim_seconds(nav) - t0
        if ended:
            return NavResult(ended[0], ended[1], elapsed)
        status = nav.getResult().name  # SUCCEEDED / CANCELED / FAILED / UNKNOWN
        if status == 'SUCCEEDED':
            return NavResult('reached', '', elapsed)
        return NavResult('failed', 'Nav2 status %s' % status, elapsed)

    def close(self):
        self._listener.unregister()  # stop its spin thread before rclpy shuts down (else a traceback at exit)
        self._tf_node.destroy_node()
        self.node.destroy_node()
        rclpy.try_shutdown()
