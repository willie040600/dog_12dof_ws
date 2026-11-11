"""The robot behind the tools: goals to Nav2, the pose from TF (spec 2026-11-10, 5.3). Needs rclpy."""
import threading
import time

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.logging import LoggingSeverity
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from robot_dog_agent.robot import CANCELED, NavResult, turn_in_place
from robot_dog_navigation.nav_eval import make_pose, sim_seconds, yaw_of

PROGRESS_PERIOD = 2.0  # s of wall time between on_progress calls
POSE_WAIT = 5.0        # s of wall time to wait for map->base_footprint
HEADING_SLACK = 0.1    # rad; a heading this close to the wanted one is left alone
LOCALIZATION_WAIT = 60.0  # the same at start-up, when AMCL / slam_toolbox may still be coming up


class Nav2Robot:
    """Blocks until Nav2 is active and the robot is localized (AMCL or slam_toolbox, whichever is running).

    timeout: sim seconds a navigation may take before it is canceled.
    """

    def __init__(self, timeout=120.0):
        # no rclpy SIGINT handler: Ctrl+C must reach go_to as KeyboardInterrupt instead of shutting rclpy down
        if not rclpy.ok():  # the window starts rclpy itself, for the camera
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.timeout = timeout
        self._cancel = threading.Event()
        self.node = self._tf_node = self._listener = None
        try:
            self.node = BasicNavigator()
            self.node.set_parameters([Parameter('use_sim_time', value=True)])
            self.tf_buffer = Buffer()
            # own node for TF: BasicNavigator spins itself, so a listener thread on it would put it in two executors
            self._tf_node = rclpy.create_node('robot_dog_agent_tf',
                                              parameter_overrides=[Parameter('use_sim_time', value=True)])
            self._listener = TransformListener(self.tf_buffer, self._tf_node, spin_thread=True)
            # 'robot_localization' = do not wait for AMCL the BasicNavigator way: that publishes an initial pose at
            # the origin, which throws AMCL off whenever the chat starts with the dog somewhere else.
            # Its INFO lines ("... service not available, waiting...") are the only sign of life while Nav2 is not up
            self.node.waitUntilNav2Active(localizer='robot_localization')
            self.node.get_logger().set_level(LoggingSeverity.WARN)  # from here on they would clutter the chat
            self.pose(wait=LOCALIZATION_WAIT)
        except BaseException:
            self.close()  # else the TF thread keeps the process alive after the error
            raise

    def cancel(self):
        """From any thread: end the navigation under way and refuse every one after it, until clear_cancel()."""
        self._cancel.set()

    def clear_cancel(self):
        """The robot may move again (the window calls this when the next command starts)."""
        self._cancel.clear()

    def pose(self, wait=POSE_WAIT):
        """wait: wall seconds to wait for the transform; 0 = look once (safe from the UI thread)."""
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
        t0 = sim_seconds(nav)
        angle = turn_in_place(self.pose(), x, y, yaw)
        if angle is not None:
            # a turn on the spot: Nav2's spin ends about 5 deg past the angle (measured); a goal at the same
            # position stops as soon as it is within the goal checker's 0.25 rad, about 12 deg short
            if abs(angle) < HEADING_SLACK / 5.0:
                return NavResult('reached', '', 0.0)
            return self._wait(lambda: nav.spin(spin_dist=angle, time_allowance=30), t0, None)
        result = self._wait(lambda: nav.goToPose(make_pose(nav, x, y, yaw)), t0, on_progress)
        if result.status != 'reached':
            return result
        # Nav2 stops turning once the heading is inside its 0.25 rad tolerance (measured: up to 0.37 rad off in
        # truth); turn the rest of the way. A trim that fails (no room to spin) still leaves the dog at the place
        pose = self.pose()
        rest = turn_in_place(pose, pose[0], pose[1], yaw)
        if abs(rest) > HEADING_SLACK:
            trim = self._wait(lambda: nav.spin(spin_dist=rest, time_allowance=15), t0, None)
            if trim.status == 'canceled':
                return trim
        return NavResult('reached', '', sim_seconds(nav) - t0)

    def _wait(self, send, t0, on_progress):
        """Send a task with send() -> accepted and wait for it to end; the NavResult counts sim time from t0."""
        nav = self.node
        nav.feedback = None  # BasicNavigator keeps the last task's feedback, which may be of another type
        if self._cancel.is_set():  # Stop came before this started: while waiting for the model, or a step ago
            return NavResult('canceled', CANCELED, sim_seconds(nav) - t0)
        if not send():
            return NavResult('failed', 'Nav2 rejected the goal', sim_seconds(nav) - t0)
        last, ended = time.monotonic(), None
        try:
            while not nav.isTaskComplete():
                if ended is None and self._cancel.is_set():
                    ended = ('canceled', CANCELED)
                    nav.cancelTask()
                if ended is None and sim_seconds(nav) - t0 > self.timeout:
                    ended = ('timeout', 'no arrival within %.0f s' % self.timeout)
                    nav.cancelTask()
                if on_progress and ended is None and time.monotonic() - last >= PROGRESS_PERIOD:
                    last = time.monotonic()
                    feedback = nav.getFeedback()
                    if getattr(feedback, 'distance_remaining', None) is not None:
                        on_progress(feedback.distance_remaining)
        except KeyboardInterrupt:
            ended = ('canceled', CANCELED)
            nav.cancelTask()
            while not nav.isTaskComplete():
                pass
        except BaseException:
            nav.cancelTask()  # never leave the dog walking with nobody watching
            raise
        elapsed = sim_seconds(nav) - t0
        if ended:
            return NavResult(ended[0], ended[1], elapsed)
        status = nav.getResult().name  # SUCCEEDED / CANCELED / FAILED / UNKNOWN
        if status == 'SUCCEEDED':
            return NavResult('reached', '', elapsed)
        return NavResult('failed', 'Nav2 status %s' % status, elapsed)

    def close(self):
        """Also after a start-up that failed half-way: whatever exists is torn down."""
        if self._listener is not None:
            self._listener.unregister()  # stop its spin thread before rclpy shuts down (else a traceback at exit)
        for node in (self._tf_node, self.node):
            if node is not None:
                node.destroy_node()
        self._listener = self._tf_node = self.node = None
        rclpy.try_shutdown()
