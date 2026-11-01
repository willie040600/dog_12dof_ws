"""Logging and start-up checks shared by the Isaac Sim entry scripts (plain Python, no omni imports)."""
import os


def log(msg):
    print('[robot_dog] ' + msg, flush=True)


def try_step(label, fn, *args):
    """Run one set-up step; report ok / FAILED instead of aborting, so other sensors still come up."""
    try:
        fn(*args)
        log('%-34s ok' % label)
        return True
    except Exception as e:
        log('%-34s FAILED: %s' % (label, e))
        return False


def require_ros_jazzy():
    """Exit before Isaac Sim starts when ROS 2 Jazzy is not sourced (the ROS 2 bridge would fail late)."""
    distro = os.environ.get('ROS_DISTRO')
    if distro != 'jazzy':
        log('ROS 2 Jazzy is not sourced (ROS_DISTRO=%s): run "source /opt/ros/jazzy/setup.bash" first' % distro)
        raise SystemExit(2)
