"""/cmd_vel handling and safety checks for the policy runner (plain Python, sim-time based)."""
import math

AXES = ('lin_vel_x', 'lin_vel_y', 'ang_vel_z')


class CommandBuffer:
    """Latest (vx, vy, wz), clipped to the training ranges; zero once it is older than timeout."""

    def __init__(self, ranges, timeout=0.5):
        self._ranges = [ranges[a] for a in AXES]
        self._timeout = timeout
        self._cmd = (0.0, 0.0, 0.0)
        self._stamp = None

    def update(self, cmd, t):
        # non-finite components (a malformed Twist) become 0 instead of reaching the policy
        self._cmd = tuple(min(max(float(c), lo), hi) if math.isfinite(c) else 0.0
                          for c, (lo, hi) in zip(cmd, self._ranges))
        self._stamp = t

    def get(self, t):
        if self._stamp is None or t - self._stamp > self._timeout:
            return (0.0, 0.0, 0.0)
        return self._cmd


class IdleHold:
    """Hold the standing pose once the command has been ~zero for hold_after seconds and the body is still.

    Waiting for the body to stop (the policy brakes it) matters: freezing the targets mid-stride made the
    dog lurch.
    """

    def __init__(self, enabled, hold_after=0.3, lin_eps=0.02, ang_eps=0.05, still_lin=0.05, still_ang=0.1):
        self._enabled = enabled
        self._hold_after, self._lin_eps, self._ang_eps = hold_after, lin_eps, ang_eps
        self._still_lin, self._still_ang = still_lin, still_ang
        self._idle_since = None
        self._holding = False
        self._resumed = False

    def update(self, cmd, t, speed=(0.0, 0.0)):
        """speed = (planar body speed m/s, |yaw rate| rad/s)."""
        idle = abs(cmd[0]) < self._lin_eps and abs(cmd[1]) < self._lin_eps and abs(cmd[2]) < self._ang_eps
        if not idle:
            self._idle_since = None
        elif self._idle_since is None:
            self._idle_since = t
        still = speed[0] < self._still_lin and speed[1] < self._still_ang
        hold = (self._enabled and idle and t - self._idle_since >= self._hold_after
                and (still or self._holding))
        self._resumed = self._holding and not hold
        self._holding = hold
        return hold

    def resumed(self):
        """True on the first policy step after a hold; the caller resets last_action then."""
        return self._resumed


def is_fallen(base_height, projected_gravity_z, min_height=0.12, max_tilt_deg=60.0):
    return base_height < min_height or projected_gravity_z > -math.cos(math.radians(max_tilt_deg))
