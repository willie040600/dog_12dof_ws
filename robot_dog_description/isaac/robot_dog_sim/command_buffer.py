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
        self._cmd = tuple(min(max(float(c), lo), hi) for c, (lo, hi) in zip(cmd, self._ranges))
        self._stamp = t

    def get(self, t):
        if self._stamp is None or t - self._stamp > self._timeout:
            return (0.0, 0.0, 0.0)
        return self._cmd


class IdleHold:
    """Hold the standing pose once the command has been ~zero for hold_after seconds."""

    def __init__(self, enabled, hold_after=0.3, lin_eps=0.02, ang_eps=0.05):
        self._enabled = enabled
        self._hold_after, self._lin_eps, self._ang_eps = hold_after, lin_eps, ang_eps
        self._idle_since = None
        self._holding = False
        self._resumed = False

    def update(self, cmd, t):
        idle = abs(cmd[0]) < self._lin_eps and abs(cmd[1]) < self._lin_eps and abs(cmd[2]) < self._ang_eps
        if not idle:
            self._idle_since = None
        elif self._idle_since is None:
            self._idle_since = t
        hold = self._enabled and idle and t - self._idle_since >= self._hold_after
        self._resumed = self._holding and not hold
        self._holding = hold
        return hold

    def resumed(self):
        """True on the first policy step after a hold; the caller resets last_action then."""
        return self._resumed


def is_fallen(base_height, projected_gravity_z, min_height=0.12, max_tilt_deg=60.0):
    return base_height < min_height or projected_gravity_z > -math.cos(math.radians(max_tilt_deg))
