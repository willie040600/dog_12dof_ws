"""Pass criteria for the gait evaluation and the stand test (spec section 4.3). Plain Python."""
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

LIN_TOL = 0.1          # m/s
ANG_TOL = 0.15         # rad/s
REL_TOL = 0.5          # error must also be below half the command
DRIFT_TOL = 0.1        # m, horizontal drift over STAND_MEASURE_S on a zero command
WARMUP_S = 2.0
MEASURE_S = 5.0
STAND_MEASURE_S = 10.0
STAND_HEIGHT_RANGE = (0.20, 0.28)  # m, base height with zero actions (kp 20 sags under load)
MIN_HEIGHT = 0.18  # m, mean base height while tracking a command; v7 crawled at ~0.09

AXES = ('vx', 'vy', 'wz')


@dataclass(frozen=True)
class GaitCase:
    name: str
    command: Tuple[float, float, float]
    axis: Optional[int]  # 0 vx, 1 vy, 2 wz, None = stand still


def _case(axis, value):
    cmd = [0.0, 0.0, 0.0]
    cmd[axis] = value
    return GaitCase('%s%+.2f' % (AXES[axis], value), tuple(cmd), axis)


CASES = tuple(
    [_case(0, s * v) for v in (0.1, 0.3, 0.5) for s in (1, -1)]
    + [_case(1, s * v) for v in (0.1, 0.3) for s in (1, -1)]
    + [_case(2, s * v) for v in (0.2, 0.5, 0.8) for s in (1, -1)]
    + [GaitCase('stand', (0.0, 0.0, 0.0), None)]
)


def judge_case(case, mean_vel, drift, falls, height=None):
    """(passed, reason) for one case; mean_vel = achieved (vx, vy, wz) in the body frame."""
    if falls > 0:
        return False, '%d fall(s)' % falls
    if height is not None and height < MIN_HEIGHT:
        return False, 'height %.3f m < %.2f (crawling)' % (height, MIN_HEIGHT)
    if case.axis is None:
        if drift < DRIFT_TOL:
            return True, 'drift %.3f m' % drift
        return False, 'drift %.3f m >= %.2f' % (drift, DRIFT_TOL)
    # also within half the command, so standing still cannot pass a small command (error = |command|)
    target = case.command[case.axis]
    tol = min(ANG_TOL if case.axis == 2 else LIN_TOL, REL_TOL * abs(target))
    err = abs(mean_vel[case.axis] - target)
    if err < tol:
        return True, '%s error %.3f' % (AXES[case.axis], err)
    return False, '%s error %.3f >= %.2f' % (AXES[case.axis], err, tol)


def judge_stand(heights: Sequence[float], falls: int):
    lo, hi = STAND_HEIGHT_RANGE
    if falls > 0:
        return False, '%d fall(s)' % falls
    bad = [h for h in heights if not lo <= h <= hi]
    if bad:
        return False, '%d of %d heights outside [%.2f, %.2f] m' % (len(bad), len(heights), lo, hi)
    return True, 'all %d heights in [%.2f, %.2f] m' % (len(heights), lo, hi)
