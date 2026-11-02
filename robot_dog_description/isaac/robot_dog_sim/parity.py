"""Shared command sequence and comparison for the Isaac Lab vs Isaac Sim parity test (spec M2). Plain Python."""

# (label, seconds, (vx m/s, vy m/s, wz rad/s)), 60 s in total
SEGMENTS = (
    ("stand", 5.0, (0.0, 0.0, 0.0)),
    ("forward 0.3", 8.0, (0.3, 0.0, 0.0)),
    ("forward 0.5", 6.0, (0.5, 0.0, 0.0)),
    ("slow forward 0.1", 6.0, (0.1, 0.0, 0.0)),
    ("sideways left 0.3", 6.0, (0.0, 0.3, 0.0)),
    ("slow sideways right 0.1", 6.0, (0.0, -0.1, 0.0)),
    ("turn left 0.8", 5.0, (0.0, 0.0, 0.8)),
    ("slow turn right 0.2", 6.0, (0.0, 0.0, -0.2)),
    ("walk and turn", 7.0, (0.3, 0.0, 0.5)),
    ("stand", 5.0, (0.0, 0.0, 0.0)),
)
WARMUP_FRACTION = 1.0 / 3.0  # skip the transition into each segment


def command_at(step, dt):
    """Command of the segment that policy step `step` falls into (the last one after the end)."""
    t = step * dt
    for _, seconds, cmd in SEGMENTS:
        if t < seconds - 1e-9:
            return cmd
        t -= seconds
    return SEGMENTS[-1][2]


def total_steps(dt):
    return sum(round(seconds / dt) for _, seconds, _ in SEGMENTS)


def segment_means(samples, dt):
    """Mean per segment of each sample field ((vx, vy, wz[, height])), one sample per policy step."""
    means, start = [], 0
    for _, seconds, _ in SEGMENTS:
        steps = round(seconds / dt)
        part = samples[start + int(steps * WARMUP_FRACTION):start + steps]
        means.append(tuple(sum(s[k] for s in part) / len(part) for k in range(len(part[0]))))
        start += steps
    return means


def compare(ref, got, lin_tol=0.05, ang_tol=0.1):
    """One message per segment whose mean velocity differs by more than the tolerance."""
    failures = []
    for (label, _, _), r, g in zip(SEGMENTS, ref, got):
        d = [abs(a - b) for a, b in zip(r, g)]
        if d[0] > lin_tol or d[1] > lin_tol or d[2] > ang_tol:
            failures.append("%s: ref (%+.3f %+.3f %+.3f) got (%+.3f %+.3f %+.3f)" % (label, *r, *g))
    return failures
