from robot_dog_sim.parity import SEGMENTS, WARMUP_FRACTION, compare, segment_means

DT = 0.02


def test_segments_last_60_seconds():
    assert abs(sum(s for _, s, _ in SEGMENTS) - 60.0) < 1e-9
    assert len(SEGMENTS) == 10


def test_segment_means_constant_input():
    n = round(sum(s for _, s, _ in SEGMENTS) / DT)
    means = segment_means([(0.3, -0.1, 0.5)] * n, DT)
    assert len(means) == len(SEGMENTS)
    for m in means:
        assert all(abs(a - b) < 1e-9 for a, b in zip(m, (0.3, -0.1, 0.5)))


def test_segment_means_drop_warmup():
    samples = []
    for _, seconds, _ in SEGMENTS:
        steps = round(seconds / DT)
        warm = int(steps * WARMUP_FRACTION)
        samples += [(9.0, 9.0, 9.0)] * warm + [(1.0, 2.0, 3.0)] * (steps - warm)
    for m in segment_means(samples, DT):
        assert all(abs(a - b) < 1e-9 for a, b in zip(m, (1.0, 2.0, 3.0)))


def test_compare_boundaries():
    ref = [(0.3, 0.0, 0.0)] * len(SEGMENTS)
    assert compare(ref, [(0.349, 0.0, 0.099)] * len(SEGMENTS)) == []
    bad = compare(ref, [(0.351, 0.0, 0.0)] + [(0.3, 0.0, 0.0)] * (len(SEGMENTS) - 1))
    assert len(bad) == 1 and SEGMENTS[0][0] in bad[0]
    assert len(compare(ref, [(0.3, 0.0, 0.101)] * len(SEGMENTS))) == len(SEGMENTS)


def test_verdict_requires_no_falls():
    from robot_dog_sim.parity import verdict

    assert verdict([], sim_fallen=False, ref_falls=0)
    assert not verdict([], sim_fallen=True, ref_falls=0)
    assert not verdict([], sim_fallen=False, ref_falls=1)
    assert not verdict(['forward 0.3: ...'], sim_fallen=False, ref_falls=0)
