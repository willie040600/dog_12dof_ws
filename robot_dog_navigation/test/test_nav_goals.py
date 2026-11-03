import math

from robot_dog_navigation.nav_goals import (GOALS, clearance, judge, load_truth, pose_error, reachable,
                                            sideways_share)

BODY = load_truth('apartment_body')


def apartment_xy(x, y):
    """World -> apartment coordinates (the truth map's origin is the apartment's corner)."""
    return x - BODY[2][0], y - BODY[2][1]


def test_goals_are_clear_and_reachable():
    assert len(GOALS) == 5
    for name, x, y, _ in GOALS:
        assert clearance(BODY, x, y) >= 0.5, name
        assert reachable(BODY, x, y), name


def test_spawn_is_clear_and_reachable():
    assert clearance(BODY, 0.0, 0.0) >= 0.5 and reachable(BODY, 0.0, 0.0)


def test_goals_cover_three_areas():
    areas = set()
    for _, x, y, _ in GOALS:
        ax, ay = apartment_xy(x, y)
        areas.add('ldk' if ay < 3.0 else ('master' if ax < 5.0 else 'second'))
    assert areas == {'ldk', 'master', 'second'}


def test_bed_centre_is_unreachable():
    assert not reachable(BODY, 0.4, 2.7) or clearance(BODY, 0.4, 2.7) < 0.16


def test_off_map_is_blocked():
    assert clearance(BODY, 20.0, 0.0) == 0.0 and not reachable(BODY, 20.0, 0.0)


def test_pose_error_wraps_yaw():
    d, e = pose_error(('g', 1.0, 2.0, 3.1), 1.0, 2.0, -3.1)
    assert d == 0.0 and abs(e - (2 * math.pi - 6.2)) < 1e-9


GOAL = ('g', 1.0, 2.0, 0.0)


def test_judge_success():
    ok, why = judge(GOAL, (1.1, 2.1, 0.1), 'SUCCEEDED', fell=False, elapsed=30.0)
    assert ok, why


def test_judge_failures():
    cases = [((1.0, 2.0, 0.0), 'FAILED', False, 30.0, 'status'),
             ((1.0, 2.0, 0.0), 'SUCCEEDED', True, 30.0, 'fall'),
             ((1.0, 2.0, 0.0), 'SUCCEEDED', False, 121.0, 'timeout'),
             ((1.3, 2.0, 0.0), 'SUCCEEDED', False, 30.0, 'distance'),
             ((1.0, 2.0, 0.4), 'SUCCEEDED', False, 30.0, 'yaw')]
    for pose, status, fell, elapsed, word in cases:
        ok, why = judge(GOAL, pose, status, fell=fell, elapsed=elapsed)
        assert not ok and word in why, (word, why)


def test_sideways_share():
    forward = [(0.4, 0.02)] * 50
    crab = [(0.2, 0.25)] * 50          # 51 degrees off the heading
    backward = [(-0.2, 0.0)] * 50
    standing = [(0.01, 0.03)] * 50     # below the speed threshold: ignored
    assert sideways_share(forward) == 0.0
    assert sideways_share(crab) == 1.0
    assert sideways_share(backward) == 1.0
    assert sideways_share(forward + crab) == 0.5
    assert sideways_share(forward + standing) == 0.0
    assert sideways_share([]) == 0.0


def test_sideways_share_averages_out_gait_sway():
    sway = [(0.3, 0.3 if i % 2 else -0.3) for i in range(50)]   # side to side every sample, straight on average
    assert sideways_share(sway) == 0.0
