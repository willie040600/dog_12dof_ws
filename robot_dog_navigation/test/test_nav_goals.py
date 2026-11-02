import math

from robot_dog_navigation.nav_goals import GOALS, judge, pose_error, room_layout

ROOM = room_layout()


def test_goals_are_inside_and_clear():
    assert len(GOALS) == 5
    for name, x, y, _ in GOALS:
        assert abs(x) < ROOM.ROOM_HALF and abs(y) < ROOM.ROOM_HALF, name
        assert ROOM.clearance(x, y) >= 0.5, name


def test_goals_cover_all_zones_and_cross_doorways():
    zones = [ROOM.zone(x) for _, x, _, _ in GOALS]
    assert set(zones) == {0, 1, 2}
    path = [ROOM.zone(0.0)] + zones
    assert sum(a != b for a, b in zip(path, path[1:])) >= 2


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
