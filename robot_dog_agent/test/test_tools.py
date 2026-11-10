import math

import pytest

from robot_dog_agent import places, tools
from robot_dog_agent.robot import FakeRobot, NavResult, turn_in_place
from robot_dog_agent.tools import Toolbox

TRUTH = places.load_body_truth()


def box(robot):
    return Toolbox(robot, TRUTH)


def test_declarations_match_the_place_table():
    by_name = {d['name']: d for d in tools.DECLARATIONS}
    assert set(by_name) == {'get_pose', 'go_to_place', 'go_to_pose', 'move_relative'}
    assert all(d['type'] == 'function' and d['description'] for d in tools.DECLARATIONS)
    assert by_name['go_to_place']['parameters']['properties']['place']['enum'] == [p.id for p in places.PLACES]
    assert by_name['go_to_pose']['parameters']['required'] == ['x', 'y']
    text = tools.system_instruction()
    assert all(p.id in text and p.name in text for p in places.PLACES) and 'Traditional Chinese' in text


def test_get_pose():
    r = box(FakeRobot((2.4, -0.6, math.pi))).call('get_pose', {})
    assert r == {'status': 'ok', 'x': 2.4, 'y': -0.6, 'yaw_deg': 180, 'area': 'living, dining and kitchen area',
                 'nearest_place': 'kitchen', 'distance_to_place': 0.1}


def test_go_to_place_uses_the_table():
    robot = FakeRobot()
    r = box(robot).call('go_to_place', {'place': 'master_bedroom'})
    assert robot.goals == [(-1.5, 3.0, math.pi / 2)]
    assert r == {'status': 'reached', 'x': -1.5, 'y': 3.0, 'yaw_deg': 90, 'seconds': 1.0}


def test_go_to_pose_faces_the_direction_of_travel():
    robot = FakeRobot()
    box(robot).call('go_to_pose', {'x': 0.0, 'y': 0.3})
    box(robot).call('go_to_pose', {'x': 0.05, 'y': 0.3})           # closer than 0.1 m: keep the heading
    box(robot).call('go_to_pose', {'x': -0.5, 'y': 0.3, 'yaw_deg': 0})
    assert [round(g[2], 6) for g in robot.goals] == [round(math.pi / 2, 6), round(math.pi / 2, 6), 0.0]


def test_move_relative_starts_from_the_current_pose():
    robot = FakeRobot((0.0, 0.0, math.pi / 2))
    box(robot).call('move_relative', {'forward_m': 0.3, 'turn_left_deg': 90})
    x, y, yaw = robot.goals[0]
    assert abs(x) < 1e-9 and abs(y - 0.3) < 1e-9 and abs(abs(yaw) - math.pi) < 1e-9


def test_blocked_target_never_reaches_the_robot():
    robot = FakeRobot()
    r = box(robot).call('go_to_pose', {'x': 0.4, 'y': 2.7})        # the middle of the bed
    assert r['status'] == 'blocked' and r['reason'].startswith('not connected') and (r['x'], r['y']) == (0.0, 0.0)
    assert robot.goals == []


def test_navigation_failure_is_reported_with_the_final_pose():
    robot = FakeRobot(results=[NavResult('timeout', 'no arrival within 120 s', 120.0)])
    r = box(robot).call('go_to_place', {'place': 'kitchen'})
    assert r == {'status': 'timeout', 'reason': 'no arrival within 120 s', 'x': 0.0, 'y': 0.0, 'yaw_deg': 0,
                 'seconds': 120.0}


def test_bad_arguments_return_error():
    robot = FakeRobot()
    for name, args in [('go_to_place', {}), ('go_to_place', {'place': 'fridge'}), ('go_to_pose', {'x': '1', 'y': 0}),
                       ('go_to_pose', {'x': 1.0}), ('go_to_pose', {'x': float('nan'), 'y': 0.0}),
                       ('go_to_pose', {'x': True, 'y': 0.0}), ('move_relative', {}),
                       ('move_relative', {'forward_m': 6.0}), ('fly', {}), ('go_to_pose', None)]:
        r = box(robot).call(name, args)
        assert r['status'] == 'error' and r['reason'], (name, args)
    assert robot.goals == []
    assert box(robot).call('move_relative', {'forward_m': 0.2, 'speed': 9})['status'] == 'reached'   # extras ignored


def test_robot_without_a_pose():
    robot = FakeRobot()
    robot.pose_error = 'no transform from map to base_footprint yet'
    for name, args in [('get_pose', {}), ('move_relative', {'forward_m': 0.2}), ('go_to_place', {'place': 'kitchen'})]:
        assert box(robot).call(name, args) == {'status': 'error',
                                               'reason': 'no transform from map to base_footprint yet'}
    assert robot.goals == []


def test_progress_is_passed_on():
    seen = []
    robot = FakeRobot()
    Toolbox(robot, TRUTH, on_progress=seen.append).call('go_to_place', {'place': 'kitchen'})
    assert robot.progress_callbacks == [seen.append]


def test_turn_in_place_is_a_target_on_the_spot():
    assert turn_in_place((1.0, 2.0, 0.0), 1.05, 2.0, math.pi / 2) == pytest.approx(math.pi / 2)
    assert turn_in_place((1.0, 2.0, 3.0), 1.0, 2.0, -3.0) == pytest.approx(2 * math.pi - 6.0)   # the short way round
    assert turn_in_place((1.0, 2.0, 0.0), 1.0, 2.0, 0.0) == 0.0
    assert turn_in_place((1.0, 2.0, 0.0), 1.2, 2.0, 1.0) is None                                # a walk, not a turn
