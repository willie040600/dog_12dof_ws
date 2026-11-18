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


# --- plan 6: cancel from another thread ---

def test_fake_robot_cancel_holds_until_it_is_cleared():
    robot = FakeRobot()
    robot.cancel()
    assert robot.go_to(1.0, 0.0, 0.0) == NavResult('canceled', 'canceled by the user', 0.0)
    assert robot.go_to(2.0, 0.0, 0.0).status == 'canceled'           # a model that tries the next step anyway
    assert robot.goals == [] and robot.pose() == (0.0, 0.0, 0.0)
    robot.clear_cancel()
    assert robot.go_to(1.0, 0.0, 0.0).status == 'reached'


def test_fake_robot_clear_cancel():
    robot = FakeRobot()
    robot.cancel()
    robot.clear_cancel()
    assert robot.go_to(1.0, 0.0, 0.0).status == 'reached'


def test_fake_robot_pose_takes_a_wait_and_close_is_recorded():
    robot = FakeRobot((1.0, 2.0, 0.5))
    assert robot.pose(wait=0) == (1.0, 2.0, 0.5) and robot.closed is False
    robot.close()
    assert robot.closed is True


def test_a_canceled_navigation_is_reported_by_the_toolbox():
    robot = FakeRobot()
    robot.cancel()
    r = box(robot).call('go_to_place', {'place': 'kitchen'})
    assert r['status'] == 'canceled' and (r['x'], r['y']) == (0.0, 0.0)


# --- plan 7 ---

def test_tool_line():
    assert tools.tool_line('go_to_pose', {'x': 1.2, 'y': 0.5}, {'status': 'reached'}) == \
        '  [工具] go_to_pose {"x": 1.2, "y": 0.5} → reached'
    assert tools.tool_line('go_to_place', {'place': 'kitchen'}, {'status': 'blocked', 'reason': 'on an obstacle'}) == \
        '  [工具] go_to_place {"place": "kitchen"} → blocked（on an obstacle）'


# --- plan 8: eyes and the six tools ---

from robot_dog_agent.objects import ObjectMemory  # noqa: E402
from robot_dog_agent.vision import Seen  # noqa: E402

EYE_TOOLS = {'list_objects', 'go_to_object', 'find_object', 'look_around', 'survey', 'forget_objects'}


class FakeEyes:
    """look() hands out the scripted sightings in turn (or raises one), remembering them like the real Eyes."""

    def __init__(self, memory, script=(), on_look=None):
        self.memory, self.script, self.targets, self.on_look = memory, list(script), [], on_look

    def look(self, target=None):
        self.targets.append(target)
        if self.on_look:
            self.on_look(len(self.targets))
        found = self.script.pop(0) if self.script else []
        if isinstance(found, BaseException):
            raise found
        for s in found:
            self.memory.add(s)
        return found


def eye_box(tmp_path, robot=None, script=(), on_look=None):
    robot = robot or FakeRobot()
    eyes = FakeEyes(ObjectMemory(str(tmp_path / 'objects.json')), script, on_look)
    notes = []
    return Toolbox(robot, TRUTH, eyes=eyes, on_note=notes.append), robot, eyes, notes


def test_without_eyes_nothing_changes():
    assert tools.declarations() == tools.DECLARATIONS and tools.system_instruction() == tools.system_instruction(False)
    assert {d['name'] for d in tools.declarations()} == {'get_pose', 'go_to_place', 'go_to_pose', 'move_relative'}
    assert 'list_objects' not in tools.system_instruction()


def test_with_eyes_six_more_tools():
    names = {d['name'] for d in tools.declarations(True)}
    assert names == {'get_pose', 'go_to_place', 'go_to_pose', 'move_relative'} | EYE_TOOLS
    assert all(d['type'] == 'function' and d['description'] for d in tools.declarations(True))
    assert 'list_objects' in tools.system_instruction(True) and 'find_object' in tools.system_instruction(True)


def test_eye_tools_without_eyes_are_errors():
    for name in EYE_TOOLS:
        r = box(FakeRobot()).call(name, {'object_id': 'obj_1', 'name': '沙發'})
        assert r == {'status': 'error', 'reason': 'no camera on this robot'}, name


def test_list_objects(tmp_path):
    t, _, eyes, _ = eye_box(tmp_path)
    eyes.memory.add(Seen('沙發', 0.0, -0.754, 1.0, 1.8))
    assert t.call('list_objects', {}) == {'status': 'ok', 'objects': [
        {'id': 'obj_1', 'name': '沙發', 'x': 0.0, 'y': -0.75, 'area': 'living, dining and kitchen area'}]}


def test_go_to_an_object(tmp_path):
    t, robot, eyes, _ = eye_box(tmp_path)
    eyes.memory.add(Seen('沙發', 0.0, -0.75, 1.0, 1.8))
    r = t.call('go_to_object', {'object_id': 'obj_1'})
    x, y, yaw = robot.pose()
    assert r['status'] == 'reached' and r['object'] == 'obj_1'
    assert 0.5 - 1e-9 <= math.hypot(x, y + 0.75) <= 1.0 + 1e-9
    assert abs(places.wrap(yaw - math.atan2(-0.75 - y, -x))) < 1e-9


def test_go_to_an_unknown_object(tmp_path):
    t, robot, _, _ = eye_box(tmp_path)
    for args in ({'object_id': 'obj_9'}, {}, {'object_id': 3}):
        assert t.call('go_to_object', args)['status'] == 'error', args
    assert robot.goals == []


def test_go_to_an_object_with_nowhere_to_stand(tmp_path):
    t, robot, eyes, _ = eye_box(tmp_path)
    eyes.memory.add(Seen('窗外的樹', 20.0, 0.0, 3.0, 1.0))
    r = t.call('go_to_object', {'object_id': 'obj_1'})
    assert r['status'] == 'blocked' and r['reason'] == 'no reachable place to stand near it' and robot.goals == []


def test_survey_visits_every_point_and_looks_four_ways(tmp_path):
    t, robot, eyes, notes = eye_box(tmp_path)
    eyes.memory.add(Seen('舊的', 1.0, 1.0, 1.0, 1.0))
    r = t.call('survey', {})
    visits = [g[:2] for g in robot.goals if g[:2] in {(p.x, p.y) for p in places.PLACES}]
    assert [v for i, v in enumerate(visits) if i == 0 or v != visits[i - 1]] == [(p.x, p.y) for p in places.PLACES]
    assert len(eyes.targets) == 32 and eyes.targets == [None] * 32
    assert notes == ['巡邏中 %d/8' % i for i in range(1, 9)]
    assert r == {'status': 'ok', 'remembered': 0, 'looks': 32, 'failed_looks': 0, 'skipped': []}


def test_survey_remembers_what_it_sees(tmp_path):
    t, _, eyes, _ = eye_box(tmp_path, script=[[Seen('沙發', 0.0, -0.75, 1.0, 1.8)]])
    assert t.call('survey', {})['remembered'] == 1 and eyes.memory.items[0].name == '沙發'


def test_survey_skips_unreachable_points(tmp_path):
    robot = FakeRobot(results=[NavResult('failed', 'Nav2 status FAILED', 3.0)])
    t, _, eyes, _ = eye_box(tmp_path, robot=robot)
    r = t.call('survey', {})
    assert r['status'] == 'ok' and r['skipped'] == ['start'] and r['looks'] == 28


def test_survey_counts_failed_looks(tmp_path):
    t, _, _, _ = eye_box(tmp_path, script=[ValueError('quota')])
    r = t.call('survey', {})
    assert r['status'] == 'ok' and r['failed_looks'] == 1 and r['looks'] == 32


def test_survey_stops_on_cancel(tmp_path):
    robot = FakeRobot()
    t, _, eyes, _ = eye_box(tmp_path, robot=robot, on_look=lambda n: n == 8 and robot.cancel())
    r = t.call('survey', {})
    assert r['status'] == 'canceled' and len(eyes.targets) == 8 and 'remembered' in r


def test_find_goes_to_what_it_finds(tmp_path):
    washer = Seen('洗衣機', 2.5, -0.6, 1.5, 0.6)
    t, robot, eyes, _ = eye_box(tmp_path, script=[[], [], [], [], [washer]])
    r = t.call('find_object', {'name': '洗衣機'})
    assert r['status'] == 'reached' and r['object'] == 'obj_1' and r['found_at'] in {p.id for p in places.PLACES}
    assert eyes.targets == ['洗衣機'] * 5
    x, y, _ = robot.pose()
    assert 0.5 - 1e-9 <= math.hypot(x - 2.5, y + 0.6) <= 1.0 + 1e-9


def test_find_starts_at_the_nearest_point(tmp_path):
    robot = FakeRobot((3.5, 0.3, 0.0))                     # at the dining table
    t, _, _, _ = eye_box(tmp_path, robot=robot, script=[[Seen('洗衣機', 2.5, -0.6, 1.5, 0.6)]])
    assert t.call('find_object', {'name': '洗衣機'})['found_at'] == 'dining_table'


def test_find_gives_up_after_every_point(tmp_path):
    t, _, eyes, _ = eye_box(tmp_path)
    r = t.call('find_object', {'name': '鋼琴'})
    assert r == {'status': 'not_found', 'reason': 'looked from every observation point', 'looks': 32}


def test_find_stops_on_cancel(tmp_path):
    robot = FakeRobot()
    t, _, eyes, _ = eye_box(tmp_path, robot=robot, on_look=lambda n: n == 3 and robot.cancel())
    assert t.call('find_object', {'name': '鋼琴'})['status'] == 'canceled' and len(eyes.targets) == 3


def test_find_needs_a_name(tmp_path):
    t, robot, _, _ = eye_box(tmp_path)
    for args in ({'name': ''}, {'name': '  '}, {'name': 3}, {}):
        assert t.call('find_object', args)['status'] == 'error', args
    assert robot.goals == []


def test_look_around(tmp_path):
    t, robot, eyes, _ = eye_box(tmp_path, script=[[Seen('沙發', 0.0, -0.75, 1.0, 1.8)], [], [Seen('餐桌', 2.0, 0.5, 2.0, 1.0)]])
    r = t.call('look_around', {})
    assert r == {'status': 'ok', 'seen': ['沙發', '餐桌'], 'remembered': 2} and len(eyes.targets) == 4
    assert all(g[:2] == (0.0, 0.0) for g in robot.goals)  # turns on the spot only


def test_forget_objects(tmp_path):
    t, _, eyes, _ = eye_box(tmp_path)
    eyes.memory.add(Seen('沙發', 0.0, -0.75, 1.0, 1.8))
    assert t.call('forget_objects', {}) == {'status': 'ok'} and eyes.memory.items == []
