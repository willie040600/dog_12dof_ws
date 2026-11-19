import threading
import time

import cv2
import numpy as np

from robot_dog_agent.phone_state import (MAX_PATH_POINTS, EventLog, MapTracker, grid_png, read_objects, state_json)


class Clock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t


class FakeSession:
    def __init__(self, state='idle', pose=(0.0, 0.0, 0.0), status='待命'):
        self.state, self._pose, self._status = state, pose, status

    def pose(self):
        return self._pose

    def status_text(self):
        return self._status


def test_event_log_numbers_and_after():
    log = EventLog()
    assert [log.add('log', str(i)) for i in range(3)] == [1, 2, 3]
    events, nxt = log.after(0, 0)
    assert [e['id'] for e in events] == [1, 2, 3] and nxt == 3
    assert events[0] == {'id': 1, 'kind': 'log', 'text': '0'}
    assert log.after(3, 0) == ([], 3)


def test_event_log_wakes_a_waiter():
    log, out = EventLog(), {}

    def wait():
        start = time.monotonic()
        out['events'], _ = log.after(0, 5)
        out['took'] = time.monotonic() - start
    thread = threading.Thread(target=wait)
    thread.start()
    time.sleep(0.1)
    log.add('reply', 'hi')
    thread.join(2)
    assert [e['text'] for e in out['events']] == ['hi'] and out['took'] < 0.5


def test_event_log_keeps_the_newest_300():
    log = EventLog()
    for i in range(305):
        log.add('log', str(i))
    events, nxt = log.after(0, 0)
    assert len(events) == 300 and events[0]['id'] == 6 and nxt == 305


def test_event_log_number_from_another_boot_returns_at_once():
    log = EventLog()
    log.add('log', 'a')
    start = time.monotonic()
    assert log.after(57, 5) == ([], 1)  # a phone still counting from before a restart: no waiting
    assert time.monotonic() - start < 0.5


def test_event_log_close_wakes_waiters():
    log, out = EventLog(), {}

    def wait():
        start = time.monotonic()
        log.after(0, 25)
        out['took'] = time.monotonic() - start
    thread = threading.Thread(target=wait)
    thread.start()
    time.sleep(0.1)
    log.close()
    thread.join(2)
    assert out['took'] < 1.0
    assert log.after(0, 25) == ([], 0)


PATH = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]


def test_path_only_for_the_running_command():
    clock = Clock(10.0)
    tracker = MapTracker(clock)
    assert tracker.snapshot((PATH, 9.0))[1] == []           # idle
    tracker.on_state('busy')
    assert tracker.snapshot((PATH, 9.0))[1] == []           # arrived before this command
    assert tracker.snapshot((PATH, 10.5))[1] == [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
    tracker.on_state('idle')
    assert tracker.snapshot((PATH, 10.5))[1] == []
    assert tracker.snapshot(None)[1] == []


def test_path_version_changes_with_the_path():
    tracker = MapTracker(Clock(0.0))
    tracker.on_state('busy')
    v1 = tracker.snapshot((PATH, 1.0))[0]
    assert tracker.snapshot((PATH, 1.0))[0] == v1
    v2 = tracker.snapshot((PATH + [(2.0, 1.0)], 1.0))[0]
    assert v2 == v1 + 1


def test_long_path_is_sampled():
    tracker = MapTracker(Clock(0.0))
    tracker.on_state('busy')
    points = [(i * 0.01, 0.0) for i in range(1000)]
    path = tracker.snapshot((points, 1.0))[1]
    assert len(path) == MAX_PATH_POINTS == 300
    assert path[0] == [0.0, 0.0] and path[-1] == [9.99, 0.0]


def test_target_held_two_seconds():
    clock = Clock(5.0)
    tracker = MapTracker(clock)
    tracker.on_state('busy')
    tracker.set_target(1.2, 0.5)
    assert tracker.snapshot(None)[2] == [1.2, 0.5]
    clock.t = 10.0
    tracker.on_state('idle')
    clock.t = 11.9
    assert tracker.snapshot(None)[2] == [1.2, 0.5]
    clock.t = 12.1
    assert tracker.snapshot(None)[2] is None


def test_next_command_clears_the_target():
    clock = Clock(0.0)
    tracker = MapTracker(clock)
    tracker.on_state('busy')
    tracker.set_target(1.0, 1.0)
    tracker.on_state('idle')
    clock.t = 0.5
    tracker.on_state('busy')
    assert tracker.snapshot(None)[2] is None


MAP_INFO = {'version': 1, 'resolution': 0.05, 'origin': [-5.0, -3.0], 'width': 200, 'height': 120}


def test_state_json_fields():
    tracker = MapTracker(Clock(0.0))
    out = state_json(FakeSession(pose=(0.12, -0.3, 0.0)), tracker, None, MAP_INFO, [{'name': '沙發', 'x': 1, 'y': 2}], -1)
    assert set(out) == {'state', 'status', 'pose', 'pose_text', 'path_version', 'path', 'target', 'objects', 'map'}
    assert out['state'] == 'idle' and out['status'] == '待命'
    assert out['pose'] == [0.12, -0.3, 0.0] and out['pose_text'].startswith('位置 (0.12, -0.30)')
    assert out['map'] == MAP_INFO and out['objects'] == [{'name': '沙發', 'x': 1, 'y': 2}]


def test_state_json_path_only_when_version_differs():
    tracker = MapTracker(Clock(0.0))
    tracker.on_state('busy')
    session = FakeSession(state='busy')
    first = state_json(session, tracker, (PATH, 1.0), None, [], -1)
    assert len(first['path']) == 3
    again = state_json(session, tracker, (PATH, 1.0), None, [], first['path_version'])
    assert again['path'] is None and again['path_version'] == first['path_version']


def test_state_json_no_pose_no_map():
    out = state_json(FakeSession(pose=None), MapTracker(Clock(0.0)), None, None, [], -1)
    assert out['pose'] is None and out['pose_text'] == '位置未知' and out['map'] is None


def test_grid_png_round_trip():
    grid = np.array([[0, 100, -1],
                     [100, 0, 0]], dtype=np.int8)
    image = cv2.imdecode(np.frombuffer(grid_png(grid), np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (2, 3)
    assert list(image[0]) == [40, 245, 245] and list(image[1]) == [245, 40, 160]


def test_read_objects_retries_then_keeps_last():
    calls = []

    def flaky():
        calls.append(1)
        if len(calls) == 1:
            raise RuntimeError('dictionary changed size during iteration')
        return [('沙發', 1.0, 2.0)]
    assert read_objects(flaky, []) == [('沙發', 1.0, 2.0)]

    def broken():
        raise RuntimeError('dictionary changed size during iteration')
    assert read_objects(broken, ['last']) == ['last']
