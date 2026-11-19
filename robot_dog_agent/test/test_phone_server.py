import json
import os
import socket
import threading
import time
import urllib.request
from urllib.error import HTTPError

import cv2
import numpy as np
import pytest

from robot_dog_agent.phone_code import Guard
from robot_dog_agent.phone_server import PhoneServer, Sources

CODE = '123456'
FAKE_KEY = 'AIzaFAKE-KEY-0123456789'
SAMPLES = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'robot_dog_phone', 'app', 'src', 'test',
                       'resources', 'api')


class FakeSession:
    """The Session calls the server makes; like the real one, nothing but idle starts a command."""

    def __init__(self, slow=0.0):
        self.state, self.calls, self.slow, self.api_key = 'idle', [], slow, FAKE_KEY
        self.on_event = lambda kind, text: None
        self._pose = (0.12, -0.3, 0.0)

    def _start(self, *call):
        if self.state != 'idle':
            return False
        time.sleep(self.slow)  # the gap between the check and the change, where a second request could slip in
        self.state = 'busy'
        self.calls.append(call)
        self.on_event('state', 'busy')
        return True

    def submit(self, text):
        return self._start('submit', text)

    def go_to(self, x, y):
        return self._start('go_to', x, y)

    def survey(self):
        return self._start('survey')

    def stop(self):
        self.calls.append(('stop',))

    def finish(self):
        self.state = 'idle'
        self.on_event('state', 'idle')

    def pose(self):
        return self._pose

    def status_text(self):
        return {'idle': '待命', 'busy': '處理中…'}[self.state]


GRID = np.zeros((120, 200), dtype=np.int8)


class Feeds:
    def __init__(self):
        self.map = (GRID, 0.05, (-5.0, -3.0), 1)
        self.path = None
        self.frame = None
        self.objects = [('沙發', 1.0, 2.0)]


def make(session=None, feeds=None, camera=True, can_survey=True, **kw):
    session, feeds = session or FakeSession(), feeds or Feeds()
    sources = Sources(latest_frame=(lambda: feeds.frame) if camera else None, latest_map=lambda: feeds.map,
                      latest_path=lambda: feeds.path, objects=lambda: feeds.objects, can_survey=can_survey)
    server = PhoneServer(session, Guard(CODE, sleep=lambda s: None), sources, host='127.0.0.1', port=0, **kw)
    session.on_event = server.on_event
    server.start()
    return server, session, feeds


@pytest.fixture
def served():
    made = []

    def build(**kw):
        made.append(make(**kw))
        return made[-1]
    yield build
    for server, _, _ in made:
        server.close()


def call(server, path, body=None, code=CODE, method=None):
    headers = {'Authorization': 'Bearer ' + code} if code else {}
    data = None if body is None else (body if isinstance(body, bytes) else json.dumps(body).encode())
    request = urllib.request.Request('http://127.0.0.1:%d%s' % (server.port, path), data=data, headers=headers,
                                     method=method)
    try:
        with urllib.request.urlopen(request, timeout=10) as r:
            return r.status, r.read(), r.headers
    except HTTPError as e:
        return e.code, e.read(), e.headers


def body(response):
    return json.loads(response[1].decode('utf-8'))


def test_ping_needs_no_code(served):
    server, _, _ = served()
    status, data, _ = call(server, '/api/ping', code=None)
    assert status == 200 and json.loads(data) == {'app': 'robot_dog_phone', 'version': 1}


def test_everything_else_needs_the_code(served):
    server, session, _ = served()
    assert call(server, '/api/hello')[0] == 200
    for path in ('/api/hello', '/api/state', '/api/events?after=0', '/api/map.png', '/api/camera.mjpg'):
        assert call(server, path, code=None)[0] == 401
    for path in ('/api/hello', '/api/state', '/api/map.png'):  # 9 wrong in all: the 10th would lock the address
        assert call(server, path, code='000000')[0] == 401
    assert call(server, '/api/command', {'text': '去廚房'}, code='000000')[0] == 401
    assert session.calls == []


def test_hello(served):
    server, _, _ = served()
    hello = body(call(server, '/api/hello'))
    assert len(hello['places']) == 8 and hello['places'][5]['short'] == '廚房'
    assert set(hello['places'][0]) == {'id', 'name', 'short', 'x', 'y'}
    assert hello['can_survey'] is True and hello['camera'] is True
    assert len(hello['boot']) == 8 and int(hello['boot'], 16) >= 0
    other, _, _ = served(camera=False, can_survey=False)
    hello = body(call(other, '/api/hello'))
    assert hello['camera'] is False and hello['can_survey'] is False


def test_state_and_path_versions(served):
    server, session, feeds = served()
    assert body(call(server, '/api/command', {'text': '去廚房'}))['ok'] is True
    feeds.path = ([(0.0, 0.0), (1.0, 0.5)], time.monotonic() + 1.0)
    state = body(call(server, '/api/state?path=-1'))
    assert state['state'] == 'busy' and state['path'] == [[0.0, 0.0], [1.0, 0.5]]
    again = body(call(server, '/api/state?path=%d' % state['path_version']))
    assert again['path'] is None
    assert state['objects'] == [{'name': '沙發', 'x': 1.0, 'y': 2.0}]
    assert state['map'] == {'version': 1, 'resolution': 0.05, 'origin': [-5.0, -3.0], 'width': 200, 'height': 120}
    session.finish()
    assert body(call(server, '/api/state?path=%d' % state['path_version']))['path'] == []


def test_events_long_poll(served):
    server, session, _ = served(long_poll=3.0)
    server.on_event('log', 'ready')
    events = body(call(server, '/api/events?after=0'))
    assert events['events'] == [{'id': 1, 'kind': 'log', 'text': 'ready'}] and events['next'] == 1
    assert events['boot'] == body(call(server, '/api/hello'))['boot']
    threading.Timer(0.3, lambda: server.on_event('reply', '到了')).start()
    start = time.monotonic()
    events = body(call(server, '/api/events?after=1'))
    assert [e['text'] for e in events['events']] == ['到了'] and time.monotonic() - start < 2.0
    start = time.monotonic()
    events = body(call(server, '/api/events?after=2'))
    assert events['events'] == [] and events['next'] == 2 and 2.5 < time.monotonic() - start < 5.0


def test_command_go_to_survey_stop_reach_the_session(served):
    server, session, _ = served()
    assert body(call(server, '/api/command', {'text': '去廚房'})) == {'ok': True}
    assert body(call(server, '/api/stop', b'')) == {'ok': True}
    session.finish()
    assert body(call(server, '/api/go_to', {'x': 1.2, 'y': 0.5}))['ok'] is True
    session.finish()
    assert body(call(server, '/api/survey', b''))['ok'] is True
    assert session.calls == [('submit', '去廚房'), ('stop',), ('go_to', 1.2, 0.5), ('survey',)]
    events = body(call(server, '/api/events?after=0'))['events']
    assert [(e['kind'], e['text']) for e in events][:2] == [('state', 'busy'), ('state', 'idle')]


def test_busy_gives_ok_false(served):
    server, session, _ = served()
    session.state = 'busy'
    assert body(call(server, '/api/command', {'text': '去廚房'}))['ok'] is False
    assert body(call(server, '/api/go_to', {'x': 1.0, 'y': 1.0}))['ok'] is False
    assert session.calls == []


def test_survey_without_eyes_is_refused(served):
    server, session, _ = served(can_survey=False)
    assert body(call(server, '/api/survey', b''))['ok'] is False and session.calls == []


def test_two_commands_at_once_only_one_runs(served):
    server, session, _ = served(session=FakeSession(slow=0.2))
    results = []
    threads = [threading.Thread(target=lambda: results.append(body(call(server, '/api/command', {'text': '去廚房'}))))
               for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(5)
    assert sorted(r['ok'] for r in results) == [False, True] and len(session.calls) == 1


def test_go_to_sets_the_target(served):
    server, session, _ = served()
    call(server, '/api/go_to', {'x': 1.2, 'y': 0.5})
    assert body(call(server, '/api/state'))['target'] == [1.2, 0.5]
    session.state = 'busy'
    call(server, '/api/command', {'text': 'x'})  # refused: the target stays
    assert body(call(server, '/api/state'))['target'] == [1.2, 0.5]


def test_bad_json_is_400(served):
    server, session, _ = served()
    assert call(server, '/api/command', b'not json')[0] == 400
    assert call(server, '/api/command', {'words': 'x'})[0] == 400
    assert call(server, '/api/go_to', {'x': 'a', 'y': 1})[0] == 400
    assert call(server, '/api/go_to', {'x': True, 'y': 1})[0] == 400
    assert call(server, '/api/go_to', {'x': float('nan'), 'y': 1})[0] == 400
    assert call(server, '/api/nothing')[0] == 404
    assert session.calls == []


def test_map_png_and_404(served):
    server, _, feeds = served()
    status, data, headers = call(server, '/api/map.png')
    image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_GRAYSCALE)
    assert status == 200 and headers['Content-Type'] == 'image/png' and image.shape == (120, 200)
    feeds.map = None
    assert call(server, '/api/map.png')[0] == 404
    assert body(call(server, '/api/state'))['map'] is None


def test_map_version_follows_the_feed(served):
    server, _, feeds = served()
    assert body(call(server, '/api/state'))['map']['version'] == 1
    feeds.map = (np.zeros((150, 260), dtype=np.int8), 0.05, (-6.0, -3.0), 2)
    info = body(call(server, '/api/state'))['map']
    assert info['version'] == 2 and info['width'] == 260 and info['origin'] == [-6.0, -3.0]
    image = cv2.imdecode(np.frombuffer(call(server, '/api/map.png')[1], np.uint8), cv2.IMREAD_GRAYSCALE)
    assert image.shape == (150, 260)


def red_frame():
    rgb = np.zeros((2, 4, 3), dtype=np.uint8)
    rgb[:, :, 0] = 255
    return 4, 2, rgb.tobytes(), time.monotonic()


def read_part(stream):
    assert stream.readline() == b'--frame\r\n'
    headers = {}
    while True:
        line = stream.readline().strip()
        if not line:
            break
        name, value = line.decode().split(':', 1)
        headers[name.strip().lower()] = value.strip()
    data = stream.read(int(headers['content-length']))
    assert stream.readline() == b'\r\n'
    return headers, data


def test_camera_stream_sends_jpegs(served):
    server, _, feeds = served()
    feeds.frame = red_frame()
    request = urllib.request.Request('http://127.0.0.1:%d/api/camera.mjpg' % server.port,
                                     headers={'Authorization': 'Bearer ' + CODE})
    with urllib.request.urlopen(request, timeout=5) as r:
        assert r.headers['Content-Type'] == 'multipart/x-mixed-replace; boundary=frame'
        headers, data = read_part(r)
        assert headers['content-type'] == 'image/jpeg' and data[:2] == b'\xff\xd8'
        feeds.frame = red_frame()
        _, data = read_part(r)
        image = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        assert image.shape == (2, 4, 3) and image[0, 0, 2] > 200  # BGR: red


def test_no_camera_is_404(served):
    server, _, _ = served(camera=False)
    assert call(server, '/api/camera.mjpg')[0] == 404


def test_client_hangs_up_mid_stream(served, capfd):
    server, _, feeds = served()
    feeds.frame = red_frame()
    sock = socket.create_connection(('127.0.0.1', server.port), timeout=5)
    sock.sendall(b'GET /api/camera.mjpg HTTP/1.1\r\nHost: x\r\nAuthorization: Bearer ' + CODE.encode() + b'\r\n\r\n')
    received = b''
    while b'--frame' not in received:  # the headers may come in a packet of their own
        received += sock.recv(65536)
    sock.close()
    for _ in range(5):  # new frames go to a closed socket
        time.sleep(0.15)
        feeds.frame = red_frame()
    assert call(server, '/api/ping', code=None)[0] == 200
    assert 'Traceback' not in capfd.readouterr().err


def test_nothing_contains_the_key(served):
    server, session, feeds = served()
    feeds.frame = red_frame()
    server.on_event('log', 'ready')
    call(server, '/api/command', {'text': '去廚房'})
    for path in ('/api/ping', '/api/hello', '/api/state?path=-1', '/api/events?after=0', '/api/map.png'):
        assert FAKE_KEY.encode() not in call(server, path)[1]


def sample_server(served):
    """A server in a fixed state: busy going to a clicked point, with a path, a thing and the map."""
    server, session, feeds = served(boot='0a1b2c3d')
    server.on_event('log', '可以開始了：打字、或按下面的地點按鈕。')
    call(server, '/api/go_to', {'x': 1.2, 'y': 0.5})
    feeds.path = ([(0.12, -0.3), (0.6, 0.1), (1.2, 0.5)], time.monotonic() + 1.0)
    server.on_event('log', '  [工具] go_to_pose {"x": 1.2, "y": 0.5} → reached')
    server.on_event('user', '去廚房')
    server.on_event('reply', '到廚房了。')
    server.on_event('error', 'Gemini 沒有回應')
    return server


def test_android_samples_are_current(served):
    server = sample_server(served)
    produced = {'hello.json': body(call(server, '/api/hello')),
                'state.json': body(call(server, '/api/state?path=-1')),
                'events.json': body(call(server, '/api/events?after=0'))}
    for name, value in produced.items():
        path = os.path.join(SAMPLES, name)
        if os.environ.get('UPDATE_SAMPLES'):
            os.makedirs(SAMPLES, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(value, f, ensure_ascii=False, indent=2)
                f.write('\n')
        with open(path, encoding='utf-8') as f:
            assert json.load(f) == value, name + ' is out of date: run with UPDATE_SAMPLES=1'
