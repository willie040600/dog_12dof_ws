"""The HTTP server the phone app talks to (spec 2026-11-19, 4). Standard library only, plus the pure modules here.

JSON in and out, except the map (PNG) and the camera (MJPEG). Every request but /api/ping needs the pairing code.
It knows the session and a few "latest data" functions, nothing about ROS: the tests run it with fakes.
"""
import json
import math
import secrets
import threading
import time
from collections import namedtuple
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from robot_dog_agent import places
from robot_dog_agent.phone_state import DECIMALS, EventLog, MapTracker, grid_png, read_objects, state_json
from robot_dog_agent.vision import encode_jpeg

# latest_frame() -> (width, height, RGB bytes, time.monotonic()) | None, or None for no camera;
# latest_map() -> (grid, resolution, origin, version) | None; latest_path() -> (points, time) | None, or None;
# objects() -> [(name, x, y)], or None; can_survey: bool
Sources = namedtuple('Sources', 'latest_frame latest_map latest_path objects can_survey')

LONG_POLL = 25.0     # s an /api/events request waits for something new
JPEG_QUALITY = 70
FRAME_PERIOD = 0.1   # s between camera pictures
KEEPALIVE = 1.0      # s without a picture before an empty part is sent: shows "no picture" on the phone, and a
                     # write is the only way to notice that the phone has gone (else its thread would stay)
STALE_AFTER = 3.0    # s; an older picture is not sent
ERRORS = {401: '配對碼錯誤', 429: '錯太多次，請一分鐘後再試', 400: '看不懂這個請求', 404: '沒有這個東西'}


class _Server(ThreadingHTTPServer):
    daemon_threads = True

    def handle_error(self, request, client_address):
        import sys
        if isinstance(sys.exc_info()[1], (ConnectionError, TimeoutError)):
            return  # a phone that went away (Wi-Fi, app in the background): nothing to report
        super().handle_error(request, client_address)


class PhoneServer:
    def __init__(self, session, guard, sources, host='0.0.0.0', port=8765, clock=time.monotonic, long_poll=LONG_POLL,
                 boot=None):
        self.session, self.guard, self.sources, self.clock, self.long_poll = session, guard, sources, clock, long_poll
        self.boot = boot or secrets.token_hex(4)
        self.events, self.tracker = EventLog(), MapTracker(clock)
        self._command_lock = threading.Lock()  # Session checks "idle" without a lock: one command at a time here
        self._closing = threading.Event()
        self._objects = []
        self._httpd = _Server((host, port), self._handler())
        self._thread = None

    @property
    def port(self):
        return self._httpd.server_address[1]

    def on_event(self, kind, text):
        """Session.on_event: called from the session's threads."""
        self.events.add(kind, text)
        if kind == 'state':
            self.tracker.on_state(text)

    def start(self):
        self._thread = threading.Thread(target=self._httpd.serve_forever, args=(0.1,), daemon=True)
        self._thread.start()

    def close(self):
        self._closing.set()
        self.events.close()
        self._httpd.shutdown()
        self._httpd.server_close()

    # the answers -----------------------------------------------------------------------------------------------

    def hello(self):
        return {'boot': self.boot,
                'places': [{'id': p.id, 'name': p.name, 'short': p.name.split('（')[0], 'x': p.x, 'y': p.y}
                           for p in places.PLACES],
                'can_survey': bool(self.sources.can_survey), 'camera': self.sources.latest_frame is not None}

    def _map(self):
        latest = self.sources.latest_map() if self.sources.latest_map else None
        return latest

    def state(self, path_version_seen):
        latest = self._map()
        info = None
        if latest is not None:
            grid, resolution, origin, version = latest
            info = {'version': version, 'resolution': resolution, 'origin': [float(origin[0]), float(origin[1])],
                    'width': int(grid.shape[1]), 'height': int(grid.shape[0])}
        if self.sources.objects:
            self._objects = read_objects(self.sources.objects, self._objects)
        objects = [{'name': n, 'x': round(float(x), DECIMALS), 'y': round(float(y), DECIMALS)} for n, x, y in self._objects]
        path = self.sources.latest_path() if self.sources.latest_path else None
        return state_json(self.session, self.tracker, path, info, objects, path_version_seen)

    def command(self, name, args):
        """ok of a Session call; one at a time, so two phones cannot both start a command."""
        with self._command_lock:
            if name == 'command':
                return self.session.submit(args['text'])
            if name == 'survey':
                return bool(self.sources.can_survey) and self.session.survey()
            if self.session.state != 'idle':  # refused anyway: keep the running command's target
                return False
            # the target first: a refused point (a wall) ends its job, busy then idle, before go_to returns, and the
            # target must already be there for idle to start its 2 s
            self.tracker.set_target(args['x'], args['y'])
            ok = self.session.go_to(args['x'], args['y'])
            if not ok:
                self.tracker.clear_target()
            return ok

    def stream_camera(self, write):
        """Write JPEG parts until the phone goes away or the server closes."""
        shown, written = None, time.monotonic()
        while not self._closing.is_set():
            frame = self.sources.latest_frame()
            jpeg = None
            if frame is not None:
                width, height, rgb, stamp = frame
                if stamp != shown and self.clock() - stamp <= STALE_AFTER and len(rgb) == width * height * 3:
                    jpeg, shown = encode_jpeg(rgb, width, height, JPEG_QUALITY), stamp
            if jpeg is None and time.monotonic() - written >= KEEPALIVE:
                jpeg = b''
            if jpeg is not None:
                write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n' % len(jpeg)
                      + jpeg + b'\r\n')
                written = time.monotonic()
            self._closing.wait(FRAME_PERIOD)

    # HTTP ------------------------------------------------------------------------------------------------------

    def _handler(self):
        server = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def _send(self, status, data, content_type):
                self.send_response(status)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(data)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(data)

            def _json(self, value, status=200):
                self._send(status, json.dumps(value, ensure_ascii=False).encode('utf-8'),
                           'application/json; charset=utf-8')

            def _error(self, status):
                self._json({'error': ERRORS[status]}, status)

            def _allowed(self):
                status = server.guard.check(self.headers.get('Authorization'), self.client_address[0])
                if status != 200:
                    self._error(status)
                return status == 200

            def do_GET(self):
                url = urlparse(self.path)
                query = parse_qs(url.query)
                if url.path == '/api/ping':
                    return self._json({'app': 'robot_dog_phone', 'version': 1})
                if url.path not in ('/api/hello', '/api/state', '/api/events', '/api/map.png', '/api/camera.mjpg'):
                    return self._error(404)
                if not self._allowed():
                    return
                try:
                    if url.path == '/api/hello':
                        return self._json(server.hello())
                    if url.path == '/api/state':
                        return self._json(server.state(int(query.get('path', ['-1'])[0])))
                    if url.path == '/api/events':
                        events, nxt = server.events.after(int(query.get('after', ['0'])[0]), server.long_poll)
                        return self._json({'boot': server.boot, 'events': events, 'next': nxt})
                except ValueError:
                    return self._error(400)
                if url.path == '/api/map.png':
                    latest = server._map()
                    if latest is None:
                        return self._error(404)
                    return self._send(200, grid_png(latest[0]), 'image/png')
                if server.sources.latest_frame is None:
                    return self._error(404)
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()

                def write(data):
                    self.wfile.write(data)
                    self.wfile.flush()
                try:
                    server.stream_camera(write)
                except (ConnectionError, TimeoutError):
                    pass
                self.close_connection = True

            def do_POST(self):
                url = urlparse(self.path)
                if url.path not in ('/api/command', '/api/go_to', '/api/survey', '/api/stop'):
                    return self._error(404)
                if not self._allowed():
                    return
                try:
                    raw = self.rfile.read(int(self.headers.get('Content-Length') or 0))
                    args = json.loads(raw.decode('utf-8')) if raw.strip() else {}
                except (ValueError, UnicodeDecodeError):
                    return self._error(400)
                if url.path == '/api/stop':
                    server.session.stop()
                    return self._json({'ok': True})
                if url.path == '/api/survey':
                    return self._json({'ok': bool(server.command('survey', {}))})
                if url.path == '/api/command':
                    if not isinstance(args, dict) or not isinstance(args.get('text'), str):
                        return self._error(400)
                    return self._json({'ok': bool(server.command('command', args))})
                if not isinstance(args, dict) or not all(_number(args.get(k)) for k in ('x', 'y')):
                    return self._error(400)
                return self._json({'ok': bool(server.command('go_to', {'x': float(args['x']),
                                                                        'y': float(args['y'])}))})
        return Handler


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
