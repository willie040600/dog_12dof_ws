import math
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')   # before PyQt5 is imported: no window appears

from PyQt5.QtWidgets import QApplication  # noqa: E402

from robot_dog_agent import gui  # noqa: E402
from robot_dog_agent.gui import NO_CAMERA, MainWindow  # noqa: E402

APP = QApplication.instance() or QApplication([])
RED, GREEN = bytes([255, 0, 0]), bytes([0, 255, 0])


class FakeSession:
    def __init__(self, state='idle'):
        self.state, self.accept, self.pose_value = state, True, None
        self.submitted, self.stops, self.shutdowns = [], 0, 0

    def submit(self, text):
        self.submitted.append(text)
        return self.accept

    def stop(self):
        self.stops += 1

    def shutdown(self):
        self.shutdowns += 1
        return True

    def pose(self):
        return self.pose_value

    def status_text(self):
        return '待命'


def window(session=None, **kw):
    session = session or FakeSession()
    w = MainWindow(session, **kw)
    w.on_event('state', session.state)
    return w, session


def has_picture(label):
    return label.pixmap() is not None and not label.pixmap().isNull()


def test_place_buttons_send_go_commands():
    w, session = window()
    names = ['起點', '客廳西側', '主臥室', '次臥室', '餐桌旁', '廚房', '玄關', '浴室門口']
    assert [b.text() for b in w.place_buttons] == names
    for button in w.place_buttons:
        button.click()
    assert session.submitted == ['去' + name for name in names]


def test_enter_and_the_send_button_submit_and_clear():
    w, session = window()
    w.input.setText('去廚房')
    w.input.returnPressed.emit()
    assert w.input.text() == ''
    w.input.setText('你在哪')
    w.send_button.click()
    assert session.submitted == ['去廚房', '你在哪'] and w.input.text() == ''


def test_a_refused_submit_keeps_the_text():
    w, session = window()
    session.accept = False
    w.input.setText('去廚房')
    w.send_button.click()
    assert w.input.text() == '去廚房'


def test_widgets_follow_the_state():
    w, session = window()

    def enabled():
        entry = [w.input.isEnabled(), w.send_button.isEnabled()] + [b.isEnabled() for b in w.place_buttons]
        assert len(set(entry)) == 1
        return entry[0], w.stop_button.isEnabled()

    seen = {}
    for state in ('idle', 'busy', 'starting', 'failed', 'idle'):
        w.on_event('state', state)
        seen[state] = enabled()
    assert seen == {'idle': (True, False), 'busy': (False, True), 'starting': (False, False),
                    'failed': (False, False)}


def test_events_become_lines_in_the_log():
    w, _ = window()
    for kind, text in [('user', '去廚房'), ('log', '  [工具] x'), ('reply', '到了'), ('error', '被限流')]:
        w.on_event(kind, text)
    assert w.log.toPlainText() == '你：去廚房\n  [工具] x\n狗：到了\n[錯誤] 被限流'


def test_stop_button_calls_stop():
    w, session = window(FakeSession('busy'))
    w.stop_button.click()
    assert session.stops == 1


def test_a_fresh_frame_is_shown():
    w, _ = window(latest_frame=lambda: (2, 1, RED + GREEN, 100.0), clock=lambda: 101.0)
    w.refresh_frame()
    assert has_picture(w.image_label) and w.image_label.text() == ''


def test_a_stale_or_missing_frame_shows_the_hint():
    now = [101.0]
    w, _ = window(latest_frame=lambda: (2, 1, RED + GREEN, 100.0), clock=lambda: now[0])
    w.refresh_frame()
    assert has_picture(w.image_label)
    now[0] = 104.0                                              # the frame is now older than 3 s
    w.refresh_frame()
    assert not has_picture(w.image_label) and w.image_label.text() == NO_CAMERA
    for kw in (dict(latest_frame=lambda: None), dict(latest_frame=None), dict()):
        w, _ = window(**kw)
        w.refresh_frame()
        assert not has_picture(w.image_label) and w.image_label.text() == NO_CAMERA, kw
    w, _ = window(no_camera_text='沒有')
    w.refresh_frame()
    assert w.image_label.text() == '沒有'


def test_a_frame_that_cannot_be_shown_is_skipped():
    w, _ = window(latest_frame=lambda: (2, 1, RED, 100.0), clock=lambda: 100.5)   # 3 bytes for 2 pixels
    w.refresh_frame()
    assert not has_picture(w.image_label) and w.image_label.text() == NO_CAMERA


def test_a_new_size_is_shown():
    frames = [(2, 1, RED + GREEN, 100.0)]
    w, _ = window(latest_frame=lambda: frames[0], clock=lambda: 100.5)
    w.refresh_frame()
    frames[0] = (1, 2, RED + GREEN, 100.2)
    w.refresh_frame()
    assert has_picture(w.image_label)


def test_status_bar():
    w, session = window()
    w.refresh_status()
    assert w.status_left.text() == '位置未知' and w.status_right.text() == '待命'
    session.pose_value = (2.48, -0.61, -math.pi / 2)
    w.refresh_status()
    assert w.status_left.text() == '位置 (2.48, -0.61) 朝向 -90°　客餐廚'


def test_closing_shuts_the_session_down():
    w, session = window()
    w.close()
    assert session.shutdowns == 1 and w.clean_shutdown is True


def test_events_after_close_are_ignored():
    w, _ = window()
    w.on_event('reply', '到了')
    w.close()
    w.on_event('reply', '晚到')
    w.on_event('state', 'busy')
    w.refresh_frame()
    w.refresh_status()
    assert w.log.toPlainText() == '狗：到了'


def test_events_cross_threads_through_the_signal():
    w, _ = window()
    w.events.emit('reply', '到了')
    APP.processEvents()
    assert w.log.toPlainText() == '狗：到了'


def test_main_without_a_key(monkeypatch, capsys):
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    assert gui.main(['--no-robot']) == 2
    assert 'GEMINI_API_KEY' in capsys.readouterr().out


def test_create_without_a_robot_has_the_saved_map(monkeypatch):
    import argparse
    w, session, feeds = gui.create(argparse.Namespace(model='m', no_robot=True), 'k')
    try:
        session._job.join(5.0)
        APP.processEvents()
        assert feeds == [] and session.state == 'idle' and w.input.isEnabled()
        w.refresh_frame()
        w.refresh_map()
        assert w.image_label.text() == gui.NO_ROBOT_CAMERA and w.map_widget.has_map
        assert '可以開始了' in w.log.toPlainText()
    finally:
        w.close()


# --- plan 7: the map ---

from robot_dog_agent.map_view import load_saved_map  # noqa: E402

SAVED = load_saved_map()


def saved_map(version=1):
    return lambda: SAVED + (version,)


class MapSession(FakeSession):
    def __init__(self, state='idle'):
        super().__init__(state)
        self.went = []

    def go_to(self, x, y):
        self.went.append((x, y))
        return self.accept


def test_a_map_click_sends_go_to_and_marks_the_target():
    w, session = window(MapSession(), latest_map=saved_map())
    w.refresh_map()
    assert w.map_widget.has_map
    w.map_widget.clicked.emit(2.6, 2.0)
    assert session.went == [(2.6, 2.0)] and w.map_widget.target == (2.6, 2.0)


def test_a_refused_go_to_marks_nothing():
    w, session = window(MapSession(), latest_map=saved_map())
    session.accept = False
    w.map_widget.clicked.emit(2.6, 2.0)
    assert w.map_widget.target is None


def test_the_map_is_clickable_only_when_idle():
    w, _ = window(MapSession())
    seen = {}
    for state in ('idle', 'busy', 'starting', 'failed'):
        w.on_event('state', state)
        seen[state] = w.map_widget.clickable
    assert seen == {'idle': True, 'busy': False, 'starting': False, 'failed': False}


def test_the_dog_follows_the_session_pose():
    w, session = window(MapSession(), latest_map=saved_map())
    session.pose_value = (1.0, 0.5, 0.3)
    w.refresh_map()
    assert w.map_widget.pose == (1.0, 0.5, 0.3)


def test_an_old_path_is_not_drawn():
    now, path = [10.0], [([(0.0, 0.0), (1.0, 0.0)], 9.0)]
    w, session = window(MapSession(), latest_map=saved_map(), latest_path=lambda: path[0], clock=lambda: now[0])
    w.on_event('state', 'busy')
    w.refresh_map()
    assert w.map_widget.path == []
    path[0] = ([(0.0, 0.0), (1.0, 0.0)], 11.0)
    w.refresh_map()
    assert w.map_widget.path == [(0.0, 0.0), (1.0, 0.0)]
    w.on_event('state', 'idle')
    w.refresh_map()
    assert w.map_widget.path == []


def test_the_target_is_cleared_two_seconds_after_the_command():
    now = [19.0]
    w, session = window(MapSession(), latest_map=saved_map(), clock=lambda: now[0])
    w.map_widget.clicked.emit(2.6, 2.0)
    w.on_event('state', 'busy')
    now[0] = 20.0
    w.on_event('state', 'idle')
    now[0] = 21.5
    w.refresh_map()
    assert w.map_widget.target == (2.6, 2.0)
    now[0] = 22.5
    w.refresh_map()
    assert w.map_widget.target is None


def test_the_map_is_redrawn_only_for_a_new_version():
    versions = [1]
    w, _ = window(MapSession(), latest_map=lambda: SAVED + (versions[0],))
    calls = []
    original = w.map_widget.set_map
    w.map_widget.set_map = lambda *a: (calls.append(a), original(*a))
    for v in (1, 1, 2):
        versions[0] = v
        w.refresh_map()
    assert len(calls) == 2


def test_no_map_function_keeps_the_hint():
    w, _ = window(MapSession())
    w.refresh_map()
    assert not w.map_widget.has_map


# --- final review ---

def test_an_old_target_is_cleared_when_the_next_command_starts():
    now = [19.0]
    w, session = window(MapSession(), latest_map=saved_map(), clock=lambda: now[0])
    w.map_widget.clicked.emit(0.4, 2.7)                     # e.g. the bed: refused almost at once
    w.on_event('state', 'busy')
    w.on_event('state', 'idle')
    now[0] = 20.0                                           # within the 2 s hold, a place button starts a command
    w.on_event('state', 'busy')
    w.refresh_map()
    assert w.map_widget.target is None


def test_a_new_map_click_keeps_its_own_target():
    w, session = window(MapSession(), latest_map=saved_map())
    w.map_widget.clicked.emit(0.4, 2.7)
    w.on_event('state', 'busy')
    w.on_event('state', 'idle')

    def go_to(x, y):                                        # the real Session sends 'busy' inside go_to
        session.went.append((x, y))
        w.on_event('state', 'busy')
        return True

    session.go_to = go_to
    w.map_widget.clicked.emit(2.6, 2.0)
    assert w.map_widget.target == (2.6, 2.0)
