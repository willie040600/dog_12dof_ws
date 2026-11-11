import math

from robot_dog_agent import session as session_module
from robot_dog_agent.gemini import GeminiError
from robot_dog_agent.robot import FakeRobot
from robot_dog_agent.session import INTERNAL, NOT_LOCALIZED, READY, Session, pose_text


class DoneJob:
    def join(self, timeout=None):
        pass

    def is_alive(self):
        return False


class StuckJob(DoneJob):
    def is_alive(self):
        return True


def sync(fn):
    """A runner that does the work on the spot."""
    fn()
    return DoneJob()


class CountingRobot(FakeRobot):
    cancels = 0

    def cancel(self):
        self.cancels += 1
        super().cancel()


class FakeAgent:
    """Answers ask() with the given values in turn: a string, an exception to raise, or a callable to run."""

    def __init__(self, answers=()):
        self.answers, self.asked = list(answers), []

    def ask(self, text):
        self.asked.append(text)
        answer = self.answers.pop(0) if self.answers else '好'
        if isinstance(answer, BaseException):
            raise answer
        return answer() if callable(answer) else answer


def started(answers=(), robot=None, run=sync):
    """(session, events, robot, agent, callbacks) of a session whose start has finished; events starts empty."""
    robot, agent, events, callbacks = robot or CountingRobot(), FakeAgent(answers), [], {}
    session = Session(run=sync)
    session.on_event = lambda kind, text: events.append((kind, text))

    def make_agent(r, **kw):
        assert r is robot
        callbacks.update(kw)
        return agent

    session.start(lambda: robot, make_agent)
    del events[:]
    session._run = run
    return session, events, robot, agent, callbacks


def test_start_makes_the_session_idle():
    robot, events = FakeRobot(), []
    session = Session(run=sync)
    session.on_event = lambda kind, text: events.append((kind, text))
    assert session.state == 'starting' and session.robot is None
    session.start(lambda: robot, lambda r, **kw: FakeAgent())
    assert session.state == 'idle' and session.robot is robot
    assert events == [('state', 'idle'), ('log', READY)]


def test_start_failure():
    events = []
    session = Session(run=sync)
    session.on_event = lambda kind, text: events.append((kind, text))

    def no_robot():
        raise RuntimeError('no transform')

    session.start(no_robot, lambda r, **kw: FakeAgent())
    assert session.state == 'failed'
    assert events == [('error', NOT_LOCALIZED % 'no transform'), ('state', 'failed')]
    assert session.submit('去廚房') is False


def test_start_failure_of_another_kind():
    events = []
    session = Session(run=sync)
    session.on_event = lambda kind, text: events.append((kind, text))

    def broken(robot, **kw):
        raise ValueError('some detail')

    robot = FakeRobot()
    session.start(lambda: robot, broken)
    assert session.state == 'failed' and events == [('error', INTERNAL % 'ValueError'), ('state', 'failed')]
    assert session.shutdown() is True and robot.closed          # the robot that was made is still closed


def test_submit_runs_one_command():
    session, events, _, agent, _ = started(['到了'])
    assert session.submit('  去廚房 ') is True
    assert agent.asked == ['去廚房']
    assert events == [('state', 'busy'), ('user', '去廚房'), ('reply', '到了'), ('state', 'idle')]


def test_blank_and_busy_submits_are_refused():
    pending = []
    session, events, _, agent, _ = started(run=lambda fn: (pending.append(fn), DoneJob())[1])
    assert session.submit('') is False and session.submit('   ') is False and events == []
    assert session.submit('去廚房') is True and session.state == 'busy'
    assert session.submit('去玄關') is False
    pending[0]()
    assert agent.asked == ['去廚房'] and session.state == 'idle'


def test_not_ready_submits_are_refused():
    assert Session(run=sync).submit('去廚房') is False


def test_gemini_error_and_other_errors_end_the_command():
    session, events, _, _, _ = started([GeminiError('被限流'), ValueError('x'), KeyboardInterrupt()])
    for expected in ('被限流', INTERNAL % 'ValueError', INTERNAL % 'KeyboardInterrupt'):
        del events[:]
        assert session.submit('去廚房') is True
        assert events[-2:] == [('error', expected), ('state', 'idle')] and session.state == 'idle'


def test_tool_lines_and_notices_are_log_events():
    session, events, _, _, callbacks = started()
    callbacks['say']('  [工具] go_to_place {} → reached')
    assert events == [('log', '  [工具] go_to_place {} → reached')]


def test_progress_sets_remaining_and_a_finished_tool_clears_it():
    pending = []
    session, _, _, _, callbacks = started(run=lambda fn: (pending.append(fn), DoneJob())[1])
    session.submit('去廚房')
    assert session.remaining is None and session.status_text() == '處理中…'
    callbacks['on_progress'](1.23)
    assert session.remaining == 1.23 and session.status_text() == '導航中，剩 1.2 m'
    callbacks['on_call'](None, {'status': 'reached'})
    assert session.remaining is None and session.status_text() == '處理中…'


def test_remaining_does_not_leak_into_the_next_command():
    session, _, _, _, callbacks = started([lambda: callbacks['on_progress'](2.0) or '好'])
    session.submit('去廚房')
    assert session.remaining is None


def test_status_text_for_every_state():
    session = Session(run=sync)
    texts = {}
    for state in ('starting', 'idle', 'busy', 'failed'):
        session.state = state
        texts[state] = session.status_text()
    assert texts == {'starting': '等 Nav2 啟動…', 'idle': '待命', 'busy': '處理中…', 'failed': '啟動失敗'}


def test_stop_only_cancels_while_busy():
    pending = []
    session, _, robot, _, _ = started(run=lambda fn: (pending.append(fn), DoneJob())[1])
    session.stop()
    assert robot.cancels == 0
    session.submit('去廚房')
    session.stop()
    session.stop()
    assert robot.cancels == 2
    Session(run=sync).stop()                                    # not started: nothing to cancel, no error


def test_a_stale_cancel_does_not_hit_the_next_command():
    robot = CountingRobot()
    session, _, _, _, _ = started([lambda: robot.go_to(1.0, 0.0, 0.0).status], robot=robot)
    robot.cancel()                                              # e.g. Stop pressed just as the last command ended
    events = []
    session.on_event = lambda kind, text: events.append((kind, text))
    session.submit('去廚房')
    assert ('reply', 'reached') in events


def test_pose_and_pose_text():
    robot = CountingRobot((2.48, -0.61, -math.pi / 2))
    session, _, _, _, _ = started(robot=robot)
    assert session.pose() == (2.48, -0.61, -math.pi / 2)
    assert pose_text(session.pose()) == '位置 (2.48, -0.61) 朝向 -90°　客餐廚'
    assert pose_text((-1.5, 3.0, math.pi / 2)) == '位置 (-1.50, 3.00) 朝向 90°　主臥室'
    robot.pose_error = 'no transform'
    assert session.pose() is None and pose_text(None) == '位置未知'
    assert Session(run=sync).pose() is None


def test_shutdown_cancels_and_closes_the_robot():
    session, _, robot, _, _ = started()
    assert session.shutdown() is True and robot.closed and robot.cancels == 1


def test_shutdown_reports_a_job_that_will_not_end():
    session, _, robot, _, _ = started(run=lambda fn: StuckJob())
    session.submit('去廚房')
    assert session.shutdown(timeout=0.05) is False and not robot.closed


def test_shutdown_before_the_robot_exists():
    assert Session(run=sync).shutdown() is True


def test_the_default_runner_is_a_background_thread():
    session, events, _, agent, _ = started(['到了'], run=session_module.in_thread)
    assert session.submit('去廚房') is True
    session._job.join(5.0)
    assert not session._job.is_alive() and ('reply', '到了') in events and session.state == 'idle'
