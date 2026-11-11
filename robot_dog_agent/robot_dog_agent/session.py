"""One command at a time, run off the UI thread, reported as events (spec 2026-11-11, 4). Plain Python.

The window owns a Session: submit() starts agent.ask in the background and returns at once; what happens is
told to session.on_event(kind, text), from the background thread:
    'state'  the new state: starting / idle / busy / failed
    'user'   the command as it was sent
    'log'    a tool call line, or a notice from the Gemini client
    'reply'  the model's answer
    'error'  why a command or the start-up failed
"""
import math
import threading

from robot_dog_agent import places

READY = '可以開始了：打字、或按下面的地點按鈕。'
NOT_LOCALIZED = 'Nav2 啟動了，但機器狗還沒有定位（%s）。請確認 nav.launch.py 正常執行。'
INTERNAL = '程式內部錯誤（%s），這一句沒有完成。'
AREA_ZH = {'living, dining and kitchen area': '客餐廚', 'master bedroom': '主臥室', 'second bedroom': '次臥室',
           'bathroom': '浴室'}
STARTING_WAIT = 0.5  # s that shutdown waits for a start-up that is still under way
STATUS = {'starting': '等 Nav2 啟動…', 'idle': '待命', 'busy': '處理中…', 'failed': '啟動失敗'}


class Closing(BaseException):
    """Raised into a running command once shutdown has begun, to end it without another round with the model."""


def in_thread(fn):
    """Run fn in a daemon thread; the thread is the job (join, is_alive)."""
    thread = threading.Thread(target=fn, daemon=True)
    thread.start()
    return thread


def pose_text(pose):
    """The left half of the status bar for pose = (x, y, yaw), or for None."""
    if pose is None:
        return '位置未知'
    area = places.area_of(pose[0], pose[1])
    return '位置 (%.2f, %.2f) 朝向 %.0f°　%s' % (pose[0], pose[1], math.degrees(pose[2]), AREA_ZH.get(area, area))


class Session:
    """run(fn) -> job starts fn in the background; a job has join(timeout) and is_alive()."""

    def __init__(self, run=None):
        self.on_event = lambda kind, text: None
        self.state, self.remaining, self.robot = 'starting', None, None  # remaining: m left of the navigation
        self._agent, self._job, self._run = None, None, run or in_thread
        self._closing = False

    def _set_state(self, state):
        self.state = state
        self.on_event('state', state)

    def start(self, make_robot, make_agent):
        """Build the robot (may block until Nav2 is up) and the agent in the background, then become idle.

        make_agent(robot, say=, on_call=, on_progress=) -> agent, the keywords of chat.build_agent.
        """
        def job():
            try:
                self.robot = make_robot()
                self._agent = make_agent(self.robot, say=lambda text: self.on_event('log', text),
                                         on_call=self._tool_done, on_progress=self._progress)
            except RuntimeError as e:
                self.on_event('error', NOT_LOCALIZED % e)
                return self._set_state('failed')
            except Exception as e:  # only the class name: a message could quote the key
                self.on_event('error', INTERNAL % type(e).__name__)
                return self._set_state('failed')
            self._set_state('idle')
            self.on_event('log', READY)
        self._job = self._run(job)

    def _progress(self, distance):
        self.remaining = distance if distance > 0.0 else None  # 0: Nav2 has no path (yet)

    def _tool_done(self, call, result):
        self.remaining = None
        if self._closing:  # the robot is stopped and refuses new goals; nobody will read the model's answer
            raise Closing()

    def submit(self, text):
        """Start a command; False when there is nothing to send or the session is not idle."""
        text = text.strip()
        if not text or self.state != 'idle':
            return False
        self.remaining = None
        self._set_state('busy')
        self.on_event('user', text)
        self.robot.clear_cancel()  # a Stop left over from the last command must not hit this one

        def job():
            try:
                kind, out = 'reply', self._agent.ask(text)
            except BaseException as e:
                gemini = type(e).__name__ == 'GeminiError'  # its message is written for the user
                kind, out = 'error', str(e) if gemini else INTERNAL % type(e).__name__
            self.remaining = None
            self.on_event(kind, out)
            self._set_state('idle')
        self._job = self._run(job)
        return True

    def stop(self):
        """Cancel the navigation of the running command, now or as soon as the model asks for one."""
        if self.state == 'busy':
            self.robot.cancel()

    def pose(self):
        """(x, y, yaw) now, or None when it is not known; never waits."""
        if self.robot is None:
            return None
        try:
            return self.robot.pose(wait=0)
        except RuntimeError:
            return None

    def status_text(self):
        if self.state == 'busy' and self.remaining is not None:
            return '導航中，剩 %.1f m' % self.remaining
        return STATUS[self.state]

    def shutdown(self, timeout=10.0):
        """Cancel, wait for the background job and close the robot. False when the job did not end in time
        (still waiting for Nav2): the robot is then left alone, and the caller should end the process."""
        self._closing = True
        if self.robot is not None:
            self.robot.cancel()
        if self._job is not None:
            # while starting, the job waits for Nav2 and cannot be interrupted: do not make the user wait for it
            self._job.join(STARTING_WAIT if self.state == 'starting' else timeout)
            if self._job.is_alive():
                return False
        if self.robot is not None:
            self.robot.close()
        return True
