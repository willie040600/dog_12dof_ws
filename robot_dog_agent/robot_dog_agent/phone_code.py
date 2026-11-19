"""The pairing code a phone needs before it may talk to the dog (spec 2026-11-19, 3). Plain Python.

Six digits, kept in a file so the phone enters it once. It is not the Gemini key: it may be printed in the terminal.
"""
import hmac
import os
import secrets
import threading
import time

CODE_PATH = os.path.expanduser('~/.config/robot_dog/phone_code')
WRONG_WAIT = 1.0  # s before a wrong code is answered
MAX_WRONG = 10    # wrong codes from one address within WINDOW s that lock it
WINDOW = 60.0     # s
LOCK = 60.0       # s an address stays locked


def load_or_create(path=CODE_PATH, new=False):
    """The code in path, or a new random one written there (mode 600) when there is none, it is bad, or new."""
    if not new:
        try:
            with open(path) as f:
                code = f.read().strip()
            if len(code) == 6 and code.isdigit():
                return code
        except OSError:
            pass
    code = '%06d' % secrets.randbelow(10 ** 6)
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    tmp = path + '.tmp'
    fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(code + '\n')
    os.chmod(tmp, 0o600)
    os.replace(tmp, path)
    return code


class Guard:
    """Checks the Authorization header of each request; slows down and locks out guessing. Thread-safe."""

    def __init__(self, code, clock=time.monotonic, sleep=time.sleep):
        self._expected = ('Bearer ' + code).encode()
        self.clock, self.sleep = clock, sleep
        self._lock = threading.Lock()
        self._wrong, self._locked_until = {}, {}  # address -> times of wrong codes; address -> end of its lock

    def check(self, authorization, ip):
        """200 when authorization is 'Bearer <code>', 401 when not (after WRONG_WAIT s), 429 while ip is locked."""
        with self._lock:
            now = self.clock()
            if now < self._locked_until.get(ip, float('-inf')):
                return 429
            if hmac.compare_digest((authorization or '').encode(), self._expected):
                return 200
            times = [t for t in self._wrong.get(ip, []) if now - t < WINDOW] + [now]
            if len(times) >= MAX_WRONG:
                self._locked_until[ip] = now + LOCK
                times = []
            self._wrong[ip] = times
        self.sleep(WRONG_WAIT)
        return 401
