import os
import stat

from robot_dog_agent.phone_code import LOCK, MAX_WRONG, WRONG_WAIT, Guard, load_or_create


def test_creates_a_six_digit_code_with_mode_600(tmp_path):
    path = str(tmp_path / 'sub' / 'phone_code')
    code = load_or_create(path)
    assert len(code) == 6 and code.isdigit()
    assert open(path).read().strip() == code
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600


def test_reuses_the_code(tmp_path):
    path = str(tmp_path / 'phone_code')
    assert load_or_create(path) == load_or_create(path)


def test_new_code_replaces_it(tmp_path):
    path = str(tmp_path / 'phone_code')
    with open(path, 'w') as f:
        f.write('123456\n')
    codes = {load_or_create(path, new=True) for _ in range(5)}
    assert codes != {'123456'}
    assert open(path).read().strip() == load_or_create(path)


def test_bad_file_is_replaced(tmp_path):
    path = str(tmp_path / 'phone_code')
    with open(path, 'w') as f:
        f.write('abc')
    code = load_or_create(path)
    assert len(code) == 6 and code.isdigit()


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def guard():
    clock, sleeps = Clock(), []
    return Guard('123456', clock=clock, sleep=sleeps.append), clock, sleeps


def test_right_code_passes():
    g, _, sleeps = guard()
    assert g.check('Bearer 123456', '10.0.0.2') == 200 and sleeps == []


def test_missing_or_wrong_code_is_401_after_a_wait():
    g, _, sleeps = guard()
    for value in (None, 'Bearer 000000', '123456', 'Bearer  123456', ''):
        assert g.check(value, '10.0.0.2') == 401
    assert sleeps == [WRONG_WAIT] * 5 and WRONG_WAIT == 1.0


def test_ten_wrong_in_a_minute_locks_the_ip():
    g, clock, _ = guard()
    for i in range(MAX_WRONG):
        clock.t = float(i)
        assert g.check('Bearer 000000', '10.0.0.2') == 401
    assert MAX_WRONG == 10
    assert g.check('Bearer 123456', '10.0.0.2') == 429
    assert g.check('Bearer 123456', '10.0.0.3') == 200


def test_locked_even_with_the_right_code():
    g, clock, sleeps = guard()
    for i in range(MAX_WRONG):
        clock.t = float(i)
        g.check('Bearer 000000', '10.0.0.2')
    waited = len(sleeps)
    clock.t = 68.0
    assert g.check('Bearer 123456', '10.0.0.2') == 429
    assert len(sleeps) == waited          # no wait while locked
    clock.t = 9.0 + LOCK + 1.0
    assert g.check('Bearer 123456', '10.0.0.2') == 200


def test_wrong_ones_older_than_a_minute_do_not_count():
    g, clock, _ = guard()
    for _ in range(9):
        g.check('Bearer 000000', '10.0.0.2')
    clock.t = 61.0
    assert g.check('Bearer 000000', '10.0.0.2') == 401
    assert g.check('Bearer 123456', '10.0.0.2') == 200
