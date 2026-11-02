import math

from robot_dog_sim.command_buffer import CommandBuffer, IdleHold, is_fallen

RANGES = {'lin_vel_x': [-0.8, 0.8], 'lin_vel_y': [-0.5, 0.5], 'ang_vel_z': [-1.0, 1.0]}


def test_timeout_zeroes_command():
    b = CommandBuffer(RANGES)
    assert b.get(0.0) == (0.0, 0.0, 0.0)  # nothing received yet
    b.update((0.3, 0.0, 0.0), 0.0)
    assert b.get(0.49) == (0.3, 0.0, 0.0)
    assert b.get(0.51) == (0.0, 0.0, 0.0)


def test_commands_are_clipped_to_training_ranges():
    b = CommandBuffer(RANGES)
    b.update((1.5, -0.9, 2.0), 1.0)
    assert b.get(1.0) == (0.8, -0.5, 1.0)


def test_hold_after_idle_period():
    h = IdleHold(enabled=True)
    assert h.update((0.0, 0.0, 0.0), 0.0) is False
    assert h.update((0.0, 0.0, 0.0), 0.29) is False
    assert h.update((0.0, 0.0, 0.0), 0.31) is True
    assert h.resumed() is False
    assert h.update((0.1, 0.0, 0.0), 0.33) is False
    assert h.resumed() is True
    assert h.update((0.1, 0.0, 0.0), 0.35) is False
    assert h.resumed() is False


def test_hold_disabled():
    h = IdleHold(enabled=False)
    assert h.update((0.0, 0.0, 0.0), 0.0) is False
    assert h.update((0.0, 0.0, 0.0), 5.0) is False


def test_small_command_is_not_idle():
    h = IdleHold(enabled=True)
    for t in (0.0, 0.2, 0.4, 0.6):
        assert h.update((0.03, 0.0, 0.0), t) is False


def test_is_fallen():
    assert not is_fallen(0.25, -1.0)
    assert is_fallen(0.10, -1.0)
    assert is_fallen(0.25, -math.cos(math.radians(70)))
    assert not is_fallen(0.25, -math.cos(math.radians(50)))


def test_hold_waits_until_the_body_is_still():
    # freezing the targets mid-stride made the dog lurch (0.19 m/s + 0.85 rad/s 1 s after stopping)
    h = IdleHold(enabled=True)
    assert h.update((0.0, 0.0, 0.0), 0.0, speed=(0.30, 0.0)) is False
    assert h.update((0.0, 0.0, 0.0), 0.5, speed=(0.20, 0.0)) is False
    assert h.update((0.0, 0.0, 0.0), 0.6, speed=(0.01, 0.30)) is False
    assert h.update((0.0, 0.0, 0.0), 0.7, speed=(0.01, 0.02)) is True
