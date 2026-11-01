from types import SimpleNamespace

import torch

from robot_dog_lab.tasks.locomotion.velocity.mdp import RobotDogVelocityCommand, small_axis_commands


def test_small_axis_commands_have_one_small_axis():
    c = small_axis_commands(6000, (0.05, 0.2), (0.1, 0.3), "cpu")
    nonzero = c != 0
    assert c.shape == (6000, 3)
    assert torch.all(nonzero.sum(dim=1) == 1)
    lin = c[:, :2][nonzero[:, :2]].abs()
    ang = c[:, 2][nonzero[:, 2]].abs()
    assert lin.min() >= 0.05 and lin.max() <= 0.2
    assert ang.min() >= 0.1 and ang.max() <= 0.3
    share = nonzero.float().mean(dim=0)
    assert torch.all(share > 0.28)  # each axis about a third
    assert 0.45 < (c.sum(dim=1) > 0).float().mean() < 0.55  # both signs


def _fake_term(n, rel_small):
    cfg = SimpleNamespace(rel_small_envs=rel_small, small_lin_range=(0.05, 0.2), small_ang_range=(0.1, 0.3))
    big = torch.tensor([[0.6, -0.4, 0.9]]).repeat(n, 1)
    return SimpleNamespace(cfg=cfg, device="cpu", num_envs=n, vel_command_b=big.clone())


def test_apply_small_commands_replaces_a_fraction():
    term = _fake_term(4000, 0.25)
    RobotDogVelocityCommand._apply_small_commands(term, torch.arange(4000))
    small = (term.vel_command_b.abs().max(dim=1).values <= 0.3)
    assert 0.2 < small.float().mean() < 0.3
    untouched = term.vel_command_b[~small]
    assert torch.all(untouched == torch.tensor([0.6, -0.4, 0.9]))


def test_apply_small_commands_only_touches_given_envs():
    term = _fake_term(100, 1.0)
    RobotDogVelocityCommand._apply_small_commands(term, torch.arange(50))
    assert torch.all(term.vel_command_b[50:] == torch.tensor([0.6, -0.4, 0.9]))
    assert torch.all(term.vel_command_b[:50].abs().max(dim=1).values <= 0.3)


def test_zero_fraction_changes_nothing():
    term = _fake_term(100, 0.0)
    RobotDogVelocityCommand._apply_small_commands(term, torch.arange(100))
    assert torch.all(term.vel_command_b == torch.tensor([0.6, -0.4, 0.9]))
