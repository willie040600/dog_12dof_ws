from types import SimpleNamespace

import torch

from isaaclab_tasks.core.velocity.mdp.rewards import feet_air_time
from robot_dog_lab.tasks.locomotion.velocity.mdp import feet_air_time_any_motion

# 4 envs x 4 feet; every foot touches down this step after 0.3 s in the air
AIR = torch.full((4, 4), 0.3)
FIRST = torch.ones(4, 4)


def _env(cmds):
    sensor = SimpleNamespace(
        compute_first_contact=lambda dt: SimpleNamespace(torch=FIRST),
        data=SimpleNamespace(last_air_time=SimpleNamespace(torch=AIR)),
    )
    return SimpleNamespace(
        step_dt=0.02,
        scene=SimpleNamespace(sensors={"contact_forces": sensor}),
        command_manager=SimpleNamespace(get_command=lambda name: torch.tensor(cmds, dtype=torch.float32)),
    )


SENSOR = SimpleNamespace(name="contact_forces", body_ids=slice(None))


def _reward(cmds):
    return feet_air_time_any_motion(_env(cmds), "base_velocity", SENSOR, threshold=0.2)


def test_small_and_turning_commands_reward_steps():
    # Isaac Lab's gate (|cmd_xy| > 0.1) gives these nothing, so the policy stood still for them
    r = _reward([[0.1, 0.0, 0.0], [0.0, -0.1, 0.0], [0.0, 0.0, 0.2], [0.0, 0.0, -0.2]])
    assert torch.allclose(r, torch.full((4,), 4 * (0.3 - 0.2)))


def test_zero_and_tiny_commands_give_no_step_reward():
    r = _reward([[0.0, 0.0, 0.0], [0.03, 0.03, 0.0], [0.0, 0.0, 0.04], [0.04, 0.0, -0.04]])
    assert torch.all(r == 0.0)


def test_matches_isaac_lab_where_its_gate_is_open():
    cmds = [[0.5, 0.0, 0.0], [0.0, 0.3, 0.5], [-0.2, 0.2, 0.0], [0.11, 0.0, -0.8]]
    ref = feet_air_time(_env(cmds), "base_velocity", SENSOR, threshold=0.2)
    assert torch.allclose(_reward(cmds), ref)
