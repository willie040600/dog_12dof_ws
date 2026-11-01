import pytest

from robot_dog_lab.tasks.locomotion.velocity.flat_env_cfg import RobotDogFlatEnvCfg


def test_commands_match_spec():
    c = RobotDogFlatEnvCfg().commands.base_velocity
    assert c.heading_command is False
    assert c.rel_standing_envs == pytest.approx(0.1)
    assert tuple(c.ranges.lin_vel_x) == (-0.8, 0.8)
    assert tuple(c.ranges.lin_vel_y) == (-0.5, 0.5)
    assert tuple(c.ranges.ang_vel_z) == (-1.0, 1.0)


def test_tracking_reward_std():
    r = RobotDogFlatEnvCfg().rewards
    assert r.track_lin_vel_xy_exp.params["std"] == pytest.approx(0.25)
    assert r.track_ang_vel_z_exp.params["std"] == pytest.approx(0.25)


def test_timing_and_action_scale():
    cfg = RobotDogFlatEnvCfg()
    assert cfg.sim.dt == pytest.approx(0.005)
    assert cfg.decimation == 4
    assert cfg.actions.joint_pos.scale == pytest.approx(0.25)
