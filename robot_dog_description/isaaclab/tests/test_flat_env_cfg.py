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
    # v1 with std 0.25 never learned to walk (stand-still optimum); 0.5 is Isaac Lab's default
    assert r.track_lin_vel_xy_exp.params["std"] == pytest.approx(0.5)
    assert r.track_ang_vel_z_exp.params["std"] == pytest.approx(0.5)


def test_feet_air_time_rewards_short_steps():
    # this dog's swing is ~0.2-0.3 s; Isaac Lab's 0.5 s made every step a penalty (runs v1, v2 stood still)
    assert RobotDogFlatEnvCfg().rewards.feet_air_time.params["threshold"] == pytest.approx(0.2)


def test_eval_env_cfg_is_deterministic():
    from robot_dog_lab.eval_env import make_eval_env_cfg

    cfg = make_eval_env_cfg(17)
    assert cfg.scene.num_envs == 17
    assert cfg.events.push_robot is None
    assert cfg.events.reset_base.params["pose_range"] == {}
    assert cfg.events.reset_robot_joints.params["position_range"] == (1.0, 1.0)
    assert cfg.commands.base_velocity.rel_standing_envs == 0.0
    assert cfg.observations.policy.enable_corruption is False


def test_timing_and_action_scale():
    cfg = RobotDogFlatEnvCfg()
    assert cfg.sim.dt == pytest.approx(0.005)
    assert cfg.decimation == 4
    assert cfg.actions.joint_pos.scale == pytest.approx(0.25)
