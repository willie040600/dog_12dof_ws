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
    # 0.25 so ignoring a 0.1 m/s command costs 15% of the reward (0.5: 4%, v4 stood still below 0.1);
    # stage 1 from scratch overrides it to 0.5 on the command line (see isaaclab/README.md)
    assert r.track_lin_vel_xy_exp.params["std"] == pytest.approx(0.25)
    assert r.track_ang_vel_z_exp.params["std"] == pytest.approx(0.25)


def test_legs_have_armature():
    # without it the explicit DC-motor PD diverges on the light legs (dt 0.005, calf I ~1.3e-3 kg m^2)
    from robot_dog_lab.assets.robot_dog import ROBOT_DOG_CFG

    assert ROBOT_DOG_CFG.actuators["legs"].armature == pytest.approx(0.01)


def test_feet_air_time_rewards_short_steps():
    # this dog's swing is ~0.2-0.3 s; Isaac Lab's 0.5 s made every step a penalty (runs v1, v2 stood still)
    assert RobotDogFlatEnvCfg().rewards.feet_air_time.params["threshold"] == pytest.approx(0.2)


def test_feet_air_time_uses_any_motion_gate():
    # Isaac Lab's gate ignores wz and |cmd_xy| <= 0.1, which left a dead zone (v4, v5)
    from robot_dog_lab.tasks.locomotion.velocity.mdp import feet_air_time_any_motion

    term = RobotDogFlatEnvCfg().rewards.feet_air_time
    assert term.func is feet_air_time_any_motion
    assert term.params["sensor_cfg"].body_names == ".*_calf"


def test_commands_include_small_single_axis_ones():
    # (0.1, 0, 0) looked like the 10% standing envs (exact zeros) and v4-v6 stood still for it
    from robot_dog_lab.tasks.locomotion.velocity.mdp import RobotDogVelocityCommand, RobotDogVelocityCommandCfg

    c = RobotDogFlatEnvCfg().commands.base_velocity
    assert isinstance(c, RobotDogVelocityCommandCfg)
    assert c.class_type is RobotDogVelocityCommand
    assert c.rel_small_envs == pytest.approx(0.25)
    assert tuple(c.small_lin_range) == (0.05, 0.2)
    assert tuple(c.small_ang_range) == (0.1, 0.3)
    assert c.asset_name == "robot" and c.debug_vis is True


def test_eval_env_cfg_is_deterministic():
    from robot_dog_lab.eval_env import make_eval_env_cfg

    cfg = make_eval_env_cfg(17)
    assert cfg.scene.num_envs == 17
    assert cfg.events.push_robot is None
    assert cfg.events.reset_base.params["pose_range"] == {}
    assert cfg.events.reset_robot_joints.params["position_range"] == (1.0, 1.0)
    assert cfg.commands.base_velocity.rel_standing_envs == 0.0
    assert cfg.commands.base_velocity.rel_small_envs == 0.0
    assert cfg.observations.policy.enable_corruption is False


def test_eval_env_cfg_uses_the_nominal_robot():
    # play_mode keeps the startup mass/CoM randomization; eval and the plan 2 parity test need the URDF robot
    from robot_dog_lab.eval_env import make_eval_env_cfg

    cfg = make_eval_env_cfg(17)
    assert cfg.events.add_base_mass is None
    assert cfg.events.base_com is None
    assert cfg.seed == 42


def test_timing_and_action_scale():
    cfg = RobotDogFlatEnvCfg()
    assert cfg.sim.dt == pytest.approx(0.005)
    assert cfg.decimation == 4
    assert cfg.actions.joint_pos.scale == pytest.approx(0.25)


def test_base_height_reward():
    # without it v7 crawled with the base ~0.09 m high (standing pose 0.237 m)
    from isaaclab.envs import mdp

    term = RobotDogFlatEnvCfg().rewards.base_height_l2
    assert term.func is mdp.base_height_l2
    assert term.weight == pytest.approx(-50.0)
    assert term.params["target_height"] == pytest.approx(0.23)
