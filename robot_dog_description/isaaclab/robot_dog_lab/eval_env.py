"""Deterministic single-purpose variant of the training env, for stand test, gait eval, export and parity."""
from robot_dog_lab.tasks.locomotion.velocity.flat_env_cfg import RobotDogFlatEnvCfg


def make_eval_env_cfg(num_envs, episode_length_s=1000.0):
    cfg = RobotDogFlatEnvCfg()
    cfg.play_mode()  # no pushes, no observation noise
    # the train/play CLIs resolve the physics preset; building the cfg directly we pick it ourselves
    cfg.sim.physics = cfg.sim.physics.isaacsim_physx
    cfg.scene.num_envs = num_envs
    cfg.episode_length_s = episode_length_s
    cmd = cfg.commands.base_velocity
    cmd.resampling_time_range = (1000.0, 1000.0)
    cmd.rel_standing_envs = 0.0
    cmd.rel_small_envs = 0.0
    cmd.debug_vis = False
    # spawn exactly at the env origin, standing pose, at rest
    cfg.events.reset_base.params["pose_range"] = {}
    cfg.events.reset_base.params["velocity_range"] = {}
    cfg.events.reset_robot_joints.params["position_range"] = (1.0, 1.0)
    return cfg
