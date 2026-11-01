"""Velocity-tracking locomotion for robot_dog (Isaac Lab 3.0 manager-based workflow)."""

import gymnasium as gym

from . import agents

gym.register(
    id="RobotDog-Velocity-Flat",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:RobotDogFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:RobotDogFlatPPORunnerCfg",
    },
)
