"""robot_dog-specific reward terms."""

from __future__ import annotations

from typing import TYPE_CHECKING

import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers import SceneEntityCfg
    from isaaclab.sensors import ContactSensor


def feet_air_time_any_motion(
    env: ManagerBasedRLEnv,
    command_name: str,
    sensor_cfg: SceneEntityCfg,
    threshold: float,
    xy_threshold: float = 0.05,
    wz_threshold: float = 0.05,
) -> torch.Tensor:
    """Isaac Lab's ``feet_air_time`` with a gate that also opens for small and turn-in-place commands.

    The original rewards steps only while ``|cmd_xy| > 0.1``, so a 0.1 m/s command or a pure yaw command
    never paid for stepping and the policy stood still for them (runs v4, v5). Nav2 sends both. Here the
    gate is ``|cmd_xy| > xy_threshold or |wz| > wz_threshold``; standing envs (exactly zero) stay closed.
    """
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    first_contact = contact_sensor.compute_first_contact(env.step_dt).torch[:, sensor_cfg.body_ids]
    last_air_time = contact_sensor.data.last_air_time.torch[:, sensor_cfg.body_ids]
    reward = torch.sum((last_air_time - threshold) * first_contact, dim=1)
    cmd = env.command_manager.get_command(command_name)
    moving = (torch.linalg.norm(cmd[:, :2], dim=1) > xy_threshold) | (torch.abs(cmd[:, 2]) > wz_threshold)
    return reward * moving
