"""robot_dog-specific command and reward terms."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

import torch

from isaaclab.envs.mdp import UniformVelocityCommand, UniformVelocityCommandCfg
from isaaclab.utils import configclass

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


def small_axis_commands(n: int, lin_range: tuple[float, float], ang_range: tuple[float, float], device) -> torch.Tensor:
    """n commands (vx, vy, wz), each moving along one random axis at a small magnitude with a random sign."""
    axis = torch.randint(0, 3, (n,), device=device)
    lo = torch.where(axis == 2, ang_range[0], lin_range[0])
    hi = torch.where(axis == 2, ang_range[1], lin_range[1])
    magnitude = lo + (hi - lo) * torch.rand(n, device=device)
    sign = torch.where(torch.rand(n, device=device) < 0.5, -1.0, 1.0)
    cmd = torch.zeros(n, 3, device=device)
    cmd[torch.arange(n, device=device), axis] = sign * magnitude
    return cmd


class RobotDogVelocityCommand(UniformVelocityCommand):
    """Uniform velocity commands plus a share of small single-axis ones.

    With uniform sampling, a command like (0.1, 0, 0) almost never occurs: the only commands with
    vy = wz = 0 are the standing envs' exact zeros, so the policy treated it as "stand" (runs v4-v6).
    Nav2 sends exactly such commands near a goal.
    """

    cfg: RobotDogVelocityCommandCfg

    def _resample_command(self, env_ids: Sequence[int]):
        super()._resample_command(env_ids)
        self._apply_small_commands(env_ids)

    def _apply_small_commands(self, env_ids):
        ids = torch.arange(self.num_envs, device=self.device)[env_ids]
        ids = ids[torch.rand(len(ids), device=self.device) < self.cfg.rel_small_envs]
        if len(ids):
            self.vel_command_b[ids] = small_axis_commands(
                len(ids), self.cfg.small_lin_range, self.cfg.small_ang_range, self.device)


@configclass
class RobotDogVelocityCommandCfg(UniformVelocityCommandCfg):
    class_type: type = RobotDogVelocityCommand

    rel_small_envs: float = 0.0
    """Share of resampled envs that get a small single-axis command (standing envs still override it)."""
    small_lin_range: tuple[float, float] = (0.05, 0.2)
    """Magnitude range for small vx / vy commands (m/s)."""
    small_ang_range: tuple[float, float] = (0.1, 0.3)
    """Magnitude range for small wz commands (rad/s)."""
