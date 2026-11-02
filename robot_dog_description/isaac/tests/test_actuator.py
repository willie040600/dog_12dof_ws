"""Run in the Isaac Lab uv env (needs torch and isaaclab)."""
import torch

from isaaclab.actuators import DCMotorCfg
from isaaclab.utils import instantiate
from isaaclab.utils.types import ArticulationActions
from robot_dog_sim.actuator import dc_motor_torque

ACT = {'type': 'DCMotor', 'stiffness': 20.0, 'damping': 0.5, 'armature': 0.01,
       'effort_limit': 25.0, 'saturation_effort': 25.0, 'velocity_limit': 30.0}


def _isaac_lab_motor(n):
    names = ['j%d' % i for i in range(12)]
    cfg = DCMotorCfg(joint_names_expr=names, stiffness=20.0, damping=0.5, actuator_effort_limit=25.0,
                     actuator_velocity_limit=30.0, saturation_effort=25.0)
    return instantiate(cfg, joint_names=names, joint_ids=list(range(12)), num_envs=n, device='cpu',
                       stiffness=cfg.stiffness, damping=cfg.damping)


def test_dc_motor_matches_isaac_lab():
    torch.manual_seed(0)
    n = 1000
    q_target = torch.rand(n, 12) * 4 - 2
    q = torch.rand(n, 12) * 4 - 2
    qd = torch.rand(n, 12) * 80 - 40  # beyond the 30 rad/s limit, so the clipping region is covered
    motor = _isaac_lab_motor(n)
    action = ArticulationActions(joint_positions=q_target, joint_velocities=torch.zeros(n, 12),
                                 joint_efforts=torch.zeros(n, 12))
    ref = motor.compute(action, joint_pos=q, joint_vel=qd).joint_efforts
    assert torch.allclose(dc_motor_torque(q_target, q, qd, ACT), ref, atol=1e-5)


def test_dc_motor_saturates_with_speed():
    big = torch.tensor([10.0])
    assert float(dc_motor_torque(big, torch.zeros(1), torch.tensor([30.0]), ACT)) <= 1e-6
    assert float(dc_motor_torque(big, torch.zeros(1), torch.zeros(1), ACT)) == 25.0
