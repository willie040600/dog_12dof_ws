"""DC-motor torque, the same formula as Isaac Lab's DCMotor (compute + _clip_effort)."""
import torch


def dc_motor_torque(q_target, q, qd, actuator):
    """PD torque toward q_target, clipped by the four-quadrant torque-speed curve.

    actuator is policy.json's "actuator" object; the velocity target is zero, as in training.
    """
    kp, kd = actuator['stiffness'], actuator['damping']
    sat, limit, vmax = actuator['saturation_effort'], actuator['effort_limit'], actuator['velocity_limit']
    tau = kp * (q_target - q) - kd * qd
    v = torch.clamp(qd, -vmax * (1.0 + limit / sat), vmax * (1.0 + limit / sat))
    tau_max = torch.clamp(sat * (1.0 - v / vmax), max=limit)
    tau_min = torch.clamp(sat * (-1.0 - v / vmax), min=-limit)
    return torch.clamp(tau, min=tau_min, max=tau_max)
