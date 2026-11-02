"""Run an exported robot_dog policy on an Isaac Sim articulation, step for step as in Isaac Lab training.

Use with stepping.Lockstep: lock.step(runner.pre_physics) runs the DC-motor torques before every physics
step, then call runner.policy_step(command, t) once per cycle (= every `decimation` physics steps).
Import only after SimulationApp has started.
"""
import torch

from .actuator import dc_motor_torque
from .command_buffer import IdleHold, is_fallen
from .observations import build_observation, check_obs_terms, joint_index_map, quat_rotate_inverse_wxyz
from .policy_meta import PolicyMetaError


def _row(x):
    """First (only) articulation's row of a warp array, as a float32 CPU tensor."""
    return torch.as_tensor(x.numpy()[0], dtype=torch.float32)


class PolicyRunner:
    def __init__(self, meta, policy_path, articulation, hold=True):
        check_obs_terms(meta)
        self.meta = meta
        self.articulation = articulation
        self.dof_indices = joint_index_map(meta['joint_names'], list(articulation.dof_names))
        self.policy = torch.jit.load(policy_path, map_location='cpu').eval()
        n_obs, n_act = sum(t['dim'] for t in meta['obs_terms']), len(meta['joint_names'])
        with torch.inference_mode():
            out = self.policy(torch.zeros(1, n_obs))
        if tuple(out.shape) != (1, n_act):
            raise PolicyMetaError('policy maps %d observations to %s, expected (1, %d)' % (n_obs, tuple(out.shape),
                                                                                            n_act))
        self.default = torch.tensor(meta['default_joint_pos'], dtype=torch.float32)
        self.q_target = self.default.clone()
        self.last_action = torch.zeros(n_act)
        self._hold = IdleHold(hold)
        self.fallen = False
        self.base_pos = self.base_quat = self.lin_vel_w = self.ang_vel_w = None

    def _joints(self):
        q = _row(self.articulation.get_dof_positions())[self.dof_indices]
        qd = _row(self.articulation.get_dof_velocities())[self.dof_indices]
        return q, qd

    def pre_physics(self):
        q, qd = self._joints()
        tau = dc_motor_torque(self.q_target, q, qd, self.meta['actuator'])
        self.articulation.set_dof_efforts([tau.tolist()], dof_indices=self.dof_indices)

    def policy_step(self, command, t):
        pos, quat = self.articulation.get_world_poses()
        lin, ang = self.articulation.get_velocities()  # root centre-of-mass velocity, world frame
        self.base_pos, self.base_quat, self.lin_vel_w, self.ang_vel_w = _row(pos), _row(quat), _row(lin), _row(ang)
        gravity_b = quat_rotate_inverse_wxyz(self.base_quat, torch.tensor([0.0, 0.0, -1.0]))
        self.fallen = is_fallen(float(self.base_pos[2]), float(gravity_b[2]))

        hold = self._hold.update(command, t)
        if self._hold.resumed():
            self.last_action.zero_()
        if hold:
            self.q_target = self.default.clone()
            self.last_action.zero_()
            return
        q, qd = self._joints()
        obs = build_observation(self.meta, self.base_quat, self.lin_vel_w, self.ang_vel_w, command, q, qd,
                                self.last_action)
        with torch.inference_mode():
            action = self.policy(obs.unsqueeze(0))[0]
        if self.meta['clip_actions'] is not None:
            c = self.meta['clip_actions']
            action = action.clamp(-c, c)
        self.last_action = action.clone()
        self.q_target = self.default + self.meta['action_scale'] * action
