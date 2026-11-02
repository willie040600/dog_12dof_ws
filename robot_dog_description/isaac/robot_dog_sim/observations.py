"""Policy observation exactly as Isaac Lab builds it for RobotDog-Velocity-Flat (48 values).

Quaternions are wxyz (Isaac Sim); Isaac Lab 3.0 itself uses xyzw. Velocities are the base's centre-of-mass
velocities, like Isaac Lab's root_lin_vel_b / root_ang_vel_b.
"""
import torch

from .policy_meta import PolicyMetaError

OBS_TERMS = ('base_lin_vel', 'base_ang_vel', 'projected_gravity', 'velocity_commands', 'joint_pos', 'joint_vel',
             'actions')
GRAVITY = (0.0, 0.0, -1.0)


def quat_rotate_inverse_wxyz(q, v):
    """Rotate world vector v into the frame of unit quaternion q = (w, x, y, z)."""
    w, u = q[0], q[1:]
    t = 2.0 * torch.linalg.cross(u, v)
    return v - w * t + torch.linalg.cross(u, t)


def joint_index_map(policy_joint_names, dof_names):
    missing = [n for n in policy_joint_names if n not in dof_names]
    if missing:
        raise PolicyMetaError('policy joints %s not in the articulation\n  policy:       %s\n  articulation: %s'
                              % (missing, list(policy_joint_names), list(dof_names)))
    return [list(dof_names).index(n) for n in policy_joint_names]


def check_obs_terms(meta):
    names = tuple(t['name'] for t in meta['obs_terms'])
    if names != OBS_TERMS:
        raise PolicyMetaError('obs_terms %s, this runner builds %s' % (list(names), list(OBS_TERMS)))
    for t in meta['obs_terms']:
        if t['scale'] is not None or t['clip'] is not None:
            raise PolicyMetaError('obs_terms %s has scale/clip, which this runner does not apply' % t['name'])


def build_observation(meta, root_quat_wxyz, com_lin_vel_w, ang_vel_w, command, joint_pos, joint_vel, last_action):
    """joint_pos / joint_vel / last_action are in the policy's joint order."""
    default = torch.as_tensor(meta['default_joint_pos'], dtype=joint_pos.dtype, device=joint_pos.device)
    q = root_quat_wxyz
    return torch.cat([
        quat_rotate_inverse_wxyz(q, com_lin_vel_w),
        quat_rotate_inverse_wxyz(q, ang_vel_w),
        quat_rotate_inverse_wxyz(q, torch.tensor(GRAVITY, dtype=q.dtype, device=q.device)),
        torch.as_tensor(command, dtype=q.dtype, device=q.device),
        joint_pos - default,
        joint_vel,
        last_action,
    ])
