"""policy.json: what the Isaac Sim policy runner needs to reproduce the Isaac Lab training setup.

Written by isaaclab/scripts/export_policy.py next to policy.pt, read by the Isaac Sim side.
Plain Python (json only) so both sides and the tests can import it.
"""
import json
import numbers
import os


class PolicyMetaError(ValueError):
    pass


REQUIRED_KEYS = ('task', 'run', 'checkpoint', 'joint_names', 'default_joint_pos', 'action_scale', 'clip_actions',
                 'obs_terms', 'sim_dt', 'decimation', 'actuator', 'physx', 'materials', 'command_ranges',
                 'spawn_height')
_GROUND_KEYS = ('static_friction', 'dynamic_friction', 'restitution', 'friction_combine_mode',
                'restitution_combine_mode')
_ROBOT_KEYS = ('static_friction', 'dynamic_friction', 'restitution')
_PHYSX_KEYS = ('solver_position_iterations', 'solver_velocity_iterations', 'self_collisions')


def _is_number(x):
    return isinstance(x, numbers.Real) and not isinstance(x, bool)


def _require(section, keys, where):
    missing = [k for k in keys if k not in section]
    if missing:
        raise PolicyMetaError('%s is missing %s' % (where, ', '.join(missing)))


def obs_dim(meta):
    return sum(t['dim'] for t in meta['obs_terms'])


def validate_policy_meta(meta, policy_input_dim=None, policy_output_dim=None):
    _require(meta, REQUIRED_KEYS, 'policy.json')

    names = meta['joint_names']
    if not names:
        raise PolicyMetaError('joint_names is empty')
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise PolicyMetaError('duplicate joint_names: %s' % ', '.join(duplicates))
    if len(meta['default_joint_pos']) != len(names):
        raise PolicyMetaError('default_joint_pos has %d entries, joint_names has %d'
                              % (len(meta['default_joint_pos']), len(names)))

    for i, term in enumerate(meta['obs_terms']):
        _require(term, ('name', 'dim', 'scale', 'clip'), 'obs_terms[%d]' % i)
        if not isinstance(term['name'], str):
            raise PolicyMetaError('obs_terms[%d].name must be a string' % i)
        if not isinstance(term['dim'], int) or isinstance(term['dim'], bool) or term['dim'] <= 0:
            raise PolicyMetaError('obs_terms[%d] (%s): dim must be a positive int' % (i, term['name']))
        if term['scale'] is not None and not _is_number(term['scale']):
            raise PolicyMetaError('obs_terms[%d] (%s): scale must be null or a number' % (i, term['name']))
        clip = term['clip']
        if clip is not None and not (len(clip) == 2 and all(_is_number(c) for c in clip)):
            raise PolicyMetaError('obs_terms[%d] (%s): clip must be null or [lo, hi]' % (i, term['name']))
    if policy_input_dim is not None and policy_input_dim != obs_dim(meta):
        raise PolicyMetaError('policy input dim %d != obs_terms total dim %d' % (policy_input_dim, obs_dim(meta)))
    if policy_output_dim is not None and policy_output_dim != len(names):
        raise PolicyMetaError('policy output dim %d != %d joint_names' % (policy_output_dim, len(names)))

    if not _is_number(meta['sim_dt']) or meta['sim_dt'] <= 0:
        raise PolicyMetaError('sim_dt must be > 0')
    if not isinstance(meta['decimation'], int) or isinstance(meta['decimation'], bool) or meta['decimation'] < 1:
        raise PolicyMetaError('decimation must be an int >= 1')

    act = meta['actuator']
    _require(act, ('type', 'stiffness', 'damping', 'effort_limit', 'saturation_effort', 'velocity_limit'),
             'actuator')
    if act['type'] != 'DCMotor':
        raise PolicyMetaError('actuator.type is %r, only DCMotor is supported' % act['type'])
    for k in ('stiffness', 'effort_limit', 'saturation_effort', 'velocity_limit'):
        if not _is_number(act[k]) or act[k] <= 0:
            raise PolicyMetaError('actuator.%s must be > 0' % k)
    if not _is_number(act['damping']) or act['damping'] < 0:
        raise PolicyMetaError('actuator.damping must be >= 0')

    _require(meta['physx'], _PHYSX_KEYS, 'physx')
    _require(meta['materials'], ('ground', 'robot'), 'materials')
    _require(meta['materials']['ground'], _GROUND_KEYS, 'materials.ground')
    _require(meta['materials']['robot'], _ROBOT_KEYS, 'materials.robot')

    ranges = meta['command_ranges']
    _require(ranges, ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'), 'command_ranges')
    for k in ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'):
        r = ranges[k]
        if not (len(r) == 2 and all(_is_number(v) for v in r) and r[0] <= r[1]):
            raise PolicyMetaError('command_ranges.%s must be [lo, hi] with lo <= hi, got %r' % (k, r))


def save_policy_meta(meta, path):
    validate_policy_meta(meta)
    text = json.dumps(meta, indent=2)  # serialize fully before touching the file
    with open(path, 'w') as f:
        f.write(text + '\n')


def load_policy_meta(path, policy_input_dim=None, policy_output_dim=None):
    with open(os.path.expanduser(path)) as f:
        meta = json.load(f)
    validate_policy_meta(meta, policy_input_dim, policy_output_dim)
    return meta
