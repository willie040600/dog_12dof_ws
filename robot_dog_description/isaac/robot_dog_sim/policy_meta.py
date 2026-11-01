"""policy.json: what the Isaac Sim policy runner needs to reproduce the Isaac Lab training setup.

Written by isaaclab/scripts/export_policy.py next to policy.pt, read by the Isaac Sim side.
Plain Python (json only) so both sides and the tests can import it.
"""
import json
import math
import numbers
import os


class PolicyMetaError(ValueError):
    pass


REQUIRED_KEYS = ('task', 'run', 'checkpoint', 'joint_names', 'default_joint_pos', 'action_scale', 'clip_actions',
                 'obs_terms', 'sim_dt', 'decimation', 'actuator', 'physx', 'materials', 'command_ranges',
                 'spawn_height')
COMBINE_MODES = ('average', 'min', 'multiply', 'max')  # PhysX material combine modes


def _object(x, where):
    if not isinstance(x, dict):
        raise PolicyMetaError('%s must be an object' % where)
    return x


def _list(x, where):
    if not isinstance(x, list):
        raise PolicyMetaError('%s must be a list' % where)
    return x


def _require(section, keys, where):
    _object(section, where)
    missing = [k for k in keys if k not in section]
    if missing:
        raise PolicyMetaError('%s is missing %s' % (where, ', '.join(missing)))


def _number(x, where, low=None, low_inclusive=True):
    """Finite real number (not a bool), optionally bounded below."""
    if not (isinstance(x, numbers.Real) and not isinstance(x, bool) and math.isfinite(x)):
        raise PolicyMetaError('%s must be a finite number, got %r' % (where, x))
    if low is not None and (x < low if low_inclusive else x <= low):
        raise PolicyMetaError('%s must be %s %g, got %r' % (where, '>=' if low_inclusive else '>', low, x))
    return x


def _int(x, where, low):
    if not isinstance(x, int) or isinstance(x, bool) or x < low:
        raise PolicyMetaError('%s must be an int >= %d, got %r' % (where, low, x))
    return x


def obs_dim(meta):
    return sum(t['dim'] for t in meta['obs_terms'])


def validate_policy_meta(meta, policy_input_dim=None, policy_output_dim=None):
    _require(meta, REQUIRED_KEYS, 'policy.json')
    for k in ('task', 'run', 'checkpoint'):
        if not isinstance(meta[k], str):
            raise PolicyMetaError('%s must be a string' % k)

    names = _list(meta['joint_names'], 'joint_names')
    if not names:
        raise PolicyMetaError('joint_names is empty')
    if not all(isinstance(n, str) for n in names):
        raise PolicyMetaError('joint_names must all be strings')
    duplicates = sorted({n for n in names if names.count(n) > 1})
    if duplicates:
        raise PolicyMetaError('duplicate joint_names: %s' % ', '.join(duplicates))
    default_q = _list(meta['default_joint_pos'], 'default_joint_pos')
    if len(default_q) != len(names):
        raise PolicyMetaError('default_joint_pos has %d entries, joint_names has %d' % (len(default_q), len(names)))
    for i, q in enumerate(default_q):
        _number(q, 'default_joint_pos[%d]' % i)

    _number(meta['action_scale'], 'action_scale', 0.0, low_inclusive=False)
    if meta['clip_actions'] is not None:
        _number(meta['clip_actions'], 'clip_actions', 0.0, low_inclusive=False)

    for i, term in enumerate(_list(meta['obs_terms'], 'obs_terms')):
        where = 'obs_terms[%d]' % i
        _require(term, ('name', 'dim', 'scale', 'clip'), where)
        if not isinstance(term['name'], str):
            raise PolicyMetaError('%s.name must be a string' % where)
        _int(term['dim'], '%s (%s).dim' % (where, term['name']), 1)
        if term['scale'] is not None:
            _number(term['scale'], '%s (%s).scale' % (where, term['name']))
        clip = term['clip']
        if clip is not None:
            if not (isinstance(clip, list) and len(clip) == 2):
                raise PolicyMetaError('%s (%s).clip must be null or [lo, hi]' % (where, term['name']))
            for c in clip:
                _number(c, '%s (%s).clip' % (where, term['name']))
    if policy_input_dim is not None and policy_input_dim != obs_dim(meta):
        raise PolicyMetaError('policy input dim %d != obs_terms total dim %d' % (policy_input_dim, obs_dim(meta)))
    if policy_output_dim is not None and policy_output_dim != len(names):
        raise PolicyMetaError('policy output dim %d != %d joint_names' % (policy_output_dim, len(names)))

    _number(meta['sim_dt'], 'sim_dt', 0.0, low_inclusive=False)
    _int(meta['decimation'], 'decimation', 1)
    _number(meta['spawn_height'], 'spawn_height', 0.0, low_inclusive=False)

    act = meta['actuator']
    _require(act, ('type', 'stiffness', 'damping', 'armature', 'effort_limit', 'saturation_effort',
                   'velocity_limit'), 'actuator')
    if act['type'] != 'DCMotor':
        raise PolicyMetaError('actuator.type is %r, only DCMotor is supported' % act['type'])
    for k in ('stiffness', 'effort_limit', 'saturation_effort', 'velocity_limit'):
        _number(act[k], 'actuator.%s' % k, 0.0, low_inclusive=False)
    for k in ('damping', 'armature'):
        _number(act[k], 'actuator.%s' % k, 0.0)

    physx = meta['physx']
    _require(physx, ('solver_position_iterations', 'solver_velocity_iterations', 'self_collisions'), 'physx')
    _int(physx['solver_position_iterations'], 'physx.solver_position_iterations', 1)
    _int(physx['solver_velocity_iterations'], 'physx.solver_velocity_iterations', 0)
    if not isinstance(physx['self_collisions'], bool):
        raise PolicyMetaError('physx.self_collisions must be true or false')

    materials = meta['materials']
    _require(materials, ('ground', 'robot'), 'materials')
    _require(materials['ground'], ('static_friction', 'dynamic_friction', 'restitution', 'friction_combine_mode',
                                   'restitution_combine_mode'), 'materials.ground')
    _require(materials['robot'], ('static_friction', 'dynamic_friction', 'restitution'), 'materials.robot')
    for part in ('ground', 'robot'):
        for k in ('static_friction', 'dynamic_friction', 'restitution'):
            _number(materials[part][k], 'materials.%s.%s' % (part, k), 0.0)
    for k in ('friction_combine_mode', 'restitution_combine_mode'):
        if materials['ground'][k] not in COMBINE_MODES:
            raise PolicyMetaError('materials.ground.%s must be one of %s, got %r'
                                  % (k, ', '.join(COMBINE_MODES), materials['ground'][k]))

    ranges = meta['command_ranges']
    _require(ranges, ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'), 'command_ranges')
    for k in ('lin_vel_x', 'lin_vel_y', 'ang_vel_z'):
        r = ranges[k]
        if not (isinstance(r, list) and len(r) == 2):
            raise PolicyMetaError('command_ranges.%s must be [lo, hi], got %r' % (k, r))
        for v in r:
            _number(v, 'command_ranges.%s' % k)
        if r[0] > r[1]:
            raise PolicyMetaError('command_ranges.%s must have lo <= hi, got %r' % (k, r))


def save_policy_meta(meta, path):
    validate_policy_meta(meta)
    text = json.dumps(meta, indent=2, allow_nan=False)  # serialize fully before touching the file
    with open(path, 'w') as f:
        f.write(text + '\n')


def load_policy_meta(path, policy_input_dim=None, policy_output_dim=None):
    with open(os.path.expanduser(path)) as f:
        meta = json.load(f)
    validate_policy_meta(meta, policy_input_dim, policy_output_dim)
    return meta
