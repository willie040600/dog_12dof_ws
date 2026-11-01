import pytest

from robot_dog_sim.policy_meta import (PolicyMetaError, load_policy_meta, obs_dim, save_policy_meta,
                                       validate_policy_meta)

LEGS = ('FL', 'FR', 'RL', 'RR')
PARTS = ('hip', 'thigh', 'calf')
STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}


@pytest.fixture
def valid_meta():
    names = ['%s_%s_joint' % (leg, part) for leg in LEGS for part in PARTS]
    terms = [('base_lin_vel', 3), ('base_ang_vel', 3), ('projected_gravity', 3), ('velocity_commands', 3),
             ('joint_pos', 12), ('joint_vel', 12), ('actions', 12)]
    return {
        'task': 'RobotDog-Velocity-Flat',
        'run': '2026-11-01_12-00-00_v1',
        'checkpoint': 'model_999.pt',
        'joint_names': names,
        'default_joint_pos': [STAND[n.split('_')[1]] for n in names],
        'action_scale': 0.25,
        'clip_actions': None,
        'obs_terms': [{'name': n, 'dim': d, 'scale': None, 'clip': None} for n, d in terms],
        'sim_dt': 0.005,
        'decimation': 4,
        'actuator': {'type': 'DCMotor', 'stiffness': 20.0, 'damping': 0.5, 'armature': 0.01,
                     'effort_limit': 25.0, 'saturation_effort': 25.0, 'velocity_limit': 30.0},
        'physx': {'solver_position_iterations': 4, 'solver_velocity_iterations': 0, 'self_collisions': False},
        'materials': {
            'ground': {'static_friction': 1.0, 'dynamic_friction': 1.0, 'restitution': 0.0,
                       'friction_combine_mode': 'multiply', 'restitution_combine_mode': 'multiply'},
            'robot': {'static_friction': 0.8, 'dynamic_friction': 0.6, 'restitution': 0.0},
        },
        'command_ranges': {'lin_vel_x': [-0.8, 0.8], 'lin_vel_y': [-0.5, 0.5], 'ang_vel_z': [-1.0, 1.0]},
        'spawn_height': 0.32,
    }


def test_valid_meta_passes(valid_meta):
    validate_policy_meta(valid_meta, policy_input_dim=48, policy_output_dim=12)
    assert obs_dim(valid_meta) == 48


def test_missing_keys_are_all_named(valid_meta):
    del valid_meta['materials']
    del valid_meta['sim_dt']
    with pytest.raises(PolicyMetaError, match='materials') as e:
        validate_policy_meta(valid_meta)
    assert 'sim_dt' in str(e.value)


def test_obs_dim_mismatch_reports_both(valid_meta):
    with pytest.raises(PolicyMetaError) as e:
        validate_policy_meta(valid_meta, policy_input_dim=45)
    assert '45' in str(e.value) and '48' in str(e.value)


def test_output_dim_mismatch(valid_meta):
    with pytest.raises(PolicyMetaError, match='joint'):
        validate_policy_meta(valid_meta, policy_output_dim=8)


def test_joint_lengths_must_match(valid_meta):
    valid_meta['default_joint_pos'] = valid_meta['default_joint_pos'][:11]
    with pytest.raises(PolicyMetaError, match='default_joint_pos'):
        validate_policy_meta(valid_meta)


def test_duplicate_joint_names(valid_meta):
    valid_meta['joint_names'][1] = valid_meta['joint_names'][0]
    with pytest.raises(PolicyMetaError, match='duplicate'):
        validate_policy_meta(valid_meta)


def test_bad_actuator_type(valid_meta):
    valid_meta['actuator']['type'] = 'IdealPD'
    with pytest.raises(PolicyMetaError, match='DCMotor'):
        validate_policy_meta(valid_meta)


def test_armature_is_required(valid_meta):
    # without armature the explicit PD diverges on the light legs, so the Isaac Sim side must set it
    del valid_meta['actuator']['armature']
    with pytest.raises(PolicyMetaError, match='armature'):
        validate_policy_meta(valid_meta)


def test_negative_armature_rejected(valid_meta):
    valid_meta['actuator']['armature'] = -0.01
    with pytest.raises(PolicyMetaError, match='armature'):
        validate_policy_meta(valid_meta)


def test_bad_command_range(valid_meta):
    valid_meta['command_ranges']['ang_vel_z'] = [1.0, -1.0]
    with pytest.raises(PolicyMetaError, match='ang_vel_z'):
        validate_policy_meta(valid_meta)


def test_roundtrip(valid_meta, tmp_path):
    p = tmp_path / 'policy.json'
    save_policy_meta(valid_meta, str(p))
    assert load_policy_meta(str(p), policy_input_dim=48, policy_output_dim=12) == valid_meta


def test_save_refuses_invalid_and_writes_nothing(valid_meta, tmp_path):
    del valid_meta['task']
    p = tmp_path / 'policy.json'
    with pytest.raises(PolicyMetaError):
        save_policy_meta(valid_meta, str(p))
    assert not p.exists()


def test_save_leaves_no_file_on_unserializable_value(valid_meta, tmp_path):
    valid_meta['spawn_height'] = object()
    p = tmp_path / 'policy.json'
    with pytest.raises((TypeError, PolicyMetaError)):
        save_policy_meta(valid_meta, str(p))
    assert not p.exists()
