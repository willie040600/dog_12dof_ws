"""Run in the Isaac Lab uv env (needs torch and isaaclab)."""
import copy
import math
import os

import pytest
import torch

from isaaclab.utils.math import quat_apply_inverse
from robot_dog_sim.observations import (build_observation, check_obs_terms, joint_index_map,
                                        quat_rotate_inverse_wxyz)
from robot_dog_sim.policy_meta import PolicyMetaError, load_policy_meta

V7 = os.path.join(os.path.dirname(__file__), '..', 'policies', '2026-11-01_20-43-36_v7', 'policy.json')


@pytest.fixture
def meta():
    return load_policy_meta(V7)


def test_rotate_inverse_matches_isaac_lab():
    torch.manual_seed(0)
    for _ in range(200):
        q = torch.nn.functional.normalize(torch.randn(4), dim=0)  # wxyz
        v = torch.randn(3)
        ref = quat_apply_inverse(q[[1, 2, 3, 0]], v)  # Isaac Lab 3.0 is xyzw
        assert torch.allclose(quat_rotate_inverse_wxyz(q, v), ref, atol=1e-6)


def test_projected_gravity_level_and_pitched():
    g = torch.tensor([0.0, 0.0, -1.0])
    assert torch.allclose(quat_rotate_inverse_wxyz(torch.tensor([1.0, 0, 0, 0]), g), g)
    h = math.radians(15)
    q = torch.tensor([math.cos(h), 0.0, math.sin(h), 0.0])
    assert torch.allclose(quat_rotate_inverse_wxyz(q, g), torch.tensor([0.5, 0.0, -0.8660254]), atol=1e-6)


def test_joint_index_map_by_name(meta):
    names = meta['joint_names']
    dof = list(reversed(names))
    idx = joint_index_map(names, dof)
    assert [dof[i] for i in idx] == names
    with pytest.raises(PolicyMetaError, match='FL_calf_joint'):
        joint_index_map(names, [n for n in dof if n != 'FL_calf_joint'])


def test_obs_terms_checked(meta):
    check_obs_terms(meta)
    swapped = copy.deepcopy(meta)
    swapped['obs_terms'][0], swapped['obs_terms'][1] = swapped['obs_terms'][1], swapped['obs_terms'][0]
    with pytest.raises(PolicyMetaError):
        check_obs_terms(swapped)
    scaled = copy.deepcopy(meta)
    scaled['obs_terms'][2]['scale'] = 2.0
    with pytest.raises(PolicyMetaError):
        check_obs_terms(scaled)


def test_build_observation_layout(meta):
    q = torch.linspace(-1, 1, 12)
    qd = torch.linspace(2, 3, 12)
    last = torch.arange(12, dtype=torch.float32)
    obs = build_observation(meta, torch.tensor([1.0, 0, 0, 0]), torch.tensor([0.2, 0.0, 0.0]),
                            torch.tensor([0.0, 0.0, 0.4]), (0.3, -0.1, 0.5), q, qd, last)
    default = torch.tensor(meta['default_joint_pos'])
    assert obs.shape == (48,)
    assert torch.allclose(obs[0:3], torch.tensor([0.2, 0.0, 0.0]))
    assert torch.allclose(obs[3:6], torch.tensor([0.0, 0.0, 0.4]))
    assert torch.allclose(obs[6:9], torch.tensor([0.0, 0.0, -1.0]))
    assert torch.allclose(obs[9:12], torch.tensor([0.3, -0.1, 0.5]))
    assert torch.allclose(obs[12:24], q - default)
    assert torch.allclose(obs[24:36], qd)
    assert torch.allclose(obs[36:48], last)
