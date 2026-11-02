import glob
import os

from robot_dog_sim.policy_meta import load_policy_meta

POLICIES = sorted(glob.glob(os.path.join(os.path.dirname(__file__), '..', 'policies', '*', 'policy.json')))


def test_every_committed_policy_loads():
    # a policy swap once left tests pointing at a deleted folder; every committed policy must stay loadable
    assert POLICIES
    for path in POLICIES:
        meta = load_policy_meta(path, policy_input_dim=48, policy_output_dim=12)
        assert os.path.isfile(os.path.join(os.path.dirname(path), 'policy.pt')), path
        assert meta['checkpoint'].startswith('model_')
