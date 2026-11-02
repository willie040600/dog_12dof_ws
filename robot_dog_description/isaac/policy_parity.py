"""Parity test (spec M2): run the exported policy in Isaac Sim and compare with Isaac Lab's reference run.

Same sequence (robot_dog_sim.parity.SEGMENTS) and step order as isaaclab/scripts/parity_reference.py: each
policy step observes with that step's command, acts, runs `decimation` physics steps, then records the
body-frame (vx, vy, wz). No ROS, no sensors. Exit code 0 when every segment is within 0.05 m/s / 0.1 rad/s.

  cd ~/IsaacLab && uv run --extra isaacsim python <this file> --policy <policies/run> \
      --reference <policies/run>/parity_reference.json [--headless]
"""
import argparse
import json
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('--policy', required=True, help='folder with policy.pt + policy.json')
parser.add_argument('--reference', required=True, help='parity_reference.json from the Isaac Lab side')
parser.add_argument('--usd-dir', default=os.path.join(tempfile.gettempdir(), 'robot_dog_isaac_usd'))
parser.add_argument('--headless', action='store_true')
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402  must start before any other omni import

simulation_app = SimulationApp({'headless': args.headless})

import torch  # noqa: E402
from isaacsim.core.experimental.prims import Articulation  # noqa: E402

from robot_dog_sim import kit, world  # noqa: E402
from robot_dog_sim.observations import quat_rotate_inverse_wxyz  # noqa: E402
from robot_dog_sim.parity import SEGMENTS, command_at, compare, segment_means, total_steps, verdict  # noqa: E402
from robot_dog_sim.policy_meta import load_policy_meta  # noqa: E402
from robot_dog_sim.policy_runner import PolicyRunner  # noqa: E402
from robot_dog_sim.robot import (apply_policy_dof_settings, author_policy_physics, convert_urdf,  # noqa: E402
                                 load_robot, robot_parts)
from robot_dog_sim.robot_spec import DEFAULT_URDF  # noqa: E402
from robot_dog_sim.stepping import Lockstep  # noqa: E402
from robot_dog_sim.util import log  # noqa: E402

kit.enable_extensions()
stage = kit.new_stage(simulation_app)


def body_sample(articulation):
    """(vx, vy, wz) in the body frame and the base height, like parity_reference.py records them."""
    pos, quat = articulation.get_world_poses()
    lin, ang = articulation.get_velocities()
    q = torch.as_tensor(quat.numpy()[0], dtype=torch.float32)
    v = quat_rotate_inverse_wxyz(q, torch.as_tensor(lin.numpy()[0], dtype=torch.float32))
    w = quat_rotate_inverse_wxyz(q, torch.as_tensor(ang.numpy()[0], dtype=torch.float32))
    return float(v[0]), float(v[1]), float(w[2]), float(pos.numpy()[0][2])


def main():
    policy_dir = os.path.abspath(os.path.expanduser(args.policy))
    meta = load_policy_meta(os.path.join(policy_dir, 'policy.json'))
    with open(os.path.expanduser(args.reference)) as f:
        ref = json.load(f)

    world.flat_world(stage, meta)
    root = load_robot(stage, simulation_app, convert_urdf(DEFAULT_URDF, args.usd_dir), meta['spawn_height'])
    art_prim, base, joints = robot_parts(root)
    author_policy_physics(root, art_prim, meta)
    dt = meta['sim_dt'] * meta['decimation']
    lock = Lockstep(simulation_app, meta['sim_dt'], meta['decimation'])
    articulation = Articulation(art_prim.GetPath().pathString)
    runner = PolicyRunner(meta, os.path.join(policy_dir, 'policy.pt'), articulation, hold=False)
    apply_policy_dof_settings(articulation, meta, runner.dof_indices)
    log('parity: %s, armatures %s' % (os.path.basename(policy_dir),
                                      articulation.get_dof_armatures().numpy()[0][:3].round(4).tolist()))

    samples, fallen_at = [], None
    for i in range(total_steps(dt)):
        runner.policy_step(command_at(i, dt), i * dt)
        lock.step(runner.pre_physics)
        samples.append(body_sample(articulation))
        if runner.fallen and fallen_at is None:
            fallen_at = i * dt
    got = segment_means(samples, dt)
    for (label, _, cmd), r, g in zip(SEGMENTS, ref['means'], got):
        log('%-24s cmd=(%+.2f %+.2f %+.2f) lab=(%+.3f %+.3f %+.3f) z=%.3f  sim=(%+.3f %+.3f %+.3f) z=%.3f'
            % (label, *cmd, *r, *g))
    failures = compare(ref['means'], got)
    if fallen_at is not None:
        log('PARITY FAIL Isaac Sim: base below 0.12 m or tilted > 60 deg first at t=%.2f s' % fallen_at)
    if ref.get('falls', 0):
        log('PARITY FAIL Isaac Lab reference: %d fall(s)' % ref['falls'])
    for msg in failures:
        log('PARITY FAIL ' + msg)
    ok = verdict(failures, fallen_at is not None, ref.get('falls', 0))
    log('parity %s: %d/%d segments within 0.05 m/s / 0.1 rad/s' % (
        'PASS' if ok else 'FAIL', len(SEGMENTS) - len(failures), len(SEGMENTS)))
    return 0 if ok else 1


exit_code = 1
try:
    exit_code = main()
except Exception:
    traceback.print_exc()  # close() ends the process with _exit(), so print before it
finally:
    simulation_app.close(exit_code=exit_code)
