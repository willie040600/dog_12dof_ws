"""Record the Isaac Lab side of the parity test (spec M2): body velocities along robot_dog_sim.parity.SEGMENTS.

One env, nominal robot (make_eval_env_cfg). Each policy step: set that step's command, recompute the policy
observation from the current state, act, step; then record the body-frame (vx, vy, wz). Isaac Sim's
policy_parity.py runs the same order and compares against the JSON written here.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/parity_reference.py \
        --policy_dir <policies/run> --out <policies/run>/parity_reference.json
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(DESC, "isaaclab"))
sys.path.insert(0, os.path.join(DESC, "isaac"))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description="Isaac Lab reference run for the parity test.")
parser.add_argument("--policy_dir", required=True)
parser.add_argument("--out", required=True)
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_sim.parity import SEGMENTS, command_at, segment_means, total_steps  # noqa: E402
from robot_dog_sim.policy_meta import load_policy_meta  # noqa: E402


def main():
    policy_dir = os.path.abspath(os.path.expanduser(args_cli.policy_dir))
    meta = load_policy_meta(os.path.join(policy_dir, "policy.json"))
    env_cfg = make_eval_env_cfg(1)
    with launch_simulation(env_cfg, args_cli):
        env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)
        uenv = env.unwrapped
        if list(uenv.action_manager.get_term("joint_pos")._joint_names) != meta["joint_names"]:
            print("[PARITY-REF] policy joint order does not match the task", flush=True)
            raise SystemExit(1)
        policy = torch.jit.load(os.path.join(policy_dir, "policy.pt"), map_location=uenv.device).eval()
        robot = uenv.scene["robot"]
        cmd_term = uenv.command_manager.get_term("base_velocity")
        dt = uenv.step_dt
        env.reset()
        samples, falls = [], 0
        with torch.inference_mode():
            for i in range(total_steps(dt)):
                cmd_term.vel_command_b[:] = torch.tensor([command_at(i, dt)], device=uenv.device)
                obs = uenv.observation_manager.compute_group("policy")
                _, _, terminated, _, _ = env.step(policy(obs))
                falls += int(terminated.sum())
                v = robot.data.root_lin_vel_b.torch[0]
                w = robot.data.root_ang_vel_b.torch[0]
                z = robot.data.root_pos_w.torch[0, 2] - uenv.scene.env_origins[0, 2]
                samples.append((float(v[0]), float(v[1]), float(w[2]), float(z)))
        means = segment_means(samples, dt)
        with open(os.path.expanduser(args_cli.out), "w") as f:
            json.dump({"policy": os.path.basename(policy_dir), "checkpoint": meta["checkpoint"], "dt": dt,
                       "segments": [s[0] for s in SEGMENTS], "means": means, "falls": falls}, f, indent=2)
        for (label, _, cmd), m in zip(SEGMENTS, means):
            print("[PARITY-REF] %-24s cmd=(%+.2f %+.2f %+.2f) got=(%+.3f %+.3f %+.3f) z=%.3f" % (label, *cmd, *m), flush=True)
        print("[PARITY-REF] falls=%d, wrote %s" % (falls, args_cli.out), flush=True)
        env.close()
        raise SystemExit(0 if falls == 0 else 1)


if __name__ == "__main__":
    main()
