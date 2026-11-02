"""Evaluate an exported robot_dog policy against the spec 4.3 criteria (robot_dog_lab.gait_criteria).

One env per case (17), all in parallel, commands held fixed. Each case is judged on the mean body
velocity over [WARMUP_S, WARMUP_S + MEASURE_S); the stand case on horizontal drift over
STAND_MEASURE_S. Any termination fails the case. Exit code 0 when all 17 pass, 1 otherwise.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/gait_eval.py \
        --policy_dir ~/dog_12dof_ws/robot_dog_description/isaac/policies/<run> [--viz kit]
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(DESC, "isaaclab"))
sys.path.insert(0, os.path.join(DESC, "isaac"))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description="Gait evaluation of an exported robot_dog policy.")
parser.add_argument("--policy_dir", required=True, help="Folder with policy.pt + policy.json (export_policy.py).")
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_lab.gait_criteria import CASES, MEASURE_S, STAND_MEASURE_S, WARMUP_S, judge_case  # noqa: E402
from robot_dog_sim.policy_meta import load_policy_meta  # noqa: E402


def main():
    policy_dir = os.path.abspath(os.path.expanduser(args_cli.policy_dir))
    meta = load_policy_meta(os.path.join(policy_dir, "policy.json"))
    env_cfg = make_eval_env_cfg(len(CASES))
    with launch_simulation(env_cfg, args_cli):
        env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)
        uenv = env.unwrapped
        if list(uenv.action_manager.get_term("joint_pos")._joint_names) != meta["joint_names"]:
            print("[GAIT] policy joint order does not match the current task; re-export the policy", flush=True)
            raise SystemExit(1)
        policy = torch.jit.load(os.path.join(policy_dir, "policy.pt"), map_location=uenv.device).eval()
        robot = uenv.scene["robot"]
        cmd_term = uenv.command_manager.get_term("base_velocity")
        cmds = torch.tensor([c.command for c in CASES], device=uenv.device)

        dt = uenv.step_dt
        warmup, measure, total = round(WARMUP_S / dt), round(MEASURE_S / dt), round((WARMUP_S + STAND_MEASURE_S) / dt)
        n = len(CASES)
        vel_sum = torch.zeros(n, 3, device=uenv.device)
        height_sum = torch.zeros(n, device=uenv.device)
        falls = torch.zeros(n, dtype=torch.long, device=uenv.device)
        start_xy = None

        obs, _ = env.reset()
        with torch.inference_mode():
            for i in range(total):
                cmd_term.vel_command_b[:] = cmds
                obs, _, terminated, _, _ = env.step(policy(obs["policy"]))
                falls += terminated.long()
                if i == warmup:
                    start_xy = robot.data.root_pos_w.torch[:, :2].clone()
                if warmup <= i < warmup + measure:
                    vel_sum[:, :2] += robot.data.root_lin_vel_b.torch[:, :2]
                    vel_sum[:, 2] += robot.data.root_ang_vel_b.torch[:, 2]
                    height_sum += robot.data.root_pos_w.torch[:, 2] - uenv.scene.env_origins[:, 2]
        mean_vel = (vel_sum / measure).tolist()
        height = (height_sum / measure).tolist()
        drift = torch.linalg.norm(robot.data.root_pos_w.torch[:, :2] - start_xy, dim=-1).tolist()
        falls = falls.tolist()

        passed = 0
        print("[GAIT] policy %s" % policy_dir, flush=True)
        for k, case in enumerate(CASES):
            ok, why = judge_case(case, tuple(mean_vel[k]), drift[k], falls[k], height[k])
            passed += ok
            print("[GAIT] %-8s cmd=(%+.2f %+.2f %+.2f) got=(%+.3f %+.3f %+.3f) z=%.3f drift=%.3f falls=%d %s %s" % (
                case.name, *case.command, *mean_vel[k], height[k], drift[k], falls[k], "PASS" if ok else "FAIL", why), flush=True)
        print("[GAIT] PASS %d/%d" % (passed, len(CASES)), flush=True)
        env.close()
        raise SystemExit(0 if passed == len(CASES) else 1)  # inside the block, so launch_simulation keeps it


if __name__ == "__main__":
    main()
