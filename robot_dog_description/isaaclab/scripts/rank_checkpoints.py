"""Score every model_<N>.pt of one run with the gait_eval cases in a single Isaac process and name the best.

Same env, cases, timing and judge_case as gait_eval.py, but each checkpoint is loaded straight into the
runner (no export). Ranked by passed cases, then by mean relative error. Export the winner afterwards with
export_policy.py --checkpoint <name> and confirm with gait_eval.py.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/rank_checkpoints.py \
        --run_dir ~/IsaacLab/logs/rsl_rl/robot_dog_flat/<run> [--every 50]
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description="Rank a run's checkpoints with the gait_eval cases.")
parser.add_argument("--run_dir", required=True)
parser.add_argument("--every", type=int, default=50, help="Only checkpoints whose number is a multiple of this.")
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.utils import to_dict  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_lab.gait_criteria import CASES, MEASURE_S, STAND_MEASURE_S, WARMUP_S, judge_case  # noqa: E402
from robot_dog_lab.tasks.locomotion.velocity.agents.rsl_rl_ppo_cfg import RobotDogFlatPPORunnerCfg  # noqa: E402


def main():
    run_dir = os.path.abspath(os.path.expanduser(args_cli.run_dir))
    found = sorted((int(m.group(1)), n) for n in os.listdir(run_dir) if (m := re.match(r"^model_(\d+)\.pt$", n)))
    last = found[-1][0]
    ckpts = [(k, n) for k, n in found if k % args_cli.every == 0 or k == last]
    env_cfg = make_eval_env_cfg(len(CASES))
    agent_cfg = RobotDogFlatPPORunnerCfg()
    with launch_simulation(env_cfg, args_cli):
        env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)
        uenv = env.unwrapped
        agent_cfg.device = str(uenv.device)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = OnPolicyRunner(wrapped, to_dict(agent_cfg), log_dir=None, device=agent_cfg.device)
        robot = uenv.scene["robot"]
        cmd_term = uenv.command_manager.get_term("base_velocity")
        cmds = torch.tensor([c.command for c in CASES], device=uenv.device)
        dt = uenv.step_dt
        warmup, measure = round(WARMUP_S / dt), round(MEASURE_S / dt)
        total = round((WARMUP_S + STAND_MEASURE_S) / dt)
        results = []
        for k, name in ckpts:
            runner.load(os.path.join(run_dir, name))
            policy = runner.get_inference_policy(device=uenv.device)
            env.reset()
            obs = wrapped.get_observations()
            vel = torch.zeros(len(CASES), 3, device=uenv.device)
            height = torch.zeros(len(CASES), device=uenv.device)
            falls = torch.zeros(len(CASES), dtype=torch.long, device=uenv.device)
            with torch.inference_mode():
                for i in range(total):
                    cmd_term.vel_command_b[:] = cmds
                    obs, _, dones, extras = wrapped.step(policy(obs))
                    falls += (dones.bool() & ~extras["time_outs"].bool()).long()
                    if i == warmup:
                        start = robot.data.root_pos_w.torch[:, :2].clone()
                    if warmup <= i < warmup + measure:
                        vel[:, :2] += robot.data.root_lin_vel_b.torch[:, :2]
                        vel[:, 2] += robot.data.root_ang_vel_b.torch[:, 2]
                        height += robot.data.root_pos_w.torch[:, 2] - uenv.scene.env_origins[:, 2]
            vel, height = (vel / measure).tolist(), (height / measure).tolist()
            drift = torch.linalg.norm(robot.data.root_pos_w.torch[:, :2] - start, dim=-1).tolist()
            passed, rel_err, failed = 0, 0.0, []
            for j, c in enumerate(CASES):
                ok, _ = judge_case(c, tuple(vel[j]), drift[j], int(falls[j]), height[j])
                passed += ok
                if not ok:
                    failed.append(c.name)
                rel_err += drift[j] / 0.1 if c.axis is None else abs(vel[j][c.axis] - c.command[c.axis]) / abs(
                    c.command[c.axis])
            results.append((passed, -rel_err / len(CASES), k, name))
            print("[RANK] %-15s pass %2d/17  mean rel err %.3f  mean z %.3f  failed: %s" % (
                name, passed, rel_err / len(CASES), sum(height) / len(height), ",".join(failed) or "-"), flush=True)
        best = max(results)
        print("[RANK] BEST %s (%d/17, mean rel err %.3f)" % (best[3], best[0], -best[1]), flush=True)
        env.close()
        raise SystemExit(0)  # inside the block, so launch_simulation keeps the status


if __name__ == "__main__":
    main()
