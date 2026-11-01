"""Zero-action stand test: the URDF, actuator model and standing pose hold the dog up.

Every env gets zero actions (joint targets = standing pose) for --seconds. Passes when every base
height is inside gait_criteria.STAND_HEIGHT_RANGE and no env terminated. Exit code 0 / 1.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/stand_test.py [--viz kit]
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description="Zero-action stand test for robot_dog.")
parser.add_argument("--num_envs", type=int, default=16)
parser.add_argument("--seconds", type=float, default=5.0)
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_lab.gait_criteria import judge_stand  # noqa: E402


def main():
    env_cfg = make_eval_env_cfg(args_cli.num_envs)
    with launch_simulation(env_cfg, args_cli):
        env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)
        uenv = env.unwrapped
        robot = uenv.scene["robot"]
        env.reset()
        actions = torch.zeros(uenv.num_envs, uenv.action_manager.total_action_dim, device=uenv.device)
        falls = 0
        with torch.inference_mode():
            for _ in range(round(args_cli.seconds / uenv.step_dt)):
                _, _, terminated, _, _ = env.step(actions)
                falls += int(terminated.sum())
        heights = (robot.data.root_pos_w.torch[:, 2] - uenv.scene.env_origins[:, 2]).tolist()
        ok, why = judge_stand(heights, falls)
        print("[STAND] height mean=%.3f min=%.3f max=%.3f falls=%d" % (
            sum(heights) / len(heights), min(heights), max(heights), falls), flush=True)
        print("[STAND] %s: %s" % ("PASS" if ok else "FAIL", why), flush=True)
        env.close()
        raise SystemExit(0 if ok else 1)  # inside the block, so launch_simulation keeps the status


if __name__ == "__main__":
    main()
