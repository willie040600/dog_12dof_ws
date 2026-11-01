"""Train robot_dog locomotion with Isaac Lab's unified train CLI.

Registers the RobotDog-* tasks, then hands all arguments to isaaclab_rl's train entry point, e.g.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/train.py \
        --task RobotDog-Velocity-Flat --run_name v1
    ... --num_envs 64 --max_iterations 5 --run_name smoke      # quick smoke test

Logs and checkpoints go to ~/IsaacLab/logs/rsl_rl/robot_dog_flat/<time>_<run_name>/.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import warp as wp  # noqa: E402

wp.config.enable_backward = False

import robot_dog_lab.tasks  # noqa: E402,F401  (registers RobotDog-* gym ids)
from isaaclab_rl.entrypoints import run_train_cli  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if not any(a.startswith("--rl_library") for a in argv):
        argv = ["--rl_library", "rsl_rl"] + argv  # the only agent config robot_dog_lab registers
    raise SystemExit(run_train_cli(argv))
