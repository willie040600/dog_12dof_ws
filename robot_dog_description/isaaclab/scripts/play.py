"""Play a trained robot_dog policy with Isaac Lab's unified play CLI, e.g.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/play.py \
        --task RobotDog-Velocity-Flat --checkpoint <run dir>/model_<N>.pt --viz kit
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import warp as wp  # noqa: E402

wp.config.enable_backward = False

import robot_dog_lab.tasks  # noqa: E402,F401  (registers RobotDog-* gym ids)
from isaaclab_rl.entrypoints import run_play_cli  # noqa: E402

if __name__ == "__main__":
    argv = sys.argv[1:]
    if not any(a.startswith("--rl_library") for a in argv):
        argv = ["--rl_library", "rsl_rl"] + argv  # the only agent config robot_dog_lab registers
    raise SystemExit(run_play_cli(argv))
