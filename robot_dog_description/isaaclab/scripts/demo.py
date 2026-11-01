"""Show an exported robot_dog policy walking a scripted command sequence, camera following the dog.

Each phase holds one (vx, vy, wz) command; the terminal prints the commanded and achieved velocity
per phase. Green arrow = command, blue arrow = actual velocity. Close the window or wait for --loops.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/demo.py \
        --policy_dir ~/dog_12dof_ws/robot_dog_description/isaac/policies/<run> --viz kit
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(DESC, "isaaclab"))
sys.path.insert(0, os.path.join(DESC, "isaac"))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

parser = argparse.ArgumentParser(description="Scripted walking demo of an exported robot_dog policy.")
parser.add_argument("--policy_dir", required=True, help="Folder with policy.pt + policy.json (export_policy.py).")
parser.add_argument("--loops", type=int, default=2, help="How many times to run the sequence.")
add_launcher_args(parser)
args_cli = parser.parse_args()

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_sim.policy_meta import load_policy_meta  # noqa: E402

# (label, seconds, (vx m/s, vy m/s, wz rad/s))
PHASES = (
    ("stand", 2.0, (0.0, 0.0, 0.0)),
    ("forward 0.3", 4.0, (0.3, 0.0, 0.0)),
    ("forward 0.5", 3.0, (0.5, 0.0, 0.0)),
    ("slow forward 0.1", 4.0, (0.1, 0.0, 0.0)),
    ("sideways left 0.3", 4.0, (0.0, 0.3, 0.0)),
    ("slow sideways right 0.1", 4.0, (0.0, -0.1, 0.0)),
    ("turn left 0.8", 3.0, (0.0, 0.0, 0.8)),
    ("slow turn right 0.2", 4.0, (0.0, 0.0, -0.2)),
    ("walk and turn", 5.0, (0.3, 0.0, 0.5)),
    ("backward 0.3", 3.0, (-0.3, 0.0, 0.0)),
    ("stand", 3.0, (0.0, 0.0, 0.0)),
)


def main():
    policy_dir = os.path.abspath(os.path.expanduser(args_cli.policy_dir))
    meta = load_policy_meta(os.path.join(policy_dir, "policy.json"))
    env_cfg = make_eval_env_cfg(1)
    env_cfg.commands.base_velocity.debug_vis = True  # command / actual velocity arrows
    env_cfg.viewer.origin_type = "asset_root"
    env_cfg.viewer.asset_name = "robot"
    env_cfg.viewer.eye = (1.4, -1.4, 0.7)
    env_cfg.viewer.lookat = (0.0, 0.0, 0.0)
    with launch_simulation(env_cfg, args_cli):
        env = gym.make("RobotDog-Velocity-Flat", cfg=env_cfg)
        uenv = env.unwrapped
        if list(uenv.action_manager.get_term("joint_pos")._joint_names) != meta["joint_names"]:
            print("[DEMO] policy joint order does not match the current task; re-export the policy", flush=True)
            raise SystemExit(1)
        policy = torch.jit.load(os.path.join(policy_dir, "policy.pt"), map_location=uenv.device).eval()
        robot = uenv.scene["robot"]
        cmd_term = uenv.command_manager.get_term("base_velocity")
        print("[DEMO] policy %s" % policy_dir, flush=True)
        obs, _ = env.reset()
        with torch.inference_mode():
            for loop in range(args_cli.loops):
                for label, seconds, cmd in PHASES:
                    steps = round(seconds / uenv.step_dt)
                    cmd_t = torch.tensor([cmd], device=uenv.device)
                    got = torch.zeros(3, device=uenv.device)
                    for i in range(steps):
                        cmd_term.vel_command_b[:] = cmd_t
                        obs, _, terminated, _, _ = env.step(policy(obs["policy"]))
                        if terminated.any():
                            print("[DEMO] fell during '%s'" % label, flush=True)
                        if i >= steps // 3:  # skip the transition into the phase
                            got[:2] += robot.data.root_lin_vel_b.torch[0, :2]
                            got[2] += robot.data.root_ang_vel_b.torch[0, 2]
                    got /= steps - steps // 3
                    print("[DEMO] %-24s cmd=(%+.2f %+.2f %+.2f)  got=(%+.2f %+.2f %+.2f)" % (
                        label, *cmd, *got.tolist()), flush=True)
        env.close()
        raise SystemExit(0)


if __name__ == "__main__":
    main()
