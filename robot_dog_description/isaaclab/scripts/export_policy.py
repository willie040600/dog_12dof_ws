"""Export a trained robot_dog checkpoint as policy.pt (TorchScript) + policy.json for the Isaac Sim runner.

policy.json is filled from the live env and config objects (joint order, default pose, observation
terms, physics, actuator, materials), not typed by hand. The export is refused when the action term
does not cover each robot joint exactly once, when the TorchScript output differs from the runner's
inference policy, or when policy.json fails robot_dog_sim.policy_meta validation.

    cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes PYTHONUNBUFFERED=1
    uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaaclab/scripts/export_policy.py \
        --run_dir ~/IsaacLab/logs/rsl_rl/robot_dog_flat/<run> [--checkpoint model_<N>.pt] [--out <dir>] [--force]

Default --out: robot_dog_description/isaac/policies/<run folder name>/
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(DESC, "isaaclab"))
sys.path.insert(0, os.path.join(DESC, "isaac"))

from isaaclab.app import add_launcher_args, launch_simulation  # noqa: E402

from robot_dog_lab.checkpoints import check_out_dir, latest_checkpoint, staged_dir  # noqa: E402

parser = argparse.ArgumentParser(description="Export robot_dog policy.pt + policy.json.")
parser.add_argument("--run_dir", required=True, help="Isaac Lab run folder with model_<N>.pt files.")
parser.add_argument("--checkpoint", default=None, help="Checkpoint file name (default: highest model_<N>.pt).")
parser.add_argument("--out", default=None, help="Output folder (default: isaac/policies/<run folder name>).")
parser.add_argument("--force", action="store_true", help="Overwrite a non-empty output folder.")
add_launcher_args(parser)
args_cli = parser.parse_args()

run_dir = os.path.abspath(os.path.expanduser(args_cli.run_dir))
run_name = os.path.basename(run_dir.rstrip("/"))
checkpoint = args_cli.checkpoint or latest_checkpoint(run_dir)
out_dir = os.path.abspath(os.path.expanduser(args_cli.out or os.path.join(DESC, "isaac", "policies", run_name)))
try:
    check_out_dir(out_dir, args_cli.force)  # before Isaac starts, so a refusal is immediate
except FileExistsError as e:
    print("[EXPORT] %s" % e, flush=True)
    raise SystemExit(1)

import gymnasium as gym  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.utils import to_dict  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper  # noqa: E402

import robot_dog_lab.tasks  # noqa: E402,F401
from robot_dog_lab.eval_env import make_eval_env_cfg  # noqa: E402
from robot_dog_lab.tasks.locomotion.velocity.agents.rsl_rl_ppo_cfg import RobotDogFlatPPORunnerCfg  # noqa: E402
from robot_dog_sim.policy_meta import save_policy_meta, validate_policy_meta  # noqa: E402

TASK = "RobotDog-Velocity-Flat"


def _num(x):
    return None if x is None else float(x)


def _pair(x):
    return None if x is None else [float(x[0]), float(x[1])]


def _mean(r):
    return (float(r[0]) + float(r[1])) / 2.0


def build_meta(env_cfg, agent_cfg, uenv):
    robot = uenv.scene["robot"]
    joint_names = list(uenv.action_manager.get_term("joint_pos")._joint_names)
    if len(joint_names) != 12 or sorted(joint_names) != sorted(robot.joint_names):
        print("[EXPORT] action term joints %s do not cover the robot joints %s once each"
              % (joint_names, list(robot.joint_names)), flush=True)
        raise SystemExit(1)
    default_q = robot.data.default_joint_pos.torch[0]
    obs_mgr = uenv.observation_manager
    obs_terms = []
    for name, shape, term in zip(obs_mgr.active_terms["policy"], obs_mgr.group_obs_term_dim["policy"],
                                 obs_mgr._group_obs_term_cfgs["policy"]):
        dim = 1
        for s in shape:
            dim *= int(s)
        obs_terms.append({"name": name, "dim": dim, "scale": _num(term.scale), "clip": _pair(term.clip)})

    robot_cfg = env_cfg.scene.robot
    legs = robot_cfg.actuators["legs"]
    physx = robot_cfg.spawn.articulation_props[0]
    rigid = robot_cfg.spawn.rigid_props
    scene = env_cfg.sim.physics  # the resolved PhysX config (make_eval_env_cfg picks isaacsim_physx)
    ground = env_cfg.scene.terrain.physics_material
    robot_mat = env_cfg.events.physics_material.params
    ranges = env_cfg.commands.base_velocity.ranges
    return {
        "task": TASK,
        "run": run_name,
        "checkpoint": checkpoint,
        "joint_names": joint_names,
        "default_joint_pos": [float(default_q[robot.joint_names.index(n)]) for n in joint_names],
        "action_scale": float(env_cfg.actions.joint_pos.scale),
        "clip_actions": _num(agent_cfg.clip_actions),
        "obs_terms": obs_terms,
        "sim_dt": float(env_cfg.sim.dt),
        "decimation": int(env_cfg.decimation),
        "actuator": {"type": "DCMotor", "stiffness": float(legs.stiffness), "damping": float(legs.damping),
                     "armature": float(legs.armature),
                     "effort_limit": float(legs.actuator_effort_limit),
                     "saturation_effort": float(legs.saturation_effort),
                     "velocity_limit": float(legs.actuator_velocity_limit)},
        "physx": {"solver_position_iterations": int(physx.solver_position_iteration_count),
                  "solver_velocity_iterations": int(physx.solver_velocity_iteration_count),
                  "self_collisions": bool(physx.enabled_self_collisions)},
        "materials": {
            "ground": {"static_friction": float(ground.static_friction),
                       "dynamic_friction": float(ground.dynamic_friction),
                       "restitution": float(ground.restitution),
                       "friction_combine_mode": str(ground.friction_combine_mode),
                       "restitution_combine_mode": str(ground.restitution_combine_mode)},
            "robot": {"static_friction": _mean(robot_mat["static_friction_range"]),
                      "dynamic_friction": _mean(robot_mat["dynamic_friction_range"]),
                      "restitution": _mean(robot_mat["restitution_range"])},
        },
        "command_ranges": {"lin_vel_x": _pair(ranges.lin_vel_x), "lin_vel_y": _pair(ranges.lin_vel_y),
                           "ang_vel_z": _pair(ranges.ang_vel_z)},
        "spawn_height": float(robot_cfg.init_state.pos[2]),
        "rigid_body": {k: float(getattr(rigid, k)) for k in ("max_depenetration_velocity", "linear_damping",
                                                             "angular_damping", "max_linear_velocity",
                                                             "max_angular_velocity")},
        "physx_scene": {"solver_type": int(scene.solver_type),
                        "bounce_threshold_velocity": float(scene.bounce_threshold_velocity),
                        "friction_offset_threshold": float(scene.friction_offset_threshold),
                        "friction_correlation_distance": float(scene.friction_correlation_distance),
                        "enable_stabilization": bool(scene.enable_stabilization)},
    }


def main():
    env_cfg = make_eval_env_cfg(1)
    agent_cfg = RobotDogFlatPPORunnerCfg()
    with launch_simulation(env_cfg, args_cli):
        env = gym.make(TASK, cfg=env_cfg)
        uenv = env.unwrapped
        agent_cfg.device = str(uenv.device)
        wrapped = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
        runner = OnPolicyRunner(wrapped, to_dict(agent_cfg), log_dir=None, device=agent_cfg.device)
        runner.load(os.path.join(run_dir, checkpoint))

        # everything goes into a staging folder that replaces out_dir only after every check passed
        with staged_dir(out_dir, args_cli.force) as tmp:
            runner.export_policy_to_jit(path=tmp, filename="policy.pt")
            meta = build_meta(env_cfg, agent_cfg, uenv)
            obs = wrapped.get_observations()
            jit = torch.jit.load(os.path.join(tmp, "policy.pt"), map_location="cpu").eval()
            with torch.inference_mode():
                a_jit = jit(obs["policy"].cpu())
                a_ref = runner.get_inference_policy(device=uenv.device)(obs).cpu()
            err = float((a_jit - a_ref).abs().max())
            if err >= 1e-5:
                print("[EXPORT] TorchScript output differs from the runner policy (max %.2e)" % err, flush=True)
                raise SystemExit(1)
            validate_policy_meta(meta, policy_input_dim=int(obs["policy"].shape[1]),
                                 policy_output_dim=int(a_jit.shape[1]))
            save_policy_meta(meta, os.path.join(tmp, "policy.json"))
        print("[EXPORT] wrote %s (%s / %s)" % (out_dir, run_name, checkpoint), flush=True)
        print("[EXPORT] joint order: %s" % meta["joint_names"], flush=True)
        print("[EXPORT] obs dim %d, max |jit - runner| = %.2e" % (obs["policy"].shape[1], err), flush=True)
        env.close()
        raise SystemExit(0)  # inside the block, so launch_simulation keeps the status


if __name__ == "__main__":
    main()
