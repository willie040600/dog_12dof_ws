"""Flat-ground velocity tracking for robot_dog, based on Isaac Lab's shared velocity task."""

from isaaclab.utils import configclass, replace

from isaaclab_tasks.core.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from robot_dog_lab.assets.robot_dog import ROBOT_DOG_CFG

from .mdp import feet_air_time_any_motion


@configclass
class RobotDogFlatEnvCfg(LocomotionVelocityRoughEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        # physics: PhysX by default like the Isaac Lab flat quadruped tasks; physics=newton_mjwarp also works
        newton_mjwarp = self.sim.physics.newton_mjwarp
        newton_mjwarp.solver_cfg.njmax = 60
        newton_mjwarp.solver_cfg.nconmax = 25
        self.sim.physics.default = self.sim.physics.isaacsim_physx

        # scene: flat plane, no height scan
        self.scene.robot = replace(ROBOT_DOG_CFG, prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        self.curriculum.terrain_levels = None

        # actions: q_target = default stand pose + 0.25 * action (config/rl_defaults.yaml)
        self.actions.joint_pos.scale = 0.25

        # commands: a 0.26 m tall, 8.8 kg dog
        self.commands.base_velocity.ranges.lin_vel_x = (-0.8, 0.8)
        self.commands.base_velocity.ranges.lin_vel_y = (-0.5, 0.5)
        self.commands.base_velocity.ranges.ang_vel_z = (-1.0, 1.0)
        # Nav2 sends wz directly, so sample it instead of deriving it from a heading error
        self.commands.base_velocity.heading_command = False
        # learn to stand still on a zero command instead of stepping in place
        self.commands.base_velocity.rel_standing_envs = 0.1

        # rewards: the foot sphere is a collision of the calf link
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_calf"
        self.rewards.feet_air_time.weight = 0.25
        # reward = air time - threshold per touchdown; Isaac Lab's 0.5 s (Go1-sized) turned every step of
        # this smaller dog into a penalty, and runs v1/v2 learned to stand still
        self.rewards.feet_air_time.params["threshold"] = 0.2
        # Isaac Lab's gate (|cmd_xy| > 0.1) never pays for steps on 0.1 m/s or turn-in-place commands
        self.rewards.feet_air_time.func = feet_air_time_any_motion
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = ".*_thigh"
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        # 0.25 (Isaac Lab: 0.5) so ignoring a 0.1 m/s command costs 15% of the tracking reward instead of 4%;
        # Nav2 sends such commands. Learning to walk from scratch is easier at 0.5, so stage 1 overrides it:
        #   train.py ... env.rewards.track_lin_vel_xy_exp.params.std=0.5 env.rewards.track_ang_vel_z_exp.params.std=0.5
        # and stage 2 fine-tunes from that checkpoint with this default (isaaclab/README.md)
        self.rewards.track_lin_vel_xy_exp.params["std"] = 0.25
        self.rewards.track_ang_vel_z_exp.params["std"] = 0.25
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.dof_acc_l2.weight = -2.5e-7
        self.rewards.flat_orientation_l2.weight = -2.5

        # terminations / events: the trunk link is base_link
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base_link"
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.base_com.params["asset_cfg"].body_names = "base_link"
