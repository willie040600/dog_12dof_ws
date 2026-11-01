"""Flat-ground velocity tracking for robot_dog, based on Isaac Lab's shared velocity task."""

from isaaclab.utils import configclass, replace

from isaaclab_tasks.core.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from robot_dog_lab.assets.robot_dog import ROBOT_DOG_CFG


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

        # rewards: the foot sphere is a collision of the calf link
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = ".*_calf"
        self.rewards.feet_air_time.weight = 0.25
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = ".*_thigh"
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.75
        self.rewards.dof_torques_l2.weight = -0.0002
        self.rewards.dof_acc_l2.weight = -2.5e-7
        self.rewards.flat_orientation_l2.weight = -2.5

        # terminations / events: the trunk link is base_link
        self.terminations.base_contact.params["sensor_cfg"].body_names = "base_link"
        self.events.add_base_mass.params["asset_cfg"].body_names = "base_link"
        self.events.base_external_force_torque.params["asset_cfg"].body_names = "base_link"
        self.events.base_com.params["asset_cfg"].body_names = "base_link"
