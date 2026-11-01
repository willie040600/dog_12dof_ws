"""Isaac Lab 3.0 articulation config of robot_dog (12-DOF quadruped designed in Fusion).

The URDF is converted to USD on first use (Isaac Sim 6.2 URDF importer 3.0) and cached.
Set ROBOT_DOG_URDF to use a URDF somewhere else.
"""
import os

from isaaclab_newton.sim.schemas import NewtonArticulationCfg
from isaaclab_physx.sim.schemas import PhysxArticulationCfg, PhysxRigidBodyCfg

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

ROBOT_DOG_URDF = os.environ.get(
    "ROBOT_DOG_URDF",
    os.path.join(os.path.dirname(__file__), "..", "data", "robot_dog", "urdf", "robot_dog_rl.urdf"),
)

ROBOT_DOG_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=ROBOT_DOG_URDF,
        fix_base=False,
        merge_fixed_joints=False,  # the RL URDF has no fixed joints: sensors are already part of base_link
        robot_type="Quadruped",
        # explicit actuator model below computes the torques, so the imported drives stay passive
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0),
        ),
        activate_contact_sensors=True,
        rigid_props=PhysxRigidBodyCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=[
            PhysxArticulationCfg(
                enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
            ),
            NewtonArticulationCfg(self_collision_enabled=False),
        ],
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.32),
        joint_pos={".*_hip_joint": 0.0, ".*_thigh_joint": 0.8, ".*_calf_joint": -1.5},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        # placeholder motor: 25 N*m, 30 rad/s (same as the URDF limits); replace with your datasheet
        "legs": DCMotorCfg(
            joint_names_expr=[".*_hip_joint", ".*_thigh_joint", ".*_calf_joint"],
            actuator_effort_limit=25.0,
            saturation_effort=25.0,
            actuator_velocity_limit=30.0,
            stiffness=20.0,
            damping=0.5,
            friction=0.0,
            # reflected rotor inertia (kg m^2, typical of a geared leg motor, as in Isaac Lab's Unitree configs);
            # without it this explicit PD diverges on the light calves (I ~1.3e-3) at dt 0.005
            armature=0.01,
        ),
    },
)
"""robot_dog with a DC-motor actuator model (PD gains as in config/rl_defaults.yaml)."""
