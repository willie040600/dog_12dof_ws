"""URDF import, robot prim lookup and joint drives. Import only after SimulationApp has started."""
import dataclasses
import glob
import math
import os

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from .util import log

DEG = math.pi / 180.0


def convert_urdf(urdf_path, usd_dir):
    """URDF -> USD with the Isaac Sim 6 importer (URDFParseAndImportFile was removed in 6.0)."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    wanted = dict(urdf_path=os.path.abspath(urdf_path), usd_path=usd_dir, fix_base=False,
                  merge_fixed_joints=False, collision_from_visuals=False, allow_self_collision=False,
                  joint_drive_type='force', joint_target_type='position', robot_type='Quadruped')
    known = {f.name for f in dataclasses.fields(URDFImporterConfig)}
    skipped = sorted(set(wanted) - known)
    if skipped:
        log('importer options not in this version, skipped: %s' % ', '.join(skipped))
    os.makedirs(usd_dir, exist_ok=True)
    out = URDFImporter(URDFImporterConfig(**{k: v for k, v in wanted.items() if k in known})).import_urdf()
    out = str(out) if out else ''
    if not os.path.isfile(out):  # fall back to the newest robot USD under the output folder
        found = sorted(glob.glob(os.path.join(usd_dir, '**', 'robot_dog*.usd*'), recursive=True),
                       key=os.path.getmtime)
        if not found:
            raise RuntimeError('URDF importer produced no USD in %s' % usd_dir)
        out = found[-1]
    return out


def load_robot(stage, app, usd_path, spawn_z):
    log('robot USD: %s' % usd_path)
    prim = stage.DefinePrim('/World/robot_dog', 'Xform')
    prim.GetReferences().AddReference(usd_path)
    # the importer puts joints/articulation/mass behind an unselected "Physics" variant set;
    # pick the backend-portable one, as Isaac Lab's converter does by default
    variants = prim.GetVariantSets()
    if variants.HasVariantSet('Physics'):
        variants.GetVariantSet('Physics').SetVariantSelection('physics')
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, spawn_z))
    app.update()
    return prim


def robot_parts(root):
    """Find prims by name, so the importer's Geometry/Physics layout does not matter."""
    art = base = None
    joints = {}
    for prim in Usd.PrimRange(root):
        if art is None and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art = prim
        if prim.GetName() == 'base_link' and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            base = prim
        if prim.IsA(UsdPhysics.RevoluteJoint):
            joints[prim.GetName()] = prim
    return art or base, base, joints


def configure_drives(joints, kp, kd, max_effort, max_velocity, stand):
    """PD drive gains, limits and the standing pose as initial state.
    USD angular drives store gains per degree, so gains and targets are converted."""
    for name, prim in joints.items():
        q = stand[name.split('_')[1]]
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(kp * DEG)
        drive.CreateDampingAttr(kd * DEG)
        drive.CreateMaxForceAttr(max_effort)
        drive.CreateTargetPositionAttr(q / DEG)
        drive.CreateTargetVelocityAttr(0.0)
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(max_velocity / DEG)
        state = PhysxSchema.JointStateAPI.Apply(prim, 'angular')
        for attr in ('CreatePositionStateAttr', 'CreatePositionAttr'):
            if hasattr(state, attr):
                getattr(state, attr)(q / DEG)
                break


def author_policy_physics(root, art_prim, meta):
    """USD side, before the timeline plays: rigid-body and articulation solver settings from policy.json."""
    rb = meta['rigid_body']
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
            api.CreateMaxDepenetrationVelocityAttr(rb['max_depenetration_velocity'])
            api.CreateLinearDampingAttr(rb['linear_damping'])
            api.CreateAngularDampingAttr(rb['angular_damping'])
            api.CreateMaxLinearVelocityAttr(rb['max_linear_velocity'])
            api.CreateMaxAngularVelocityAttr(rb['max_angular_velocity'])  # Isaac Lab writes it unconverted
    px = meta['physx']
    art = PhysxSchema.PhysxArticulationAPI.Apply(art_prim)
    art.CreateSolverPositionIterationCountAttr(px['solver_position_iterations'])
    art.CreateSolverVelocityIterationCountAttr(px['solver_velocity_iterations'])
    art.CreateEnabledSelfCollisionsAttr(px['self_collisions'])


def apply_policy_dof_settings(articulation, meta, dof_indices):
    """Tensor side, after the timeline plays: torque-driven joints, armature, limits, friction, start pose.

    dof_indices maps the policy's joint order to the articulation's (observations.joint_index_map).
    """
    import warp as wp

    act = meta['actuator']
    n = len(dof_indices)
    articulation.set_dof_drive_types('force', dof_indices=dof_indices)
    articulation.set_dof_gains([[0.0] * n], [[0.0] * n], dof_indices=dof_indices)  # torques come from actuator.py
    articulation.set_dof_armatures([[act['armature']] * n], dof_indices=dof_indices)
    articulation.set_dof_max_efforts([[act['effort_limit']] * n], dof_indices=dof_indices)
    articulation.set_dof_max_velocities([[act['velocity_limit']] * n], dof_indices=dof_indices)
    # robot friction on every collision shape, as Isaac Lab's physics_material event does
    view = articulation._physics_articulation_view
    mat = wp.to_torch(view.get_material_properties()).clone()
    r = meta['materials']['robot']
    mat[..., 0], mat[..., 1], mat[..., 2] = r['static_friction'], r['dynamic_friction'], r['restitution']
    view.set_material_properties(wp.from_torch(mat.contiguous()), wp.array([0], dtype=wp.int32))
    reset_pose(articulation, meta, dof_indices)


def reset_pose(articulation, meta, dof_indices):
    """Standing pose at the spawn height, at rest (Isaac Lab's reset with the eval env's zero ranges)."""
    articulation.set_world_poses(positions=[[0.0, 0.0, meta['spawn_height']]], orientations=[[1.0, 0.0, 0.0, 0.0]])
    articulation.set_velocities(linear_velocities=[[0.0, 0.0, 0.0]], angular_velocities=[[0.0, 0.0, 0.0]])
    articulation.set_dof_positions([meta['default_joint_pos']], dof_indices=dof_indices)
    articulation.set_dof_velocities([[0.0] * len(dof_indices)], dof_indices=dof_indices)
