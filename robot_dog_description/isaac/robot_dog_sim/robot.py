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
