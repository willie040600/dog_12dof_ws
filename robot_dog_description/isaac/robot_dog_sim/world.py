"""Stage, physics scene and static geometry. Import only after SimulationApp has started."""
import os

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

from . import apartment


def setup_stage(stage, physics_hz):
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim('/World', 'Xform'))
    scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(physics_hz)


def ground_material(stage, static_friction=1.0, dynamic_friction=1.0, restitution=0.0):
    friction = UsdShade.Material.Define(stage, '/World/ground_friction')
    pm = UsdPhysics.MaterialAPI.Apply(friction.GetPrim())
    pm.CreateStaticFrictionAttr(static_friction)
    pm.CreateDynamicFrictionAttr(dynamic_friction)
    pm.CreateRestitutionAttr(restitution)
    return friction


def static_box(stage, path, center, size, color, material):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
        material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    return cube.GetPrim()


def add_lights(stage):
    UsdLux.DistantLight.Define(stage, '/World/sun').CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(stage, '/World/sky').CreateIntensityAttr(500)


def demo_world(stage, physics_hz):
    """Ground plane plus one box in front of the robot for the LiDAR and camera."""
    setup_stage(stage, physics_hz)
    friction = ground_material(stage)
    static_box(stage, '/World/ground', (0, 0, -0.05), (40, 40, 0.1), (0.6, 0.6, 0.6), friction)
    static_box(stage, '/World/box_obstacle', (1.5, 0.3, 0.25), (0.5, 0.5, 0.5), (0.8, 0.3, 0.2), friction)
    add_lights(stage)


def apply_physics_settings(stage, meta):
    """Physics scene as in training: dt, gravity and the PhysX solver settings from policy.json."""
    setup_stage(stage, int(round(1.0 / meta['sim_dt'])))
    scene = PhysxSchema.PhysxSceneAPI.Get(stage, '/World/physicsScene')
    s = meta['physx_scene']
    scene.CreateSolverTypeAttr('TGS' if s['solver_type'] == 1 else 'PGS')
    scene.CreateBounceThresholdAttr(s['bounce_threshold_velocity'])
    scene.CreateFrictionOffsetThresholdAttr(s['friction_offset_threshold'])
    scene.CreateFrictionCorrelationDistanceAttr(s['friction_correlation_distance'])
    scene.CreateEnableStabilizationAttr(s['enable_stabilization'])


def policy_ground(stage, meta):
    """Ground friction material with the training combine modes (the robot side is set on its shapes)."""
    g = meta['materials']['ground']
    material = ground_material(stage, g['static_friction'], g['dynamic_friction'], g['restitution'])
    physx = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    physx.CreateFrictionCombineModeAttr(g['friction_combine_mode'])
    physx.CreateRestitutionCombineModeAttr(g['restitution_combine_mode'])
    return material


def flat_world(stage, meta):
    """Training-like scene for the parity test: physics settings, a large ground plane, lights."""
    apply_physics_settings(stage, meta)
    static_box(stage, '/World/ground', (0, 0, -0.05), (100, 100, 0.1), (0.6, 0.6, 0.6), policy_ground(stage, meta))
    add_lights(stage)


def add_static_collision(stage, root_path, material):
    """Static triangle-mesh collision and the physics material on every visible mesh under root_path.

    The HIDDEN groups are made invisible first and get no collision. Everything is authored on this stage,
    so a referenced USD file is not changed. Returns the number of meshes given collision.
    """
    root = stage.GetPrimAtPath(root_path)
    for prim in Usd.PrimRange(root):
        if prim.GetName() in apartment.HIDDEN:
            UsdGeom.Imageable(prim).MakeInvisible()
    count = 0
    for prim in Usd.PrimRange(root):
        if prim.GetTypeName() != 'Mesh':
            continue
        visible = UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible
        if not apartment.wants_collision(prim.GetPath().pathString, visible):
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('none')
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
        count += 1
    return count


def usd_world(stage, meta, usd_path, offset_xy=(0.0, 0.0)):
    """Training physics, a USD scene shifted by -offset_xy with static collision on its meshes, and lights."""
    apply_physics_settings(stage, meta)
    scene = stage.DefinePrim('/World/scene', 'Xform')
    scene.GetReferences().AddReference(os.path.abspath(usd_path))
    UsdGeom.Xformable(scene).AddTranslateOp().Set(Gf.Vec3d(-offset_xy[0], -offset_xy[1], 0.0))
    count = add_static_collision(stage, '/World/scene', policy_ground(stage, meta))
    add_lights(stage)
    return count


def apartment_world(stage, meta):
    """The MUJI apartment, shifted so its spawn point is the world origin (spec 2026-11-03)."""
    missing = apartment.missing_textures(apartment.USD)
    if missing:
        raise RuntimeError('%s: %d textures missing next to it, e.g. %s' % (apartment.USD, len(missing), missing[0]))
    return usd_world(stage, meta, apartment.USD, apartment.SPAWN_XY)
