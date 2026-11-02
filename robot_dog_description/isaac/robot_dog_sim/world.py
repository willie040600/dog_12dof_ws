"""Stage, physics scene and static geometry. Import only after SimulationApp has started."""
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics, UsdShade


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


# the room's dimensions and obstacles live in room_layout.py (shared with nav_goals and map_check)
from .room_layout import (BOXES, CYLINDER_HEIGHT, CYLINDER_RADIUS, CYLINDERS, INNER_WALLS,  # noqa: E402
                          ROOM_HALF, WALL_HEIGHT, WALL_THICKNESS)


def static_cylinder(stage, path, center, radius, height, color, material):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr('Z')
    cyl.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(*center))
    UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cyl.GetPrim()).Bind(material, UsdShade.Tokens.weakerThanDescendants, 'physics')
    return cyl.GetPrim()


def room_world(stage, meta):
    """Training physics and ground, plus the 10 x 10 m room with two inner walls, boxes and cylinders."""
    apply_physics_settings(stage, meta)
    ground = policy_ground(stage, meta)
    static_box(stage, '/World/ground', (0, 0, -0.05), (40, 40, 0.1), (0.6, 0.6, 0.6), ground)
    wall, h, t, r = (0.75, 0.72, 0.68), WALL_HEIGHT, WALL_THICKNESS, ROOM_HALF
    for i, (center, size) in enumerate((((0, r, h / 2), (2 * r + t, t, h)), ((0, -r, h / 2), (2 * r + t, t, h)),
                                        ((r, 0, h / 2), (t, 2 * r, h)), ((-r, 0, h / 2), (t, 2 * r, h)))):
        static_box(stage, '/World/room/outer_%d' % i, center, size, wall, ground)
    for i, (x, (g0, g1)) in enumerate(INNER_WALLS):
        for j, (y0, y1) in enumerate(((-r, g0), (g1, r))):
            static_box(stage, '/World/room/inner_%d_%d' % (i, j), (x, (y0 + y1) / 2, h / 2), (t, y1 - y0, h), wall,
                       ground)
    for i, ((x, y), s) in enumerate(BOXES):
        static_box(stage, '/World/room/box_%d' % i, (x, y, s / 2), (s, s, s), (0.8, 0.3, 0.2), ground)
    for i, (x, y) in enumerate(CYLINDERS):
        static_cylinder(stage, '/World/room/cylinder_%d' % i, (x, y, CYLINDER_HEIGHT / 2), CYLINDER_RADIUS,
                        CYLINDER_HEIGHT, (0.2, 0.4, 0.8), ground)
    add_lights(stage)
