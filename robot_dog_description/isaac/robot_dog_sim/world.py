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
