"""Load robot_dog into Isaac Sim 6.2 and expose it to ROS 2 the same way as Gazebo.

Run with the Isaac Sim 6.2 pip install in ~/IsaacLab's uv environment, ROS 2 Jazzy sourced:
  source /opt/ros/jazzy/setup.bash
  cd ~/IsaacLab && export OMNI_KIT_ACCEPT_EULA=yes
  uv run --extra isaacsim python <path>/robot_dog_isaac.py            # Isaac PD drives follow position commands
  uv run --extra isaacsim python <path>/robot_dog_isaac.py --effort   # drives off, joints take torques
  ... --headless --max-frames 600                                     # smoke test: run 600 frames, then exit 0
Then on the ROS 2 side:
  ros2 launch robot_dog_description isaac.launch.py [command:=effort]

Publishes  /clock  /isaac_joint_states  /imu  /scan  /camera/image_raw  /camera/camera_info
Subscribes /isaac_joint_commands  (sensor_msgs/JointState, from ros2_control)

Written against the Isaac Sim 6.x APIs (URDFImporter, isaacsim.sensors.experimental.rtx,
ROS 2 OmniGraph nodes from isaacsim.ros2.nodes). Each sensor is set up on its own: if a
call differs in your build, a warning is printed and the rest keeps running.
"""
import argparse
import dataclasses
import glob
import math
import os
import sys
import tempfile
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from robot_dog_sim.util import log, require_ros_jazzy, try_step  # noqa: E402

parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('--urdf', default=os.path.join(HERE, '..', 'urdf', 'robot_dog_rl.urdf'))
parser.add_argument('--usd', help='reference an already converted USD instead of importing the URDF')
parser.add_argument('--usd-dir', default=os.path.join(tempfile.gettempdir(), 'robot_dog_isaac_usd'),
                    help='where the URDF importer writes the USD')
parser.add_argument('--effort', action='store_true', help='torque control: drive stiffness/damping = 0')
parser.add_argument('--kp', type=float, default=20.0, help='drive stiffness, N*m/rad')
parser.add_argument('--kd', type=float, default=0.5, help='drive damping, N*m*s/rad')
parser.add_argument('--physics-hz', type=int, default=500)
parser.add_argument('--headless', action='store_true')
parser.add_argument('--max-frames', type=int, default=0, help='exit after this many app updates (0: run until closed)')
args, _ = parser.parse_known_args()
require_ros_jazzy()

from isaacsim import SimulationApp  # noqa: E402  must start before any other omni import

simulation_app = SimulationApp({'headless': args.headless})

import omni.graph.core as og  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade  # noqa: E402

STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}   # rad
EFFORT, VELOCITY = 25.0, 30.0                      # N*m, rad/s (placeholders, as in the URDF)
SPAWN_Z = 0.32
# sensor poses in base_link (m), identical to the URDF frames
IMU_XYZ = (0.0, 0.0, 0.048)
LASER_XYZ = (0.08, 0.0, 0.085)
CAMERA_XYZ = (0.178, 0.0, 0.0575)
CAMERA_HFOV = 1.204  # rad, same as Gazebo
DEG = math.pi / 180.0

# OmniGraph node types keep the isaacsim.ros2.bridge prefix in 6.x (extension isaacsim.ros2.nodes)
ROS = 'isaacsim.ros2.bridge.'
TICK = ('Tick', 'omni.graph.action.OnPlaybackTick')
SIM_TIME = ('SimTime', 'isaacsim.core.nodes.IsaacReadSimulationTime')
CONTEXT = ('Context', ROS + 'ROS2Context')
RENDER_PRODUCT = ('RenderProduct', 'isaacsim.core.nodes.IsaacCreateRenderProduct')

_ext = omni.kit.app.get_app().get_extension_manager()
for _name in ('isaacsim.asset.importer.urdf', 'isaacsim.core.nodes', 'isaacsim.sensors.physics',
              'isaacsim.sensors.experimental.rtx', 'isaacsim.ros2.bridge', 'isaacsim.ros2.nodes'):
    _ext.set_extension_enabled_immediate(_name, True)  # no-op if already on or absent
omni.usd.get_context().new_stage()
simulation_app.update()
stage = omni.usd.get_context().get_stage()


# ---------------- scene ----------------
def setup_world():
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetDefaultPrim(stage.DefinePrim('/World', 'Xform'))
    scene = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim()).CreateTimeStepsPerSecondAttr(args.physics_hz)

    friction = UsdShade.Material.Define(stage, '/World/ground_friction')
    pm = UsdPhysics.MaterialAPI.Apply(friction.GetPrim())
    pm.CreateStaticFrictionAttr(1.0)
    pm.CreateDynamicFrictionAttr(1.0)
    pm.CreateRestitutionAttr(0.0)

    def static_box(path, center, size, color):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*center))
        xf.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(
            friction, UsdShade.Tokens.weakerThanDescendants, 'physics')

    static_box('/World/ground', (0, 0, -0.05), (40, 40, 0.1), (0.6, 0.6, 0.6))
    static_box('/World/box_obstacle', (1.5, 0.3, 0.25), (0.5, 0.5, 0.5), (0.8, 0.3, 0.2))  # for LiDAR/camera
    UsdLux.DistantLight.Define(stage, '/World/sun').CreateIntensityAttr(3000)
    UsdLux.DomeLight.Define(stage, '/World/sky').CreateIntensityAttr(500)


def convert_urdf():
    """URDF -> USD with the Isaac Sim 6 importer (URDFParseAndImportFile was removed in 6.0)."""
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    wanted = dict(urdf_path=os.path.abspath(args.urdf), usd_path=args.usd_dir, fix_base=False,
                  merge_fixed_joints=False, collision_from_visuals=False, allow_self_collision=False,
                  joint_drive_type='force', joint_target_type='position', robot_type='Quadruped')
    known = {f.name for f in dataclasses.fields(URDFImporterConfig)}
    skipped = sorted(set(wanted) - known)
    if skipped:
        log('importer options not in this version, skipped: %s' % ', '.join(skipped))
    os.makedirs(args.usd_dir, exist_ok=True)
    out = URDFImporter(URDFImporterConfig(**{k: v for k, v in wanted.items() if k in known})).import_urdf()
    out = str(out) if out else ''
    if not os.path.isfile(out):  # fall back to the newest robot USD under the output folder
        found = sorted(glob.glob(os.path.join(args.usd_dir, '**', 'robot_dog*.usd*'), recursive=True),
                       key=os.path.getmtime)
        if not found:
            raise RuntimeError('URDF importer produced no USD in %s' % args.usd_dir)
        out = found[-1]
    return out


def load_robot():
    usd = os.path.abspath(args.usd) if args.usd else convert_urdf()
    log('robot USD: %s' % usd)
    prim = stage.DefinePrim('/World/robot_dog', 'Xform')
    prim.GetReferences().AddReference(usd)
    # the importer puts joints/articulation/mass behind an unselected "Physics" variant set;
    # pick the backend-portable one, as Isaac Lab's converter does by default
    variants = prim.GetVariantSets()
    if variants.HasVariantSet('Physics'):
        variants.GetVariantSet('Physics').SetVariantSelection('physics')
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0, 0, SPAWN_Z))
    simulation_app.update()
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


def configure_joints(joints):
    """PD drive gains, limits and the standing pose as initial state.
    USD angular drives store gains per degree, so gains and targets are converted."""
    kp, kd = (0.0, 0.0) if args.effort else (args.kp, args.kd)
    for name, prim in joints.items():
        q = STAND[name.split('_')[1]]
        drive = UsdPhysics.DriveAPI.Apply(prim, 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateStiffnessAttr(kp * DEG)
        drive.CreateDampingAttr(kd * DEG)
        drive.CreateMaxForceAttr(EFFORT)
        drive.CreateTargetPositionAttr(q / DEG)
        drive.CreateTargetVelocityAttr(0.0)
        PhysxSchema.PhysxJointAPI.Apply(prim).CreateMaxJointVelocityAttr(VELOCITY / DEG)
        state = PhysxSchema.JointStateAPI.Apply(prim, 'angular')
        for attr in ('CreatePositionStateAttr', 'CreatePositionAttr'):
            if hasattr(state, attr):
                getattr(state, attr)(q / DEG)
                break


# ---------------- ROS 2 graphs ----------------
def graph(path, nodes, values, connections):
    try:
        og.Controller.edit({'graph_path': path, 'evaluator_name': 'execution'}, {
            og.Controller.Keys.CREATE_NODES: nodes,
            og.Controller.Keys.SET_VALUES: values,
            og.Controller.Keys.CONNECT: connections,
        })
    except Exception:
        stage.RemovePrim(path)  # do not leave half-built graphs behind
        raise


def joint_graph(art_path):
    """Layout of the Isaac Sim 6 ROS 2 manipulation tutorial: read joint state -> publish,
    subscribe joint commands -> articulation controller, plus /clock."""
    fields = ('jointNames', 'jointPositions', 'jointVelocities', 'jointEfforts', 'jointDofTypes',
              'stageMetersPerUnit', 'sensorTime')
    graph('/ROS/joints',
          [TICK, SIM_TIME, CONTEXT,
           ('ReadJointState', 'isaacsim.sensors.physics.IsaacReadJointState'),
           ('PubJointState', ROS + 'ROS2PublishJointState'),
           ('SubJointCmd', ROS + 'ROS2SubscribeJointState'),
           ('Articulation', 'isaacsim.core.nodes.IsaacArticulationController'),
           ('PubClock', ROS + 'ROS2PublishClock')],
          [('SimTime.inputs:resetOnStop', True),
           ('ReadJointState.inputs:prim', [Sdf.Path(art_path)]),
           ('PubJointState.inputs:topicName', '/isaac_joint_states'),
           ('SubJointCmd.inputs:topicName', '/isaac_joint_commands'),
           ('Articulation.inputs:targetPrim', [Sdf.Path(art_path)]),
           ('PubClock.inputs:topicName', '/clock')],
          [('Tick.outputs:tick', 'ReadJointState.inputs:execIn'),
           ('ReadJointState.outputs:execOut', 'PubJointState.inputs:execIn'),
           ('Tick.outputs:tick', 'SubJointCmd.inputs:execIn'),
           ('Tick.outputs:tick', 'Articulation.inputs:execIn'),
           ('Tick.outputs:tick', 'PubClock.inputs:execIn'),
           ('SimTime.outputs:simulationTime', 'PubJointState.inputs:timeStamp'),
           ('SimTime.outputs:simulationTime', 'PubClock.inputs:timeStamp'),
           ('Context.outputs:context', 'PubJointState.inputs:context'),
           ('Context.outputs:context', 'SubJointCmd.inputs:context'),
           ('Context.outputs:context', 'PubClock.inputs:context')]
          + [('ReadJointState.outputs:%s' % f, 'PubJointState.inputs:%s' % f) for f in fields]
          + [('SubJointCmd.outputs:%s' % a, 'Articulation.inputs:%s' % a)
             for a in ('jointNames', 'positionCommand', 'velocityCommand', 'effortCommand')])


def create_imu(base_path):
    path = base_path + '/imu_sensor'
    try:
        omni.kit.commands.execute('IsaacSensorCreateImuSensor', path='imu_sensor', parent=base_path,
                                  translation=Gf.Vec3d(*IMU_XYZ), orientation=Gf.Quatd(1, 0, 0, 0))
    except Exception as e:
        log('IsaacSensorCreateImuSensor unavailable (%s), defining the IMU prim directly' % e)
    if not stage.GetPrimAtPath(path).IsValid():
        prim = stage.DefinePrim(path, 'IsaacImuSensor')
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*IMU_XYZ))
    return path


def imu_graph(base_path):
    imu_path = create_imu(base_path)
    graph('/ROS/imu',
          [TICK, SIM_TIME, CONTEXT, ('ReadIMU', 'isaacsim.sensors.physics.IsaacReadIMU'),
           ('PubImu', ROS + 'ROS2PublishImu')],
          [('ReadIMU.inputs:imuPrim', [Sdf.Path(imu_path)]), ('ReadIMU.inputs:readGravity', True),
           ('PubImu.inputs:frameId', 'imu_link'), ('PubImu.inputs:topicName', '/imu')],
          [('Tick.outputs:tick', 'ReadIMU.inputs:execIn'),
           ('ReadIMU.outputs:execOut', 'PubImu.inputs:execIn'),
           ('ReadIMU.outputs:angVel', 'PubImu.inputs:angularVelocity'),
           ('ReadIMU.outputs:linAcc', 'PubImu.inputs:linearAcceleration'),
           ('ReadIMU.outputs:orientation', 'PubImu.inputs:orientation'),
           ('SimTime.outputs:simulationTime', 'PubImu.inputs:timeStamp'),
           ('Context.outputs:context', 'PubImu.inputs:context')])


def camera_graph(base_path):
    path = base_path + '/camera'
    cam = UsdGeom.Camera.Define(stage, path)
    # USD cameras look along -Z with +Y up; rows map them to +X forward, +Z up
    UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d(0, -1, 0, 0,
                                                            0, 0, 1, 0,
                                                            -1, 0, 0, 0,
                                                            CAMERA_XYZ[0], CAMERA_XYZ[1], CAMERA_XYZ[2], 1))
    aperture = 20.955  # mm
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture * 480 / 640)
    cam.CreateFocalLengthAttr(aperture / (2 * math.tan(CAMERA_HFOV / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
    graph('/ROS/camera',
          [TICK, CONTEXT, RENDER_PRODUCT,
           ('PubImage', ROS + 'ROS2CameraHelper'),
           ('PubInfo', ROS + 'ROS2CameraInfoHelper')],
          [('RenderProduct.inputs:cameraPrim', [Sdf.Path(path)]),
           ('RenderProduct.inputs:width', 640), ('RenderProduct.inputs:height', 480),
           ('PubImage.inputs:type', 'rgb'), ('PubImage.inputs:topicName', '/camera/image_raw'),
           ('PubImage.inputs:frameId', 'camera_optical_link'),
           ('PubInfo.inputs:topicName', '/camera/camera_info'),
           ('PubInfo.inputs:frameId', 'camera_optical_link')],
          [('Tick.outputs:tick', 'RenderProduct.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubImage.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubInfo.inputs:execIn'),
           ('RenderProduct.outputs:renderProductPath', 'PubImage.inputs:renderProductPath'),
           ('RenderProduct.outputs:renderProductPath', 'PubInfo.inputs:renderProductPath'),
           ('Context.outputs:context', 'PubImage.inputs:context'),
           ('Context.outputs:context', 'PubInfo.inputs:context')])


def create_lidar(path, config):
    try:  # Isaac Sim 6.x API
        from isaacsim.sensors.experimental.rtx import Lidar
        Lidar.create(path, config=config, translations=[list(LASER_XYZ)], tick_rate=10.0)
    except ImportError:  # legacy command (deprecated since 6.0)
        parent, name = path.rsplit('/', 1)
        omni.kit.commands.execute('IsaacSensorCreateRtxLidar', path=name, parent=parent, config=config,
                                  translation=Gf.Vec3d(*LASER_XYZ), orientation=Gf.Quatd(1, 0, 0, 0))
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError('no lidar prim at %s' % path)


def lidar_graph(base_path):
    path = base_path + '/laser'
    try:
        create_lidar(path, 'Example_Rotary_2D')   # planar scan -> sensor_msgs/LaserScan on /scan
        kind, topic = 'laser_scan', '/scan'
    except Exception as e:
        log('2D lidar config unavailable (%s); using Example_Rotary -> PointCloud2 on /scan/points' % e)
        stage.RemovePrim(path)
        create_lidar(path, 'Example_Rotary')
        kind, topic = 'point_cloud', '/scan/points'
    graph('/ROS/lidar',
          [TICK, CONTEXT, RENDER_PRODUCT, ('PubScan', ROS + 'ROS2RtxLidarHelper')],
          [('RenderProduct.inputs:cameraPrim', [Sdf.Path(path)]),
           ('PubScan.inputs:type', kind), ('PubScan.inputs:topicName', topic),
           ('PubScan.inputs:frameId', 'laser_link')],
          [('Tick.outputs:tick', 'RenderProduct.inputs:execIn'),
           ('RenderProduct.outputs:execOut', 'PubScan.inputs:execIn'),
           ('RenderProduct.outputs:renderProductPath', 'PubScan.inputs:renderProductPath'),
           ('Context.outputs:context', 'PubScan.inputs:context')])


def main():
    setup_world()
    root = load_robot()
    art, base, joints = robot_parts(root)
    if base is None or len(joints) != 12:
        raise RuntimeError('expected base_link and 12 revolute joints, found %s / %d' % (base, len(joints)))
    configure_joints(joints)
    log('articulation root %s, base %s, %s control' % (
        art.GetPath(), base.GetPath(), 'effort' if args.effort else 'position (kp=%g kd=%g)' % (args.kp, args.kd)))

    base_path = base.GetPath().pathString
    if not try_step('joint states / commands / clock', joint_graph, art.GetPath().pathString):
        log('ros2_control cannot reach the robot without the joint graph')
    try_step('IMU -> /imu', imu_graph, base_path)
    try_step('camera -> /camera/image_raw', camera_graph, base_path)
    try_step('LiDAR', lidar_graph, base_path)

    omni.timeline.get_timeline_interface().play()
    frames = 0
    while simulation_app.is_running() and (args.max_frames <= 0 or frames < args.max_frames):
        simulation_app.update()
        frames += 1


exit_code = 0
try:
    main()
except Exception:
    traceback.print_exc()  # close() ends the process with _exit(), so print before it
    exit_code = 1
finally:
    simulation_app.close(exit_code=exit_code)
