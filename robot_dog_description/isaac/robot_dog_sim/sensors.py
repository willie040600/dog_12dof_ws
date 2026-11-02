"""IMU, camera and RTX LiDAR prims under base_link. Import only after SimulationApp has started."""
import math

import omni.kit.commands
from pxr import Gf, UsdGeom

from .util import log


def create_imu(stage, base_path, xyz):
    path = base_path + '/imu_sensor'
    try:
        omni.kit.commands.execute('IsaacSensorCreateImuSensor', path='imu_sensor', parent=base_path,
                                  translation=Gf.Vec3d(*xyz), orientation=Gf.Quatd(1, 0, 0, 0))
    except Exception as e:
        log('IsaacSensorCreateImuSensor unavailable (%s), defining the IMU prim directly' % e)
    if not stage.GetPrimAtPath(path).IsValid():
        prim = stage.DefinePrim(path, 'IsaacImuSensor')
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(*xyz))
    return path


def create_camera(stage, base_path, xyz, hfov, size):
    path = base_path + '/camera'
    cam = UsdGeom.Camera.Define(stage, path)
    # USD cameras look along -Z with +Y up; rows map them to +X forward, +Z up
    UsdGeom.Xformable(cam).AddTransformOp().Set(Gf.Matrix4d(0, -1, 0, 0,
                                                            0, 0, 1, 0,
                                                            -1, 0, 0, 0,
                                                            xyz[0], xyz[1], xyz[2], 1))
    aperture = 20.955  # mm
    width, height = size
    cam.CreateHorizontalApertureAttr(aperture)
    cam.CreateVerticalApertureAttr(aperture * height / width)
    cam.CreateFocalLengthAttr(aperture / (2 * math.tan(hfov / 2)))
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 20.0))
    return path


def _set_range(prim, near, far):
    for name, value in (('omni:sensor:Core:nearRangeM', near), ('omni:sensor:Core:farRangeM', far)):
        attr = prim.GetAttribute(name)
        if not attr.IsValid():
            raise RuntimeError('lidar has no %s attribute' % name)
        attr.Set(float(value))


def _rtx_lidar(stage, path, config, xyz):
    try:  # Isaac Sim 6.x API
        from isaacsim.sensors.experimental.rtx import Lidar
        Lidar.create(path, config=config, translations=[list(xyz)], tick_rate=10.0)
    except ImportError:  # legacy command (deprecated since 6.0)
        parent, name = path.rsplit('/', 1)
        omni.kit.commands.execute('IsaacSensorCreateRtxLidar', path=name, parent=parent, config=config,
                                  translation=Gf.Vec3d(*xyz), orientation=Gf.Quatd(1, 0, 0, 0))
    if not stage.GetPrimAtPath(path).IsValid():
        raise RuntimeError('no lidar prim at %s' % path)


def create_lidar(stage, base_path, xyz, range_m=None):
    """Planar RTX LiDAR -> (prim path, 'laser_scan', '/scan'); falls back to a 3D one on /scan/points.

    range_m = (near, far) overrides the profile's omni:sensor:Core:nearRangeM / farRangeM.
    """
    path = base_path + '/laser'
    try:
        _rtx_lidar(stage, path, 'Example_Rotary_2D', xyz)
        if range_m is not None:
            _set_range(stage.GetPrimAtPath(path), *range_m)
        return path, 'laser_scan', '/scan'
    except Exception as e:
        log('2D lidar config unavailable (%s); using Example_Rotary -> PointCloud2 on /scan/points' % e)
        stage.RemovePrim(path)
        _rtx_lidar(stage, path, 'Example_Rotary', xyz)
        return path, 'point_cloud', '/scan/points'
