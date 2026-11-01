"""RobotDog12DOF: build a 12-DOF quadruped in Fusion for ROS 2 Jazzy / RL.

Frame convention (REP-103): X forward, Y left, Z up, base_link origin at the
trunk center. Every link is a top-level component modeled in place (identity
occurrence transform) with the legs straight down, which is the URDF zero pose
(all joint angles 0). Do not leave joints rotated when exporting.

Links:  base_link + {FL,FR,RL,RR}_{hip,thigh,calf}
Joints: {leg}_hip_joint   (abduction, axis +X)
        {leg}_thigh_joint (hip pitch, axis +Y)
        {leg}_calf_joint  (knee,      axis +Y)

Parts rigidly attached to the trunk (IMU, LiDAR, camera, hip motor mounts) are
bodies inside base_link, so any exporter turns them into one link. The foot
sphere is a body of the calf for the same reason. Joints use geometryOne =
child, geometryTwo = parent, as Fusion_URDF_Exporter_ROS2 expects.

Run: Utilities > Scripts and Add-Ins > RobotDog12DOF.
"""
import adsk.core
import adsk.fusion
import json
import math
import os
import traceback

# ---------------- dimensions (mm) ----------------
TRUNK_L, TRUNK_W, TRUNK_H = 340, 110, 90
TRUNK_FILLET = 10
HIP_X, HIP_Y = 200, 40            # hip abduction axis (front/rear, left/right)
MOUNT_R, MOUNT_X0, MOUNT_X1 = 32, 170, 180   # abduction motor mount on base_link
ROTOR_R, ROTOR_HALF = 30, 20      # abduction rotor, x = HIP_X +- ROTOR_HALF
ARM_HALF, ARM_Y1 = 12, 80         # hip arm from rotor out to the thigh motor
HFE_R, HFE_Y0, HFE_Y1 = 36, 76, 112          # thigh (hip pitch) motor housing
LEG_Y = 140                       # thigh / knee joint origins and foot plane
THIGH_Y0, THIGH_Y1 = 112, 131
THIGH_LEN, THIGH_HALF_W = 160, 14
THIGH_HUB_R, THIGH_KNEE_R = 28, 22
CALF_Y0, CALF_Y1 = 132, 148
CALF_LEN, CALF_HALF_W = 160, 10
CALF_HUB_R, FOOT_R = 18, 20

IMU_BOX = (-15, 15, -15, 15, 45, 51)
LIDAR_X, LIDAR_PED_R, LIDAR_R = 80, 28, 35
LIDAR_Z = (45, 58, 98)            # pedestal bottom, lidar bottom, lidar top
CAM_BOX = (148, 173, -45, 45, 45, 70)
CAM_LENS_R, CAM_LENS_X1 = 9, 178
LASER_Z = 85                      # scan plane height inside the LiDAR body

# Sensor frames (mm, in base_link). They carry no geometry of their own: the
# sensor bodies above belong to base_link, these are only the ROS frame_ids.
SENSOR_FRAMES = {
    'imu_link': ((IMU_BOX[0] + IMU_BOX[1]) / 2, (IMU_BOX[2] + IMU_BOX[3]) / 2, (IMU_BOX[4] + IMU_BOX[5]) / 2),
    'laser_link': (LIDAR_X, 0, LASER_Z),
    'camera_link': (CAM_LENS_X1, 0, (CAM_BOX[4] + CAM_BOX[5]) / 2),
}

# ---------------- joint limits (rad) ----------------
LIMITS = {'hip': (-0.80, 0.80), 'thigh': (-1.57, 3.49), 'calf': (-2.70, 0.0)}

# ---------------- materials / appearances ----------------
MAT = {'trunk': 'ABS Plastic', 'metal': 'Aluminum', 'calf': 'ABS Plastic', 'foot': 'Rubber, Black'}
LOOK = {
    'trunk': 'Paint - Enamel Glossy (White)',
    'metal': 'Aluminum - Anodized Glossy (Grey)',
    'thigh': 'Paint - Enamel Glossy (Dark Grey)',
    'calf': 'Plastic - Matte (Gray)',
    'foot': 'Rubber - Soft',
    'sensor': 'Plastic - Matte (Black)',
    'lens': 'Aluminum - Anodized Glossy (Blue)',
}

LEGS = [('FL', 1, 1), ('FR', 1, -1), ('RL', -1, 1), ('RR', -1, -1)]

NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
AXES = 'xyz'


def vr(mm):
    return adsk.core.ValueInput.createByReal(mm / 10.0)


def P(x, y, z):
    return adsk.core.Point3D.create(x / 10.0, y / 10.0, z / 10.0)


def pt(axis, s, a, b):
    """Point with coordinate s on `axis`; a, b on the remaining axes in xyz order."""
    others = [c for c in AXES if c != axis]
    v = {axis: s, others[0]: a, others[1]: b}
    return P(v['x'], v['y'], v['z'])


class Builder:
    def __init__(self, design):
        self.app = adsk.core.Application.get()
        self.design = design
        self.root = design.rootComponent
        self._planes = {}
        self._looks = {}
        self._mats = {}

    # ---------- library lookups ----------
    @staticmethod
    def _by_name(coll, name):
        try:
            return coll.itemByName(name) if coll.count else None
        except RuntimeError:  # some libraries raise instead of returning None
            return None

    def look(self, key):
        name = LOOK[key]
        if name not in self._looks:
            a = self._by_name(self.design.appearances, name)
            if not a:
                for lib in self.app.materialLibraries:
                    la = self._by_name(lib.appearances, name)
                    if la:
                        a = self.design.appearances.addByCopy(la, name)
                        break
            self._looks[name] = a
        return self._looks[name]

    def mat(self, key):
        name = MAT[key]
        if name not in self._mats:
            m = None
            for lib in self.app.materialLibraries:
                m = self._by_name(lib.materials, name)
                if m:
                    break
            self._mats[name] = m
        return self._mats[name]

    def style(self, body, look_key, mat_key=None):
        a = self.look(look_key)
        if a:
            body.appearance = a
        if mat_key:
            m = self.mat(mat_key)
            if m:
                body.material = m

    # ---------- construction helpers ----------
    def plane(self, comp, axis, off):
        key = (comp.name, axis, round(off, 4))
        if key in self._planes:
            return self._planes[key]
        base = {'x': comp.yZConstructionPlane, 'y': comp.xZConstructionPlane,
                'z': comp.xYConstructionPlane}[axis]
        if abs(off) < 1e-9:
            pl = base
        else:
            planes = comp.constructionPlanes
            pin = planes.createInput()
            pin.setByOffset(base, vr(off))
            pl = planes.add(pin)
            # offset direction follows the base plane normal; flip if needed
            if abs(getattr(pl.geometry.origin, axis) - off / 10.0) > 1e-6:
                pl.deleteMe()
                pin = planes.createInput()
                pin.setByOffset(base, vr(-off))
                pl = planes.add(pin)
            pl.isLightBulbOn = False
        self._planes[key] = pl
        return pl

    def _finish(self, feat, name, target):
        body = target if target else feat.bodies.item(0)
        if not target and name:
            body.name = name
        return body

    def _extrude(self, comp, sk, length, op, target, name):
        prof = sk.profiles.item(0)
        ext_in = comp.features.extrudeFeatures.createInput(prof, op)
        ext_in.setSymmetricExtent(vr(length), True)
        if target:
            ext_in.participantBodies = [target]
        feat = comp.features.extrudeFeatures.add(ext_in)
        sk.isVisible = False
        return self._finish(feat, name, target)

    def box(self, comp, x0, x1, y0, y1, z0, z1, target=None, name=None, axis='z'):
        lo = {'x': x0, 'y': y0, 'z': z0}
        hi = {'x': x1, 'y': y1, 'z': z1}
        mid = (lo[axis] + hi[axis]) / 2.0
        a, b = [c for c in AXES if c != axis]
        sk = comp.sketches.add(self.plane(comp, axis, mid))
        p0 = sk.modelToSketchSpace(pt(axis, mid, lo[a], lo[b]))
        p1 = sk.modelToSketchSpace(pt(axis, mid, hi[a], hi[b]))
        sk.sketchCurves.sketchLines.addTwoPointRectangle(p0, p1)
        return self._extrude(comp, sk, hi[axis] - lo[axis], JOIN if target else NEW, target, name)

    def cyl(self, comp, axis, ca, cb, r, s0, s1, target=None, name=None):
        mid = (s0 + s1) / 2.0
        sk = comp.sketches.add(self.plane(comp, axis, mid))
        c = sk.modelToSketchSpace(pt(axis, mid, ca, cb))
        sk.sketchCurves.sketchCircles.addByCenterRadius(c, r / 10.0)
        return self._extrude(comp, sk, abs(s1 - s0), JOIN if target else NEW, target, name)

    def sphere(self, comp, cx, cy, cz, r, target=None, name=None):
        sk = comp.sketches.add(self.plane(comp, 'y', cy))
        to = sk.modelToSketchSpace
        line = sk.sketchCurves.sketchLines.addByTwoPoints(to(P(cx - r, cy, cz)), to(P(cx + r, cy, cz)))
        sk.sketchCurves.sketchArcs.addByThreePoints(line.startSketchPoint, to(P(cx, cy, cz + r)),
                                                    line.endSketchPoint)
        rev_in = comp.features.revolveFeatures.createInput(sk.profiles.item(0), line, JOIN if target else NEW)
        rev_in.setAngleExtent(False, adsk.core.ValueInput.createByString('360 deg'))
        if target:
            rev_in.participantBodies = [target]
        feat = comp.features.revolveFeatures.add(rev_in)
        sk.isVisible = False
        return self._finish(feat, name, target)

    def fillet_all(self, comp, body, r):
        edges = adsk.core.ObjectCollection.create()
        for e in body.edges:
            edges.add(e)
        fin = comp.features.filletFeatures.createInput()
        try:
            fin.edgeSetInputs.addConstantRadiusEdgeSet(edges, vr(r), True)
        except AttributeError:
            fin.addConstantRadiusEdgeSet(edges, vr(r), True)
        comp.features.filletFeatures.add(fin)

    def frame_points(self, comp, pts):
        """Sketch points used as joint geometry (one sketch per component)."""
        sk = comp.sketches.add(comp.xYConstructionPlane)
        sk.name = 'joint_frames'
        out = [sk.sketchPoints.add(sk.modelToSketchSpace(P(*p))) for p in pts]
        sk.isVisible = False
        return out

    def new_link(self, name):
        occ = self.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        occ.component.name = name
        return occ

    def joint(self, name, child_occ, child_pt, parent_occ, parent_pt, axis, limits):
        g1 = adsk.fusion.JointGeometry.createByPoint(child_pt.createForAssemblyContext(child_occ))
        g2 = adsk.fusion.JointGeometry.createByPoint(parent_pt.createForAssemblyContext(parent_occ))
        jin = self.root.joints.createInput(g1, g2)
        jin.setAsRevoluteJointMotion({'x': adsk.fusion.JointDirections.XAxisJointDirection,
                                      'y': adsk.fusion.JointDirections.YAxisJointDirection}[axis])
        j = self.root.joints.add(jin)
        j.name = name
        lim = j.jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = limits[0]
        lim.isMaximumValueEnabled = True
        lim.maximumValue = limits[1]
        return j

    # ---------- links ----------
    def build_base(self):
        occ = self.new_link('base_link')
        comp = occ.component
        hl, hw, hh = TRUNK_L / 2, TRUNK_W / 2, TRUNK_H / 2
        trunk = self.box(comp, -hl, hl, -hw, hw, -hh, hh, name='trunk')
        self.fillet_all(comp, trunk, TRUNK_FILLET)
        self.style(trunk, 'trunk', 'trunk')

        for leg, sx, sy in LEGS:
            x0, x1 = sorted((sx * MOUNT_X0, sx * MOUNT_X1))
            m = self.cyl(comp, 'x', sy * HIP_Y, 0, MOUNT_R, x0, x1, name='hip_mount_' + leg)
            self.style(m, 'metal', 'metal')

        imu = self.box(comp, *IMU_BOX, name='imu')
        self.style(imu, 'sensor', 'trunk')

        z0, z1, z2 = LIDAR_Z
        ped = self.cyl(comp, 'z', LIDAR_X, 0, LIDAR_PED_R, z0, z1, name='lidar_mount')
        lidar = self.cyl(comp, 'z', LIDAR_X, 0, LIDAR_R, z1, z2, name='lidar')
        for b in (ped, lidar):
            self.style(b, 'sensor', 'trunk')

        cam = self.box(comp, *CAM_BOX, name='camera')
        self.style(cam, 'sensor', 'trunk')
        cz = (CAM_BOX[4] + CAM_BOX[5]) / 2
        lens = self.cyl(comp, 'x', 0, cz, CAM_LENS_R, CAM_BOX[1] - 1, CAM_LENS_X1, name='camera_lens')
        self.style(lens, 'lens', 'trunk')

        pts = {}
        for leg, sx, sy in LEGS:
            pts[leg] = self.frame_points(comp, [(sx * HIP_X, sy * HIP_Y, 0)])[0]
        occ.isGrounded = True
        return occ, pts

    def build_hip(self, leg, sx, sy):
        occ = self.new_link(leg + '_hip')
        comp = occ.component
        hx, hy = sx * HIP_X, sy * HIP_Y
        body = self.cyl(comp, 'x', hy, 0, ROTOR_R, hx - ROTOR_HALF, hx + ROTOR_HALF, name=leg + '_hip')
        y0, y1 = sorted((hy, sy * ARM_Y1))
        self.box(comp, hx - ARM_HALF, hx + ARM_HALF, y0, y1, -ARM_HALF, ARM_HALF, target=body, axis='y')
        y0, y1 = sorted((sy * HFE_Y0, sy * HFE_Y1))
        self.cyl(comp, 'y', hx, 0, HFE_R, y0, y1, target=body)
        self.style(body, 'metal', 'metal')
        p_hip, p_thigh = self.frame_points(comp, [(hx, hy, 0), (hx, sy * LEG_Y, 0)])
        return occ, p_hip, p_thigh

    def build_thigh(self, leg, sx, sy):
        occ = self.new_link(leg + '_thigh')
        comp = occ.component
        hx = sx * HIP_X
        y0, y1 = sorted((sy * THIGH_Y0, sy * THIGH_Y1))
        body = self.box(comp, hx - THIGH_HALF_W, hx + THIGH_HALF_W, y0, y1, -THIGH_LEN, 0,
                        name=leg + '_thigh', axis='y')
        self.cyl(comp, 'y', hx, 0, THIGH_HUB_R, y0, y1, target=body)
        self.cyl(comp, 'y', hx, -THIGH_LEN, THIGH_KNEE_R, y0, y1, target=body)
        self.style(body, 'thigh', 'metal')
        p_thigh, p_knee = self.frame_points(comp, [(hx, sy * LEG_Y, 0), (hx, sy * LEG_Y, -THIGH_LEN)])
        return occ, p_thigh, p_knee

    def build_calf(self, leg, sx, sy):
        occ = self.new_link(leg + '_calf')
        comp = occ.component
        hx = sx * HIP_X
        y0, y1 = sorted((sy * CALF_Y0, sy * CALF_Y1))
        zk, zf = -THIGH_LEN, -THIGH_LEN - CALF_LEN
        body = self.box(comp, hx - CALF_HALF_W, hx + CALF_HALF_W, y0, y1, zf, zk,
                        name=leg + '_calf', axis='y')
        self.cyl(comp, 'y', hx, zk, CALF_HUB_R, y0, y1, target=body)
        self.style(body, 'calf', 'calf')
        foot = self.sphere(comp, hx, sy * LEG_Y, zf, FOOT_R, name=leg + '_foot')
        self.style(foot, 'foot', 'foot')
        p_knee = self.frame_points(comp, [(hx, sy * LEG_Y, zk)])[0]
        return occ, p_knee

    def build(self):
        base, base_pts = self.build_base()
        for leg, sx, sy in LEGS:
            hip, h_hip, h_thigh = self.build_hip(leg, sx, sy)
            thigh, t_thigh, t_knee = self.build_thigh(leg, sx, sy)
            calf, c_knee = self.build_calf(leg, sx, sy)
            self.joint(leg + '_hip_joint', hip, h_hip, base, base_pts[leg], 'x', LIMITS['hip'])
            self.joint(leg + '_thigh_joint', thigh, t_thigh, hip, h_thigh, 'y', LIMITS['thigh'])
            self.joint(leg + '_calf_joint', calf, c_knee, thigh, t_knee, 'y', LIMITS['calf'])


def _target_design(app):
    """Build into the active design if it is empty, otherwise into a new document."""
    design = adsk.fusion.Design.cast(app.activeProduct)
    if design is None or design.rootComponent.occurrences.count or design.rootComponent.bRepBodies.count:
        doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    # Newer Fusion opens "Part Design" documents that allow a single component;
    # hybrid intent is the classic multi-component design the URDF exporters expect.
    if hasattr(design, 'designIntent'):
        design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    return design


def summary(design):
    lines = []
    total = 0.0
    for occ in design.rootComponent.occurrences:
        p = occ.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
        total += p.mass
        lines.append('%-10s %6.3f kg' % (occ.component.name, p.mass))
    lines.append('total      %6.3f kg' % total)
    for j in design.rootComponent.joints:
        m = j.jointMotion
        lines.append('%-16s %s -> %s axis=%s' % (
            j.name, j.occurrenceOne.component.name, j.occurrenceTwo.component.name,
            [round(v, 3) for v in m.rotationAxisVector.asArray()]))
    return '\n'.join(lines)


def collision_shapes(sx, sy):
    """Primitive collision shapes per link, in each link frame (m, rad).

    Simple shapes keep contact cheap and stable for RL; meshes stay visual only.
    """
    m = 0.001
    half_pi = math.pi / 2
    hfe_cy = sy * ((HFE_Y0 + HFE_Y1) / 2 - HIP_Y)
    thigh_cy = sy * ((THIGH_Y0 + THIGH_Y1) / 2 - LEG_Y)
    calf_cy = sy * ((CALF_Y0 + CALF_Y1) / 2 - LEG_Y)
    calf_box = CALF_LEN - FOOT_R
    return {
        'hip': [
            {'type': 'cylinder', 'radius': ROTOR_R * m, 'length': 2 * ROTOR_HALF * m,
             'xyz': [0, 0, 0], 'rpy': [0, half_pi, 0]},
            {'type': 'cylinder', 'radius': HFE_R * m, 'length': (HFE_Y1 - HFE_Y0) * m,
             'xyz': [0, hfe_cy * m, 0], 'rpy': [half_pi, 0, 0]},
        ],
        'thigh': [
            {'type': 'box', 'size': [2 * THIGH_HALF_W * m, (THIGH_Y1 - THIGH_Y0) * m, THIGH_LEN * m],
             'xyz': [0, thigh_cy * m, -THIGH_LEN / 2 * m], 'rpy': [0, 0, 0]},
        ],
        'calf': [
            {'type': 'box', 'size': [2 * CALF_HALF_W * m, (CALF_Y1 - CALF_Y0) * m, calf_box * m],
             'xyz': [0, calf_cy * m, -calf_box / 2 * m], 'rpy': [0, 0, 0]},
            {'type': 'sphere', 'radius': FOOT_R * m, 'xyz': [0, 0, -CALF_LEN * m], 'rpy': [0, 0, 0],
             'foot': True},
        ],
    }


def export(out_dir):
    """Export per-link STL (world frame, as Fusion writes it) and a JSON with
    mass properties, joints, sensor frames and collision shapes."""
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    moved = [j.name for j in root.joints if abs(j.jointMotion.rotationValue) > 1e-6]
    if moved:
        raise RuntimeError('Set these joints back to 0 before exporting: ' + ', '.join(moved))

    mesh_dir = os.path.join(out_dir, 'meshes_raw')
    os.makedirs(mesh_dir, exist_ok=True)
    em = design.exportManager
    links = {}
    for occ in root.occurrences:
        name = occ.component.name
        p = occ.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        _, xx, yy, zz, xy, yz, xz = p.getXYZMomentsOfInertia()
        _, i1, i2, i3 = p.getPrincipalMomentsOfInertia()
        links[name] = {
            'mass_kg': p.mass,
            'com_world_cm': list(p.centerOfMass.asArray()),
            'inertia_world_origin_kgcm2': [xx, yy, zz, xy, yz, xz],
            'principal_moments_kgcm2': [i1, i2, i3],
            'stl': name + '.stl',
        }
        opts = em.createSTLExportOptions(occ, os.path.join(mesh_dir, name + '.stl'))
        opts.isBinaryFormat = True
        opts.sendToPrintUtility = False
        opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
        em.execute(opts)

    joints = []
    for j in root.joints:
        lim = j.jointMotion.rotationLimits
        joints.append({
            'name': j.name,
            'parent': j.occurrenceTwo.component.name,
            'child': j.occurrenceOne.component.name,
            'origin_world_cm': list(j.geometryOrOriginTwo.origin.asArray()),
            'axis': [round(v, 6) for v in j.jointMotion.rotationAxisVector.asArray()],
            'lower': lim.minimumValue,
            'upper': lim.maximumValue,
        })

    collisions = {'base_link': [{'type': 'box', 'size': [TRUNK_L / 1000, TRUNK_W / 1000, TRUNK_H / 1000],
                                 'xyz': [0, 0, 0], 'rpy': [0, 0, 0]}]}
    for leg, sx, sy in LEGS:
        for part, shapes in collision_shapes(sx, sy).items():
            collisions['%s_%s' % (leg, part)] = shapes

    data = {
        'robot': 'robot_dog',
        'links': links,
        'joints': joints,
        'sensor_frames_m': {k: [c / 1000 for c in v] for k, v in SENSOR_FRAMES.items()},
        'foot_offset_m': [0, 0, -CALF_LEN / 1000],
        'collisions': collisions,
    }
    with open(os.path.join(out_dir, 'robot_dog_fusion.json'), 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)
    return data


def build():
    app = adsk.core.Application.get()
    design = _target_design(app)
    Builder(design).build()
    vp = app.activeViewport
    vp.viewOrientation = adsk.core.ViewOrientations.IsoTopRightViewOrientation
    vp.fit()
    return design


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = build()
        ui.messageBox('12-DOF robot dog built.\n\n' + summary(design), 'RobotDog12DOF')
    except Exception:
        if ui:
            ui.messageBox('Failed:\n' + traceback.format_exc(), 'RobotDog12DOF')
