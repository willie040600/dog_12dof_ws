#!/usr/bin/env python3
"""Turn the Fusion export (robot_dog_fusion.json + meshes_raw/*.stl) into the
generated parts of the robot_dog_description package:

  meshes/<link>.stl          link-frame meshes in meters
  urdf/robot_dog_core.xacro  links, joints, sensor and foot frames (ROS 2)
  urdf/robot_dog_rl.urdf     plain URDF for RL simulators (Isaac Lab, MuJoCo, ...)
  isaaclab/robot_dog_lab/data/robot_dog/   copy of the RL URDF + meshes for the Isaac Lab project

Pure standard library, so it runs on Windows or inside a ROS 2 workspace.
Usage: python tools/generate_description.py [--export fusion/] [--pkg .]
"""
import argparse
import json
import math
import os
import shutil
import struct

# Actuator placeholders: replace with the datasheet values of your motors.
EFFORT = {'hip': 25.0, 'thigh': 25.0, 'calf': 25.0}      # N*m
VELOCITY = {'hip': 30.0, 'thigh': 30.0, 'calf': 30.0}    # rad/s
DAMPING, FRICTION = 0.01, 0.0
PKG = 'robot_dog_description'

COLORS = {
    'base_link': ('white', '0.92 0.92 0.88 1'),
    'hip': ('metal', '0.55 0.56 0.60 1'),
    'thigh': ('dark_grey', '0.25 0.25 0.27 1'),
    'calf': ('light_grey', '0.70 0.70 0.70 1'),
}


def fmt(v):
    s = '%.6g' % round(v, 10)  # drop float noise such as 1e-19 products of inertia
    return '0' if s in ('-0', '0') else s


def vec(v):
    return ' '.join(fmt(x) for x in v)


# ---------------- mass properties ----------------
def eig_sym3(a):
    """Eigenvalues of a symmetric 3x3 matrix (Jacobi rotations)."""
    a = [row[:] for row in a]
    for _ in range(50):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) < 1e-15:
            break
        theta = (a[q][q] - a[p][p]) / (2 * a[p][q])
        t = math.copysign(1.0, theta) / (abs(theta) + math.sqrt(theta * theta + 1))
        c = 1 / math.sqrt(t * t + 1)
        s = t * c
        for k in range(3):
            akp, akq = a[k][p], a[k][q]
            a[k][p], a[k][q] = c * akp - s * akq, s * akp + c * akq
        for k in range(3):
            apk, aqk = a[p][k], a[q][k]
            a[p][k], a[q][k] = c * apk - s * aqk, s * apk + c * aqk
    return sorted(a[i][i] for i in range(3))


def inertia_at_com(link, sign):
    """Fusion gives moments about the world origin (kg*cm^2). Returns the URDF
    inertia tensor at the COM in kg*m^2; `sign` selects the product convention."""
    xx, yy, zz, xy, yz, xz = [v * 1e-4 for v in link['inertia_world_origin_kgcm2']]
    t = [[xx, sign * xy, sign * xz], [sign * xy, yy, sign * yz], [sign * xz, sign * yz, zz]]
    m = link['mass_kg']
    c = [v * 0.01 for v in link['com_world_cm']]
    c2 = sum(v * v for v in c)
    return [[t[i][j] - m * ((c2 if i == j else 0.0) - c[i] * c[j]) for j in range(3)] for i in range(3)]


def com_inertia(name, link):
    """Pick the product-of-inertia sign whose eigenvalues match Fusion's principal moments."""
    principal = sorted(v * 1e-4 for v in link['principal_moments_kgcm2'])
    best = None
    for sign in (1.0, -1.0):
        t = inertia_at_com(link, sign)
        err = max(abs(a - b) for a, b in zip(eig_sym3(t), principal)) / max(principal)
        if best is None or err < best[0]:
            best = (err, t)
    if best[0] > 1e-3:
        raise ValueError('%s: inertia does not match Fusion principal moments (rel err %.2e)' % (name, best[0]))
    return best[1]


# ---------------- meshes ----------------
def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    n = struct.unpack_from('<I', data, 80)[0]
    tris = [struct.unpack_from('<12f', data, 84 + 50 * i) for i in range(n)]
    return tris


def write_stl(path, tris):
    with open(path, 'wb') as f:
        f.write(b'robot_dog link mesh, meters, link frame'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            f.write(struct.pack('<12fH', *t, 0))


def stl_unit_scale(tris, expected_extent_m):
    xs = [t[i] for t in tris for i in (3, 6, 9)]
    extent = max(xs) - min(xs)
    for scale in (0.001, 0.01, 1.0):  # mm, cm, m
        if abs(extent * scale - expected_extent_m) / expected_extent_m < 0.25:
            return scale
    raise ValueError('cannot infer STL units (x extent %.4g)' % extent)


def convert_meshes(export_dir, pkg_dir, data, frames):
    raw = os.path.join(export_dir, 'meshes_raw')
    out = os.path.join(pkg_dir, 'meshes')
    os.makedirs(out, exist_ok=True)
    trunk_len = data['collisions']['base_link'][0]['size'][0]
    scale = stl_unit_scale(read_stl(os.path.join(raw, 'base_link.stl')), trunk_len)
    for name, link in data['links'].items():
        o = frames[name]
        tris = read_stl(os.path.join(raw, link['stl']))
        moved = []
        for t in tris:
            v = list(t[:3])
            for k in range(3):
                v += [t[3 + 3 * k + i] * scale - o[i] for i in range(3)]
            moved.append(v)
        write_stl(os.path.join(out, name + '.stl'), moved)
    return scale


# ---------------- URDF writing ----------------
def link_kind(name):
    return 'base_link' if name == 'base_link' else name.split('_', 1)[1]


def geometry_xml(shape, ind):
    if shape['type'] == 'box':
        g = '<box size="%s"/>' % vec(shape['size'])
    elif shape['type'] == 'cylinder':
        g = '<cylinder radius="%s" length="%s"/>' % (fmt(shape['radius']), fmt(shape['length']))
    else:
        g = '<sphere radius="%s"/>' % fmt(shape['radius'])
    return ind + '<geometry>' + g + '</geometry>'


def link_xml(name, link, frame, collisions, mesh_uri, with_material):
    t = com_inertia(name, link)
    com = [link['com_world_cm'][i] * 0.01 - frame[i] for i in range(3)]
    kind = link_kind(name)
    lines = ['  <link name="%s">' % name,
             '    <inertial>',
             '      <origin xyz="%s" rpy="0 0 0"/>' % vec(com),
             '      <mass value="%s"/>' % fmt(link['mass_kg']),
             '      <inertia ixx="%s" ixy="%s" ixz="%s" iyy="%s" iyz="%s" izz="%s"/>' % tuple(
                 fmt(v) for v in (t[0][0], t[0][1], t[0][2], t[1][1], t[1][2], t[2][2])),
             '    </inertial>',
             '    <visual>',
             '      <origin xyz="0 0 0" rpy="0 0 0"/>',
             '      <geometry><mesh filename="%s"/></geometry>' % mesh_uri(name)]
    if with_material:
        lines.append('      <material name="%s"/>' % COLORS[kind][0])
    lines.append('    </visual>')
    for i, c in enumerate(collisions):
        cname = '%s_foot_collision' % name[:2] if c.get('foot') else '%s_collision_%d' % (name, i)
        lines += ['    <collision name="%s">' % cname,
                  '      <origin xyz="%s" rpy="%s"/>' % (vec(c['xyz']), vec(c['rpy'])),
                  geometry_xml(c, '      '),
                  '    </collision>']
    lines.append('  </link>')
    return '\n'.join(lines)


def joint_xml(j, frames, damping):
    kind = j['name'].split('_')[1]
    p, c = frames[j['parent']], frames[j['child']]
    xyz = [c[i] - p[i] for i in range(3)]
    return '\n'.join([
        '  <joint name="%s" type="revolute">' % j['name'],
        '    <parent link="%s"/>' % j['parent'],
        '    <child link="%s"/>' % j['child'],
        '    <origin xyz="%s" rpy="0 0 0"/>' % vec(xyz),
        '    <axis xyz="%s"/>' % vec(j['axis']),
        '    <limit lower="%s" upper="%s" effort="%s" velocity="%s"/>' % (
            fmt(j['lower']), fmt(j['upper']), fmt(EFFORT[kind]), fmt(VELOCITY[kind])),
        '    <dynamics damping="%s" friction="%s"/>' % (fmt(damping), fmt(FRICTION)),
        '  </joint>'])


def frame_xml(name, parent, xyz, rpy=(0, 0, 0)):
    return '\n'.join([
        '  <link name="%s"/>' % name,
        '  <joint name="%s_joint" type="fixed">' % name.replace('_link', ''),
        '    <parent link="%s"/>' % parent,
        '    <child link="%s"/>' % name,
        '    <origin xyz="%s" rpy="%s"/>' % (vec(xyz), vec(rpy)),
        '  </joint>'])


def body(data, frames, mesh_uri, with_material, damping):
    parts = []
    if with_material:
        parts += ['  <material name="%s"><color rgba="%s"/></material>' % v for v in COLORS.values()]
    for name, link in data['links'].items():
        parts.append(link_xml(name, link, frames[name], data['collisions'][name], mesh_uri, with_material))
    for j in data['joints']:
        parts.append(joint_xml(j, frames, damping))
    return parts


def generate(export_dir, pkg_dir):
    with open(os.path.join(export_dir, 'robot_dog_fusion.json'), encoding='utf-8') as f:
        data = json.load(f)
    # link frames: base_link at the origin, every child at its parent joint (world, m)
    frames = {'base_link': [0.0, 0.0, 0.0]}
    for j in data['joints']:
        frames[j['child']] = [v * 0.01 for v in j['origin_world_cm']]
    scale = convert_meshes(export_dir, pkg_dir, data, frames)

    urdf_dir = os.path.join(pkg_dir, 'urdf')
    os.makedirs(urdf_dir, exist_ok=True)
    header = '<!-- Generated by tools/generate_description.py from the Fusion export. Do not edit by hand. -->'

    # ROS 2: package:// meshes, materials, plus massless frames for sensors and feet
    ros = body(data, frames, lambda n: 'package://%s/meshes/%s.stl' % (PKG, n), True, DAMPING)
    sf = data['sensor_frames_m']
    ros.append('  <!-- Sensor frames: the sensor bodies are part of base_link; these only give ROS frame_ids. -->')
    ros.append(frame_xml('imu_link', 'base_link', sf['imu_link']))
    ros.append(frame_xml('laser_link', 'base_link', sf['laser_link']))
    ros.append(frame_xml('camera_link', 'base_link', sf['camera_link']))
    ros.append(frame_xml('camera_optical_link', 'camera_link', [0, 0, 0], [-math.pi / 2, 0, -math.pi / 2]))
    ros.append('  <!-- Foot frames (contact point at the foot sphere center) for kinematics / state estimation. -->')
    for j in data['joints']:
        if j['name'].endswith('_calf_joint'):
            leg = j['name'][:2]
            ros.append(frame_xml('%s_foot' % leg, j['child'], data['foot_offset_m']))
    # newline='\n' keeps LF endings when generated on Windows (files are used on Ubuntu)
    with open(os.path.join(urdf_dir, 'robot_dog_core.xacro'), 'w', encoding='utf-8', newline='\n') as f:
        f.write('<?xml version="1.0"?>\n%s\n<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n' % header)
        f.write('\n'.join(ros) + '\n</robot>\n')

    # RL: plain URDF, only the 13 physical links, relative mesh paths. Joint damping is 0 so
    # effort control in Isaac Lab/PhysX is not damped by the imported drive; no simulator tags.
    rl = body(data, frames, lambda n: '../meshes/%s.stl' % n, False, 0.0)
    rl_urdf = os.path.join(urdf_dir, 'robot_dog_rl.urdf')
    with open(rl_urdf, 'w', encoding='utf-8', newline='\n') as f:
        f.write('<?xml version="1.0"?>\n%s\n<robot name="robot_dog">\n' % header)
        f.write('\n'.join(rl) + '\n</robot>\n')

    # keep the Isaac Lab project's copy (urdf/ + meshes/, same relative layout) in sync
    lab_pkg = os.path.join(pkg_dir, 'isaaclab', 'robot_dog_lab')
    lab_data = os.path.join(lab_pkg, 'data', 'robot_dog')
    if os.path.isdir(lab_pkg):
        os.makedirs(os.path.join(lab_data, 'urdf'), exist_ok=True)
        shutil.copy2(rl_urdf, os.path.join(lab_data, 'urdf'))
        shutil.copytree(os.path.join(pkg_dir, 'meshes'), os.path.join(lab_data, 'meshes'), dirs_exist_ok=True)

    total = sum(l['mass_kg'] for l in data['links'].values())
    print('STL unit scale %.3g -> meters; %d links, %d joints, total mass %.3f kg'
          % (scale, len(data['links']), len(data['joints']), total))


if __name__ == '__main__':
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--export', default=os.path.join(here, '..', 'fusion'))
    ap.add_argument('--pkg', default=os.path.join(here, '..'))
    args = ap.parse_args()
    generate(os.path.abspath(args.export), os.path.abspath(args.pkg))
