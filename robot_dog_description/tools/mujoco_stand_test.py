#!/usr/bin/env python3
"""Physics sanity check of urdf/robot_dog_rl.urdf in MuJoCo (pip install mujoco).

Drops the dog onto a floor with a PD controller holding the standing pose and
reports base height / tilt, then checks the joint sign convention.
Usage: python mujoco_stand_test.py [--seconds 4] [--image out.png]
"""
import argparse
import math
import os

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, '..', 'urdf', 'robot_dog_rl.urdf')
LEGS = ('FL', 'FR', 'RL', 'RR')
STAND = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}
KP, KD, TAU_MAX = 20.0, 0.5, 25.0


def load_model():
    # the URDF carries no simulator tags, so point MuJoCo at the meshes here
    with open(URDF, encoding='utf-8') as f:
        xml = f.read()
    meshdir = os.path.abspath(os.path.join(os.path.dirname(URDF), '..', 'meshes')).replace(os.sep, '/')
    xml = xml.replace('<robot name="robot_dog">', '<robot name="robot_dog"><mujoco><compiler meshdir="%s" '
                      'discardvisual="false"/></mujoco>' % meshdir, 1)
    spec = mujoco.MjSpec.from_string(xml)
    spec.body('base_link').add_freejoint()
    spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[5, 5, 0.1], friction=[1.0, 0.005, 0.0001])
    spec.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])
    spec.option.timestep = 0.001
    model = spec.compile()
    # URDF visuals become mesh geoms; keep them out of contact (collision = primitives only)
    for g in range(model.ngeom):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            model.geom_contype[g] = model.geom_conaffinity[g] = 0
    return model


def joint_ids(model):
    names = ['%s_%s_joint' % (leg, part) for leg in LEGS for part in ('hip', 'thigh', 'calf')]
    ids = [model.joint(n).id for n in names]
    return names, [model.jnt_qposadr[i] for i in ids], [model.jnt_dofadr[i] for i in ids]


def stand_test(model, seconds):
    data = mujoco.MjData(model)
    names, qadr, dadr = joint_ids(model)
    target = np.array([STAND[n.split('_')[1]] for n in names])
    data.qpos[2] = 0.32
    data.qpos[qadr] = target
    mujoco.mj_forward(model, data)
    for _ in range(int(seconds / model.opt.timestep)):
        tau = KP * (target - data.qpos[qadr]) - KD * data.qvel[dadr]
        data.qfrc_applied[dadr] = np.clip(tau, -TAU_MAX, TAU_MAX)
        mujoco.mj_step(model, data)
    w, x, y, z = data.qpos[3:7]
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
    feet = 0
    for i in range(data.ncon):
        c = data.contact[i]
        for g in (c.geom1, c.geom2):
            if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                feet += 1
    tau_now = KP * (target - data.qpos[qadr]) - KD * data.qvel[dadr]
    return data, {
        'base_z': data.qpos[2], 'roll_deg': roll, 'pitch_deg': pitch,
        'xy_drift': float(np.hypot(*data.qpos[:2])), 'foot_contacts': feet,
        'max_joint_err_rad': float(np.max(np.abs(target - data.qpos[qadr]))),
        'max_hold_torque': float(np.max(np.abs(tau_now))),
    }


def sign_test(model):
    """Positive thigh -> foot swings back (-x); negative calf -> foot swings forward;
    positive hip on a left leg -> foot moves outward (+y)."""
    data = mujoco.MjData(model)
    names, qadr, _ = joint_ids(model)
    foot = lambda: data.xpos[model.body('FL_calf').id] + data.xmat[model.body('FL_calf').id].reshape(3, 3) @ [0, 0, -0.16]
    mujoco.mj_kinematics(model, data)
    p0 = foot().copy()
    res = {}
    for joint, q, axis, sign in (('FL_thigh_joint', 0.3, 0, -1), ('FL_calf_joint', -0.3, 0, 1), ('FL_hip_joint', 0.3, 1, 1)):
        data.qpos[:] = model.qpos0
        data.qpos[qadr[names.index(joint)]] = q
        mujoco.mj_kinematics(model, data)
        res[joint] = bool((foot() - p0)[axis] * sign > 0)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=4.0)
    ap.add_argument('--image', help='save a rendered PNG of the final pose')
    args = ap.parse_args()
    model = load_model()
    print('bodies=%d joints=%d total mass=%.3f kg' % (model.nbody, model.njnt, sum(model.body_mass)))
    data, r = stand_test(model, args.seconds)
    print('after %.1fs: base_z=%.3f m roll=%.2f deg pitch=%.2f deg drift=%.3f m foot contacts=%d '
          'max joint err=%.3f rad max hold torque=%.2f N*m'
          % (args.seconds, r['base_z'], r['roll_deg'], r['pitch_deg'], r['xy_drift'], r['foot_contacts'],
             r['max_joint_err_rad'], r['max_hold_torque']))
    print('joint sign convention:', sign_test(model))
    ok = 0.22 < r['base_z'] < 0.30 and abs(r['roll_deg']) < 5 and abs(r['pitch_deg']) < 5 and r['foot_contacts'] >= 4
    print('STAND TEST', 'PASS' if ok else 'FAIL')
    if args.image:
        renderer = mujoco.Renderer(model, 480, 640)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [data.qpos[0], data.qpos[1], 0.15]
        cam.distance, cam.azimuth, cam.elevation = 1.2, 135, -20
        renderer.update_scene(data, cam)
        import PIL.Image  # optional dependency, only for --image
        PIL.Image.fromarray(renderer.render()).save(args.image)
        print('saved', args.image)


if __name__ == '__main__':
    main()
