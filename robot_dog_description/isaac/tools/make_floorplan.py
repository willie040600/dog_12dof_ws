"""Truth floor plans of the apartment for map_check and the nav_eval goals (spec 2026-11-03, 4.1).

Slices every mesh that collides in the sim (robot_dog_sim.apartment.wants_collision) at the LiDAR height and
over the body band, in world coordinates (apartment - SPAWN_XY), and writes map_saver-style maps to
robot_dog_navigation/maps/truth/. Needs pxr only; rerun after the apartment USD changes:

    cd ~/IsaacLab && uv run --extra isaacsim python ~/dog_12dof_ws/robot_dog_description/isaac/tools/make_floorplan.py
"""
import os
import sys

import numpy as np
from pxr import Usd, UsdGeom

ISAAC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ISAAC)

from robot_dog_sim import apartment  # noqa: E402
from robot_dog_sim.floorplan import band_grid, save_map, slice_grid  # noqa: E402

OUT = os.path.join(ISAAC, '..', '..', 'robot_dog_navigation', 'maps', 'truth')
RES = 0.05
EXTENT = (0.0, 0.0, 10.0, 6.0)  # the apartment's floor, in its own coordinates (200 x 120 cells)


def world_triangles(stage):
    """(N, 3, 3) triangles of the colliding meshes, in world coordinates with the spawn point at the origin."""
    xc = UsdGeom.XformCache()
    hidden = set()
    tris = []
    for prim in stage.Traverse():
        if prim.GetName() in apartment.HIDDEN:
            hidden.add(prim.GetPath().pathString)
        if prim.GetTypeName() != 'Mesh':
            continue
        path = prim.GetPath().pathString
        visible = UsdGeom.Imageable(prim).ComputeVisibility() != UsdGeom.Tokens.invisible
        if not apartment.wants_collision(path, visible):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = np.array(mesh.GetPointsAttr().Get() or [], dtype=float)
        if not len(points):
            continue
        m = np.array(xc.GetLocalToWorldTransform(prim), dtype=float)   # row-vector convention
        points = (np.c_[points, np.ones(len(points))] @ m)[:, :3]
        points[:, :2] -= apartment.SPAWN_XY
        counts, indices = mesh.GetFaceVertexCountsAttr().Get(), np.array(mesh.GetFaceVertexIndicesAttr().Get())
        k = 0
        for c in counts:                                                # fan-triangulate each polygon
            face = indices[k:k + c]
            tris += [points[[face[0], face[i], face[i + 1]]] for i in range(1, c - 1)]
            k += c
    return np.array(tris), sorted(hidden)


def main():
    stage = Usd.Stage.Open(apartment.USD)
    tris, hidden = world_triangles(stage)
    bounds = (EXTENT[0] - apartment.SPAWN_XY[0], EXTENT[1] - apartment.SPAWN_XY[1],
              EXTENT[2] - apartment.SPAWN_XY[0], EXTENT[3] - apartment.SPAWN_XY[1])
    print('[FLOORPLAN] %d triangles, hidden %s, bounds %s' % (len(tris), hidden, tuple(round(v, 2) for v in bounds)))
    os.makedirs(OUT, exist_ok=True)
    for name, grid in (('apartment_lidar', slice_grid(tris, apartment.LIDAR_HEIGHT, bounds, RES)),
                       ('apartment_body', band_grid(tris, *apartment.BODY_BAND, bounds, res=RES))):
        save_map(grid, RES, bounds[:2], os.path.join(OUT, name))
        print('[FLOORPLAN] %s: %d x %d cells, %d occupied' % (name, grid.shape[1], grid.shape[0], grid.sum()))


if __name__ == '__main__':
    main()
