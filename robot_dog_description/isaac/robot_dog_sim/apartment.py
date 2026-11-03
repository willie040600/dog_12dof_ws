"""The MUJI apartment navigation world (spec 2026-11-03): file, spawn point and slicing heights.
Plain Python, except missing_textures (imports pxr when called).

World coordinates = apartment coordinates - SPAWN_XY: the apartment is loaded shifted so the spawn point is the
world origin, and odom, the SLAM map and AMCL all start there.
"""
import os

USD = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'worlds', 'MUJI_Apartment_3D.usd')
SPAWN_XY = (3.5, 1.8)        # living room, between the sofa and the TV stand (0.70 m clear)
LIDAR_HEIGHT = 0.31          # standing base height 0.226 m + laser_joint 0.085 m
BODY_BAND = (0.03, 0.35)     # heights that block the dog's body
HIDDEN = ('Ceiling_Group',)  # not shown and not collided with (lets the sun in, top view in the GUI)
FLAT_TOP = 0.02              # flat decor lying on the floor (the 8 mm rug) is walked over, not collided with:
                             # its edge is a step the policy, trained on flat ground, cannot climb


def wants_collision(path, visible, z_range):
    """True for a mesh that should collide: visible, not under a HIDDEN group, and not flat decor on the floor.

    z_range: (bottom, top) of the mesh in world z. Floor slabs reach below 0 and still collide.
    """
    if not visible or any(name in path.split('/') for name in HIDDEN):
        return False
    return not (z_range[0] >= -1e-3 and z_range[1] <= FLAT_TOP)


def missing_textures(usd_path):
    """Texture files the USD references that are not on disk (relative paths resolve next to the USD). Needs pxr."""
    from pxr import Sdf, Usd
    stage = Usd.Stage.Open(usd_path)
    missing = set()
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            if attr.GetTypeName() != Sdf.ValueTypeNames.Asset:
                continue
            value = attr.Get()
            if value and value.path and not value.resolvedPath:
                missing.add(value.path)
    return sorted(missing)
