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


def wants_collision(path, visible):
    """True for a mesh that should collide: visible and not under a HIDDEN group."""
    return visible and not any(name in path.split('/') for name in HIDDEN)


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
