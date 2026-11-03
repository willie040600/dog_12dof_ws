import os

from robot_dog_sim import apartment


def test_usd_and_constants():
    assert os.path.isfile(apartment.USD) and apartment.USD.endswith('worlds/MUJI_Apartment_3D.usd')
    assert apartment.SPAWN_XY == (3.5, 1.8) and apartment.LIDAR_HEIGHT == 0.31
    assert apartment.BODY_BAND == (0.03, 0.35) and apartment.HIDDEN == ('Ceiling_Group',)


def test_texture_folder_complete():
    # the USD's own references are checked with pxr when it is loaded (apartment.missing_textures)
    base = os.path.join(os.path.dirname(apartment.USD), 'textures')
    files = [f for _, _, fs in os.walk(base) for f in fs if f.endswith('.jpg')]
    assert len(files) == 27


def test_wants_collision():
    assert apartment.wants_collision('/World/scene/Walls_Group/Wall_01/Wall_01', True)
    assert not apartment.wants_collision('/World/scene/Ceiling_Group/Ceiling/Ceiling', True)
    assert not apartment.wants_collision('/World/scene/Decor_Group/Lamp/Lamp', False)
    assert apartment.wants_collision('/World/scene/Furniture_Group/Ceiling_Group_lookalike/Mesh', True)
