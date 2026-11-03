import os

from robot_dog_sim import apartment


def test_usd_and_constants():
    assert os.path.isfile(apartment.USD) and apartment.USD.endswith('worlds/MUJI_Apartment_3D.usd')
    assert apartment.SPAWN_XY == (3.5, 1.8) and apartment.LIDAR_HEIGHT == 0.31
    assert apartment.BODY_BAND == (0.03, 0.35) and apartment.HIDDEN[0] == 'Ceiling_Group'


def test_texture_folder_complete():
    # the USD's own references are checked with pxr when it is loaded (apartment.missing_textures)
    base = os.path.join(os.path.dirname(apartment.USD), 'textures')
    files = [f for _, _, fs in os.walk(base) for f in fs if f.endswith('.jpg')]
    assert len(files) == 27


def test_wants_collision():
    z = (0.0, 0.8)
    assert apartment.wants_collision('/World/scene/Walls_Group/Wall_01/Wall_01', True, z)
    assert not apartment.wants_collision('/World/scene/Ceiling_Group/Ceiling/Ceiling', True, z)
    assert not apartment.wants_collision('/World/scene/Decor_Group/Lamp/Lamp', False, z)
    assert apartment.wants_collision('/World/scene/Furniture_Group/Ceiling_Group_lookalike/Mesh', True, z)


def test_flat_decor_on_the_floor_does_not_collide():
    # the 8 mm living-room rug: its edge is a step the policy (trained on flat ground) cannot climb
    assert not apartment.wants_collision('/World/scene/Decor_Group/Rug_Living/Rug_Living', True, (0.0, 0.008))
    assert apartment.wants_collision('/World/scene/Floors_Group/Floor_LDK/Floor_LDK', True, (-0.1, 0.0))
    assert apartment.wants_collision('/World/scene/Bath_Group/ShowerTray_Body/ShowerTray_Body', True, (0.0, 0.05))


def test_door_leaves_and_handles_are_removed():
    # the four door leaves stood open into the rooms and hid walls from the LiDAR; the frames stay
    details = '/World/scene/asset/Details_Group/%s/%s'
    for i in range(1, 5):
        leaf, handle = 'Leaf_Door_0%d' % i, 'Handle_Door_0%d' % i
        assert not apartment.wants_collision(details % (leaf, leaf), True, (0.0, 2.1))
        assert not apartment.wants_collision(details % (handle, handle), True, (0.98, 1.02))
    casing = 'Casing_Door_03_A_Left'
    assert apartment.wants_collision(details % (casing, casing), True, (0.0, 2.1))
    assert apartment.wants_collision('/World/scene/asset/Furniture_Group/Wardrobe_Door_1/Wardrobe_Door_1', True,
                                     (0.0, 2.0))


def test_gui_start_view():
    # picked in the GUI (above the living room, looking north over the whole apartment), world coordinates
    assert apartment.VIEW_EYE == (1.418, -3.166, 9.121) and apartment.VIEW_TARGET == (1.503, 1.738, 0.0)
