import numpy as np

from robot_dog_agent.map_view import FREE, OCCUPIED, UNKNOWN, MapView, grid_to_gray, load_saved_map


def test_grid_to_gray_values_and_flip():
    grid = np.array([[0, 100, -1],       # row 0: smallest y -> last line of the image
                     [64, 65, 0]], dtype=np.int8)
    assert grid_to_gray(grid) == bytes([FREE, OCCUPIED, FREE, FREE, OCCUPIED, UNKNOWN])


VIEW = MapView(0.5, (-1.0, -1.0), 4, 2, 100, 100)   # 2 m x 1 m map in a 100 x 100 widget


def test_view_scales_and_centres():
    assert VIEW.scale == 25.0 and VIEW.left == 0.0 and VIEW.top == 25.0


def test_to_pixel_corners():
    assert VIEW.to_pixel(-1.0, -1.0) == (0.0, 75.0)     # bottom-left of the map
    assert VIEW.to_pixel(1.0, 0.0) == (100.0, 25.0)     # top-right


def test_round_trip():
    x, y = VIEW.to_world(*VIEW.to_pixel(0.3, -0.4))
    assert abs(x - 0.3) < 1e-9 and abs(y + 0.4) < 1e-9
    assert VIEW.to_world(50.0, 50.0) == (0.0, -0.5)


def test_outside_the_image_is_none():
    assert VIEW.to_world(50.0, 10.0) is None and VIEW.to_world(50.0, 90.0) is None
    assert VIEW.to_world(-1.0, 50.0) is None


def test_view_follows_a_new_map_size():
    wide = MapView(0.5, (-2.0, -1.0), 8, 2, 100, 100)   # the map grew to the left (SLAM)
    assert wide.scale == 12.5 and wide.to_world(0.0, 50.0) == (-2.0, -0.5)


def test_load_saved_map():
    grid, res, origin = load_saved_map()
    assert grid.shape == (119, 197) and res == 0.05 and origin == (-3.417, -1.809)
