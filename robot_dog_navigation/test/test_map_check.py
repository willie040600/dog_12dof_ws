import numpy as np

from robot_dog_navigation.map_check import check_map, load_map, occupied_points

RES, ORIGIN = 0.05, (-1.0, -1.0)   # 2 x 2 m, the world origin in the middle


def room(block_filled=True):
    """Synthetic truth: four walls around a 2 x 2 m room and a 0.4 m block east of the origin."""
    g = np.zeros((40, 40), np.int8)
    g[0, :] = g[-1, :] = g[:, 0] = g[:, -1] = 100
    if block_filled:
        g[16:24, 28:36] = 100
    else:
        g[16:24, 28] = g[16:24, 35] = g[16, 28:36] = g[23, 28:36] = 100
    return g, RES, ORIGIN


def points_of(grid):
    return occupied_points(grid, RES, ORIGIN)


def test_truth_itself_passes():
    r = check_map(points_of(room()[0]), room(), room())
    assert r['aligned'] > 0.99 and r['coverage'] > 0.95


def test_shifted_map_is_not_aligned():
    assert check_map(points_of(room()[0]) + [0.3, 0.3], room(), room())['aligned'] < 0.5


def test_missing_wall_lowers_coverage():
    g = room()[0].copy()
    g[-1, :] = 0                                  # the north wall never seen
    assert 0.6 < check_map(points_of(g), room(), room())['coverage'] < 0.9


def test_hidden_surfaces_do_not_count():
    # the map only has the block's outline, the truth has it filled: its inside is not visible from free space
    r = check_map(points_of(room(block_filled=False)[0]), room(), room())
    assert r['coverage'] > 0.95 and r['aligned'] > 0.99


def test_load_map_values_and_row_order(tmp_path):
    # 3 x 3: top row occupied (0 = black), middle free (254), bottom unknown (205); map_saver trinary values
    (tmp_path / 'm.pgm').write_bytes(b'P5\n3 3\n255\n' + bytes([0, 0, 0, 254, 254, 254, 205, 205, 205]))
    (tmp_path / 'm.yaml').write_text('image: m.pgm\nmode: trinary\nresolution: 0.5\norigin: [-1.0, -2.0, 0.0]\n'
                                     'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    grid, res, origin = load_map(str(tmp_path / 'm.yaml'))
    assert res == 0.5 and tuple(origin) == (-1.0, -2.0)
    assert grid[2].tolist() == [100, 100, 100]   # image top row = largest y
    assert grid[1].tolist() == [0, 0, 0]
    assert grid[0].tolist() == [-1, -1, -1]
    pts = occupied_points(grid, res, origin)
    assert np.allclose(sorted(pts[:, 0]), [-0.75, -0.25, 0.25]) and np.allclose(pts[:, 1], -0.75)
