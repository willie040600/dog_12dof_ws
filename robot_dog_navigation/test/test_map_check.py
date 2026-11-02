import numpy as np

from robot_dog_navigation.map_check import check_map, load_map, occupied_points
from robot_dog_navigation.nav_goals import room_layout

ROOM = room_layout()
RES = 0.05


def _synthetic_points(keep=lambda name: True):
    xs = np.arange(-5.5, 5.5, RES) + RES / 2
    pts = []
    for name, box in ROOM.obstacles():
        if not keep(name):
            continue
        for x in xs:
            for y in xs:
                if ROOM.box_distance(x, y, box) <= RES / 2:
                    pts.append((x, y))
    return np.array(pts)


def test_true_layout_passes():
    r = check_map(_synthetic_points(), ROOM.obstacles())
    assert r['aligned'] > 0.99 and r['outer_wall_coverage'] > 0.95


def test_shifted_map_is_not_aligned():
    assert check_map(_synthetic_points() + [0.3, 0.3], ROOM.obstacles())['aligned'] < 0.5


def test_missing_outer_walls_lower_coverage():
    r = check_map(_synthetic_points(lambda n: n not in ('outer_north', 'outer_east')), ROOM.obstacles())
    assert 0.4 < r['outer_wall_coverage'] < 0.6


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
