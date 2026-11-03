import os

import numpy as np
import yaml

from robot_dog_sim.floorplan import band_grid, save_map, slice_grid

TRUTH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'robot_dog_navigation', 'maps',
                     'truth')
BOUNDS = (0.0, 0.0, 2.0, 2.0)  # 40 x 40 cells at 0.05 m


def box_tris(xmin, ymin, zmin, xmax, ymax, zmax):
    """12 triangles of an axis-aligned box."""
    c = np.array([[x, y, z] for x in (xmin, xmax) for y in (ymin, ymax) for z in (zmin, zmax)], dtype=float)
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, d, e in faces:
        tris += [[c[a], c[b], c[d]], [c[a], c[d], c[e]]]
    return np.array(tris)


def cell(x, y):
    return int(y / 0.05), int(x / 0.05)


def test_slice_cube_outline():
    g = slice_grid(box_tris(0.5, 0.5, 0.0, 1.5, 1.5, 1.0), 0.3, BOUNDS)
    assert g.shape == (40, 40)
    for x, y in ((0.5, 1.0), (1.5, 1.0), (1.0, 0.5), (1.0, 1.5)):  # the four sides (on a cell edge: the cell above)
        assert g[cell(x, y)], (x, y)
    assert not g[cell(1.0, 1.0)] and not g[cell(0.2, 0.2)]          # inside and outside


def test_slice_above_cube_is_empty():
    assert not slice_grid(box_tris(0.5, 0.5, 0.0, 1.5, 1.5, 1.0), 1.5, BOUNDS).any()


def test_band_includes_low_obstacle():
    plate = box_tris(0.5, 0.5, 0.0, 1.5, 1.5, 0.05)
    assert band_grid(plate, 0.03, 0.35, BOUNDS)[cell(0.5, 1.0)]
    assert not slice_grid(plate, 0.31, BOUNDS).any()


def test_save_map_roundtrip(tmp_path):
    g = np.zeros((4, 6), bool)
    g[0, 1] = g[3, 5] = True                                # row 0 = smallest y
    save_map(g, 0.5, (-1.0, -2.0), str(tmp_path / 'm'))
    meta = yaml.safe_load((tmp_path / 'm.yaml').read_text())
    assert meta['image'] == 'm.pgm' and meta['mode'] == 'trinary' and meta['resolution'] == 0.5
    assert meta['origin'] == [-1.0, -2.0, 0.0]
    data = (tmp_path / 'm.pgm').read_bytes()
    assert data.startswith(b'P5\n6 4\n255\n')
    px = np.frombuffer(data[len(b'P5\n6 4\n255\n'):], np.uint8).reshape(4, 6)[::-1]   # back to row 0 = smallest y
    assert ((px == 0) == g).all() and set(np.unique(px)) == {0, 254}


def test_truth_yaml_origin_is_minus_spawn():
    for name in ('apartment_lidar', 'apartment_body'):
        meta = yaml.safe_load(open(os.path.join(TRUTH, name + '.yaml')))
        assert meta['origin'] == [-3.5, -1.8, 0.0] and meta['resolution'] == 0.05, name
