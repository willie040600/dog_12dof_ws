"""Compare a map_saver map (.yaml + .pgm) with the apartment's truth floor plans (spec 2026-11-03, 4.3).

The dog starts at the world origin, so slam_toolbox's map frame is the world frame, the frame the truth maps in
maps/truth/ are written in (robot_dog_description/isaac/tools/make_floorplan.py).
"""
import os

import numpy as np
import yaml
from scipy import ndimage


def _read_pgm(path):
    with open(path, 'rb') as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:  # magic, width, height, maxval (comments start with '#')
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b'#':
            i = data.index(b'\n', i)
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b'P5' or maxval > 255:
        raise ValueError('%s: only 8-bit binary PGM (P5) is supported' % path)
    pixels = np.frombuffer(data[i + 1:i + 1 + w * h], dtype=np.uint8)
    return pixels.reshape(h, w), maxval


def load_map(yaml_path):
    """(grid int8: 100 occupied / 0 free / -1 unknown, row 0 = smallest y; resolution; (origin_x, origin_y))."""
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    image = meta['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    pixels, maxval = _read_pgm(image)
    occ = pixels.astype(float) / maxval if meta.get('negate', 0) else (maxval - pixels.astype(float)) / maxval
    grid = np.full(pixels.shape, -1, dtype=np.int8)
    grid[occ > meta['occupied_thresh']] = 100
    grid[occ < meta['free_thresh']] = 0
    return grid[::-1].copy(), float(meta['resolution']), (float(meta['origin'][0]), float(meta['origin'][1]))


def occupied_points(grid, resolution, origin):
    rows, cols = np.nonzero(grid == 100)
    return np.stack([origin[0] + (cols + 0.5) * resolution, origin[1] + (rows + 0.5) * resolution], axis=1)


def to_cell(res, origin, x, y):
    """(row, col) of world point(s) x, y in a grid with this resolution and origin (row 0 = smallest y)."""
    return (np.floor((np.asarray(y) - origin[1]) / res).astype(int),
            np.floor((np.asarray(x) - origin[0]) / res).astype(int))


def _sample(field, rows, cols, outside):
    """field[rows, cols], with `outside` where the index is off the grid."""
    inside = (rows >= 0) & (rows < field.shape[0]) & (cols >= 0) & (cols < field.shape[1])
    out = np.full(np.shape(rows), outside, dtype=float)
    out[inside] = field[rows[inside], cols[inside]]
    return out


def reachable_free(truth_body):
    """Free cells of the body-band truth map connected (4-neighbour) to the cell of the world origin."""
    grid, res, origin = truth_body
    labels, _ = ndimage.label(grid == 0)
    r, c = to_cell(res, origin, 0.0, 0.0)
    return labels == labels[r, c] if labels[r, c] else np.zeros(grid.shape, bool)


def check_map(points, truth_lidar, truth_body, tol=0.10, near=0.15, visible=0.10):
    """{'aligned': share of map points within tol of an occupied truth cell (LiDAR plane),
        'coverage': share of the visible truth surface with a map point within near}.

    Visible truth surface: occupied LiDAR-plane cells within `visible` of free space the dog can reach from the
    origin (body band), so walls' backs and the inside of furniture do not count.
    """
    grid, res, origin = truth_lidar
    if truth_body[0].shape != grid.shape or truth_body[1] != res or tuple(truth_body[2]) != tuple(origin):
        raise ValueError('the LiDAR and body truth maps must share size, resolution and origin')
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points):
        return {'aligned': 0.0, 'coverage': 0.0}
    occupied = grid == 100
    to_truth = ndimage.distance_transform_edt(~occupied) * res
    r, c = to_cell(res, origin, points[:, 0], points[:, 1])
    aligned = float((_sample(to_truth, r, c, np.inf) <= tol).mean())
    seen = np.zeros(grid.shape, bool)
    inside = (r >= 0) & (r < grid.shape[0]) & (c >= 0) & (c < grid.shape[1])
    seen[r[inside], c[inside]] = True
    to_seen = ndimage.distance_transform_edt(~seen) * res
    to_free = ndimage.distance_transform_edt(~reachable_free(truth_body)) * res
    surface = occupied & (to_free <= visible)
    coverage = float((to_seen[surface] <= near).mean()) if surface.any() else 0.0
    return {'aligned': aligned, 'coverage': coverage}
