"""Horizontal slices of a triangle mesh as occupancy grids, saved like map_saver maps (spec 2026-11-03, 4.1).

Plain numpy: tools/make_floorplan.py reads the USD with pxr and hands the triangles in.
Grids are [row, col] with row 0 at the smallest y.
"""
import os

import numpy as np


def _shape(bounds, res):
    xmin, ymin, xmax, ymax = bounds
    return int(round((ymax - ymin) / res)), int(round((xmax - xmin) / res))


def slice_grid(tris, z, bounds, res=0.05):
    """Cells crossed by the intersection of the triangles (N, 3, 3) with the plane at height z."""
    rows, cols = _shape(bounds, res)
    grid = np.zeros((rows, cols), bool)
    tris = np.asarray(tris, dtype=float).reshape(-1, 3, 3)
    zs = tris[:, :, 2] - z
    tris = tris[(zs.min(axis=1) < 0) & (zs.max(axis=1) > 0)]
    if not len(tris):
        return grid
    zs = tris[:, :, 2] - z
    points = []  # the two crossing points of every edge pair that straddles the plane
    for a, b in ((0, 1), (1, 2), (2, 0)):
        za, zb = zs[:, a], zs[:, b]
        cross = (za < 0) != (zb < 0)
        t = np.where(cross, za / np.where(cross, za - zb, 1.0), 0.0)
        p = tris[:, a, :2] + t[:, None] * (tris[:, b, :2] - tris[:, a, :2])
        points.append(np.where(cross[:, None], p, np.nan))
    p = np.stack(points, axis=1)  # (N, 3, 2): exactly two of the three are not nan
    order = np.argsort(np.isnan(p[:, :, 0]), axis=1, kind='stable')
    p0 = np.take_along_axis(p, order[:, :1, None], axis=1)[:, 0]
    p1 = np.take_along_axis(p, order[:, 1:2, None], axis=1)[:, 0]
    steps = int(np.ceil(np.nanmax(np.linalg.norm(p1 - p0, axis=1)) / (res / 2))) + 1
    s = np.linspace(0.0, 1.0, steps)[None, :, None]
    samples = (p0[:, None] + s * (p1 - p0)[:, None]).reshape(-1, 2)
    c = np.floor((samples[:, 0] - bounds[0]) / res).astype(int)
    r = np.floor((samples[:, 1] - bounds[1]) / res).astype(int)
    keep = (r >= 0) & (r < rows) & (c >= 0) & (c < cols)
    grid[r[keep], c[keep]] = True
    return grid


def band_grid(tris, z0, z1, bounds, step=0.04, res=0.05):
    """Union of slices from z0 to z1 (inclusive) every `step`."""
    grid = np.zeros(_shape(bounds, res), bool)
    for z in np.arange(z0, z1 + 1e-9, step):
        grid |= slice_grid(tris, z, bounds, res)
    return grid


def save_map(grid, res, origin_xy, path_no_ext):
    """Write <path>.pgm (occupied 0, free 254, top row = largest y) and <path>.yaml in map_saver's format."""
    rows, cols = grid.shape
    pixels = np.where(grid[::-1], 0, 254).astype(np.uint8)
    with open(path_no_ext + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (cols, rows))
        f.write(pixels.tobytes())
    with open(path_no_ext + '.yaml', 'w') as f:
        f.write('image: %s.pgm\nmode: trinary\nresolution: %s\norigin: [%s, %s, 0.0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.196\n'
                % (os.path.basename(path_no_ext), res, float(origin_xy[0]), float(origin_xy[1])))
