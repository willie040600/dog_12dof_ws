"""Compare a map_saver map (.yaml + .pgm) with the known room layout (spec M4).

The dog starts at the world origin, so slam_toolbox's map frame is the world frame: occupied cells should
lie on the walls and obstacles of robot_dog_sim.room_layout.
"""
import os

import numpy as np
import yaml


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


def _distance_to_boxes(points, boxes):
    """Distance of each point to the nearest axis-aligned box (0 inside)."""
    b = np.asarray(boxes, dtype=float)  # (B, 4) xmin ymin xmax ymax
    px, py = points[:, :1], points[:, 1:2]
    dx = np.maximum(np.maximum(b[:, 0] - px, 0.0), px - b[:, 2])
    dy = np.maximum(np.maximum(b[:, 1] - py, 0.0), py - b[:, 3])
    return np.hypot(dx, dy).min(axis=1)


def check_map(points, obstacles, tol=0.10, step=0.1, near=0.15):
    """{'aligned': share of occupied points within tol of an obstacle,
        'outer_wall_coverage': share of the outer walls' inner faces with an occupied point within `near`}."""
    boxes = [b for _, b in obstacles]
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    aligned = float((_distance_to_boxes(points, boxes) <= tol).mean()) if len(points) else 0.0
    samples = []
    for name, (xmin, ymin, xmax, ymax) in obstacles:
        if name == 'outer_north':
            samples += [(x, ymin) for x in np.arange(xmin + 0.1, xmax - 0.1, step)]
        elif name == 'outer_south':
            samples += [(x, ymax) for x in np.arange(xmin + 0.1, xmax - 0.1, step)]
        elif name == 'outer_east':
            samples += [(xmin, y) for y in np.arange(ymin + 0.1, ymax - 0.1, step)]
        elif name == 'outer_west':
            samples += [(xmax, y) for y in np.arange(ymin + 0.1, ymax - 0.1, step)]
    samples = np.array(samples)
    if not len(points):
        return {'aligned': 0.0, 'outer_wall_coverage': 0.0}
    d = np.hypot(samples[:, None, 0] - points[None, :, 0], samples[:, None, 1] - points[None, :, 1]).min(axis=1)
    return {'aligned': aligned, 'outer_wall_coverage': float((d <= near).mean())}
