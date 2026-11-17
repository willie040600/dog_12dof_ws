"""The map for the window: occupancy as grey pixels, and map <-> widget coordinates (spec 2026-11-17, 5).

numpy only: no ROS, no Qt. Grids are [row, col] with row 0 at the smallest y, as in map_check.load_map and in
nav_msgs/OccupancyGrid.
"""
import os

import numpy as np

from robot_dog_navigation.map_check import load_map

FREE, OCCUPIED, UNKNOWN = 245, 40, 160  # grey levels
OCCUPIED_AT = 65                        # occupancy (0-100) from which a cell is drawn as an obstacle (Nav2: 0.65)


def grid_to_gray(grid):
    """rows * cols grey bytes, the first line at the largest y (top of the picture)."""
    grid = np.asarray(grid)
    gray = np.full(grid.shape, FREE, dtype=np.uint8)
    gray[grid >= OCCUPIED_AT] = OCCUPIED
    gray[grid < 0] = UNKNOWN
    return gray[::-1].tobytes()


class MapView:
    """A map of width x height cells drawn as large as fits, centred, in a widget of the given pixel size."""

    def __init__(self, resolution, origin, width, height, widget_width, widget_height):
        self.resolution, self.origin, self.width, self.height = resolution, origin, width, height
        self.scale = min(widget_width / width, widget_height / height)  # pixels per cell
        self.left = (widget_width - width * self.scale) / 2.0
        self.top = (widget_height - height * self.scale) / 2.0

    def to_pixel(self, x, y):
        """Widget pixel of map point (x, y)."""
        col = (x - self.origin[0]) / self.resolution
        row = (y - self.origin[1]) / self.resolution
        return self.left + col * self.scale, self.top + (self.height - row) * self.scale

    def to_world(self, px, py):
        """Map point under widget pixel (px, py), or None outside the picture of the map."""
        col = (px - self.left) / self.scale
        row = self.height - (py - self.top) / self.scale
        if not (0.0 <= col <= self.width and 0.0 <= row <= self.height):
            return None
        return self.origin[0] + col * self.resolution, self.origin[1] + row * self.resolution


def load_saved_map():
    """(grid, resolution, origin) of robot_dog_navigation's maps/apartment.yaml, from the checkout or the share."""
    import robot_dog_navigation
    candidates = [os.path.join(os.path.dirname(os.path.abspath(robot_dog_navigation.__file__)), '..', 'maps')]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(get_package_share_directory('robot_dog_navigation'), 'maps'))
    except Exception:
        pass
    for path in candidates:
        yaml_path = os.path.join(path, 'apartment.yaml')
        if os.path.isfile(yaml_path):
            return load_map(yaml_path)
    raise FileNotFoundError('apartment.yaml not found in %s' % candidates)
