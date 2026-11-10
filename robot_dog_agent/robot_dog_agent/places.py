"""Named places in the apartment, relative-move geometry and the target check (spec 2026-11-10, 4). Plain Python.

Coordinates are the map frame = world frame = apartment coordinates - (3.5, 1.8): the dog spawns at the origin
facing +x, with +y to its left.
"""
import math
from collections import namedtuple

from robot_dog_navigation.map_check import to_cell
from robot_dog_navigation.nav_goals import clearance, load_truth, reachable

Place = namedtuple('Place', 'id name x y yaw')  # yaw in rad

PLACES = (
    Place('start', '起點（客廳中央，沙發和電視櫃之間）', 0.0, 0.0, 0.0),
    Place('living_room', '客廳西側', -2.5, -0.3, math.pi),
    Place('master_bedroom', '主臥室', -1.5, 3.0, math.pi / 2),
    Place('second_bedroom', '次臥室', 3.0, 2.2, 0.0),
    Place('dining_table', '餐桌旁（餐桌東側，面向餐桌）', 3.6, 0.3, math.pi),
    Place('kitchen', '廚房（流理台前，面向流理台）', 2.5, -0.6, -math.pi / 2),
    Place('entrance', '玄關（面向大門）', -2.0, -0.9, -math.pi / 2),
    Place('bathroom_door', '浴室門口（門外，面向浴室）', 3.6, -0.5, 0.0),
)
MIN_CLEARANCE = 0.30         # m; half the body's diagonal is 0.27 m
APARTMENT_OFFSET = (3.5, 1.8)  # apartment coordinates = world + this (robot_dog_sim.apartment.SPAWN_XY)


def load_body_truth():
    """The truth floor plan of everything that blocks the dog's body: (grid, resolution, origin)."""
    return load_truth('apartment_body')


def find_place(place_id):
    for place in PLACES:
        if place.id == place_id:
            return place
    return None


def wrap(angle):
    """Angle in (-pi, pi]."""
    return math.pi - (math.pi - angle) % (2.0 * math.pi)


def relative_target(pose, forward=0.0, left=0.0, turn_left_deg=0.0):
    """(x, y, yaw) reached from pose = (x, y, yaw) by moving forward/left in the body frame and turning left."""
    x, y, yaw = pose
    return (x + forward * math.cos(yaw) - left * math.sin(yaw),
            y + forward * math.sin(yaw) + left * math.cos(yaw),
            wrap(yaw + math.radians(turn_left_deg)))


def check_target(truth, x, y):
    """None when the dog can stand at (x, y), else the reason why not."""
    grid, res, origin = truth
    r, c = to_cell(res, origin, x, y)
    if not (0 <= r < grid.shape[0] and 0 <= c < grid.shape[1]):
        return 'outside the map'
    if grid[r, c] == 100:
        return 'on an obstacle'
    if not reachable(truth, x, y):
        return 'not connected to the free space the robot is in (inside furniture or a closed-off area)'
    gap = clearance(truth, x, y)
    if gap < MIN_CLEARANCE - 1e-9:
        return 'the target point is only %.2f m from an obstacle; the robot needs %.2f m of room' % (gap, MIN_CLEARANCE)
    return None


def area_of(x, y):
    """The part of the apartment (x, y) is in."""
    ax, ay = x + APARTMENT_OFFSET[0], y + APARTMENT_OFFSET[1]
    if ax >= 7.8 and ay <= 2.1:
        return 'bathroom'
    if ay < 3.0:
        return 'living, dining and kitchen area'
    return 'master bedroom' if ax < 5.0 else 'second bedroom'


def nearest_place(x, y):
    """(place, distance in m) of the named place closest to (x, y)."""
    place = min(PLACES, key=lambda p: math.hypot(p.x - x, p.y - y))
    return place, math.hypot(place.x - x, place.y - y)
