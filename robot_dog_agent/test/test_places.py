import itertools
import math

from robot_dog_agent import places
from robot_dog_navigation.nav_goals import clearance, reachable

TRUTH = places.load_body_truth()


def test_place_table():
    assert [p.id for p in places.PLACES] == ['start', 'living_room', 'master_bedroom', 'second_bedroom',
                                             'dining_table', 'kitchen', 'entrance', 'bathroom_door']
    k = places.find_place('kitchen')
    assert (k.x, k.y) == (2.5, -0.6) and abs(k.yaw + math.pi / 2) < 1e-9 and places.find_place('fridge') is None


def test_places_are_clear_reachable_and_apart():
    for p in places.PLACES:
        assert clearance(TRUTH, p.x, p.y) >= 0.45 and reachable(TRUTH, p.x, p.y), p.id
        assert places.check_target(TRUTH, p.x, p.y) is None, p.id
    assert min(math.hypot(a.x - b.x, a.y - b.y) for a, b in itertools.combinations(places.PLACES, 2)) >= 0.7


def test_relative_target():
    x, y, yaw = places.relative_target((1.0, 2.0, math.pi / 2), forward=1.0)
    assert abs(x - 1.0) < 1e-9 and abs(y - 3.0) < 1e-9 and abs(yaw - math.pi / 2) < 1e-9
    x, y, yaw = places.relative_target((0.0, 0.0, 0.0), left=0.5, turn_left_deg=-90)
    assert abs(x) < 1e-9 and abs(y - 0.5) < 1e-9 and abs(yaw + math.pi / 2) < 1e-9
    assert abs(places.relative_target((0, 0, math.radians(170)), turn_left_deg=30)[2] - math.radians(-160)) < 1e-9


def test_check_target_reasons():
    assert places.check_target(TRUTH, 0.0, 0.0) is None
    assert places.check_target(TRUTH, 20.0, 0.0) == 'outside the map'
    assert places.check_target(TRUTH, -3.45, 0.0) == 'on an obstacle'           # the west wall
    assert places.check_target(TRUTH, 0.4, 2.7).startswith('not connected')     # the middle of the bed
    assert places.check_target(TRUTH, 0.0, -0.55) == \
        'the target point is only 0.25 m from an obstacle; the robot needs 0.30 m of room'   # by the sofa


def test_area_and_nearest():
    assert places.area_of(0.0, 0.0) == 'living, dining and kitchen area'
    assert places.area_of(-1.5, 3.0) == 'master bedroom' and places.area_of(3.0, 2.2) == 'second bedroom'
    assert places.area_of(5.0, -1.0) == 'bathroom'
    place, dist = places.nearest_place(2.4, -0.6)
    assert place.id == 'kitchen' and abs(dist - 0.1) < 1e-9
