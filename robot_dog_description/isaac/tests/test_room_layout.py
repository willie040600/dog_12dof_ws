from robot_dog_sim.room_layout import BOXES, CYLINDERS, clearance, obstacles, zone


def test_spawn_is_clear():
    assert clearance(0.0, 0.0) >= 1.0


def test_inside_a_box_is_zero():
    (x, y), _ = BOXES[0]
    assert clearance(x, y) == 0.0


def test_zones():
    assert (zone(-3.0), zone(0.0), zone(3.0)) == (0, 1, 2)


def test_obstacle_count():
    names = [n for n, _ in obstacles()]
    assert sum(n.startswith('outer') for n in names) == 4
    assert sum(n.startswith('inner') for n in names) == 4
    assert sum(n.startswith('box') for n in names) == len(BOXES) == 4
    assert sum(n.startswith('cylinder') for n in names) == len(CYLINDERS) == 3
