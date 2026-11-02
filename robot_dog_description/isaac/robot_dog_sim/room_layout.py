"""The navigation room (spec 5.5) as plain data: world.py builds it, nav_goals and map_check read it."""
import math

ROOM_HALF = 5.0          # 10 x 10 m
WALL_HEIGHT, WALL_THICKNESS = 1.0, 0.1
INNER_WALLS = ((-1.7, (2.5, 3.5)), (1.7, (-3.5, -2.5)))   # (x, doorway y range): three zones, 1 m doorways
BOXES = (((-3.5, -3.0), 0.5), ((-3.3, 1.0), 0.4), ((3.2, 2.5), 0.6), ((3.5, -1.0), 0.45))   # ((x, y), size)
CYLINDERS = ((0.0, 3.0), (1.0, -3.0), (-1.0, -2.5))   # (x, y)
CYLINDER_RADIUS, CYLINDER_HEIGHT = 0.15, 0.8


def obstacles():
    """[(name, (xmin, ymin, xmax, ymax))] of every wall segment, box and (bounding square of a) cylinder."""
    r, t = ROOM_HALF, WALL_THICKNESS
    out = [('outer_north', (-r - t / 2, r - t / 2, r + t / 2, r + t / 2)),
           ('outer_south', (-r - t / 2, -r - t / 2, r + t / 2, -r + t / 2)),
           ('outer_east', (r - t / 2, -r, r + t / 2, r)),
           ('outer_west', (-r - t / 2, -r, -r + t / 2, r))]
    for i, (x, (g0, g1)) in enumerate(INNER_WALLS):
        for j, (y0, y1) in enumerate(((-r, g0), (g1, r))):
            out.append(('inner_%d_%d' % (i, j), (x - t / 2, y0, x + t / 2, y1)))
    for i, ((x, y), s) in enumerate(BOXES):
        out.append(('box_%d' % i, (x - s / 2, y - s / 2, x + s / 2, y + s / 2)))
    for i, (x, y) in enumerate(CYLINDERS):
        c = CYLINDER_RADIUS
        out.append(('cylinder_%d' % i, (x - c, y - c, x + c, y + c)))
    return out


def box_distance(x, y, box):
    xmin, ymin, xmax, ymax = box
    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)
    return math.hypot(dx, dy)


def clearance(x, y):
    """Distance from (x, y) to the nearest obstacle outline (0 inside one)."""
    return min(box_distance(x, y, b) for _, b in obstacles())


def zone(x):
    """0 west of the first inner wall, 1 between them, 2 east of the second."""
    return 0 if x < INNER_WALLS[0][0] else (1 if x < INNER_WALLS[1][0] else 2)
