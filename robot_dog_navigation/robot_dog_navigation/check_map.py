"""Check a saved map against the room layout (spec M4): aligned >= 0.95 and outer wall coverage >= 0.80.

    ros2 run robot_dog_navigation check_map <map.yaml>
"""
import sys

from robot_dog_navigation.map_check import check_map, load_map, occupied_points
from robot_dog_navigation.nav_goals import room_layout

MIN_ALIGNED, MIN_COVERAGE = 0.95, 0.80


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    grid, res, origin = load_map(sys.argv[1])
    pts = occupied_points(grid, res, origin)
    r = check_map(pts, room_layout().obstacles())
    ok = r['aligned'] >= MIN_ALIGNED and r['outer_wall_coverage'] >= MIN_COVERAGE
    print('[MAP] %d occupied cells, aligned %.3f (>= %.2f), outer wall coverage %.3f (>= %.2f): %s' % (
        len(pts), r['aligned'], MIN_ALIGNED, r['outer_wall_coverage'], MIN_COVERAGE, 'PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
