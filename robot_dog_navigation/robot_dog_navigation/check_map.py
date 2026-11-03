"""Check a saved map against the apartment's truth floor plans (spec 2026-11-03, 4.3): aligned >= 0.95,
coverage >= 0.80.

    ros2 run robot_dog_navigation check_map <map.yaml>
"""
import sys

from robot_dog_navigation.map_check import check_map, load_map, occupied_points
from robot_dog_navigation.nav_goals import load_truth

MIN_ALIGNED, MIN_COVERAGE = 0.95, 0.80


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    grid, res, origin = load_map(sys.argv[1])
    pts = occupied_points(grid, res, origin)
    r = check_map(pts, load_truth('apartment_lidar'), load_truth('apartment_body'))
    ok = r['aligned'] >= MIN_ALIGNED and r['coverage'] >= MIN_COVERAGE
    print('[MAP] %d occupied cells, aligned %.3f (>= %.2f), coverage %.3f (>= %.2f): %s' % (
        len(pts), r['aligned'], MIN_ALIGNED, r['coverage'], MIN_COVERAGE, 'PASS' if ok else 'FAIL'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
