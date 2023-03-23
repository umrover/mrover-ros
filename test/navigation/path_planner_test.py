from shapely.geometry import LineString, Point, Polygon
from path_planner import PathPlanner
from failure_zone import FailureZone


def check_path_planner(planner: PathPlanner, source, target, expected_targets):
    returned_targets = []

    planner.get_intermediate_target(source, target)
    while not planner.is_path_complete():
        returned_targets.append(planner.get_intermediate_target(source, target))
        planner.complete_intermediate_target()

    print(returned_targets)
    assert returned_targets == expected_targets


def main():
    # Test 1: No failure zones
    planner1 = PathPlanner()
    src = Point(0, 0)
    target = Point(-5, 10)
    expected = [src, target]
    check_path_planner(planner1, src, target, expected)

    # Test 2: 1 Failure Zone in way
    fz1 = FailureZone(Polygon([[1, 1], [1, 2], [2, 2], [2, 1]]))
    planner1.add_failure_zone(fz1)
    src = Point(0, 0)
    target = Point(2, 3)
    expected = [src, Point(1, 2), target]
    check_path_planner(planner1, src, target, expected)

    # Test 3: Failure Zone exists but only corner in the way
    src = Point(0, 0)
    target = Point(4, 2)
    expected = [src, target]
    check_path_planner(planner1, src, target, expected)

    # Test 4: Generic multiple failure zones
    planner2 = PathPlanner()
    src = Point(0, 0)
    target = Point(8, 0)

    fz1 = FailureZone(Polygon([[1, -1], [2, -1], [2, 1], [1, 1]]))
    planner2.add_failure_zone(fz1)

    fz2 = FailureZone(Polygon([[3, 0], [4, 0], [4, 2], [3, 2]]))
    planner2.add_failure_zone(fz2)

    fz3 = FailureZone(Polygon([[5, -0.5], [7, -0.5], [7, 1], [5, 1]]))
    planner2.add_failure_zone(fz3)
    expected = [src, Point(1, -1), Point(2, -1), Point(7, -0.5), target]
    check_path_planner(planner2, src, target, expected)

    # TEST 5: Generic, multiple failure zones
    src = Point(0, 0)
    target = Point(4.5, 1.5)
    expected = [src, Point(1, 1), Point(3, 2), Point(4, 2), target]
    check_path_planner(planner2, src, target, expected)

    # TEST 6: Add failure zone that conflicts with previous edges and plan new path
    fz4 = FailureZone(Polygon([[3, -2], [4, -2], [4, -0.25], [3, -0.25]]))
    planner2.add_failure_zone(fz4)
    src = Point(0, 0)
    target = Point(8, 0)
    expected = [
        src,
        Point(1, -1),
        Point(2, -1),
        Point(3, -0.25),
        Point(4, -0.25),
        Point(5, -0.5),
        Point(7, -0.5),
        target,
    ]
    check_path_planner(planner2, src, target, expected)

    # TEST 7: Add failure zone that overlaps existing failure zone
    fz5 = FailureZone(Polygon([[3, -1], [4, -1], [4, 1], [3, 1]]))
    planner2.add_failure_zone(fz5)
    src = Point(8, 0)
    target = Point(0, 0)
    expected = [src, Point(4, -2), Point(3, -2), Point(1, -1), target]
    check_path_planner(planner2, src, target, expected)

    # TEST 8: src in FZ
    src = Point(1.5, 0)
    target = Point(8, 0)
    expected = [src, target]
    check_path_planner(planner2, src, target, expected)

    # TEST 9: target in FZ
    src = Point(0, 0)
    target = Point(1.5, 0)
    expected = [src, target]
    check_path_planner(planner2, src, target, expected)

    # TEST 10: target blocked off by FZ
    fz6 = FailureZone(Polygon([[2, 0.5], [3, 0.5], [3, 1], [2, 1]]))
    fz7 = FailureZone(Polygon([[2, -0.5], [3, -0.5], [3, -1.5], [2, -1.5]]))
    planner2.add_failure_zone(fz6)
    planner2.add_failure_zone(fz7)
    src = Point(1.5, 0)
    target = Point(0, 0)
    expected = [src, target]
    check_path_planner(planner2, src, target, expected)

    # TEST 11: src blocked off by FZ
    src = Point(0, 0)
    target = Point(1.5, 0)
    expected = [src, target]
    check_path_planner(planner2, src, target, expected)

    print("All path_planner tests passed!")


if __name__ == "__main__":
    main()
