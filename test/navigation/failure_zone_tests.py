import numpy as np
from shapely.geometry import Polygon, LineString, Point
from navigation.failure_zone import FailureZone

def check_failure_zone(fz, start, end, res):
    start = np.array(start)
    end = np.array(end)
    line = LineString((start, end))
    assert(fz.intersects(line) == res)

fz = FailureZone(Polygon([[0, 0], [0, 1], [1, 1], [1, 0]]))
check_failure_zone(fz, [2, 0], [2, 2], False)   # no intersection vertical line
check_failure_zone(fz, [0, 2], [2, 2], False)   # no intersection horizontal line
check_failure_zone(fz, [2, 2], [3, 3], False)   # no intersection diagonal line
check_failure_zone(fz, [0, 0], [-1, -1], False) # no intersection touch at corner
check_failure_zone(fz, [1, -1], [-1, 1], False) # no intersection touch at corner
check_failure_zone(fz, [-1, 1], [1, -1], False) # no intersection touch at corner
check_failure_zone(fz, [0, 0], [1, 0], False)   # no intersection overlapping only on edge
check_failure_zone(fz, [0, 0], [1, 1], True) # intersection go through corners
check_failure_zone(fz, [0.2, 0.2], [0.7, 0.7], True) # intersection contained path
check_failure_zone(fz, [1, 0], [0, 1], True) # intersection go through corners
check_failure_zone(fz, [0.5, 0.2], [0.5, 0.21], True) # intersection contained path
check_failure_zone(fz, [1.2, 1.2], [1.2, 1.2], False) # no intersection degenerate path
check_failure_zone(fz, [-1, 0.5], [0, 0.5], False) # no intersection at T
check_failure_zone(fz, [0.5, -1], [0.5, 0], False) # no intersection at T
check_failure_zone(fz, [3, 2], [-1, 0], True) # normal intersection

fz = FailureZone(Polygon([[0, 1], [1, 2], [2, 1], [1, 0]])) # rotated fz 
check_failure_zone(fz, [3, 0], [3, 3], False)   # no intersection vertical line
check_failure_zone(fz, [0, 3], [3, 3], False)   # no intersection horizontal line
check_failure_zone(fz, [2, 2], [3, 3], False)   # no intersection diagonal line
check_failure_zone(fz, [0, 0], [0, 1], False) # no intersection touch at corner
check_failure_zone(fz, [0, 0], [2, 0], False) # no intersection touch at corner
check_failure_zone(fz, [2, 0], [0, 0], False) # no intersection touch at corner
check_failure_zone(fz, [0, 1], [2, 1], True) # intersection go through corners
check_failure_zone(fz, [1, 1], [1.1, 1.1], True) # intersection contained path
check_failure_zone(fz, [0.6, 0.6], [0.6, 0.61], True) # intersection go through corners
check_failure_zone(fz, [0.6, 0.6], [0.6, 0.6], False) # no intersection degenerate path
check_failure_zone(fz, [0, 0], [0.5, 0.5], False) # no intersection at T
check_failure_zone(fz, [0.5, 0], [0.5, 0.5], False) # no intersection at T
check_failure_zone(fz, [3, 2], [-1, 0], True) # normal intersection

print("FailureZone: All tests successfully passed!")





