from np_utils import intersect_2d, orientation_2d
import numpy as np
from dataclasses import dataclass
from shapely.geometry import Polygon, LineString

@dataclass
class FailureZone:
    """
    FailureZones are represented as rectangles aligned with the x-axis 
    """
    vertices: Polygon   

    def get_vertices(self):
        """
        """
        bounds = self.vertices.bounds 
    
    def intersects(self, path: LineString):
        return self.vertices.intersects(path) and not self.vertices.touches(path)

# """
# Unit tests for intersect_2d
# Implicitly check orientation_2d

# Delete this file later
# """
# def check_intersect_2d(s1, e1, s2, e2, res):
#     s1 = np.array(s1)
#     e1 = np.array(e1)
#     s2 = np.array(s2)
#     e2 = np.array(e2)
#     assert(intersect_2d(s1, e1, s2, e2) == res)

# check_intersect_2d([-1, 0], [1, 0], [0, -1], [0, 1], True)   # generic, right angle 
# check_intersect_2d([-1, -1], [1, 1], [-1, 1], [1, -1], True) # diagonal @ right angle
# check_intersect_2d([1, 1], [-1, -1], [-1, 1], [1, -1], True) # switch start and end
# check_intersect_2d([5, 5], [10, 10], [11, 10], [5, 6], True) # skew intersect acute
# check_intersect_2d([5, 5], [9, 9], [0, 0], [4, 4], False) # non-intersecting clean
# check_intersect_2d([0, 0], [2, 1], [2, 1], [4, 4], False) # intersect only at endpoint
# check_intersect_2d([0, 0], [2, 2], [1, 1], [0, 2], False) # intersect only at T
# check_intersect_2d([2, -2], [-2, 2], [1, 1], [0, 0], False) # intersect only at T
# check_intersect_2d([2, -2], [-2, 2], [1, 0], [0, 0], False) # intersect only at T
# check_intersect_2d([0, 0], [2, 2], [1, 1], [1.5, 1.5], True) # collinear full overlap
# check_intersect_2d([0, 0], [2, 2], [1, 1], [3, 3], True) # collinear half overlap
# check_intersect_2d([0, 0], [2, 2], [3, 3], [-5, -5], True) # collinear full overlap
# check_intersect_2d([0, 0], [1, 1], [1, 1], [2, 2], False) # collinear only at endpoint
# check_intersect_2d([0, 0], [5, 5], [-5, -5], [2, 2], True) # collinear overlap
# check_intersect_2d([1, 0], [2, 1], [2, 0], [2, 2], False)
# check_intersect_2d([1, 2], [2, 1], [2, 0], [2, 2], False)
# check_intersect_2d([0, 0], [1, 1], [0, 0], [-1, -1], False)
# check_intersect_2d([1, 0], [0, 1], [0, 0], [-1, -1], False)


# print("intersect_2d: All tests successfully passed!")

# """
# Unit tests for FailureZone
# """
# @dataclass
# class FailureZone:
#     """
#     FailureZones are represented as rectangles.
#     self.vertices should be a np.ndarray of shape (4, 2) representing 
#     the 4 2D corners of the failure zone such that v0, v2 are diagonal
#     """
#     vertices: np.ndarray    # shape (4, 2)

#     def intersect(self, path_start: np.array, path_end: np.array) -> bool:
#         """
#         Returns true if the proposed path intersects with this failure zone. 

#         Intersection is defined as going through or overlapping with an edge, 
#         not just touching a corner -- see intersection_2d for details

#         path_start: (x, y) of start
#         path_end: (x, y) of end
#         """
        
#         # degenerate paths
#         if(np.all(np.isclose(path_start, path_end))):
#             return False
        
#         # If the path intersects any of the edges, return True
#         if(intersect_2d(self.vertices[0, :], self.vertices[1, :], path_start, path_end)
#            or intersect_2d(self.vertices[1, :], self.vertices[2, :], path_start, path_end)
#            or intersect_2d(self.vertices[2, :], self.vertices[3, :], path_start, path_end)
#            or intersect_2d(self.vertices[3, :], self.vertices[0, :], path_start, path_end)):
#             return True

#         # If the path goes through exactly 2 corners of the zone return True
#         # Necessary as intersect_2d interprets ends of line segments as open 
#         if(intersect_2d(self.vertices[0, :], self.vertices[2, :], path_start, path_end)
#         or intersect_2d(self.vertices[1, :], self.vertices[3, :], path_start, path_end)):
#             return True

#         # If the path is fully inside the rectangle return True
#         # If the path is fully inside, then the midpoint of the path will 
#         # necessarily be STRICTLY inside the rectangle
#         # This means that the orientation of all sides with the midpoint will be the same
#         # Note that if the point is outside, at least 2 sides will disagree on the 
#         # orientation
        
#         midpoint = (path_start + path_end)/2
#         o1 = orientation_2d(self.vertices[0, :], self.vertices[1, :], midpoint)
#         o2 = orientation_2d(self.vertices[1, :], self.vertices[2, :], midpoint)
#         o3 = orientation_2d(self.vertices[2, :], self.vertices[3, :], midpoint)
#         o4 = orientation_2d(self.vertices[3, :], self.vertices[0, :], midpoint)

#         return max(o1, o2, o3, o4) * min(o1, o2, o3, o4) > 0 # check all 4 have same sign


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





