from np_utils import intersect_2d 
import numpy as np

"""
Unit tests for intersect_2d
Implicitly check orientation_2d
"""
def check_intersect_2d(s1, e1, s2, e2, res):
    s1 = np.array(s1)
    e1 = np.array(e1)
    s2 = np.array(s2)
    e2 = np.array(e2)
    assert(intersect_2d(s1, e1, s2, e2) == res)

check_intersect_2d([-1, 0], [1, 0], [0, -1], [0, 1], True)   # generic, right angle 
check_intersect_2d([-1, -1], [1, 1], [-1, 1], [1, -1], True) # diagonal @ right angle
check_intersect_2d([1, 1], [-1, -1], [-1, 1], [1, -1], True) # switch start and end
check_intersect_2d([5, 5], [10, 10], [11, 10], [5, 6], True) # skew intersect acute
check_intersect_2d([5, 5], [9, 9], [0, 0], [4, 4], False) # non-intersecting clean
check_intersect_2d([0, 0], [2, 1], [2, 1], [4, 4], False) # intersect only at endpoint
check_intersect_2d([0, 0], [2, 2], [1, 1], [0, 2], False) # intersect only at T
check_intersect_2d([2, -2], [-2, 2], [1, 1], [0, 0], False) # intersect only at T
check_intersect_2d([2, -2], [-2, 2], [1, 0], [0, 0], False) # intersect only at T
check_intersect_2d([0, 0], [2, 2], [1, 1], [1.5, 1.5], True) # collinear full overlap
check_intersect_2d([0, 0], [2, 2], [1, 1], [3, 3], True) # collinear half overlap
check_intersect_2d([0, 0], [2, 2], [3, 3], [-5, -5], True) # collinear full overlap
check_intersect_2d([0, 0], [1, 1], [1, 1], [2, 2], False) # collinear only at endpoint
check_intersect_2d([0, 0], [5, 5], [-5, -5], [2, 2], True) # collinear overlap

print("All tests successfully passed!")



