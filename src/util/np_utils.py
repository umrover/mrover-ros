import numpy as np


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm


def perpendicular_2d(v):
    if v.shape != (2,) and v.shape != (1, 2) and v.shape != (2, 1):
        raise Exception("vector must be 2D!!!")
    orig_shape = v.shape
    return np.reshape((np.array([-v.flatten()[1], v.flatten()[0]])), orig_shape)


def angle_to_rotate(v1, v2):
    """
    returns the angle you'd need to rotate normalized(v1) by in order to have a vector that is normalized(v2)
    """
    v1_u, v2_u = normalized(v1), normalized(v2)
    smallest_angle = np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    # Determine the sign of our effort by seeing if we are to the left or to the right of the target
    # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
    perp_alignment = v2_u[0] * -v1_u[1] + v2_u[1] * v1_u[0]
    sign = np.sign(perp_alignment)
    if sign == 0.0:
        sign = 1
    return smallest_angle * sign


def intersect_2d(start1: np.array, end1: np.array, start2: np.array, end2: np.array) -> bool:
    """
    Checks if two 2-dimensional line segments intersect.

    The line segments are considered to have open endpoints;
    as such, they are not considered intersecting if they touch
    only at one of their endpoints. Cases:
        1) They do not touch at all --> False
        2) They touch only at endpoints --> False
        3) They touch at a T --> False
        4) They intersect in the middle of both segments --> True
        5) They are collinear and they overlap --> True
    """

    """
    Algorithm:
    
    We will calculate 4 orientations with orientation_2d:
        o1 = s1, e1, s2 ; o2 = s1, e1, e2
        o3 = s2, e2, s1 ; o4 = s2, e2, e1

    If the segments are collinear, then o1, o2, o3, o4 = 0. 

    If the segments intersect in a non-collinear way, o1 and o2 will 
    have different signs, as will o3 and o4.

    Note that if any of these orientations are 0 (while not all are 0), 
    the segments intersect only at an endpoint and we return False.  
    """

    orient1 = orientation_2d(start1, end1, start2)
    orient2 = orientation_2d(start1, end1, end2)
    orient3 = orientation_2d(start2, end2, start1)
    orient4 = orientation_2d(start2, end2, end1)

    if orient1 == orient2 == 0:  # collinear case
        # ensure x-coordinates of both points are non-decreasing
        if start1[0] > end1[0]:
            temp = start1
            start1 = end1
            end1 = temp
        if start2[0] > end2[0]:
            temp = start2
            start2 = end2
            end2 = temp

        x_cross = (start1[0] <= start2[0] < end1[0]) or (start2[0] <= start1[0] < end2[0])
        y_cross = (start1[1] <= start2[1] < end1[1]) or (start2[1] <= start1[1] < end2[1])
        return x_cross or y_cross  # strict overlap in x OR strict overlap in y

    return orient1 * orient2 < 0 and orient3 * orient4 < 0  # non-collinear case


def orientation_2d(a: np.array, b: np.array, c: np.array) -> float:
    """
    Checks the orientation of three 2-dimensional points a, b, c.
    Is traversing from a to b to c clockwise, counterclockwise, or collinear?

    Returns:
        - 0 if a, b, c are collinear
        - > 0 if they are clockwise-oriented
        - < 0 if they are counter-clockwise oriented
    """

    """
    Algorithm: Note that
                slope(AC) < slope(AB) if ABC is CW
                slope(AC) > slope(AB) if ABC is CCW
                slope(AC) = slope(AB) if ABC is collinear
    """
    ab = b - a
    ac = c - a
    return (ab[1] * ac[0]) - (ac[1] * ab[0])
