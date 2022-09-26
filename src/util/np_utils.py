import numpy as np


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm


def perpendicular_2d(v):
    if v.shape != (2,) and v.shape != (1, 2) and v.shape != (2, 1):
        raise Exception("vector must be 2D!!!")
    orig_shape = v.shape
    return np.reshape((np.array([-v.flatten()[1], -v.flatten()[0]])), orig_shape)
