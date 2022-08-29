import numpy as np


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm

def perpendicular_2d(v):
    if v.shape != (2,) and v.shape != (1,2) and v.shape != (2, 1):
        print("vector must be 2D!!!")
        return v
    if v.shape == (2,):
        return np.array([-v[0], -v[1]])
    elif v.shape == (2,1):
        return np.array([ [-v.flatten()[1]], [-v.flatten()[0]] ])
    else:
        return np.array([ [-v.flatten()[1], -v.flatten()[0]] ])