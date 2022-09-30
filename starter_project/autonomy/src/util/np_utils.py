import numpy as np


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm
