from typing import Union

import numpy as np
from geometry_msgs.msg import Vector3, Quaternion, Point


def _translation_to_numpy(translation: Vector3) -> np.ndarray:
    return np.array([translation.x, translation.y, translation.z])


def _point_to_numpy(point: Point) -> np.ndarray:
    return np.array([point.x, point.y, point.z])


def _rotation_to_numpy(rotation: Quaternion) -> np.ndarray:
    return np.array([rotation.x, rotation.y, rotation.z, rotation.w])


def normalized(v):
    norm = np.linalg.norm(v)
    return v / norm


def numpify(msg: Union[Vector3, Quaternion, Point]) -> np.ndarray:
    if msg.__class__ == Vector3:
        return _translation_to_numpy(msg)
    elif msg.__class__ == Quaternion:
        return _rotation_to_numpy(msg)
    elif msg.__class__ == Point:
        return _point_to_numpy(msg)
    else:
        raise Exception("type of msg must be either Vector3 or Quaternion!!!")
