from gettext import translation
import numpy as np
from geometry_msgs.msg import Pose, Transform
from tf.transformations import quaternion_matrix
from ros_numpy import numpify
from mrover.util.tf_utils import vector3_to_point, point_to_vector3
from __future__ import annotations

# TODO: add ros_numpy as a dependency
class SE3(Pose):
    
    def __init__(self):
        ...
        
    @classmethod
    def from_tf(cls, tf: Transform) -> SE3:
        """
        Create an SE3 pose object from a Transform message.
        The translation of the Transform will become the position of the Pose,
        and the rotation of the Transform will become the orientation of the Pose.
        
        Args:
            tf: the Transform message object
        
        Returns:
            the created SE3 pose object
        """
        point = vector3_to_point(tf.translation)
        orientation = tf.rotation
        return SE3(point=point, orientation=orientation)
        
    def to_tf(self) -> Transform:
        translation = point_to_vector3(self.position)
        return Transform(translation=translation, orientation=self.orientation)
    
    def xy_direction(self) -> np.ndarray:
        rotation_matrix = quaternion_matrix(self.quaternion_array())
        
        # Extract what the x-axis (forward) is with respect to our rover rotation
        return rotation_matrix[0:3, 0]
        
    def position(self) -> np.ndarray:
        ...

    def distance_to(self, p: SE3) -> float:
        ...

    def quaternion_array(self) -> np.ndarray:
        return numpify(self.orientation)