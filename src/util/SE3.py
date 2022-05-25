from gettext import translation
import numpy as np
from geometry_msgs.msg import Pose, Transform
from tf.transformations import quaternion_matrix
from ros_numpy import numpify
from mrover.util.tf_utils import vector3_to_point, point_to_vector3
from __future__ import annotations

class SE3(Pose):
    
    def __init__(self, *args, **kwargs):
        super().__init__(args, kwargs)
        
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
        """
        Create a transform message from an SE3 pose object.
        The opposite of from_tf(), see its docs for details.
        
        Returns:
            the created Transform message object
        """
        translation = point_to_vector3(self.position)
        return Transform(translation=translation, orientation=self.orientation)
    
    def x_vector(self) -> np.ndarray:
        """
        Get the unit direction vector of the SE3 pose's X axis.
        
        Returns:
            unit direction vector [x, y, z]
        """
        rotation_matrix = quaternion_matrix(self.quaternion_array())
        
        # Extract what the x-axis (forward) is with respect to our rover rotation
        return rotation_matrix[0:3, 0]
        
    def distance_to(self, p: SE3) -> float:
        """
        Get the euclidean distance from the position of this SE3 pose 
        to the position of another SE3 pose.
        
        Args:
            p: another SE3 pose object
            
        Returns:
            euclidean distance between the two SE3 poses
        """
        return np.linalg.norm(p.position() - self.position())

    def position(self) -> np.ndarray:
        """
        Get the position vector of the SE3 pose.
        
        Returns:
            a position vector [x, y, z]
        """
        return numpify(self.position)

    def quaternion_array(self) -> np.ndarray:
        """
        Get the quaternion array of the SE3 pose.

        Returns:
            a quaternion array [x, y, z, w]
        """
        return numpify(self.orientation)