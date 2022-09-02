from __future__ import annotations
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
import mrover.msg
import mrover.srv
from util.SE3 import SE3
from visualization_msgs.msg import Marker
from typing import ClassVar, Optional
import numpy as np
from dataclasses import dataclass


@dataclass
class Rover:
    ctx: Context

    def get_pose(self) -> SE3:
        #TODO: return the rovers pose as an SE3 object 
        pass

    def send_drive_command(self, twist: Twist):
        #TODO: publish a twist message to the cmd_vel topic
        pass

    def send_drive_stop(self):
        #TODO: tell the rover to stop
        pass

@dataclass
class Environment:
    """
    Context class to represent the rover's envrionment
    Information such as locations of fiducials or obstacles
    """

    ctx: Context
    NO_FIDUCIAL: ClassVar[int] = -1

    def get_fid_pos(self, fid_id: int) -> Optional[np.ndarray]:
        """
        Retrieves the pose of the given fiducial ID from the TF tree
        if it exists, otherwise returns None
        """
        try:
            #TODO: return the position of the AR tag (hint use the SE3.from_tf_tree function)
            pass
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher

    # Use these as the primary interfaces in states
    rover: Rover
    env: Environment

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher("nav_vis", Marker)
        self.rover = Rover(self)
        self.env = Environment(self)