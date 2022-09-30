from __future__ import annotations
from re import S
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import StarterProjectTag
import mrover.srv
from util.SE3 import SE3
from visualization_msgs.msg import Marker
from typing import ClassVar, Optional
import numpy as np
from dataclasses import dataclass


@dataclass
class Rover:
    ctx: Context

    def get_pose(self) -> Optional[SE3]:
        #TODO: return the pose of the rover (or None if we don't have one (catch exception))
        pass

    def send_drive_command(self, twist: Twist):
        #TODO: send the twist message to the rover
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
    fid_pos: Optional[np.ndarray]

    def get_fid_data(self) -> Optional[StarterProjectTag]:
        """
        Retrieves the last recieved message regarding fid pose
        if it exists, otherwise returns None (hint: you will need to create an additonal instance variable in the class)
        """
        #TODO:
        pass
        

    def recieve_fid_data(self, message : StarterProjectTag):
        #TODO: (fill in the correct type for message) and handle incoming FID data messages here
        pass
        


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    fid_listener: rospy.Subscriber

    # Use these as the primary interfaces in states
    rover: Rover
    env: Environment

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher("nav_vis", Marker)
        self.rover = Rover(self)
        self.env = Environment(self, None)
        self.fid_listener = rospy.Subscriber("/tag", StarterProjectTag,self.env.recieve_fid_data)