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

    def get_fid_data(self) -> Optional[StarterProjectTag]:
        """
        Retrieves the last recieved message regarding fid pose
        if it exists, otherwise returns None (hint: you will need to create an additonal instance variable in the class)
        """

    def recieve_fid_data(self, message : StarterProjectTag):
        #TODO: (fill in the correct type for message) and handle incoming FID data messages here
        


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
        self.env = Environment(self)
        self.fid_listener = rospy.Subscriber("/tag", StarterProjectTag,self.env.recieve_fid_data)