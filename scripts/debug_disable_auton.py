#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import AutonCommand
from util.course_publish_helpers import publish_waypoints


"""
The purpose of this file is for testing courses in the sim 
without needing the auton GUI. You can add waypoints with
and without fiducials and these will get published to nav
"""


if __name__ == "__main__":
    rospy.init_node("debug_disable_auton")
    publish_enable = rospy.Publisher("auton/command", AutonCommand, queue_size=1)
    msg = AutonCommand([], False)
    while 1:
        publish_enable.publish(msg)
