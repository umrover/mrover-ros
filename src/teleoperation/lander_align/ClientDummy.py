#!/usr/bin/env python3
import rospkg
import actionlib

import os

# python linear algebra library
import numpy as np

# library for interacting with ROS and TF tree
import rospy
import mrover

from mrover.msg import LanderAlignAction
from mrover.msg import LanderAlignGoal


class ClientDummy:
    def __init__(self):
        self.landerClient = actionlib.SimpleActionClient('LanderAlignAction', LanderAlignAction)
        self.landerClient.wait_for_server()
        print("finished init")

        
    def start_lander_align(self) -> None:
        print("start")
        goal = LanderAlignGoal()
        self.landerClient.send_goal(goal)
        print("finished start")
        pass

    def stop_lander_align(self) -> None:
        print("stop")
        self.landerClient.cancel_goal()
        pass



def main():
    # initialize the node
    rospy.init_node("ClientDummy")
    dummy = ClientDummy()

    dummy.start_lander_align()
    rospy.sleep(4)
    dummy.stop_lander_align()

    # let the callback functions run asynchronously in the background
    rospy.spin()
    
    
if __name__ == "__main__":
    main()