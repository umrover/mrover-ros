#!/usr/bin/env python3

import rospy
from mrover.msg import ClickIkAction, ClickIkGoal
import actionlib
import sys

def click_ik_client():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('do_click_ik', ClickIkAction)
    print('created click ik client')
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    print('Found server')
    # Creates a goal to send to the action server.
    goal = ClickIkGoal()
    # Sends the goal to the action server.
    goal.pointInImageX = 0
    goal.pointInImageY = 0
    client.send_goal(goal, feedback_cb=feedback)
    client.wait_for_result()
    result = client.get_result()
    return result

def feedback(msg):
    print(msg)

if __name__ == "__main__":
    try:
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
        rospy.init_node('debug_click_ik')
        result = click_ik_client()
        print('result: ', result)
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)