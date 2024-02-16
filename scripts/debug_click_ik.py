#!/usr/bin/env python3

import rospy
from mrover.msg import ClickIkAction, ClickIkActionGoal, ClickIkActionFeedback, ClickIkActionResult
import actionlib
import sys

def click_ik_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('click-ik', ClickIkAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    goal = ClickIkActionGoal(pointInImageX=0, pointInImageY=0)
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == "__main__":
    try:
    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
        rospy.init_node('debug_click_ik')
        result = click_ik_client()
        print("Result:", ', '.join([str(n) for n in result.sequence]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)