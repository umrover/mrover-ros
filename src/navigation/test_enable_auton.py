#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import Waypoint, EnableAuton, GPSWaypoint


def main():
    print("Starting tests")
    rospy.init_node("debug_gps_publisher")
    # target = np.array([42.2, -83.7, 0.0])
    target = np.array([42.2, -83.7001, 0.0])
    print("Starting publisher")
    pub = rospy.Publisher("auton/enable_state", EnableAuton, queue_size=1)

    waypoints = [
        GPSWaypoint(target[0], target[1], False, True, 1)
    ]
    print("Starting")
    
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        print("message published")
        enable_msg = EnableAuton(True, waypoints)
        pub.publish(enable_msg)
        r.sleep()

    

if __name__ == "__main__":
    main()