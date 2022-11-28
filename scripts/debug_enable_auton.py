#!/usr/bin/env python3

import numpy as np

import rospy
from mrover.msg import Waypoint, EnableAuton, GPSWaypoint, WaypointType


def main():
    rospy.init_node("debug_gps_publisher")
    target = np.array([42.2, -83.7001, 0.0])
    pub = rospy.Publisher("auton/enable_state", EnableAuton, queue_size=1)
    waypoints = [GPSWaypoint(target[0], target[1], WaypointType(val=WaypointType.POST), 1)]

    r = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        enable_msg = EnableAuton(waypoints, True)
        pub.publish(enable_msg)
        r.sleep()


if __name__ == "__main__":
    main()
