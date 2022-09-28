#!/usr/bin/env python3

import numpy as np
import rospy

from mrover.src.util.course_service import CourseService
from mrover.msg import EnableAuton,  Waypoint, GPSWaypoint

from util.SE3 import SE3
from util.tf_utils import EARTH_RADIUS

class Converter:
    def __init__(self):
        rospy.init_node("gps-converter")
        rospy.Subscriber("auton/enable_state", EnableAuton, self.read_data)
        self.publish_course = CourseService()
    
    
    def convert(self, waypoint: GPSWaypoint):
        """
        Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.

        GPSWaypoint:
        float latitude_degrees
        float longitude_degrees
        bool gate #should we search for a gate around the above coordinate
        bool post #should we search for a post around the above coordinate (NOT RELEVANT IF GATE IS TRUE)
        int32 id

        Waypoint:
        int32 fiducial_id
        string tf_id
        """

        # Create odom position based on GPS latitude and longitude
        base = np.array([0, 0]) # Our base/reference is in latitude, longitude coordinates (degrees)
        x = EARTH_RADIUS * np.radians(waypoint.latitude_degrees - base[0])
        y = EARTH_RADIUS * np.radians(waypoint.longitude_degrees - base[1])
        z = 0
        odom = np.array([x, y, z])
        return (Waypoint(fiducial_id=0, tf_id=f"course{waypoint.id}"), SE3(position=odom))

    def read_data(self, data: EnableAuton):
        # If auton is enabled, publish the waypoints to the course
        if (data.enable):
            waypoints = [self.convert(i) for i in data.waypoints]
            self.publish_course(waypoints)

def main():
    converter_node = Converter()
    rospy.spin()

if __name__ == "__main__":
    main()