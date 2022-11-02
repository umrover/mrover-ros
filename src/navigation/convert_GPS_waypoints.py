#!/usr/bin/env python3

import numpy as np
import rospy
import pymap3d

from mrover.msg import EnableAuton,  Waypoint, GPSWaypoint, WaypointType
from util.course_service import CourseService

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
        """

        # Create odom position based on GPS latitude and longitude
        base = np.array([42.2, -83.7, 0.0]) # Our base/reference is in latitude, longitude, altitude coordinates (degrees)
        odom = np.array(pymap3d.geodetic2enu(waypoint.latitude_degrees, waypoint.longitude_degrees, base[2], base[0], base[1], base[2]))

        return (Waypoint(fiducial_id=0, tf_id=f"course{waypoint.id}", type=waypoint.type), SE3(position=odom))

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