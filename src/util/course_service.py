from __future__ import annotations

from typing import List, Tuple

import rospy
import tf2_ros
from mrover.msg import Waypoint, Course, EnableAuton, GPSWaypoint
from mrover.srv import PublishCourse, PublishEnableAuton
from util.SE3 import SE3
import pymap3d
import numpy as np


class CourseService(rospy.ServiceProxy):
    SERVICE_NAME = "course_service"

    tf_broadcaster: tf2_ros.StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()

    def __init__(self, **kwargs):
        super().__init__(self.SERVICE_NAME, PublishCourse, **kwargs)

    def call(self, waypoints: List[Tuple[Waypoint, SE3]]):
        all_waypoint_info = []
        for waypoint_info, pose in waypoints:
            all_waypoint_info.append(waypoint_info)
            pose.publish_to_tf_tree(self.tf_broadcaster, "map", waypoint_info.tf_id)
        course = Course(waypoints=all_waypoint_info)
        super().call(course)

    def __call__(self, waypoints: List[Tuple[Waypoint, SE3]]):
        self.call(waypoints)


class EnableService(rospy.ServiceProxy):
    SERVICE_NAME = "enable_service"

    def __init__(self, **kwargs):
        super().__init__(self.SERVICE_NAME, PublishEnableAuton, **kwargs)
        self.publish_course = CourseService()
        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")

    def convert(self, waypoint: GPSWaypoint) -> Waypoint:
        """
        Converts a GPSWaypoint into a "Waypoint" used for publishing to the CourseService.
        """

        # Create odom position based on GPS latitude and longitude
        odom = np.array(
            pymap3d.geodetic2enu(
                waypoint.latitude_degrees, waypoint.longitude_degrees, 0.0, self.ref_lat, self.ref_lon, 0.0, deg=True
            )
        )
        # zero the z-coordinate of the odom because even though the altitudes are set to zero,
        # two points on a sphere are not going to have the same z coordinate
        # navigation algorithmns currently require all coordinates to have zero as the z coordinate
        odom[2] = 0

        return Waypoint(fiducial_id=waypoint.id, tf_id=f"course{waypoint.id}", type=waypoint.type), SE3(position=odom)

    def convert_and_publish_course(self, data: EnableAuton):
        # If auton is enabled, publish the waypoints to the course
        if data.enable:
            waypoints = [self.convert(i) for i in data.waypoints]
            self.publish_course(waypoints)

    def __call__(self, enableMsg: EnableAuton):
        self.convert_and_publish_course(enableMsg)
