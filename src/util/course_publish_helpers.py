import rospy
import time

from mrover.msg import GPSWaypoint, Waypoint, AutonCommand
from typing import Tuple
from util.SE3 import SE3
import pymap3d


REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")


def publish_waypoints(waypoints):
    publish_enable = rospy.Publisher("auton/command", AutonCommand, queue_size=1)
    msg = AutonCommand(waypoints, True)
    while 1:
        publish_enable.publish(msg)


def convert_waypoint_to_gps(waypoint_pose_pair: Tuple[Waypoint, SE3]) -> GPSWaypoint:
    waypoint, pose = waypoint_pose_pair
    lat, lon, _ = pymap3d.enu2geodetic(pose.position[0], pose.position[1], pose.position[2], REF_LAT, REF_LON, 0.0)
    return GPSWaypoint(lat, lon, waypoint.type, waypoint.fiducial_id)
