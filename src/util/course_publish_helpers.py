from typing import List, Tuple

import pymap3d

import rospy
from mrover.msg import EnableAuton, GPSWaypoint, Waypoint
from mrover.srv import PublishEnableAuton
from util.SE3 import SE3

REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")


def publish_waypoints(waypoints: List[GPSWaypoint]):
    rospy.wait_for_service("enable_auton")
    try:
        publish_enable = rospy.ServiceProxy("enable_auton", PublishEnableAuton)
        msg = EnableAuton(waypoints, True)
        publish_enable(msg)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def convert_waypoint_to_gps(waypoint_pose_pair: Tuple[Waypoint, SE3]) -> GPSWaypoint:
    waypoint, pose = waypoint_pose_pair
    lat, lon, _ = pymap3d.enu2geodetic(pose.position[0], pose.position[1], pose.position[2], REF_LAT, REF_LON, 0.0)
    return GPSWaypoint(waypoint.tag_id, lat, lon, waypoint.type)
