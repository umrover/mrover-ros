import rospy
from mrover.srv import PublishEnableAuton
from mrover.msg import EnableAuton, GPSWaypoint, Waypoint
from typing import Tuple
from util.SE3 import SE3
import pymap3d


REF_LAT = rospy.get_param("gps_linearization/reference_point_latitude")
REF_LON = rospy.get_param("gps_linearization/reference_point_longitude")


def publish_waypoints(waypoints):
    rospy.wait_for_service("enable_auton")
    try:
        publish_enable = rospy.ServiceProxy("enable_auton", PublishEnableAuton)
        msg = EnableAuton(waypoints, True)
        publish_enable(msg)
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")


def convert_waypoint_to_gps(waypoint_pose_pair: Tuple[Waypoint, SE3]) -> GPSWaypoint:
    waypoint, pose = waypoint_pose_pair
    lat, lon, _ = pymap3d.enu2geodetic(pose.position[0], pose.position[1], pose.position[1], REF_LAT, REF_LON, 0.0)
    return GPSWaypoint(lat, lon, waypoint.type, waypoint.fiducial_id)
