import numpy as np

import rospy
from geometry_msgs.msg import Point, TransformStamped, Vector3
from sensor_msgs.msg import NavSatFix, geometry_msgs

EARTH_RADIUS = 6371000


def gps_to_world(gps_coord: NavSatFix, ref_coord: NavSatFix, name: str, parent: str = "world") -> TransformStamped:
    """
    Returns the GPS to cartesian world transform.

    :param gps_coord: The gps coordinate that we want in the cartesian world frame
    :param name: The name of the returned transform frame
    :param parent: The name of the reference world frame
    """
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = name
    longitude_delta = gps_coord.longitude - ref_coord.longitude
    latitude_delta = gps_coord.latitude - ref_coord.latitude
    t.transform.translation.x = np.radians(longitude_delta) * EARTH_RADIUS
    t.transform.translation.y = np.radians(latitude_delta) * EARTH_RADIUS
    t.transform.translation.z = gps_coord.altitude
    return t


def vector3_to_point(vec3: Vector3) -> Point:
    return Point(x=vec3.x, y=vec3.y, z=vec3.z)


def point_to_vector3(pt: Point) -> Vector3:
    return Vector3(x=pt.x, y=pt.y, z=pt.z)
