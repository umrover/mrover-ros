import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix, geometry_msgs

EARTH_RADIUS = 6371000


def gps_to_world(gps_coord: NavSatFix, ref_coord: NavSatFix, name: str, parent: str = "world") -> TransformStamped:
    """
    Returns the GPS to cartesian world transform using
    the GPS reference point of the world origin from the parameter server
    :param gps_coord:   The gps coordinate that we want in the cartesian world frame
    :param ref_coord:
    :param name:        The name of the returned transform frame
    :param parent:      The name of the reference world frame
    :return:
    """
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = name
    t.transform.translation.x = np.radians(gps_coord.longitude - ref_coord.longitude) * EARTH_RADIUS
    t.transform.translation.y = np.radians(gps_coord.latitude - ref_coord.latitude) * EARTH_RADIUS
    t.transform.translation.z = gps_coord.altitude
    return t
