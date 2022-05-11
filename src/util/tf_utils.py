import numpy as np

import rospy
from geometry_msgs.msg import TransformStamped, Quaternion
from sensor_msgs.msg import NavSatFix, geometry_msgs

EARTH_RADIUS = 6371000


def gps_to_world(gps_coord: NavSatFix, ref_coord: NavSatFix, parent_frame: str, world_frame: str = "world") \
        -> TransformStamped:
    """
    Convert the GPS coordinate to a cartesian coordinate transform
    Returns the transform from the specified parent frame to a world frame with
    origin at the specified GPS reference coordinate
    :param gps_coord:   The GPS coordinate that we want in cartesian coordinates
    :param ref_coord:   The reference GPS coordinate
    :param name:        The name of the returned transform frame
    :param parent:      The name of the reference world frame
    :return:
    """
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = world_frame

    # TODO: compare accuracy to geodesy.utm transforms
    lat_cos = np.cos(np.radians(gps_coord.latitude))
    scaled_longitude = (gps_coord.longitude - ref_coord.longitude) * lat_cos
    t.transform.translation.x = np.radians(scaled_longitude) * EARTH_RADIUS
    t.transform.translation.y = np.radians(
        gps_coord.latitude - ref_coord.latitude) * EARTH_RADIUS

    # use an identity quaternion since we have no rotation info from GPS
    t.transform.rotation.w = Quaternion(0, 0, 0, 1)
    return t
