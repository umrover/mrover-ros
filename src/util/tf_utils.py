from localization import EARTH_RADIUS
from sensor_msgs.msg import NavSatFix, geometry_msgs
from geometry_msgs.msg import TransformStamped, Vector3, Point
import rospy
import numpy as np

EARTH_RADIUS = 6371000

def gps_to_world(gps_coord : NavSatFix, ref_coord : NavSatFix, name : str, parent : str="world") ->  TransformStamped:
    '''
    Returns the GPS to cartesian world transform using the GPS reference point 
    of the world origin from the parameter server. Note that the 

    Args:
        gps_coord: The gps coordinate that we want in the cartesian world frame
        name: The name of the returned transform frame
        parent: The name of the reference world frame
    '''
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent
    t.child_frame_id = name
    t.transform.translation.x = np.radians(gps_coord.longitude - ref_coord.longitude) * EARTH_RADIUS
    t.transform.translation.y = np.radians(gps_coord.latitude - ref_coord.latitude) * EARTH_RADIUS
    t.transform.translation.z = gps_coord.altitude
    return t

def vector3_to_point(vec3: Vector3) -> Point:
    return Point(x=vec3.x, y=vec3.y, z=vec3.z)

def point_to_vector3(pt: Point) -> Vector3:
    return Vector3(x=pt.x, y=pt.y, z=pt.z)