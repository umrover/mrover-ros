#!/usr/bin/env python3

import numpy as np
from pymap3d.enu import geodetic2enu

import rospy
from geometry_msgs.msg import Vector3Stamped, Vector3
from sensor_msgs.msg import NavSatFix


class GPSLinearization:
    def __init__(self):
        # Read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")
        self.world_frame = rospy.get_param("world_frame")

        rospy.Subscriber("gps/fix", NavSatFix, self.single_gps_callback)
        self.position_publisher = rospy.Publisher("linearized_position", Vector3Stamped, queue_size=1)

    def single_gps_callback(self, msg: NavSatFix):
        if np.any(np.isnan([msg.latitude, msg.longitude, msg.altitude])):
            rospy.logwarn("Received NaN GPS data, ignoring")
            return

        x, y, _ = geodetic2enu(
            msg.latitude, msg.longitude, 0.0, self.ref_lat, self.ref_lon, self.ref_alt, deg=True
        )
        self.position_publisher.publish(Vector3Stamped(msg.header, Vector3(x=x, y=y)))


def main():
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
