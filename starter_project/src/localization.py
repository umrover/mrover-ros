# python linear algebra library
import numpy as np

# library for interacting with ROS
import rospy

# ROS message types we need to use
from sensor_msgs.msg import NavSatFix, Imu


class Localization:
    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        rospy.Subscriber("gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        self.SE3

    def gps_callback(msg: NavSatFix):
        """
        This function will be called every time this node receives a NavSatFix message
        on the /gps topic. It should read the GPS location from the given NavSatFix message,
        convert it to cartesian coordiantes, and then store that value in `self.SE3`
        """
        ...

    def imu_callback(msg: Imu):
        ...

    def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
        """
        This is a utility function that should convert spherical (latitude, longitude) 
        coordinates into cartesian (x, y, z) coordinates using the specified reference point 
        as the center of the tangent plane used for approximation.
        
        :param spherical_coord: the spherical coordinate to convert, 
                                given as a numpy array [latiude, longitude]
        :param reference_coord: the reference coordinate to use for conversion,
                                given as a numpy array [latiude, longitude]

        """
        ...


def main():
    # initialize the node
    rospy.init_node("localization")
    localization = Localization()


if __name__ == "__main__":
    main()
