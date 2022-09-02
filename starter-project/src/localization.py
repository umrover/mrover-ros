import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix, Imu


class Localization:

    def __init__(self):
        # create subscribers for GPS and IMU data, linking them to our callback functions
        rospy.Subscriber("gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu", Imu, self.imu_callback)
        
        
    def gps_callback(msg: NavSatFix):
        ...

    def imu_callback(msg: Imu):
        ...
        
    def spherical_to_cartesian(lat: float, lon: float) -> np.ndarray:
        ...

def main():
    # initialize the node
    rospy.init_node("localization")
    localization = Localization()
    


if __name__ == "__main__":
    main()