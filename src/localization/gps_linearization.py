#!/usr/bin/env python3
from typing import Optional, Tuple

import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from mrover.msg import ImuAndMag
from pymap3d.enu import geodetic2enu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from util.SE3 import SE3
from util.np_utils import numpify


class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it.
    """

    last_gps_msg: Optional[NavSatFix]
    last_imu_msg: Optional[ImuAndMag]
    pose_publisher: rospy.Publisher

    # reference coordinates
    ref_lat: float
    ref_lon: float
    ref_alt: float

    world_frame: str

    # covariance config
    use_dop_covariance: bool
    config_gps_covariance: np.ndarray

    def __init__(self):
        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")
        self.world_frame = rospy.get_param("world_frame")
        self.use_dop_covariance = rospy.get_param("global_ekf/use_gps_dop_covariance")
        self.config_gps_covariance = np.array(rospy.get_param("global_ekf/gps_covariance", None))

        self.last_gps_msg = None
        self.last_imu_msg = None

        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu/data", ImuAndMag, self.imu_callback)
        self.pose_publisher = rospy.Publisher("linearized_pose", PoseWithCovarianceStamped, queue_size=1)

    def gps_callback(self, msg: NavSatFix):
        """
        Callback function that receives GPS messages, assigns their covariance matrix,
        and then publishes the linearized pose.

        :param msg: The NavSatFix message containing GPS data that was just received
        """
        if np.any(np.isnan([msg.latitude, msg.longitude, msg.altitude])):
            return

        if not self.use_dop_covariance:
            msg.position_covariance = self.config_gps_covariance

        self.last_gps_msg = msg

        if self.last_imu_msg is not None:
            self.publish_pose()

    def imu_callback(self, msg: ImuAndMag):
        """
        Callback function that receives IMU messages and publishes the linearized pose.

        :param msg: The Imu message containing IMU data that was just received
        """
        self.last_imu_msg = msg

        if self.last_gps_msg is not None:
            self.publish_pose()

    @staticmethod
    def get_linearized_pose_in_world(
        gps_msg: NavSatFix, imu_msg: ImuAndMag, ref_coord: np.ndarray
    ) -> Tuple[SE3, np.ndarray]:
        """
        Linearizes the GPS geodetic coordinates into ENU cartesian coordinates,
        then combines them with the IMU orientation into a pose estimate relative
        to the world frame, with corresponding covariance matrix.

        :param gps_msg: Message containing the rover's GPS coordinates and their corresponding
                        covariance matrix.
        :param imu_msg: Message containing the rover's orientation from IMU, with
                        corresponding covariance matrix.
        :param ref_coord: numpy array containing the geodetic coordinate which will be the origin
                          of the tangent plane, [latitude, longitude, altitude]
        :returns: The pose consisting of linearized GPS and IMU orientation, and the corresponding
                  covariance matrix as a 6x6 numpy array where each row is [x, y, z, roll, pitch, yaw]
        """
        # linearize GPS coordinates into cartesian
        cartesian = np.array(geodetic2enu(gps_msg.latitude, gps_msg.longitude, gps_msg.altitude, *ref_coord, deg=True))

        # ignore Z
        cartesian[2] = 0

        imu_quat = numpify(imu_msg.imu.orientation)

        # normalize to avoid rounding errors
        imu_quat = imu_quat / np.linalg.norm(imu_quat)
        pose = SE3.from_pos_quat(position=cartesian, quaternion=imu_quat)

        covariance = np.zeros((6, 6))
        covariance[:3, :3] = np.array([gps_msg.position_covariance]).reshape(3, 3)
        covariance[3:, 3:] = np.array([imu_msg.imu.orientation_covariance]).reshape(3, 3)

        return pose, covariance

    def publish_pose(self):
        """
        Publishes the linearized pose of the rover relative to the map frame,
        as a PoseWithCovarianceStamped message.
        """
        ref_coord = np.array([self.ref_lat, self.ref_lon, self.ref_alt])
        linearized_pose, covariance = self.get_linearized_pose_in_world(self.last_gps_msg, self.last_imu_msg, ref_coord)

        pose_msg = PoseWithCovarianceStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(*linearized_pose.position),
                    orientation=Quaternion(*linearized_pose.rotation.quaternion),
                ),
                covariance=covariance.flatten().tolist(),
            ),
        )
        self.pose_publisher.publish(pose_msg)


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
