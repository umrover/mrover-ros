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
from util.SO3 import SO3
from util.np_utils import numpify
import message_filters


class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it.
    """

    last_gps_msg: Optional[np.ndarray]
    last_imu_msg: Optional[ImuAndMag]
    pose_publisher: rospy.Publisher

    # reference coordinates
    ref_lat: float
    ref_lon: float
    ref_alt: float

    world_frame: str
    both_gps: bool

    # covariance config
    use_dop_covariance: bool
    config_gps_covariance: np.ndarray

    def __init__(self):
        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param(param_name="gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")
        self.both_gps = rospy.get_param("gps_linearization/use_both_gps")
        self.world_frame = rospy.get_param("world_frame")
        self.use_dop_covariance = rospy.get_param("global_ekf/use_gps_dop_covariance")
        self.ref_coord = np.array([self.ref_lat, self.ref_lon, self.ref_alt])

        # config gps and imu convariance matrices
        self.config_gps_covariance = np.array(rospy.get_param("global_ekf/gps_covariance", None))
        self.config_imu_covariance = np.array(rospy.get_param("global_ekf/imu_orientation_covariance", None))

        self.last_gps_msg = None
        self.last_pose = None
        self.last_imu_msg = None

        if self.both_gps:
            right_gps_sub = message_filters.Subscriber("right_gps_driver/fix", NavSatFix)
            left_gps_sub = message_filters.Subscriber("left_gps_driver/fix", NavSatFix)
            sync_gps_sub = message_filters.ApproximateTimeSynchronizer([right_gps_sub, left_gps_sub], 10, 0.5)
            sync_gps_sub.registerCallback(self.duo_gps_callback)
        else:
            rospy.Subscriber("left_gps_driver/fix", NavSatFix, self.single_gps_callback)

        rospy.Subscriber("imu/data", ImuAndMag, self.imu_callback)
        self.pose_publisher = rospy.Publisher("linearized_pose", PoseWithCovarianceStamped, queue_size=1)

    def single_gps_callback(self, msg: NavSatFix):
        """
        Callback function that receives GPS messages, assigns their covariance matrix,
        and then publishes the linearized pose.

        :param msg: The NavSatFix message containing GPS data that was just received
        """
        if np.any(np.isnan([msg.latitude, msg.longitude, msg.altitude])):
            return

        if not self.use_dop_covariance:
            msg.position_covariance = self.config_gps_covariance

        # print("using single callback")

        if self.last_imu_msg is not None:
            self.last_pose = self.get_linearized_pose_in_world(msg, self.last_imu_msg, self.ref_coord)
            self.publish_pose()

    def duo_gps_callback(self, right_gps_msg: NavSatFix, left_gps_msg: NavSatFix):
        """
        Callback function that receives GPS messages, assigns their covariance matrix,
        and then publishes the linearized pose.

        :param msg: The NavSatFix message containing GPS data that was just received
        TODO: Handle invalid PVTs
        """
        # print("using duo gps")
        if np.any(np.isnan([right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude])):
            return
        if np.any(np.isnan([left_gps_msg.latitude, left_gps_msg.longitude, left_gps_msg.altitude])):
            return

        right_cartesian = np.array(
            geodetic2enu(
                right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude, *self.ref_coord, deg=True
            )
        )
        left_cartesian = np.array(
            geodetic2enu(
                left_gps_msg.latitude, left_gps_msg.longitude, left_gps_msg.altitude, *self.ref_coord, deg=True
            )
        )

        self.last_pose = GPSLinearization.compute_gps_pose(
            right_cartesian=right_cartesian, left_cartesian=left_cartesian
        )

        self.publish_pose()
        # print("double")

    def imu_callback(self, msg: ImuAndMag):
        """
        Callback function that receives IMU messages and publishes the linearized pose.

        :param msg: The Imu message containing IMU data that was just received
        """
        self.last_imu_msg = msg

        if self.last_gps_msg is not None and not self.both_gps:
            self.last_pose = self.get_linearized_pose_in_world(self.last_gps_msg, self.last_imu_msg, self.ref_coord)
            self.publish_pose()
        elif self.last_pose is not None and self.both_gps:
            self.publish_pose()

    def publish_pose(self):
        """
        Publishes the linearized pose of the rover relative to the map frame,
        as a PoseWithCovarianceStamped message.
        """
        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[:3, :3] = self.config_gps_covariance.reshape(3, 3)
        covariance_matrix[3:, 3:] = self.config_imu_covariance.reshape(3, 3)

        pose_msg = PoseWithCovarianceStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(*self.last_pose.position),
                    orientation=Quaternion(*self.last_pose.rotation.quaternion),
                ),
                covariance=covariance_matrix.flatten().tolist(),
            ),
        )
        self.pose_publisher.publish(pose_msg)

    @staticmethod
    def compute_gps_pose(right_cartesian, left_cartesian) -> np.ndarray:
        # TODO: give simulated GPS non zero altitude so we can stop erasing the z component
        # left_cartesian[2] = 0
        # right_cartesian[2] = 0
        vector_connecting = left_cartesian - right_cartesian
        vector_connecting[2] = 0
        magnitude = np.linalg.norm(vector_connecting)
        vector_connecting = vector_connecting / magnitude

        vector_perp = np.zeros(shape=(3, 1))
        vector_perp[0] = vector_connecting[1]
        vector_perp[1] = -vector_connecting[0]

        rotation_matrix = np.hstack(
            (vector_perp, np.reshape(vector_connecting, (3, 1)), np.array(object=[[0], [0], [1]]))
        )

        # temporary fix, assumes base_link is exactly in the middle of the two GPS antennas
        # TODO: use static tf from base_link to left_antenna instead
        rover_position = (left_cartesian + right_cartesian) / 2

        pose = SE3(rover_position, SO3.from_matrix(rotation_matrix=rotation_matrix))

        return pose

    @staticmethod
    def get_linearized_pose_in_world(gps_msg: NavSatFix, imu_msg: ImuAndMag, ref_coord: np.ndarray) -> np.ndarray:
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

        return pose


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
