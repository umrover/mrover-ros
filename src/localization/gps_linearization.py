#!/usr/bin/env python3
from typing import Optional
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
import threading

# from mrover.msg import rtkStatus
from tf.transformations import *

class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it.
    """

    last_gps_pose: Optional[np.ndarray]
    last_gps_pose_fixed: Optional[np.ndarray]
    last_imu_orientation: Optional[np.ndarray]

    # offset
    rtk_offset: Optional[np.ndarray]
    calculate_offset: bool

    # reference coordinates
    ref_lat: float
    ref_lon: float
    ref_alt: float

    world_frame: str

    # timeout detection
    gps_timeout_interval: int
    imu_timeout_interval: int
    gps_has_timeout: bool
    imu_has_timeout: bool
    gps_timer: threading.Timer
    imu_timer: threading.Timer

    # subscribers and publishers
    left_gps_sub : rospy.Subscriber
    right_gps_sub : rospy.Subscriber
    sync_gps_sub: rospy.Subscriber
    imu_sub: rospy.Subscriber
    pose_publisher: rospy.Publisher
    
    # covariance config
    use_dop_covariance: bool
    config_gps_covariance: np.ndarray


    # timeout callbacks, if both gps and imu fail kill node
    def gps_timeout(self):
        rospy.loginfo("GPS has timed out")
        self.gps_has_timeout = True

        if self.imu_has_timeout == True:
            rospy.signal_shutdown("Both IMU and GPS have timed out")
            
        return
    
    def imu_timeout(self):
        rospy.loginfo("IMU has timed out")
        self.imu_has_timeout = True

        if self.gps_has_timeout == True:
            rospy.signal_shutdown("Both IMU and GPS have timed out")

        return
    

    def __init__(self):

        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")
        self.world_frame = rospy.get_param("world_frame")
        self.use_dop_covariance = rospy.get_param("global_ekf/use_gps_dop_covariance")

        # config gps and imu convariance matrices
        self.config_gps_covariance = np.array(rospy.get_param("global_ekf/gps_covariance", None))
        self.config_imu_covariance = np.array(rospy.get_param("global_ekf/imu_orientation_covariance", None))

        # track last gps pose and last imu message
        self.last_gps_pose = None
        self.last_gps_pose_fixed = None
        self.last_imu_orientation = None

        # rtk offset
        self.rtk_offset = None
        self.calculate_offset = False

        # subscribe to topics
        self.right_gps_sub = message_filters.Subscriber("/right_gps_driver/fix", NavSatFix)
        self.left_gps_sub = message_filters.Subscriber("/left_gps_driver/fix", NavSatFix)
        self.imu_sub = rospy.Subscriber("imu/data", ImuAndMag, self.imu_callback)
        # left_rtk_fix_sub = message_filters.Subscriber("left_gps_driver/rtk_fix_status", rtkStatus)
        # right_rtk_fix_sub = message_filters.Subscriber("right_gps_driver/rtk_fix_status", rtkStatus)

        # sync subscribers
        self.sync_gps_sub = message_filters.ApproximateTimeSynchronizer([self.right_gps_sub, self.left_gps_sub], 10, 0.5)
        self.sync_gps_sub.registerCallback(self.gps_callback)

        # publisher
        self.pose_publisher = rospy.Publisher("linearized_pose", PoseWithCovarianceStamped, queue_size=1)

        # timers
        self.gps_has_timeout = self.imu_time_delay = False
        self.gps_timeout_interval = self.imu_timeout_interval = 5
        self.gps_timer = threading.Timer(self.gps_timeout_interval, self.gps_timeout)
        self.imu_timer = threading.Timer(self.imu_timeout_interval, self.imu_timeout)
        
        
    def gps_callback(
        self,
        right_gps_msg: NavSatFix,
        left_gps_msg: NavSatFix,
    ):
        """
        Callback function that receives GPS messages, assigns their covariance matrix,
        and then publishes the linearized pose.

        :param msg: The NavSatFix message containing GPS data that was just received
        TODO: Handle invalid PVTs
        """

        self.gps_has_timeout = False

        # global gps_timer
        self.gps_timer.cancel()
        self.gps_timer = threading.Timer(self.gps_timeout_interval, self.gps_timeout)
        self.gps_timer.start()
        
        if np.any(np.isnan([right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude])):
            return
        if np.any(np.isnan([left_gps_msg.latitude, left_gps_msg.longitude, left_gps_msg.altitude])):
            return

        ref_coord = np.array([self.ref_lat, self.ref_lon, self.ref_alt])

        right_cartesian = np.array(
            geodetic2enu(right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude, *ref_coord, deg=True)
        )
        left_cartesian = np.array(
            geodetic2enu(left_gps_msg.latitude, left_gps_msg.longitude, left_gps_msg.altitude, *ref_coord, deg=True)
        )

        pose = GPSLinearization.compute_gps_pose(right_cartesian=right_cartesian, left_cartesian=left_cartesian)

        # if imu has timed out do not use imu orientation
        if (self.imu_has_timeout == False):
            # add roll and pitch from imu to gps rotation
            if self.last_imu_orientation is not None:
                imu_euler_angles = euler_from_quaternion(self.last_imu_orientation)
                gps_euler_angles = euler_from_quaternion(pose.rotation.quaternion)
                pose = SE3(pose.position, SO3((quaternion_from_euler(imu_euler_angles[0], imu_euler_angles[1], gps_euler_angles[2], axes="sxyz")).copy()))
    
        self.last_gps_pose = pose

        # if the fix status of both gps is 2 (fixed), update the offset with the next imu messsage
        # if right_rtk_fix.RTK_FIX_TYPE == 2 and left_rtk_fix.RTK_FIX_TYPE == 2:
        self.calculate_offset = True
        self.last_gps_pose_fixed = pose

        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[:3, :3] = self.config_gps_covariance.reshape(3, 3)
        covariance_matrix[3:, 3:] = self.config_imu_covariance.reshape(3, 3)

        # publish to ekf
        pose_msg = PoseWithCovarianceStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(*pose.position),
                    orientation=Quaternion(*pose.rotation.quaternion),
                ),
                covariance=covariance_matrix.flatten().tolist(),
            ),
        )
        
        # publish pose (rtk)
        self.pose_publisher.publish(pose_msg)


    def imu_callback(self, msg: ImuAndMag):
        """
        Callback function that receives IMU messages and publishes the linearized pose.

        :param msg: The Imu message containing IMU data that was just received
        """
        # reset in the case that we want it to go back after it has set itself to true
        self.imu_has_timeout = False
        
        self.imu_timer.cancel()
        self.imu_timer = threading.Timer(self.imu_timeout_interval, self.imu_timeout)
        self.imu_timer.start()

        if self.last_gps_pose is None:
            return
        
        imu_orientation = numpify(msg.imu.orientation)
        self.last_imu_orientation = imu_orientation
        
        # if gps has timed out do not use offset
        if (self.gps_has_timeout == False):
            # if calculate_offset flag is true
            if self.calculate_offset:
                # euler angles
                imu_euler_angles = euler_from_quaternion(imu_orientation)
                gps_euler_angles = euler_from_quaternion(self.last_gps_pose_fixed.rotation.quaternion)

                # rotation matrices, use imu roll and pitch for gps rotation to make sure these angles are preserved
                imu_rotation_matrix = euler_matrix(imu_euler_angles[0], imu_euler_angles[1], imu_euler_angles[2], axes="sxyz")
                gps_rotation_matrix = euler_matrix(imu_euler_angles[0], imu_euler_angles[1], gps_euler_angles[2], axes="sxyz")

                self.rtk_offset = np.matmul(inverse_matrix(imu_rotation_matrix), gps_rotation_matrix)
                self.calculate_offset = False

            # if rtk_offset is set, apply offset to imu orientation
            if self.rtk_offset is not None:
                imu_euler_angles = euler_from_quaternion(imu_orientation)
                imu_rotation_matrix = euler_matrix(ai=imu_euler_angles[0], aj=imu_euler_angles[1], ak=imu_euler_angles[2], axes="sxyz")

                offsetted_rotation_matrix = np.matmul(imu_rotation_matrix, self.rtk_offset)
                offsetted_euler = euler_from_matrix(offsetted_rotation_matrix)
                imu_orientation = quaternion_from_euler(
                    offsetted_euler[0], offsetted_euler[1], offsetted_euler[2], axes="sxyz"
                )

        # imu quaternion
        imu_quat = imu_orientation / np.linalg.norm(imu_orientation)

        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[:3, :3] = self.config_gps_covariance.reshape(3, 3)
        covariance_matrix[3:, 3:] = self.config_imu_covariance.reshape(3, 3)

        pose_msg = PoseWithCovarianceStamped(
            header=Header(stamp=msg.header.stamp, frame_id=self.world_frame),
            pose=PoseWithCovariance(
                pose=Pose(
                    position=Point(*self.last_gps_pose.position),
                    orientation=Quaternion(*imu_quat),
                ),
                covariance=covariance_matrix.flatten().tolist(),
            ),
        )

        # publish pose (imu with correction)
        self.pose_publisher.publish(pose_msg)

    @staticmethod
    def compute_gps_pose(right_cartesian, left_cartesian) -> np.ndarray:
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


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization", disable_signals=True)
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
