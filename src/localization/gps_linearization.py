#!/usr/bin/env python3
from typing import Optional
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from mrover.msg import ImuAndMag, RTKNavSatFix
from pymap3d.enu import geodetic2enu
from std_msgs.msg import Header
from util.SE3 import SE3
from util.SO3 import SO3
from util.np_utils import numpify
import message_filters
import threading
from tf.transformations import *

class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it.
    """

    last_gps_pose: Optional[np.ndarray]
    last_imu_orientation: Optional[np.ndarray]

    # offset
    rtk_offset: np.ndarray

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

    left_gps_timer: threading.Timer
    right_gps_timer: threading.Timer
    left_gps_has_timeout: bool
    right_gps_has_timeout: bool

    # subscribers and publishers
    left_gps_sub : message_filters.Subscriber
    right_gps_sub : message_filters.Subscriber
    sync_gps_sub: message_filters.ApproximateTimeSynchronizer
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
        self.last_imu_orientation = None

        # rtk offset
        self.rtk_offset = np.array([0, 0, 0, 1])

        # subscribe to topics
        self.left_gps_sub = message_filters.Subscriber("/left_gps_driver/fix", RTKNavSatFix)
        self.right_gps_sub = message_filters.Subscriber("/right_gps_driver/fix", RTKNavSatFix)
        self.imu_sub = rospy.Subscriber("imu/data", ImuAndMag, self.imu_callback)

        # sync subscribers
        self.sync_gps_sub = message_filters.ApproximateTimeSynchronizer([self.right_gps_sub, self.left_gps_sub], 10, 0.5)
        self.sync_gps_sub.registerCallback(self.gps_callback)

        self.left_gps_sub.registerCallback(self.left_gps_callback)
        self.right_gps_sub.registerCallback(self.right_gps_callback)

        # publisher
        self.pose_publisher = rospy.Publisher("linearized_pose", PoseWithCovarianceStamped, queue_size=1)

        # timers
        self.gps_has_timeout = self.left_gps_has_timeout = self.right_gps_has_timeout = self.imu_time_delay = False
        self.gps_timeout_interval = self.imu_timeout_interval = 5
        self.gps_timer = threading.Timer(self.gps_timeout_interval, self.gps_timeout)
        self.imu_timer = threading.Timer(self.imu_timeout_interval, self.imu_timeout)

        self.left_gps_timer = threading.Timer(self.gps_timeout_interval, self.left_gps_timeout)
        self.right_gps_timer = threading.Timer(self.gps_timeout_interval, self.right_gps_timeout)

    def right_gps_timeout(self):
        print("right gps has timed out")
        self.right_gps_has_timeout = True

    def left_gps_timeout(self):
        print("left gps has timed out")
        self.left_gps_has_timeout = True
    
    def right_gps_callback(self, right_gps_msg: RTKNavSatFix):
        self.right_gps_has_timeout = False

        self.right_gps_timer.cancel()
        self.right_gps_timer = threading.Timer(self.gps_timeout_interval, self.right_gps_timeout)
        self.right_gps_timer.start()

        # what if synchronizer was not working?
        if (self.left_gps_has_timeout == False):
            return

        if np.any(np.isnan([right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude])):
            return
        
        ref_coord = np.array([self.ref_lat, self.ref_lon, self.ref_alt])

        right_cartesian = np.array(
            geodetic2enu(right_gps_msg.latitude, right_gps_msg.longitude, right_gps_msg.altitude, *ref_coord, deg=True)
        )

        pose = SE3(right_cartesian, SO3(np.array[0, 0, 0, 1]))

        if (self.last_imu_orientation is not None):
            imu_quat = self.last_imu_orientation
            gps_quat = quaternion_multiply(self.rtk_offset, imu_quat)
            pose = SE3(pose.position, SO3(gps_quat))

        self.last_gps_pose = pose

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

        self.pose_publisher(pose_msg)


    def left_gps_callback(self, left_gps_msg: RTKNavSatFix):
        print("left_GPS")
    
    
    def gps_callback(
        self,
        right_gps_msg: RTKNavSatFix,
        left_gps_msg: RTKNavSatFix
    ):
        """
        Callback function that receives GPS messages, assigns their covariance matrix,
        and then publishes the linearized pose.

        :param msg: The NavSatFix message containing GPS data that was just received
        TODO: Handle invalid PVTs
        """
        print("GPS callback")
        self.gps_has_timeout = False

        # set gps timer
        self.gps_timer.cancel()
        self.gps_timer = threading.Timer(self.gps_timeout_interval, self.gps_timeout)
        self.gps_timer.start()
        
        if np.any(np.isnan([right_gps_msg.coord.latitude, right_gps_msg.coord.longitude, right_gps_msg.coord.altitude])):
            return
        if np.any(np.isnan([left_gps_msg.coord.latitude, left_gps_msg.coord.longitude, left_gps_msg.coord.altitude])):
            return

        ref_coord = np.array([self.ref_lat, self.ref_lon, self.ref_alt])

        right_cartesian = np.array(
            geodetic2enu(right_gps_msg.coord.latitude, right_gps_msg.coord.longitude, right_gps_msg.coord.altitude, *ref_coord, deg=True)
        )
        left_cartesian = np.array(
            geodetic2enu(left_gps_msg.coord.latitude, left_gps_msg.coord.longitude, left_gps_msg.coord.altitude, *ref_coord, deg=True)
        )

        pose = GPSLinearization.compute_gps_pose(right_cartesian=right_cartesian, left_cartesian=left_cartesian)

        # if imu has not timed out, use imu roll and pitch, calculate offset if fix status is RTK_FIX
        if (self.imu_has_timeout == False and self.last_imu_orientation is not None):
            imu_quat = self.last_imu_orientation
            gps_quat = pose.rotation.quaternion

            if (right_gps_msg.fix_type == RTKNavSatFix.RTK_FIX and left_gps_msg.fix_type == RTKNavSatFix.RTK_FIX):
                imu_yaw_quat = quaternion_from_euler(0, 0, euler_from_quaternion(imu_quat)[2], "sxyz")
                gps_yaw_quat = quaternion_from_euler(0, 0, euler_from_quaternion(gps_quat)[2], "sxyz")
                self.rtk_offset = quaternion_multiply(gps_yaw_quat, quaternion_conjugate(imu_yaw_quat))

            gps_quat = quaternion_multiply(self.rtk_offset, imu_quat)
            pose = SE3(pose.position, SO3(gps_quat))

        self.last_gps_pose = pose

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
        
        self.imu_has_timeout = False
        
        # set imu timer
        self.imu_timer.cancel()
        self.imu_timer = threading.Timer(self.imu_timeout_interval, self.imu_timeout)
        self.imu_timer.start()

        if self.last_gps_pose is None:
            return
        
        # imu quaternion
        imu_quat = numpify(msg.imu.orientation)
        self.last_imu_orientation = imu_quat
        
        imu_quat = quaternion_multiply(self.rtk_offset, imu_quat)

        covariance_matrix = np.zeros((6, 6))
        covariance_matrix[:3, :3] = self.config_gps_covariance.reshape(3, 3)
        covariance_matrix[3:, 3:] = self.config_imu_covariance.reshape(3, 3)

        # publish to ekf
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
