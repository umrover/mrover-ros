#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from mrover.msg import ImuAndMag
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion
from std_msgs.msg import Header
import tf2_ros
import numpy as np
from pymap3d.enu import geodetic2enu
from tf.transformations import quaternion_about_axis, quaternion_multiply


class GPSLinearization:
    """
    This node subscribes to GPS and IMU data, linearizes the GPS data
    into ENU coordinates, then combines the linearized GPS position and the IMU
    orientation into a pose estimate for the rover and publishes it to the TF tree.
    """

    pose: SE3
    covariance: np.ndarray
    pose_publisher: rospy.Publisher
    
    # TF infrastructure
    tf_broadcaster: tf2_ros.TransformBroadcaster
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener

    # reference coordinates
    ref_lat: float
    ref_lon: float
    ref_alt: float

    # frame/publishing configuration
    publish_tf: bool
    use_odom: bool
    world_frame: str
    odom_frame: str
    rover_frame: str

    def __init__(self):
        # init to zero pose
        self.pose = SE3()

        # read required parameters, if they don't exist an error will be thrown
        self.ref_lat = rospy.get_param("gps_linearization/reference_point_latitude")
        self.ref_lon = rospy.get_param("gps_linearization/reference_point_longitude")
        self.ref_alt = rospy.get_param("gps_linearization/reference_point_altitude")

        self.publish_tf = rospy.get_param("gps_linearization/publish_tf")
        self.use_odom = rospy.get_param("gps_linearization/use_odom_frame")

        self.world_frame = rospy.get_param("world_frame")
        self.odom_frame = rospy.get_param("odom_frame")
        self.rover_frame = rospy.get_param("rover_frame")

        rospy.Subscriber("gps/fix", NavSatFix, self.gps_callback)
        rospy.Subscriber("imu/data", ImuAndMag, self.imu_callback)
        self.pose_publisher = rospy.Publisher("gps/pose", PoseWithCovarianceStamped, queue_size=1)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


    def publish_pose(self):
        """
        Publishes the pose of the rover relative to the map frame, either to the TF tree or as
        a PoseWithCovarianceStamped message. The pose will be published either as a direct 
        map->base_link transform, or an indirect map-odom transform if the odom frame is in use.
        See the wiki for more details:
        https://github.com/umrover/mrover-ros/wiki/Localization#guide-to-localization-frames
        """
        rover_in_map = self.pose

        if self.publish_tf:
            if self.use_odom:
                # TODO: fix naming conventions of transforms per Ashwins instructions
                # TODO: add transform notation to wiki
                # Get the odom to rover transform from the TF tree
                rover_in_odom = SE3.from_tf_tree(self.tf_buffer, self.odom_frame, self.rover_frame)
                odom_to_rover = rover_in_odom.transform_matrix()
                map_to_rover = rover_in_map.transform_matrix()

                # Calculate the intermediate transform from the overall transform and odom to rover
                map_to_odom = map_to_rover @ np.linalg.inv(odom_to_rover)
                odom_in_map = SE3.from_transform_matrix(map_to_odom)
                pose_out = odom_in_map
                child_frame = self.rover_frame
            else:
                # publish directly as map->base_link
                pose_out = rover_in_map
                child_frame = self.rover_frame
            
            pose_out.publish_to_tf_tree(
                self.tf_broadcaster, parent_frame=self.world_frame, child_frame=child_frame
            )
        else:
                pose_msg = PoseWithCovarianceStamped(
                    header=Header(stamp=rospy.Time.now(), frame_id=self.world_frame),
                    pose=PoseWithCovariance(
                        pose=Pose(
                            position=Point(*rover_in_map.position),
                            orientation=Quaternion(*rover_in_map.rotation.quaternion)
                        ),
                        # TODO: figure out covariance: either feed forward from GPS or make system for configuring them
                        covariance=np.diag(np.ones(6)*0.1).flatten().tolist()
                    )
                )
                self.pose_publisher.publish(pose_msg)
            

    def gps_callback(self, msg: NavSatFix):
        """
        Callback function that receives GPS messages, linearizes them,
        updates the rover pose, and publishes it to the TF tree.

        :param msg: The NavSatFix message containing GPS data that was just received
        """
        # linearize GPS coordinates into cartesian
        cartesian = np.array(
            geodetic2enu(msg.latitude, msg.longitude, msg.altitude, self.ref_lat, self.ref_lon, self.ref_alt, deg=True)
        )
        if np.any(np.isnan(cartesian)):
            return

        # ignore Z
        cartesian[2] = 0
        # TODO: locks?
        # self.covariance = msg.position_covariance
        self.pose = SE3(position=cartesian, rotation=self.pose.rotation)
        self.publish_pose()

    def imu_callback(self, msg: ImuAndMag):
        """
        Callback function that receives IMU messages, updates the rover pose,
        and publishes it to the TF tree.

        :param msg: The Imu message containing IMU data that was just received
        """
        # convert ROS msg quaternion to numpy array
        imu_quat = np.array(
            [msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w]
        )

        # normalize to avoid rounding errors
        imu_quat = imu_quat / np.linalg.norm(imu_quat)

        # get a quaternion to rotate about the Z axis by 90 degrees
        offset_quat = quaternion_about_axis(np.pi / 2, np.array([0, 0, 1]))

        # rotate the IMU quaternion by the offset to convert it to the ENU frame
        enu_quat = quaternion_multiply(offset_quat, imu_quat)
        self.pose = SE3.from_pos_quat(position=self.pose.position, quaternion=enu_quat)
        self.publish_pose()


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    GPSLinearization()
    rospy.spin()


if __name__ == "__main__":
    main()
