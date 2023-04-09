#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from util.np_utils import numpify
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf2_ros
import numpy as np


class PassthroughFilter:
    """
    This class takes the place of the EKF when we want to use raw GPS and IMU
    pose data. It takes in a transform from the world frame to the rover frame
    and manages the map->odom->baselink transform structure.
    """

    # TF infrastructure
    tf_broadcaster: tf2_ros.TransformBroadcaster
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener

    # frame/publishing configuration
    use_odom: bool
    world_frame: str
    odom_frame: str
    rover_frame: str

    def __init__(self):
        # read required parameters, if they don't exist an error will be thrown
        self.use_odom = rospy.get_param("use_odom_frame")
        self.world_frame = rospy.get_param("world_frame")
        self.odom_frame = rospy.get_param("odom_frame")
        self.rover_frame = rospy.get_param("rover_frame")

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("linearized_pose", PoseWithCovarianceStamped, self.pose_callback)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Publishes the pose of the rover relative to the map frame to the TF tree.
        The pose will be published either as a direct map->base_link transform,
        or as an indirect map-odom transform if the odom frame is in use.
        See the wiki for more details:
        https://github.com/umrover/mrover-ros/wiki/Localization#guide-to-localization-frames
        """
        rover_in_map = SE3.from_pos_quat(numpify(msg.pose.pose.position), numpify(msg.pose.pose.orientation))
        if self.use_odom:
            # Get the odom to rover transform from the TF tree
            try:
                rover_in_odom = SE3.from_tf_tree(self.tf_buffer, self.odom_frame, self.rover_frame)

            # don't do anything if you can't find that transform
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ):
                rospy.logerr(f"Could not find transform from {self.odom_frame} frame to {self.rover_frame} frame")
                return

            rover_to_odom = rover_in_odom.transform_matrix()
            rover_to_map = rover_in_map.transform_matrix()

            # Calculate the intermediate transform from the overall transform and odom to rover
            odom_to_map = rover_to_map @ np.linalg.inv(rover_to_odom)
            odom_in_map = SE3.from_transform_matrix(odom_to_map)
            pose_out = odom_in_map
            child_frame = self.odom_frame
        else:
            # publish directly as map->base_link
            pose_out = rover_in_map
            child_frame = self.rover_frame

        pose_out.publish_to_tf_tree(self.tf_broadcaster, self.world_frame, child_frame)


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("gps_linearization")
    PassthroughFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
