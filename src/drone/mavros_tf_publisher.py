#!/usr/bin/env python3
import rospy
from util.SE3 import SE3
from geometry_msgs.msg import PoseStamped
import tf2_ros


class MavrosTfPublisher:
    """
    This node subscribes to MAVROS's drone pose data,
    and publishes it to the TF tree.
    """

    def __init__(self):
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.world_frame = rospy.get_param("mavros_tf_publisher/world_frame")
        self.drone_frame = rospy.get_param("mavros_tf_publisher/drone_frame")

    def pose_callback(self, msg: PoseStamped):
        """
        Callback function that receives the drone's pose from MAVROS and publishes it to the TF tree.

        :param msg: The pose message of the drone, in local coordinates
        """
        position = msg.pose.position
        position = [position.x, position.y, position.z]

        rotation = msg.pose.orientation
        rotation = [rotation.x, rotation.y, rotation.z, rotation.w]
        self.pose = SE3.from_pos_quat(position, rotation)
        self.pose.publish_to_tf_tree(self.tf_broadcaster, parent_frame=self.world_frame, child_frame=self.drone_frame)


def main():
    # start the node and spin to wait for messages to be received
    rospy.init_node("tf_pose_publisher")
    MavrosTfPublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
