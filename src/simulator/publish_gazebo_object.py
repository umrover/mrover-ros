#!/usr/bin/env python3

import rospy
import numpy as np
import tf2_ros
from gazebo_msgs.msg import ModelStates
from util.SE3 import SE3


class GazeboObject:
    def __init__(self):
        rospy.Subscriber("gazebo/model_states", ModelStates, self.state_callback)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.obj_to_detect = rospy.get_param("publish_object/object_name")
        # To prevent tf_repeated_data warning
        self.ts = rospy.Time.now()

    def state_callback(self, msg: ModelStates):
        try:
            msg_time = rospy.Time.now()
            if self.ts != msg_time:
                self.ts = msg_time
                obj_index = msg.name.index(self.obj_to_detect)
                msg_pos = msg.pose[obj_index]
                pos = np.array([msg_pos.position.x, msg_pos.position.y, msg_pos.position.z])
                rot = np.array(
                    [msg_pos.orientation.x, msg_pos.orientation.y, msg_pos.orientation.z, msg_pos.orientation.w]
                )
                obj_pose = SE3.from_pos_quat(pos, rot)
                obj_pose.publish_to_tf_tree(self.tf_broadcaster, "map", "object_link")
        except ValueError:
            rospy.logerr("object not in sim world")
            return


def main():
    rospy.init_node("publish_gazebo_object")
    gazebo_object = GazeboObject()
    rospy.spin()


if __name__ == "__main__":
    main()
