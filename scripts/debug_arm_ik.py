#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Quaternion, Point, PointStamped
from mrover.msg import IK

if __name__ == "__main__":
    rospy.init_node("debug_arm_ik")

    pub = rospy.Publisher("arm_ik", IK, queue_size=1)

    rospy.sleep(1.0)

    pub.publish(
        IK(
            pose=Pose(
                position=Point(x=0.5, y=0.5, z=0.5),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
    )

    def on_clicked_point(clicked_point: PointStamped):
        ik = IK(
            pose=Pose(
                position=clicked_point.point,
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        pub.publish(ik)

    sub = rospy.Subscriber("clicked_point", PointStamped, on_clicked_point)

    rospy.spin()
