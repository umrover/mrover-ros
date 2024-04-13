#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, PointStamped
from std_msgs.msg import Header
from mrover.msg import IK

if __name__ == "__main__":
    rospy.init_node("debug_arm_ik")

    pub = rospy.Publisher("arm_ik", IK, queue_size=1)

    rospy.sleep(1.0)

    pub.publish(
        IK( 
            target = PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
                pose=Pose(
                    position=Point(x=0.8, y=1.0, z=0.5),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
    )

    def on_clicked_point(clicked_point: PointStamped):
        ik = IK(
            target=PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id="base_link"),
                pose=Pose(
                    position=clicked_point.point,
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                )
            )
        )
        pub.publish(ik)

    sub = rospy.Subscriber("clicked_point", PointStamped, on_clicked_point)

    rospy.spin()
