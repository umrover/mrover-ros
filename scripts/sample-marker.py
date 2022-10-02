#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from visualization_msgs.msg import Marker
 
def talker():
    
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    marker = Marker()
    marker.header.frame_id = 1
    marker.id = 2
    marker.type = 2
    marker.action = 2
    marker.pose = Pose()
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.0
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.frame_locked = False
    marker.ns = "random ns"

    pub = rospy.Publisher("marker", Marker, queue_size=10)

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        marker.ns = hello_str
        rospy.loginfo(hello_str)
        pub.publish(marker)
        rate.sleep()
 
    if __name__ == '__main__':
        try:
            talker()
        except rospy.ROSInterruptException:
            pass