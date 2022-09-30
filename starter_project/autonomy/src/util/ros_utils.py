import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def send_debug_arrow(self, rot):
    # TODO: not working
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.header.stamp = rospy.Time.now()
    marker.id = 0
    marker.action = Marker.ADD
    marker.type = Marker.ARROW
    marker.pose.orientation.x = rot[0]
    marker.pose.orientation.y = rot[1]
    marker.pose.orientation.z = rot[2]
    marker.pose.orientation.w = rot[3]
    marker.scale = 2.0
    marker.color.r = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.points = [Point(0, 0, 0), Point(2, 0, 0)]
    self.context.vis_publisher.publish(marker)
