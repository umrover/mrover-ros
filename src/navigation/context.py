import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import Course
from util.SE3 import SE3
from visualization_msgs.msg import Marker


class Context:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    course_listener: rospy.Subscriber

    course: Course = None

    def course_callback(self, course: Course):
        self.course = course

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('nav_vis', Marker)
        self.course_listener = rospy.Subscriber('course', Course, self.course_callback)

    def drive_command(self, twist: Twist):
        self.vel_cmd_publisher.publish(twist)

    def drive_stop(self):
        self.drive_command(Twist())

    def get_rover_pose(self) -> SE3:
        return self.get_transform('base_link')

    def get_transform(self, frame: str, parent_frame: str = 'odom') -> SE3:
        """
        :param frame:
        :param parent_frame:
        :return:
        """
        # TODO: use SE3 function to lookup
        stamped_transform = self.tf_buffer.lookup_transform(parent_frame, frame, rospy.Time(0))
        return SE3.from_tf(stamped_transform.transform)
