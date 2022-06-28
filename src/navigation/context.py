import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from mrover.msg import Course
from visualization_msgs.msg import Marker


class Goal:
    course_listener: rospy.Subscriber
    course: Course = None

    def course_callback(self, course: Course):
        self.course = course

    def __init__(self):
        self.course_listener = rospy.Subscriber('course', Course, self.course_callback)


class Rover:
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('nav_vis', Marker)

    def drive_command(self, twist: Twist):
        self.vel_cmd_publisher.publish(twist)

    def drive_stop(self):
        self.drive_command(Twist())


class Environment:
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener

    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


class Context:
    goal: Goal
    rover: Rover
    environment: Environment

    def __init__(self):
        self.goal = Goal()
        self.rover = Rover()
        self.environment = Environment()
