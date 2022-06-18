import rospy
import tf2_ros
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Twist
from mrover.msg import Course
from visualization_msgs.msg import Marker


# TODO: rename?
class Context:
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    fid_listener: rospy.Subscriber
    course_listener: rospy.Subscriber
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener

    fiducial_transforms: FiducialTransformArray = None
    course: Course = None

    def course_callback(self, course: Course):
        self.course = course

    def fiducial_transforms_callback(self, fiducial_transforms: FiducialTransformArray):
        self.fiducial_transforms = fiducial_transforms

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('nav_vis', Marker)
        self.fid_listener = rospy.Subscriber('fiducial_transforms', FiducialTransformArray,
                                             self.fiducial_transforms_callback)
        self.course_listener = rospy.Subscriber('course', Course, self.course_callback)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
