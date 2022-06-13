from abc import ABC

import rospy
import smach
import tf2_ros
from fiducial_msgs.msg import FiducialArray
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

from util.SE3 import SE3


# TODO: rename?
class Context:
    vel_cmd_publisher: rospy.Publisher
    vis_publisher: rospy.Publisher
    fid_listener: rospy.Subscriber
    tf_buffer: tf2_ros.Buffer
    tf_listener: tf2_ros.TransformListener
    fiducial_transforms: FiducialArray

    def fiducial_transforms_callback(self, fiducial_transforms: FiducialArray):
        self.fiducial_transforms = fiducial_transforms

    def __init__(self):
        self.vel_cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.vis_publisher = rospy.Publisher('nav_vis', Marker)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.fid_listener = rospy.Subscriber('fiducial_transforms', FiducialArray, self.fiducial_transforms_callback)


class BaseState(smach.State, ABC):
    """
    Custom base state which handles termination cleanly via smach preemption.
    """
    context: Context

    def __init__(self, context: Context, *args, **kwargs):
        kwargs['outcomes'].append('terminated')
        super().__init__(*args, **kwargs)
        self.context = context

    def execute(self, ud):
        """
        Override execute method to add logic for early termination.
        Base classes should override evaluate instead of this!
        """
        if self.preempt_requested():
            self.service_preempt()
            return 'terminated'
        return self.evaluate(ud)

    def evaluate(self, ud: smach.UserData) -> str:
        """Override me instead of execute!"""
        pass

    def rover_pose(self) -> SE3:
        return self.transform('base_link')

    def transform(self, frame: str, parent_frame: str = 'odom') -> SE3:
        """
        :param frame:
        :param parent_frame:
        :return:
        """
        stamped_transform = self.context.tf_buffer.lookup_transform(parent_frame, frame, rospy.Time(0))
        return SE3.from_tf(stamped_transform.transform)


class DoneState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['done'],
            input_keys=[],
            output_keys=[]
        )

    def evaluate(self, ud):
        cmd_vel = Twist()
        self.context.vel_cmd_publisher.publish(cmd_vel)
        return 'done'
