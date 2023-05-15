import tf2_ros
import rospy
from context import Context
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist
from util.ros_utils import get_rosparam
from state import BaseState


class ApproachPostStateTransitions(Enum):
    _settings_ = NoAlias

    finished_fiducial = "WaypointState"
    continue_fiducial_id = "ApproachPostState"
    no_fiducial = "SearchState"
    recovery_state = "RecoveryState"


class ApproachPostState(BaseState):
    STOP_THRESH = get_rosparam("single_fiducial/stop_thresh", 0.7)
    FIDUCIAL_STOP_THRESHOLD = get_rosparam("single_fiducial/fiducial_stop_threshold", 1.75)
    DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees

    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=[transition.name for transition in ApproachPostStateTransitions])  # type: ignore

    def evaluate(self, ud) -> str:
        """
        Drive towards a single fiducial if we see it and stop within a certain threshold if we see it.
        Else conduct a search to find it.
        :param ud:  State machine user data
        :return:    Next state
        """
        fid_pos = self.context.env.current_fid_pos()
        if fid_pos is None:
            # We have arrived at the waypoint where the fiducial should be but we have not seen it yet
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            self.context.rover.send_drive_command(cmd_vel)
            return ApproachPostStateTransitions.no_fiducial.name  # type: ignore

        try:
            cmd_vel, arrived = self.context.rover.driver.get_drive_command(
                fid_pos,
                self.context.rover.get_pose(in_odom_frame=True),
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
                in_odom=self.context.use_odom,
            )
            if arrived:
                self.context.course.increment_waypoint()
                self.context.env.arrived_at_post = True
                self.context.env.last_post_location = fid_pos
                return ApproachPostStateTransitions.finished_fiducial.name  # type: ignore
            self.context.rover.send_drive_command(cmd_vel)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # TODO: probably go into some waiting state
            pass

        return ApproachPostStateTransitions.continue_fiducial_id.name  # type: ignore
