import tf2_ros
from context import Context
from drive import get_drive_command
from geometry_msgs.msg import Twist
from waypoint import DRIVE_FWD_THRESH, WaypointState

STOP_THRESH = 0.7
FIDUCIAL_STOP_THRESHOLD = 1.75


class SingleFiducialState(WaypointState):
    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=["waypoint_traverse", "single_fiducial", "search"])

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
            # TODO: add more advanced logic than just driving forward
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            self.context.rover.send_drive_command(cmd_vel)
            return "search"

        try:
            cmd_vel, arrived = get_drive_command(fid_pos, self.context.rover.get_pose(), STOP_THRESH, DRIVE_FWD_THRESH)
            if arrived:
                self.context.course.increment_waypoint()
                return "waypoint_traverse"
            self.context.rover.send_drive_command(cmd_vel)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # TODO: probably go into some waiting state
            pass

        return "single_fiducial"
