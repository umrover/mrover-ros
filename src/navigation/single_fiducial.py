import tf2_ros
from context import Context
from drive import get_drive_command
from geometry_msgs.msg import Twist
from waypoint import DRIVE_FWD_THRESH, BaseWaypointState

FIDUCIAL_STOP_THRESHOLD = 1.75


class SingleFiducialState(BaseWaypointState):
    def __init__(self, context: Context):
        super().__init__(context, outcomes=[], input_keys=['fiducial_id'], output_keys=[])

    def evaluate(self, ud) -> str:
        fid_pos = self.current_fid_pos(ud)
        if fid_pos is None:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.5
            self.context.vel_cmd_publisher.publish(cmd_vel)
            return 'single_fiducial'

        try:
            cmd_vel, arrived = get_drive_command(fid_pos, self.rover_pose(), 0.7, DRIVE_FWD_THRESH)
            if arrived:
                ud.waypoint_index += 1
                return 'waypoint_traverse'
            self.context.vel_cmd_publisher.publish(cmd_vel)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # TODO: probably go into some waiting state
            pass

        return 'single_fiducial'
