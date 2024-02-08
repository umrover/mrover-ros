import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State

from navigation import search, waypoint


class ApproachPostState(State):
    STOP_THRESH = get_rosparam("single_fiducial/stop_thresh", 0.7)
    FIDUCIAL_STOP_THRESHOLD = get_rosparam("single_fiducial/fiducial_stop_threshold", 1.75)
    DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees

    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        """
        Drive towards a single fiducial if we see it and stop within a certain threshold if we see it.
        Else conduct a search to find it.
        :param ud:  State machine user data
        :return:    Next state
        """
        fid_pos = context.env.current_fid_pos()
        if fid_pos is None:
            return search.SearchState()

        try:
            cmd_vel, arrived = context.rover.driver.get_drive_command(
                fid_pos,
                context.rover.get_pose(in_odom_frame=True),
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
                in_odom=context.use_odom,
            )
            if arrived:
                context.env.arrived_at_post = True
                context.env.last_post_location = context.env.current_fid_pos(odom_override=False)
                context.course.increment_waypoint()
                return waypoint.WaypointState()
            context.rover.send_drive_command(cmd_vel)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            # TODO: probably go into some waiting state
            pass

        return self
