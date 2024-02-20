import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State

from navigation import search, recovery, approach_post, post_backup, state, water_bottle_search
from mrover.msg import WaypointType


class WaypointState(State):
    STOP_THRESH = get_rosparam("waypoint/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees
    NO_FIDUCIAL = get_rosparam("waypoint/no_fiducial", -1)

    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        """
        Handle driving to a waypoint defined by a linearized cartesian position.
        If the waypoint is associated with a tag id, go into that state early if we see it,
        otherwise wait until we get there to conduct a more thorough search.
        :param ud:  State machine user data
        :return:    Next state
        """
        current_waypoint = context.course.current_waypoint()
        if current_waypoint is None:
            return state.DoneState()

        # if we are at a post currently (from a previous leg), backup to avoid collision
        if context.env.arrived_at_post:
            context.env.arrived_at_post = False
            return post_backup.PostBackupState()

        if context.course.look_for_post():
            if context.env.current_tag_pos() is not None:
                return approach_post.ApproachPostState()

        # Attempt to find the waypoint in the TF tree and drive to it
        try:
            waypoint_pos = context.course.current_waypoint_pose().position
            cmd_vel, arrived = context.rover.driver.get_drive_command(
                waypoint_pos,
                context.rover.get_pose(),
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
            )
            if arrived:
                if current_waypoint.type.val == WaypointType.NO_SEARCH:
                    # We finished a regular waypoint, go onto the next one
                    context.course.increment_waypoint()
                elif current_waypoint.type.val == WaypointType.WATER_BOTTLE:
                    # We finished a waypoint associated with the water bottle, but we have not seen it yet
                    return water_bottle_search.WaterBottleSearchState()
                else:
                    # We finished a waypoint associated with a tag id or the mallet, but we have not seen it yet.
                    return search.SearchState()

            if context.rover.stuck:
                context.rover.previous_state = self
                return recovery.RecoveryState()

            context.rover.send_drive_command(cmd_vel)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

        return self
