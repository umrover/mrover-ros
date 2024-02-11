import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State

from navigation import search, recovery, approach_post, post_backup, state, approach_object, long_range


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
        if context.env.arrived_at_target:
            context.env.arrived_at_target = False
            return post_backup.PostBackupState()

        # returns either ApproachPostState, LongRangeState, ApproachObjectState, or None
        if context.course.check_approach() is not None:
            return context.course.check_approach()

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
                if not context.course.look_for_post() and not context.course.look_for_object():
                    # We finished a regular waypoint, go onto the next one
                    context.course.increment_waypoint()
                elif context.course.look_for_post() or context.course.look_for_object():
                    # We finished a waypoint associated with a post or mallet, but we have not seen it yet.
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
