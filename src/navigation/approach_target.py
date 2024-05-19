from typing import Optional

import numpy as np

import rospy
from navigation import search, waypoint, state, recovery
from navigation.context import Context
from util.state_lib.state import State

STOP_THRESHOLD = rospy.get_param("single_tag/stop_threshold")
STOP_THRESH_WAYPOINT = rospy.get_param("waypoint/stop_threshold")
TAG_STOP_THRESHOLD = rospy.get_param("single_tag/tag_stop_threshold")
DRIVE_FORWARD_THRESHOLD = rospy.get_param("waypoint/drive_forward_threshold")


class ApproachTargetState(State):
    def on_enter(self, context: Context) -> None:
        pass

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> Optional[np.ndarray]:
        tag_position = context.env.current_target_pos()
        return tag_position

    def determine_next(self, context: Context, is_finished: bool) -> State:
        if is_finished:
            return state.DoneState()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return self

    def on_loop(self, context: Context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_pos().
        Return to search if there is no target position.
        :return: Next state
        """
        assert context.course is not None

        target_pos = self.get_target_position(context)
        if target_pos is None:
            # TODO(quintin): Clean this up
            if self.__class__.__name__ == "LongRangeState" and not context.env.arrived_at_waypoint:
                return waypoint.WaypointState()
            return search.SearchState()

        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            STOP_THRESHOLD,
            DRIVE_FORWARD_THRESHOLD,
        )
        next_state = self.determine_next(context, arrived)
        if arrived:
            context.env.arrived_at_target = True
            context.env.last_target_location = self.get_target_position(context)
            context.course.increment_waypoint()
        else:
            context.rover.send_drive_command(cmd_vel)

        return next_state
