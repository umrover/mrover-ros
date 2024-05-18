from abc import abstractmethod, ABC
from typing import Optional

import numpy as np

import rospy
from navigation import search, waypoint
from navigation.long_range import LongRangeState
from util.state_lib.state import State

STOP_THRESHOLD = rospy.get_param("single_tag/stop_threshold")
STOP_THRESH_WAYPOINT = rospy.get_param("waypoint/stop_threshold")
TAG_STOP_THRESHOLD = rospy.get_param("single_tag/tag_stop_threshold")
DRIVE_FORWARD_THRESHOLD = rospy.get_param("waypoint/drive_forward_threshold")


class ApproachTargetBaseState(State, ABC):
    @abstractmethod
    def get_target_position(self, context) -> Optional[np.ndarray]:
        raise NotImplementedError

    @abstractmethod
    def determine_next(self, context, is_finished: bool) -> State:
        raise NotImplementedError

    def on_loop(self, context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_pos().
        Return to search if there is no target position.
        :return: Next state
        """
        target_pos = self.get_target_position(context)
        if target_pos is None:
            # TODO(quintin): Clean this up
            if type(self) is LongRangeState and not context.env.arrived_at_waypoint:
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
