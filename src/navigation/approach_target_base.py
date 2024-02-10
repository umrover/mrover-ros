import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State
from abc import abstractmethod
from typing import Optional
import numpy as np

from navigation import search


class ApproachTargetBaseState(State):
    STOP_THRESH = get_rosparam("single_fiducial/stop_thresh", 0.7)
    FIDUCIAL_STOP_THRESHOLD = get_rosparam("single_fiducial/fiducial_stop_threshold", 1.75)
    DRIVE_FWD_THRESH = get_rosparam("waypoint/drive_fwd_thresh", 0.34)  # 20 degrees

    def on_enter(self, context) -> None:
        pass

    def on_exit(self, context) -> None:
        pass

    @abstractmethod
    def get_target_pos(self, context) -> Optional[np.ndarray]:
        raise NotImplementedError

    @abstractmethod
    def determine_next(self, context, finished: bool) -> State:
        raise NotImplementedError

    def on_loop(self, context) -> State:
        """
        Drive towards a target based on what gets returned from get_target_pos().
        Return to search if there is no target position.
        :param context: rover context
        :return:    Next state
        """
        target_pos = self.get_target_pos(context)
        if target_pos is None:
            return search.SearchState()

        try:
            cmd_vel, arrived = context.rover.driver.get_drive_command(
                target_pos,
                context.rover.get_pose(in_odom_frame=True),
                self.STOP_THRESH,
                self.DRIVE_FWD_THRESH,
                in_odom=context.use_odom,
            )
            next_state = self.determine_next(context, arrived)
            if arrived:
                context.env.arrived_at_target = True
                context.env.last_target_location = self.get_target_pos(context)
                context.course.increment_waypoint()
            else:
                context.rover.send_drive_command(cmd_vel)

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            pass

        return next_state
