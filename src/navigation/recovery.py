from enum import Enum
from typing import Optional

import numpy as np

import rospy
from navigation.context import Context
from util.np_utils import rotate_2d
from util.state_lib.state import State

STOP_THRESH = rospy.get_param("recovery/stop_threshold")
DRIVE_FWD_THRESH = rospy.get_param("recovery/drive_forward_threshold")
RECOVERY_DISTANCE = rospy.get_param("recovery/recovery_distance")
GIVE_UP_TIME = rospy.get_param("recovery/give_up_time")


class JTurnAction(Enum):
    MOVING_BACK = 0
    J_TURNING = 1


class RecoveryState(State):
    waypoint_behind: Optional[np.ndarray]
    current_action: JTurnAction
    start_time: Optional[rospy.Time] = None
    waypoint_calculated: bool

    def reset(self, context: Context) -> None:
        self.waypoint_calculated = False
        self.waypoint_behind = None
        self.current_action = JTurnAction.MOVING_BACK
        context.rover.stuck = False
        self.start_time = None

    def on_enter(self, context: Context) -> None:
        self.reset(context)
        self.start_time = rospy.Time.now()

    def on_exit(self, context: Context) -> None:
        self.reset(context)

    def on_loop(self, context: Context) -> State:
        if rospy.Time.now() - self.start_time > rospy.Duration(GIVE_UP_TIME):
            return context.rover.previous_state

        # Making waypoint behind the rover to go backwards
        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        # If first round, set a waypoint directly behind the rover and command it to
        # drive backwards toward it until it arrives at that point.
        if self.current_action == JTurnAction.MOVING_BACK:
            # Only set waypoint_behind once so that it doesn't get overwritten and moved
            # further back every iteration
            if self.waypoint_behind is None:
                dir_vector = -1 * RECOVERY_DISTANCE * rover_in_map.rotation.direction_vector()
                self.waypoint_behind = rover_in_map.position + dir_vector

            cmd_vel, arrived_back = context.rover.driver.get_drive_command(
                self.waypoint_behind, rover_in_map, STOP_THRESH, DRIVE_FWD_THRESH, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            if arrived_back:
                self.current_action = JTurnAction.J_TURNING  # move to second part of turn
                self.waypoint_behind = None
                context.rover.driver.reset()

        # If second round, set a waypoint off to the side of the rover and command it to
        # turn and drive backwards towards it until it arrives at that point. So it will begin
        # by turning then it will drive backwards.
        if self.current_action == JTurnAction.J_TURNING:
            if self.waypoint_behind is None:
                dir_vector = rover_in_map.rotation.direction_vector()
                # the waypoint will be 45 degrees to the left of the rover behind it.
                dir_vector[:2] = RECOVERY_DISTANCE * rotate_2d(dir_vector[:2], 3 * np.pi / 4)
                self.waypoint_behind = rover_in_map.position + dir_vector

            cmd_vel, arrived_turn = context.rover.driver.get_drive_command(
                self.waypoint_behind, rover_in_map, STOP_THRESH, DRIVE_FWD_THRESH, drive_back=True
            )
            context.rover.send_drive_command(cmd_vel)

            # set stuck to False
            if arrived_turn:
                return context.rover.previous_state

        return self
