import smach
from typing import List
from context import Context
from aenum import Enum, NoAlias
from state import BaseState
from geometry_msgs.msg import Twist
import rospy
import numpy as np
from typing import Optional
from util.np_utils import perpendicular_2d
from util.ros_utils import get_rosparam


STOP_THRESH = get_rosparam("recovery/stop_thresh", 0.2)
DRIVE_FWD_THRESH = get_rosparam("recovery/drive_fwd_thresh", 0.34)  # 20 degrees
RECOVERY_DISTANCE = get_rosparam("recovery/recovery_distance", 1.0)


class RecoveryStateTransitions(Enum):
    _settings_ = NoAlias
    continue_waypoint_traverse = "WaypointState"
    continue_gate_traverse = "GateTraverseState"
    continue_search = "SearchState"
    continue_recovery = "RecoveryState"
    recovery_state = "RecoveryState"
    partial_gate = "PartialGateState"


class JTurnAction(Enum):
    moving_back: Enum = 0
    j_turning: Enum = 1


class RecoveryState(BaseState):
    waypoint_behind: Optional[np.ndarray]
    current_action: JTurnAction

    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=[transition.name for transition in RecoveryStateTransitions])  # type: ignore
        self.waypoint_calculated = False
        self.waypoint_behind = None
        self.current_action = JTurnAction.moving_back

    def evaluate(self, ud) -> str:
        # Making waypoint behind the rover to go backwards
        pose = self.context.rover.get_pose()
        # if first round, set a waypoint directly behind the rover and command it to
        # drive backwards toward it until it arrives at that point.
        if self.current_action == JTurnAction.moving_back:
            # Only set waypoint_behind once so that it doesn't get overwritten and moved
            # further back every iteration
            if self.waypoint_behind is None:
                dir_vector = -1 * RECOVERY_DISTANCE * pose.rotation.direction_vector()
                self.waypoint_behind = pose.position + dir_vector

            cmd_vel, arrived_back = self.context.rover.driver.get_drive_command(
                self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH, True
            )
            self.context.rover.send_drive_command(cmd_vel)

            if arrived_back:
                self.current_action = JTurnAction.j_turning  # move to second part of turn
                self.waypoint_behind = None

        # if second round, set a waypoint off to the side of the rover and command it to
        # turn and drive backwards towards it until it arrives at that point. So it will begin
        # by turning then it will drive backwards.
        if self.current_action == JTurnAction.j_turning:
            if self.waypoint_behind is None:
                dir_vector = pose.rotation.direction_vector()
                dir_vector[:2] = RECOVERY_DISTANCE * perpendicular_2d(dir_vector[:2])
                self.waypoint_behind = pose.position + dir_vector

            cmd_vel, arrived_turn = self.context.rover.driver.get_drive_command(
                self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH, True
            )
            self.context.rover.send_drive_command(cmd_vel)

            # set stuck to False
            if arrived_turn:
                self.context.rover.stuck = False  # change to subscriber
                self.waypoint_behind = None
                self.current_action = JTurnAction.moving_back
                return self.context.rover.previous_state

        return RecoveryStateTransitions.continue_recovery.name  # type: ignore
