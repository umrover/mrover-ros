import smach
from typing import List
from context import Context
from aenum import Enum, NoAlias
from state import BaseState
from drive import get_j_turn_command
from geometry_msgs.msg import Twist
import rospy
import numpy as np
from typing import Optional
from util.np_utils import perpendicular_2d


STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.34  # 20 degrees


class RecoveryStateTransitions(Enum):
    _settings_ = NoAlias
    continue_waypoint_traverse = "WaypointState"
    continue_gate_traverse = "GateTraverseState"
    continue_search = "SearchState"
    continue_recovery = "RecoveryState"
    recovery_state = "RecoveryState"
    partial_gate = "PartialGateState"


class JTurnAction(Enum):
    moving_back = 0
    j_turning = 1
    

class RecoveryState(BaseState):
    # waypoint_calculated: bool
    waypoint_behind: Optional[np.array]
    current_action: JTurnAction
    arrived: bool

    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=[transition.name for transition in RecoveryStateTransitions])
        self.waypoint_calculated = False
        self.waypoint_behind = None
        self.current_action = JTurnAction.moving_back
        self.arrived = False
    
    def evaluate(self, ud) -> str:
        # rospy.logerr(f"Executing J-Turn\n")
        # rospy.logerr(f"Driving Backwards First Time\n")

        # Making waypoint behind the rover to go backwards
        pose = self.context.rover.get_pose()
        # if first round
        if self.current_action == JTurnAction.moving_back:
            # rospy.logerr("EVALUATE")
            if self.waypoint_behind is None:
                # rospy.logerr(f"POSE{pose}")
                dir_vector = -2 * pose.rotation.direction_vector()
                self.waypoint_behind = pose.position + dir_vector
                # self.waypoint_calculated = True

            cmd_vel, self.arrived = get_j_turn_command(self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
            # rospy.logerr(f"Finished Get Drive Command\n")
            self.context.rover.send_drive_command(cmd_vel)

        if self.arrived:
            self.current_action = JTurnAction.j_turning # move to second part of turn
            # self.waypoint_calculated = False
            self.arrived = False
            self.waypoint_behind = None

        # if second round
        if self.current_action == JTurnAction.j_turning:
            if self.waypoint_behind is None:
                # rospy.logerr(f"POSE{pose}")
                dir_vector = pose.rotation.direction_vector()
                dir_vector_rot = 2 * perpendicular_2d(dir_vector[:2])
                dir_vector = np.append(dir_vector_rot, dir_vector[2])
                self.waypoint_behind = pose.position + dir_vector
                # self.waypoint_calculated = True
            
            cmd_vel, self.arrived = get_j_turn_command(self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
            self.context.rover.send_drive_command(cmd_vel)

        #set stuck to False
        # rospy.logerr(f"{self.context.rover.previous_state}\n")
        # return self.context.rover.previous_state
        if self.arrived:
            self.context.rover.stuck = False # change to subscriber
            # self.waypoint_calculated = False
            self.waypoint_behind = None
            self.current_action = JTurnAction.moving_back
            self.arrived = False
            return self.context.rover.previous_state
        
        return RecoveryStateTransitions.continue_recovery.name