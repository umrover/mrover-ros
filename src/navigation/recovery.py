import smach
from typing import List
from context import Context, Rover, Environment
from aenum import Enum, NoAlias
from state import BaseState
from drive import get_drive_command
from geometry_msgs.msg import Twist
import rospy
import numpy as np


STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.34  # 20 degrees


class RecoveryStateTransitions(Enum):
    _settings_ = NoAlias
    continue_waypoint_traverse = "WaypointState"
    continue_gate_traverse = "GateTraverseState"
    continue_search = "SearchState"
    continue_fiducial_id = "SingleFiducialState"
    recovery_state = "RecoveryState"

class RecoveryState(BaseState):
    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=[transition.name for transition in RecoveryStateTransitions])
        self.waypoint_calculated = False
        self.waypoint_behind = np.array([0,0,0])

    def rotate_by_90(self, v):
        x = v[0] * -1.0
        y = v[1]
        return np.array([y, x, v[2]])

    
    def evaluate(self, ud) -> str:
        rospy.logerr(f"Executing J-Turn\n")

        rospy.logerr(f"Driving Backwards First Time\n")
        #Making waypoint behind the rover to go backwards
        pose = self.context.rover.get_pose()
        arrived = False
        # if first round
        if Rover.move_back:
            rospy.logerr("EVALUATE")
            if not self.waypoint_calculated:
                rospy.logerr(f"POSE{pose}")
                dir_vector = -2 * pose.rotation.direction_vector()
                self.waypoint_behind = pose.position + dir_vector
                self.waypoint_calculated = True
            cmd_vel, arrived, stuck = get_drive_command(self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH, self.context.rover)
            rospy.logerr(f"Finished Get Drive Command\n")
            self.context.rover.send_drive_command(cmd_vel)

        if arrived:
            Rover.move_back = False # move to second part of turn
            self.waypoint_calculated = False
            arrived = False

        # if second round
        if not Rover.move_back:
            if not self.waypoint_calculated:
                rospy.logerr(f"POSE{pose}")
                dir_vector = 2 * self.rotate_by_90(pose.rotation.direction_vector())
                self.waypoint_behind = pose.position + dir_vector
                self.waypoint_calculated = True
            cmd_vel, arrived, stuck = get_drive_command(self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
            self.context.rover.send_drive_command(cmd_vel)

        #set stuck to False
        rospy.logerr(f"{self.context.rover.previous_state}\n")
        #return self.context.rover.previous_state
        if arrived:
            self.context.rover.stuck = False
        return RecoveryStateTransitions.continue_waypoint_traverse.name