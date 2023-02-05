import smach
from typing import List
from context import Context, Rover, Environment
from aenum import Enum, NoAlias
from state import BaseState
from drive import get_drive_command, collector
from geometry_msgs.msg import Twist
import rospy
import numpy as np


STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.34  # 20 degrees


class RecoveryStateTransitions(Enum):
    _settings_ = NoAlias
    #waypoint
    continue_waypoint_traverse = "WaypointState"
    #gate
    continue_gate_traverse = "GateTraverseState"
    #search 
    continue_search = "SearchState"
    #single fiducial
    continue_fiducial_id = "SingleFiducialState"
    recovery_state = "RecoveryState"

class RecoveryState(BaseState):
    def __init__(self, context: Context):
        super().__init__(context, add_outcomes=[transition.name for transition in RecoveryStateTransitions])
        self.waypoint_calculated = False
        self.waypoint_behind = np.array([0,0,0])
    # def __init__(self, context: Context, 
    #                 add_outcomes: List[str] = None, 
    #                 add_input_keys: List[str] = None, 
    #                 add_output_keys: List[str] = None):
    #     super().__init__(
    #         context, 
    #         add_outcomes,
    #         add_input_keys, 
    #         add_output_keys
    #     )
    #     pass

    # def __init__(
    #     self,
    #     context: Context,
    #     add_outcomes: List[str] = None,
    #     add_input_keys: List[str] = None,
    #     add_output_keys: List[str] = None,
    # ):
    #     add_outcomes = add_outcomes or []
    #     add_input_keys = add_input_keys or []
    #     add_output_keys = add_output_keys or []
    #     super().__init__(
    #         add_outcomes + ["terminated"],
    #         add_input_keys + ["waypoint_index"],
    #         add_output_keys + ["waypoint_index"],
    #     )
    def rotate_by_90(self, v):
        x = v[0] * -1.0
        y = v[1]
        return np.array([y, x, v[2]])

    
    def evaluate(self, ud) -> str:
        rospy.logerr(f"Executing J-Turn\n")

        rospy.logerr(f"Driving Backwards First Time\n")
        #Making waypoint behind the rover to go backwards
        pose = self.context.rover.get_pose()

        # if first round
        if not collector.collector_context.rover.move_back:
            if not self.waypoint_calculated:
                rospy.logerr(f"POSE{pose}")
                dir_vector = -5 * pose.rotation.direction_vector()
                self.waypoint_behind = pose.position + dir_vector
                self.waypoint_calculated = True
            cmd_vel, arrived, stuck = get_drive_command(self.waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
            rospy.logerr(f"Finished Get Drive Command\n")
            self.context.rover.send_drive_command(cmd_vel)
            collector.collector_context.rover.move_back = True # move to get_drive_command

        # if second round
        #   
        
        # #Turn In place
        # rospy.logerr(f"Turning in place\n")
        # pose = self.context.rover.get_pose()
        # dir_vector = pose.rotation.direction_vector()
        # waypoint_rotate = pose.position + self.rotate_by_90(dir_vector)
        # cmd_vel, arrived, stuck = get_drive_command(waypoint_rotate, pose, STOP_THRESH, DRIVE_FWD_THRESH)
        # self.context.rover.send_drive_command(cmd_vel)
        # rospy.logerr(f"Finished Get Drive Command\n")
        # collector.collector_context.rover.move_back = True
        
        '''
        #Drive backward
        rospy.logerr(f"Driving Backwards Second Time\n")
        pose = self.context.rover.get_pose()
        dir_vector = -1 * pose.rotation.direction_vector()
        waypoint_behind = pose.position + dir_vector
        cmd_vel, arrived, stuck = get_drive_command(waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
        rospy.logerr(f"Finished Get Drive Command\n")
        self.context.rover.send_drive_command(cmd_vel)
        '''

        #set stuck to False
        rospy.logerr(f"{self.context.rover.previous_state}\n")
        return self.context.rover.previous_state