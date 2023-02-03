import smach
from typing import List
from context import Context, Rover, Environment
from aenum import Enum, NoAlias
from state import BaseState
from drive import get_drive_command, collector
from geometry_msgs.msg import Twist
from util.np_utils import rotate_by_90


STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.34  # 20 degrees

class RecoveryState(BaseState):
    def __init__(self, context: Context):
        super().__init__(context)
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

    def evaluate(self, ud) -> str:
          
        #Making waypoint behind the rover to go backwards
        pose = self.context.rover.get_pose()
        dir_vector = -1 * pose.rotation.direction_vector()
        waypoint_behind = pose.position + dir_vector
        cmd_vel, arrived, stuck = get_drive_command(waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
        self.context.rover.send_drive_command(cmd_vel)
        collector.collector_context.rover.move_back = False  
        
        #Turn In place
        pose = self.context.rover.get_pose()
        dir_vector = pose.rotation.direction_vector()
        waypoint_rotate = pose.position + rotate_by_90(dir_vector)
        cmd_vel, arrived, stuck = get_drive_command(waypoint_rotate, pose, STOP_THRESH, DRIVE_FWD_THRESH)
        self.context.rover.send_drive_command(cmd_vel)
        collector.collector_context.rover.move_back = True
        
        #Drive backward
        pose = self.context.rover.get_pose()
        dir_vector = -1 * pose.rotation.direction_vector()
        waypoint_behind = pose.position + dir_vector
        cmd_vel, arrived, stuck = get_drive_command(waypoint_behind, pose, STOP_THRESH, DRIVE_FWD_THRESH)
        self.context.rover.send_drive_command(cmd_vel)

        #set stuck to False
        return self.context.rover.previous_state