import smach
from typing import List
from context import Context, Environment
from aenum import Enum, NoAlias
from state import BaseState

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
        #get_drive_command() to go backwards
        #send_drive_command()

        #get_drive_command() to go turn
        #send_drive_command()

        #get_drive_command() to go backwards
        #send_drive_command()

        #set stuck to False
        return self.context.rover.previous_state