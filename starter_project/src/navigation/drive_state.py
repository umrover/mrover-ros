from multiprocessing import context
from state import BaseState
import numpy as np
from context import Context
from drive import get_drive_command

class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO:
            add_outcomes=["TODO: add outcomes here"],
        )

    def evaluate(self, ud):
        target = np.array([3.0, 3.0, 0.0])
        #TODO: get the rovers pose, if it doesn't exist stay in DriveState with outcome "driving_to_point"

        #TODO: get the drive command (and completion status) based on target and pose (HINT: use get_drive_command())

        #TODO: if we are finished getting to the target, return with outcome "reached_point"

        #TODO: send the drive command to the rover

        #TODO: tell smach to stay in the DriveState by returning with outcome "driving_to_point"

        pass
