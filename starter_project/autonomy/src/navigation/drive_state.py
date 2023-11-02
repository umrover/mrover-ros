import numpy as np

from context import Context
from drive import get_drive_command
from state import BaseState


class DriveState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO:
            add_outcomes=["reached_point", "driving_to_point"],
        )

    def evaluate(self, ud):
        target = np.array([5.5, 2.0, 0.0])

        # get the rover's pose, if it doesn't exist stay in DriveState (with outcome "driving_to_point")
        pose = Context.rover.get_pose()

        # get the drive command and completion status based on target and pose
        # (HINT: use get_drive_command(), with completion_thresh set to 0.7 and turn_in_place_thresh set to 0.2)
        status = get_drive_command(target, pose, 0.7, 0.2)

        # if we are finished getting to the target, go to TagSeekState (with outcome "reached_point")
        if status[1]:
            return "reached_point"

        else:
            Context.rover.send_drive_command(status[0])
            return "driving_to_point"

        # send the drive command to the rover

        # tell smach to stay in the DriveState by returning with outcome "driving_to_point"
