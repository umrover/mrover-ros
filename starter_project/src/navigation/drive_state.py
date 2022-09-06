from state import BaseState
import numpy as np
from context import Context

class DriveState(BaseState):
	def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["TODO: add the outcomes of the DriveState to this list"],
        )

    def evaluate(self, ud):
       #TODO get the rovers pose and 

       goal = np.array([5.0, 5.0])

       #TODO: calculate the vector towards the goal

       #TODO: if we are close enough to the goal (say 0.5 meters, return outcome "reached_point")

       #TODO: calculate the turning effort

       #TODO: calculate the driving effort

       #TODO: create twist message

       #TODO: send twist message

       #TODO: return outcome "driving_to_point"
