from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO: add outcomes
            add_outcomes=["TODO: add outcomes here"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)

        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag

        # TODO: send Twist command to rover

        # TODO: stay in the TagSeekState (with outcome "working")

        pass
