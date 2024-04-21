from geometry_msgs.msg import Twist

from context import Context
from state import BaseState


class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            # TODO: add outcomes
            add_outcomes=["failure", "working", "success"],
        )

    def evaluate(self, ud):
        DISTANCE_TOLERANCE = 0.99
        ANUGLAR_TOLERANCE = 0.3
        # TODO: get the tag's location and properties (HINT: use get_fid_data() from context.env)
        tag = self.context.env.get_fid_data()

        # TODO: if we don't have a tag: go to the DoneState (with outcome "failure")
        if tag is None:
            return "failure"

        # TODO: if we are within angular and distance tolerances: go to DoneState (with outcome "success")
        if tag.closenessMetric < DISTANCE_TOLERANCE and abs(tag.xTagCenterPixel) < ANUGLAR_TOLERANCE:
            return "success"

        # TODO: figure out the Twist command to be applied to move the rover closer to the tag
        twist = Twist()
        if tag.closenessMetric >= DISTANCE_TOLERANCE:
            twist.linear.x = 1
        if abs(tag.closenessMetric) >= ANUGLAR_TOLERANCE:
            twist.angular.z = tag.xTagCenterPixel

        # TODO: send Twist command to rover
        self.context.rover.send_drive_command(twist)

        # TODO: stay in the TagSeekState (with outcome "working")
        return "working"

        pass
