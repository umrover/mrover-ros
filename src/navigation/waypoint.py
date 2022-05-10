from common import Context, BaseState


class WaypointState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['waypoint', 'waypoint_done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        next_waypoint = self.transform('/<target>')
        if next_waypoint:
            self.context.vel_cmd_publisher.publish()
        return 'waypoint'
