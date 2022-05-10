from common import Context, BaseState


class WaypointState(BaseState):
    def __init__(self, navigation: Context):
        super().__init__(
            navigation,
            outcomes=['waypoint', 'waypoint_done'],
            input_keys=[],
            output_keys=[]
        )

    def execute(self, userdata):
        next_waypoint = self.transform('/<target>')

        return 'waypoint'
