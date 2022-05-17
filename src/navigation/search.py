from common import BaseState, Context


class SearchState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['waypoint_traverse', 'waypoint_done'],
            input_keys=['waypoint_index', 'waypoints'],
            output_keys=['waypoint_index']
        )

    def evaluate(self, ud):
        pass
