from state import BaseState


class TagSeekState(BaseState):
	def __init__(self, context: Context):
        super().__init__(
            context,
            add_outcomes=["TODO: add the outcomes of the TagSeekState to this list"],
        )

    def evaluate(self, ud):
        #TODO: get the tag's location and properties


        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')

        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')

        #TODO: figure out the twist command to be applied to move rover to tag

        #TODO: send Twist command to rover

        #TODO: stay in the TagSeekState (with outcome 'working')