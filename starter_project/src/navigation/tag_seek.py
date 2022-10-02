from state import BaseState
from context import Context
from geometry_msgs.msg import Twist

class TagSeekState(BaseState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            #TODO: add outcomes
            add_outcomes=["TODO"],
        )

    def evaluate(self, ud):
        #TODO: get the tag's location and properties
        
        #TODO: if we don't have a tag: go to the Done State (with outcome 'failure')

        #TODO: if we are within angular and distance tolerances: go to Done State (with outcome 'success')
        
        #TODO: figure out the twist command to be applied to move rover to tag

        #TODO: send Twist command to rover

        #TODO: stay in the TagSeekState (with outcome 'working')
        pass