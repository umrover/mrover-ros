from typing import Optional

import numpy as np

from navigation import state, recovery
from navigation.approach_target_base import ApproachTargetBaseState
from util.state_lib.state import State


class ApproachObjectState(ApproachTargetBaseState):
    """
    State for when we see an object (mallet or water bottle).
    We are only using the ZED camera.
    Transitions:
    -If arrived at target: DoneState
    -Did not arrive at target: ApproachObjectState
    """

    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def get_target_position(self, context) -> Optional[np.ndarray]:
        object_position = context.env.current_target_pos()
        return object_position

    def determine_next(self, context, is_finished: bool) -> State:
        if is_finished:
            return state.DoneState()

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return self
