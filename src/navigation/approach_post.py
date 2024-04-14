import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State
from typing import Optional
import numpy as np

from navigation import state, recovery
from navigation.approach_target_base import ApproachTargetBaseState


class ApproachPostState(ApproachTargetBaseState):
    """
    State for when the tag is seen in the ZED camera.
    Transitions:
    -If arrived at target: DoneState
    -Did not arrive at target: ApproachPostState
    """

    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def get_target_pos(self, context) -> Optional[np.ndarray]:
        # return fid_pos, either position or None
        fid_pos = context.env.current_target_pos()
        return fid_pos

    def determine_next(self, context, finished: bool) -> State:
        if finished:
            return state.DoneState()
        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return self
