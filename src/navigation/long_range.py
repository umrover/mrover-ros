import tf2_ros

from util.ros_utils import get_rosparam
from util.state_lib.state import State
from abc import abstractmethod
from typing import Optional

from navigation import approach_post
from navigation.approach_target_base import ApproachTargetBaseState


class LongRangeState(ApproachTargetBaseState):
    """
    State for when the tag is seen only in the long range camera.
    Transitions:
    -If tag seen in ZED: ApproachPostState
    -Not seen in ZED and moved: LongRangeState
    -Don't see the tag in long range camera: SearchState
    -Stuck?
    """

    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def get_target_pos(self, context) -> Optional[int]:
        target_pos = None  # TODO: replace
        return target_pos

    def determine_next(self, context, finished: bool) -> State:
        fid_pos = context.env.current_fid_pos()
        if fid_pos is None:
            return self
        else:
            return approach_post.ApproachPostState()
