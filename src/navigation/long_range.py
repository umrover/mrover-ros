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
        target_pos = None  # TODO: get bearing from most recent message for current tag_id and create a “target” location to drive to by setting a point a certain look ahead distance far away from the rover (20-30 meters) once aligned with the bearing
        return target_pos

    def determine_next(self, context, finished: bool) -> State:
        # if we see the tag in the ZED transition to approach post
        fid_pos = context.env.current_target_pos()
        if fid_pos is None:
            # if we haven’t seen the tag in the long range cam in the last 3 seconds, go back to search 
            # TODO if time.now - last message received for tag_id > 3:
		        # return search.SearchState()
	        # otherwise continue in long range state
            return self
        else:
            return approach_post.ApproachPostState()
