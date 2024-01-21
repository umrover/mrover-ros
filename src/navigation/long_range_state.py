import tf2_ros
import rospy
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist
from util.ros_utils import get_rosparam
from context import Context
from util.state_lib.state import State
from util.state_lib.state_machine import StateMachine
from abc import ABC, abstractmethod
from state import DoneState
from search import SearchState
from util.state_lib.state_publisher_server import StatePublisher
from typing import Optional
from util.state_lib.state import State
from abc import abstractmethod
import state
from state import ApproachTargetBaseState
from state import approach_post
import numpy as np

TAG_SEARCH_EXPIRATION = 3


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
        tag_id = context.current_waypoint()["tagid"]
        bearing = context.env.tag_data_dict[tag_id][2]
        rover_position = context.rover.get_pose().position
        target_pos_x = 90 - bearing
        target_pos_y = bearing
        target_pos_z = rover_position[2]
        target_coordinates = np.ndarray
        context.tag_data_callback()
        return target_pos

    def determine_next(self, context, finished: bool) -> State:
        fid_pos = context.env.current_fid_pos()  # Need to pass in the fid_id
        tag_time = context.env.tag_data_dict[context.current_tag_id][0]
        if fid_pos is None:
            if rospy.Time() - tag_time > TAG_SEARCH_EXPIRATION:
                return state.SearchState()
            else:
                return state.LongRangeState()
        else:
            return approach_post.ApproachPostState()
