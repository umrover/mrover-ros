import tf2_ros
import rospy
from aenum import Enum, NoAlias
from geometry_msgs.msg import Twist
from util.ros_utils import get_rosparam
from context import Context
from util.state_lib.state import State
from util.state_lib.state_machine import StateMachine
from util.np_utils import normalize
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
import math


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
        tag_id = context.course.get_current_waypoint().tag_id
        tag = context.env.long_range_tags.get(tag_id)
        if tag is None:
            return None
        pose = context.rover.get_pose()
        if pose is None:
            return None

        rover_position = pose.position
        rover_direction = pose.rotation.direction_vector()

        bearing_to_tag = np.radians(tag.bearing)

        # rover_direction rotated by bearing to tag
        bearing_rotation_mat = np.array(
            [[np.cos(bearing_to_tag), -np.sin(bearing_to_tag)], [np.sin(bearing_to_tag), np.cos(bearing_to_tag)]]
        )

        direction_to_tag = bearing_rotation_mat @ rover_direction[:2]

        direction_to_tag = normalize(direction_to_tag)
        distance = 20  # TODO replace with rosparam
        tag_position = rover_position + direction_to_tag * distance

    def determine_next(self, context, finished: bool) -> State:
        fid_pos = context.env.current_fid_pos()
        if fid_pos is None:
            return state.LongRangeState()
        else:
            return approach_post.ApproachPostState()
