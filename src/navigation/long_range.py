import tf2_ros
import rospy

from util.ros_utils import get_rosparam
from util.state_lib.state import State
from util.np_utils import normalized
from typing import Optional

from navigation import approach_post, recovery
from navigation.approach_target_base import ApproachTargetBaseState
import numpy as np

DIST_AHEAD = get_rosparam("long_range/distance_ahead", 20)


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

    def get_target_pos(self, context) -> Optional[np.ndarray]:
        tag_id = context.course.current_waypoint().tag_id
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

        direction_to_tag = normalized(direction_to_tag)
        distance = DIST_AHEAD
        direction_to_tag = np.array([direction_to_tag[0], direction_to_tag[1], 0.0])
        tag_position = rover_position + direction_to_tag * distance
        return tag_position

    def determine_next(self, context, finished: bool) -> State:
        fid_pos = context.env.current_target_pos()
        if fid_pos is None:
            return self
        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return approach_post.ApproachPostState()
