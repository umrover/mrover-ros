from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from navigation import approach_post, recovery
from navigation.approach_target_base import ApproachTargetBaseState
from navigation.context import Context
from util.state_lib.state import State

DISTANCE_AHEAD = rospy.get_param("long_range/distance_ahead")


class LongRangeState(ApproachTargetBaseState):
    """
    State for when the tag is seen only in the long range camera.
    Transitions:
    -If tag seen in ZED: ApproachPostState
    -Not seen in ZED and moved: LongRangeState
    -Don't see the tag in long range camera: SearchState
    -Stuck?
    """

    def on_enter(self, context: Context) -> None:
        pass

    def on_exit(self, context: Context) -> None:
        pass

    def get_target_position(self, context: Context) -> Optional[np.ndarray]:
        assert context.course is not None

        current_waypoint = context.course.current_waypoint()
        assert current_waypoint is not None

        tag_id = current_waypoint.tag_id
        tag = context.env.long_range_tags.query(tag_id)
        if tag is None:
            return None

        pose = context.rover.get_pose()
        if pose is None:
            return None

        rover_position = pose.position
        rover_direction = pose.rotation.direction_vector()

        bearing_to_tag = np.radians(tag.tag.bearing)
        # If you have not seen the tag in a while but are waiting until the expiration time is up,
        # keep going towards where it was last seen (the direction you are heading), don't use an old bearing value
        if tag.hit_count <= 0:
            bearing_to_tag = 0

        bearing_rotation_mat = Rotation.from_rotvec([0, 0, bearing_to_tag]).as_matrix()

        direction_to_tag = bearing_rotation_mat[:2, :2] @ rover_direction[:2]

        distance = DISTANCE_AHEAD
        direction_to_tag = np.array([direction_to_tag[0], direction_to_tag[1], 0.0])
        tag_position = rover_position + direction_to_tag * distance
        return tag_position

    def determine_next(self, context: Context, is_finished: bool) -> State:
        tag_position = context.env.current_target_pos()
        if tag_position is None:
            return self

        if context.rover.stuck:
            context.rover.previous_state = self
            return recovery.RecoveryState()

        return approach_post.ApproachPostState()
