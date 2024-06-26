from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation

import rospy
from navigation import recovery
from navigation.approach_target import ApproachTargetState
from navigation.context import Context
from util.state_lib.state import State

DISTANCE_AHEAD = rospy.get_param("long_range/distance_ahead")


class LongRangeState(ApproachTargetState):
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

        target = context.env.image_targets.query(context.course.image_target_name())
        if target is None:
            return None

        rover_in_map = context.rover.get_pose_in_map()
        assert rover_in_map is not None

        rover_position = rover_in_map.position
        rover_direction = rover_in_map.rotation.direction_vector()

        bearing_to_tag = target.target.bearing
        # If you have not seen the tag in a while but are waiting until the expiration time is up,
        # keep going towards where it was last seen (the direction you are heading), don't use an old bearing value
        if target.hit_count <= 0:
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

        return ApproachTargetState()
