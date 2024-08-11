from typing import Optional

import numpy as np

import rospy
from mrover.msg import GPSPointList, WaypointType
from navigation import recovery, waypoint
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import SearchTrajectory
from util.state_lib.state import State


class SearchState(State):
    trajectory: Optional[SearchTrajectory] = None
    prev_target_pos_in_map: Optional[np.ndarray] = None
    is_recovering: bool = False

    STOP_THRESH = rospy.get_param("search/stop_threshold")
    DRIVE_FORWARD_THRESHOLD = rospy.get_param("search/drive_forward_threshold")
    SPIRAL_COVERAGE_RADIUS = rospy.get_param("search/coverage_radius")
    SEGMENTS_PER_ROTATION = rospy.get_param("search/segments_per_rotation")
    DISTANCE_BETWEEN_SPIRALS = rospy.get_param("search/distance_between_spirals")

    OBJECT_SPIRAL_COVERAGE_RADIUS = rospy.get_param("object_search/coverage_radius")
    OBJECT_DISTANCE_BETWEEN_SPIRALS = rospy.get_param("object_search/distance_between_spirals")

    def on_enter(self, context: Context) -> None:
        if SearchState.trajectory is None:
            self.new_trajectory(context)

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        rover_in_map = context.rover.get_pose_in_map()

        assert rover_in_map is not None
        assert SearchState.trajectory is not None

        # Continue executing the path from wherever it left off
        target_position_in_map = SearchState.trajectory.get_current_point()
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_position_in_map,
            rover_in_map,
            self.STOP_THRESH,
            self.DRIVE_FORWARD_THRESHOLD,
            path_start=self.prev_target_pos_in_map,
        )
        if arrived:
            self.prev_target_pos_in_map = target_position_in_map
            # If we finish the spiral without seeing the tag, move on with course
            if SearchState.trajectory.increment_point():
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in SearchState.trajectory.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        # Returns either ApproachTargetState, LongRangeState, or None
        assert context.course is not None
        for i in range(100):
            if context.env.get_target_position(f"tag{i}") is not None:
                context.course.current_waypoint().tag_id = i
                context.course.current_waypoint().type.val = WaypointType.POST
                return context.env.ReturnApproachTargetState()

        return self

    def new_trajectory(self, context) -> None:
        assert context.course is not None
        search_center = context.course.current_waypoint()

        if not self.is_recovering:
            if search_center.type.val == WaypointType.POST:
                SearchState.trajectory = SearchTrajectory.spiral_traj(
                    context.course.current_waypoint_pose_in_map().position[0:2],
                    self.SPIRAL_COVERAGE_RADIUS,
                    self.DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False,
                )
            else:  # water bottle or mallet
                SearchState.trajectory = SearchTrajectory.spiral_traj(
                    context.course.current_waypoint_pose_in_map().position[0:2],
                    self.OBJECT_SPIRAL_COVERAGE_RADIUS,
                    self.OBJECT_DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False,
                )
            self.prev_target_pos_in_map = None
