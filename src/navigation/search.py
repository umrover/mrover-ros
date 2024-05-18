from typing import Optional

import numpy as np

import rospy
from mrover.msg import GPSPointList, WaypointType
from navigation import recovery, waypoint
from navigation.context import convert_cartesian_to_gps, Context
from navigation.trajectory import SearchTrajectory
from util.state_lib.state import State


class SearchState(State):
    traj: SearchTrajectory
    prev_target: Optional[np.ndarray] = None
    is_recovering: bool = False

    STOP_THRESH = rospy.get_param("search/stop_threshold")
    DRIVE_FORWARD_THRESHOLD = rospy.get_param("search/drive_forward_threshold")
    SPIRAL_COVERAGE_RADIUS = rospy.get_param("search/coverage_radius")
    SEGMENTS_PER_ROTATION = rospy.get_param("search/segments_per_rotation")
    DISTANCE_BETWEEN_SPIRALS = rospy.get_param("search/distance_between_spirals")

    OBJECT_SPIRAL_COVERAGE_RADIUS = rospy.get_param("object_search/coverage_radius")
    OBJECT_DISTANCE_BETWEEN_SPIRALS = rospy.get_param("object_search/distance_between_spirals")

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        search_center = context.course.current_waypoint()
        assert search_center is not None

        if not self.is_recovering:
            if search_center.type.val == WaypointType.POST:
                self.traj = SearchTrajectory.spiral_traj(
                    context.rover.get_pose().position[0:2],
                    self.SPIRAL_COVERAGE_RADIUS,
                    self.DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False,
                )
            else:  # water bottle or mallet
                self.traj = SearchTrajectory.spiral_traj(
                    context.rover.get_pose().position[0:2],
                    self.OBJECT_SPIRAL_COVERAGE_RADIUS,
                    self.OBJECT_DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False,
                )
            self.prev_target = None

    def on_exit(self, context: Context) -> None:
        pass

    def on_loop(self, context: Context) -> State:
        # continue executing the path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FORWARD_THRESHOLD,
            path_start=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # If we finish the spiral without seeing the tag, move on with course
            if self.traj.increment_point():
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.is_recovering = True
            return recovery.RecoveryState()
        else:
            self.is_recovering = False

        context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in self.traj.coordinates])
        )
        context.rover.send_drive_command(cmd_vel)

        # returns either ApproachPostState, LongRangeState, ApproachObjectState, or None
        assert context.course is not None
        approach_state = context.course.get_approach_target_state()
        if approach_state is not None:
            return approach_state

        return self
