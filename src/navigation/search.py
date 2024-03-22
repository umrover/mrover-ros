from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from mrover.msg import GPSPointList, WaypointType


from util.ros_utils import get_rosparam
from util.state_lib.state import State

from navigation import approach_post, approach_object, recovery, waypoint, long_range
from navigation.context import convert_cartesian_to_gps
from navigation.trajectory import SearchTrajectory


class SearchState(State):
    traj: SearchTrajectory
    prev_target: Optional[np.ndarray] = None
    is_recovering: bool = False

    STOP_THRESH = get_rosparam("search/stop_thresh", 0.5)
    DRIVE_FWD_THRESH = get_rosparam("search/drive_fwd_thresh", 0.34)  # 20 degrees
    SPIRAL_COVERAGE_RADIUS = get_rosparam("search/coverage_radius", 20)
    SEGMENTS_PER_ROTATION = get_rosparam("search/segments_per_rotation", 8)
    DISTANCE_BETWEEN_SPIRALS = get_rosparam("search/distance_between_spirals", 3)

    OBJECT_SPIRAL_COVERAGE_RADIUS = get_rosparam("object_search/coverage_radius", 10)
    OBJECT_DISTANCE_BETWEEN_SPIRALS = get_rosparam("object_search/distance_between_spirals", 3)

    def on_enter(self, context) -> None:
        search_center = context.course.current_waypoint()

        if not self.is_recovering:
            if search_center.type.val == WaypointType.POST:
                self.traj = SearchTrajectory.spiral_traj(
                    context.rover.get_pose().position[0:2],
                    self.SPIRAL_COVERAGE_RADIUS,
                    self.DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False
                )
            else:  # water bottle or mallet
                self.traj = SearchTrajectory.spiral_traj(
                    context.rover.get_pose().position[0:2],
                    self.OBJECT_SPIRAL_COVERAGE_RADIUS,
                    self.OBJECT_DISTANCE_BETWEEN_SPIRALS,
                    self.SEGMENTS_PER_ROTATION,
                    search_center.tag_id,
                    False
                )
            self.prev_target = None

    def on_exit(self, context) -> None:
        pass

    def on_loop(self, context) -> State:
        # continue executing the path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            path_start=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # if we finish the spiral without seeing the fiducial, move on with course
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
        if context.course.check_approach() is not None:
            return context.course.check_approach()

        return self
