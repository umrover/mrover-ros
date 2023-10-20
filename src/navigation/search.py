from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from aenum import Enum, NoAlias
from mrover.msg import GPSPointList
from util.ros_utils import get_rosparam

from context import Context, convert_cartesian_to_gps
from trajectory import Trajectory

from util.state_lib.state import State
import waypoint, approach_post, recovery

@dataclass
class SearchTrajectory(Trajectory):
    # Associated fiducial for this trajectory
    fid_id: int

    @classmethod
    def gen_spiral_coordinates(
        cls, coverage_radius: float, distance_between_spirals: float, num_segments_per_rotation: int
    ) -> np.ndarray:
        """
        Generates a set of coordinates for a spiral search pattern centered at the origin
        :param coverage_radius:     radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiralradii = angles * (distance_between_spirals / (2*np.pi)) (float)
        :param num_segments_per_rotation:   number of segments that the spiral has per rotation (int)
        :return:    np.ndarray of coordinates
        """
        # the number of spirals should ensure coverage of the entire radius.
        # We add 1 to ensure that the last spiral covers the radius along the entire rotation,
        # as otherwise we will just make the outermost point touch the radius
        num_spirals = np.ceil(coverage_radius / distance_between_spirals).astype("int") + 1
        # the angles are evenly spaced between 0 and 2pi*num_segments_per_rotation (add one to the number of points because N+1 points make N segments)
        angles = np.linspace(0, 2 * np.pi * num_spirals, num_segments_per_rotation * num_spirals + 1)
        # radii are computed via following polar formula.
        # This is correct because you want the radius to increase by 'distance_between_spirals' every 2pi radians (one rotation)
        radii = angles * (distance_between_spirals / (2 * np.pi))
        # convert to cartesian coordinates
        xcoords = np.cos(angles) * radii
        ycoords = np.sin(angles) * radii
        # we want to return as a 2D matrix where each row is a coordinate pair
        # so we reshape x and y coordinates to be (n, 1) matricies then stack horizontally to get (n, 2) matrix
        return np.hstack((xcoords.reshape(-1, 1), ycoords.reshape(-1, 1)))

    @classmethod
    def spiral_traj(
        cls,
        center: np.ndarray,
        coverage_radius: float,
        distance_between_spirals: float,
        segments_per_rotation: int,
        fid_id: int,
    ) -> SearchTrajectory:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param coverage_radius:     radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param segments_per_rotation:     number of segments per spiral (int), for example, 4 segments per rotation would be a square spiral, 8 segments per rotation would be an octagonal spiral
        :param fid_id:      fiducial id to associate with this trajectory (int)
        :return:    SearchTrajectory object
        """
        zero_centered_spiral_r2 = cls.gen_spiral_coordinates(
            coverage_radius, distance_between_spirals, segments_per_rotation
        )

        # numpy broadcasting magic to add center to each row of the spiral coordinates
        spiral_coordinates_r2 = zero_centered_spiral_r2 + center
        # add a column of zeros to make it 3D
        spiral_coordinates_r3 = np.hstack(
            (spiral_coordinates_r2, np.zeros(spiral_coordinates_r2.shape[0]).reshape(-1, 1))
        )
        return SearchTrajectory(
            spiral_coordinates_r3,
            fid_id,
        )



class SearchState(State):
    STOP_THRESH = get_rosparam("search/stop_thresh", 0.2)
    DRIVE_FWD_THRESH = get_rosparam("search/drive_fwd_thresh", 0.34)  # 20 degrees
    SPIRAL_COVERAGE_RADIUS = get_rosparam("search/coverage_radius", 20)
    SEGMENTS_PER_ROTATION = get_rosparam("search/segments_per_rotation", 8)
    DISTANCE_BETWEEN_SPIRALS = get_rosparam("search/distance_between_spirals", 2.5)
    
    def on_enter(self, context) -> None:
        waypoint = context.course.current_waypoint()
        self.traj = SearchTrajectory.spiral_traj(
                context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.SEGMENTS_PER_ROTATION,
                waypoint.fiducial_id,
            )
        self.prev_target = None

    def on_exit(self, context) -> None:
        if not self.is_recovering:
            self.traj = None
            self.prev_target = None

    def on_loop(self, context) -> State:
        waypoint = context.course.current_waypoint()

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

        if context.env.current_fid_pos() is not None and context.course.look_for_post():
            return approach_post.ApproachPostState()
        return self
