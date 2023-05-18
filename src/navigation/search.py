from __future__ import annotations
from typing import ClassVar, Optional

import numpy as np
import rospy

from context import Context, Environment, convert_cartesian_to_gps
from aenum import Enum, NoAlias
from state import BaseState
from dataclasses import dataclass
from trajectory import Trajectory
from mrover.msg import GPSPointList
from util.ros_utils import get_rosparam
from math import ceil

@dataclass
class SearchTrajectory(Trajectory):
    # Associated fiducial for this trajectory
    fid_id: int

    @classmethod
    def get_polar_spiral_formula(cls, distance_between_spirals: float) -> callable:
        """
        Returns a polar function that takes in an angle and returns the radius of a spiral 
        such that the distance between each spiral is 'distance_between_spirals'
        :param distance_between_spirals:    distance between each spiral (float)
        :return:    polar function (callable)
        """
        return lambda theta: theta * (distance_between_spirals / (2*np.pi))

    @classmethod
    def gen_spiral_coordinates(cls, coverage_radius: float, distance_between_spirals: float, num_points_per_spiral: int) -> np.ndarray:
        """
        Generates a set of coordinates for a spiral search pattern centered at the origin
        :param coverage_radius:     radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param num_points_per_spiral:   number of points per spiral (int)
        :return:    np.ndarray of coordinates
        """
        num_spirals = ceil(coverage_radius / distance_between_spirals) + 1
        angles = np.linspace(0, 2*np.pi*num_spirals, num_points_per_spiral*num_spirals + 1)
        radii = cls.get_polar_spiral_formula(distance_between_spirals)(angles)
        xcoords = np.cos(angles)*radii
        ycoords = np.sin(angles)*radii
        return np.hstack((xcoords.reshape(-1, 1), ycoords.reshape(-1, 1)))

    @classmethod
    def spiral_traj(cls, center: np.ndarray, coverage_radius: float, distance_between_spirals: float, points_per_turn: int,fid_id: int) -> SearchTrajectory:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param coverage_radius:     radius of the spiral search pattern (float)
        :param distance_between_spirals:    distance between each spiral (float)
        :param points_per_turn:     number of points per spiral (int)
        :param fid_id:      fiducial id to associate with this trajectory (int)
        :return:    SearchTrajectory object
        """
        deltas = cls.gen_spiral_coordinates(coverage_radius, distance_between_spirals, points_per_turn)
        
        # At this point we use cumsum to create a new array of vectors where each vector
        # is the sum of all the previous deltas up to that index in the old array. We
        # also make sure to add the center coordinate in here too so the spiral is in
        # the correct location
        coordinates = np.cumsum(np.vstack((center, deltas)), axis=0)
        return SearchTrajectory(
            np.hstack((coordinates, np.zeros(coordinates.shape[0]).reshape(-1, 1))),
            fid_id,
        )


class SearchStateTransitions(Enum):
    _settings_ = NoAlias

    no_fiducial = "WaypointState"
    continue_search = "SearchState"
    found_fiducial_post = "ApproachPostState"
    found_fiducial_gate = "PartialGateState"
    found_gate = "GateTraverseState"
    recovery_state = "RecoveryState"


class SearchState(BaseState):
    STOP_THRESH = get_rosparam("search/stop_thresh", 0.2)
    DRIVE_FWD_THRESH = get_rosparam("search/drive_fwd_thresh", 0.34)  # 20 degrees
    SPIRAL_COVERAGE_RADIUS = get_rosparam("search/coverage_radius", 20)
    POINTS_PER_SPIRAL = get_rosparam("search/points_per_spiral", 8)
    DISTANCE_BETWEEN_SPIRALS = get_rosparam("search/distance_between_spirals", 2.5)

    def __init__(
        self,
        context: Context,
    ):
        own_transitions = [SearchStateTransitions.continue_search.name]  # type: ignore
        super().__init__(
            context,
            own_transitions,
            add_outcomes=[transition.name for transition in SearchStateTransitions],  # type: ignore
        )
        self.traj: Optional[SearchTrajectory] = None
        self.prev_target: Optional[np.ndarray] = None

    def reset(self) -> None:
        self.traj = None
        self.prev_target = None

    def evaluate(self, ud):
        # Check if a path has been generated, and it's associated with the same
        # waypoint as the previous one. Generate one if not
        waypoint = self.context.course.current_waypoint()
        if self.traj is None or self.traj.fid_id != waypoint.fiducial_id:
            self.traj = SearchTrajectory.spiral_traj(
                self.context.rover.get_pose().position[0:2],
                self.SPIRAL_COVERAGE_RADIUS,
                self.DISTANCE_BETWEEN_SPIRALS,
                self.POINTS_PER_SPIRAL,
                waypoint.fiducial_id,
            )

        # continue executing this path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = self.context.rover.driver.get_drive_command(
            target_pos,
            self.context.rover.get_pose(),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            prev_target=self.prev_target,
        )
        if arrived:
            self.prev_target = target_pos
            # if we finish the spiral without seeing the fiducial, move on with course
            if self.traj.increment_point():
                return SearchStateTransitions.no_fiducial.name  # type: ignore

        if self.context.rover.stuck:
            self.context.rover.previous_state = SearchStateTransitions.continue_search.name  # type: ignore
            return SearchStateTransitions.recovery_state.name  # type: ignore

        self.context.search_point_publisher.publish(
            GPSPointList([convert_cartesian_to_gps(pt) for pt in self.traj.coordinates])
        )
        self.context.rover.send_drive_command(cmd_vel)

        # if we see the fiduicial or gate, go to either fiducial or gate state
        if self.context.env.current_gate() is not None:
            return SearchStateTransitions.found_gate.name  # type: ignore
        elif self.context.env.current_fid_pos() is not None and self.context.course.look_for_post():
            return SearchStateTransitions.found_fiducial_post.name  # type: ignore
        elif (
            self.context.env.current_fid_pos() is not None or self.context.env.other_gate_fid_pos() is not None
        ) and self.context.course.look_for_gate():
            return SearchStateTransitions.found_fiducial_gate.name  # type: ignore
        return SearchStateTransitions.continue_search.name  # type: ignore
