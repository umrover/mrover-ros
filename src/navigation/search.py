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


@dataclass
class SearchTrajectory(Trajectory):
    # Associated fiducial for this trajectory
    fid_id: int
    # Helper for building spiral
    dirs: ClassVar[np.ndarray] = np.array([[0, -1], [-1, 0], [0, 1], [1, 0]])

    @classmethod
    def spiral_traj(cls, center: np.ndarray, num_turns: int, distance: int, fid_id: int) -> SearchTrajectory:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param num_turns:   number of times to spiral around
        :param distance:    initial distance and increment (int)
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """
        # First we will attempt to create the "delta" vectors that get added at each point
        # in the spiral to get to the next.
        deltas = np.tile(cls.dirs, (num_turns, 1))
        # We will build the coefficients for the delta vecs now that we have the correct
        # layout of unit vectors Given the distance parameter 'd', the coef layout we
        # need is [d,d,2d,2d,3d,3d...]
        dist_coefs = distance * np.repeat(np.arange(1, num_turns * 2 + 1), 2).reshape(-1, 1)
        deltas *= dist_coefs
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

    def reset(self) -> None:
        self.traj = None

    def evaluate(self, ud):
        # Check if a path has been generated, and it's associated with the same
        # waypoint as the previous one. Generate one if not
        waypoint = self.context.course.current_waypoint()
        if self.traj is None or self.traj.fid_id != waypoint.fiducial_id:
            self.traj = SearchTrajectory.spiral_traj(
                self.context.rover.get_pose().position[0:2],
                5,
                2,
                waypoint.fiducial_id,
            )

        # continue executing this path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = self.context.rover.driver.get_drive_command(
            target_pos, self.context.rover.get_pose(), self.STOP_THRESH, self.DRIVE_FWD_THRESH
        )
        if arrived:
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
