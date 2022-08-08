from __future__ import annotations
from typing import List

import numpy as np

from context import Context
from state import BaseState
from dataclasses import dataclass

@dataclass
class SearchTrajectory:
    # coordinates
    coordinates : np.ndarray
    # Associated fiducial for this trajectory
    fid_id : int

    @classmethod
    def spiral_traj(cls, center: np.ndarray, num_turns: int, distance: int, fid_id : int) -> SearchTrajectory:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param num_turns:   number of times to spiral around
        :param distance:    initial distance and increment (int)
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """
        # First we will attempt to create the "delta" vectors that get added add each point 
        # in the spiral to get to the next. 
        dirs = np.array([ [0, -1], [-1, 0], [0, 1], [1, 0] ])
        deltas = np.tile(dirs, (num_turns, 1))
        # We will build the coeficients for the delta vecs now that we have the correct
        # layout of unit vectors Given the distance parameter 'd', the coef layout we 
        # need is [d,d,2d,2d,3d,3d...]
        dist_coefs = distance * np.repeat(np.arange(1, num_turns*2+1), 2).reshape(-1, 1)
        deltas *= dist_coefs
        # At this point we use cumsum to create a new array of vectors where each vector
        # is the sum of all the previous deltas up to that index in the old array. We
        # also make sure to add the center coordinate in here too so the spiral is in
        # the correct location
        coordinates = np.cumsum(np.vstack((center, deltas)), axis=0)
        return SearchTrajectory(coordinates, fid_id)

class SearchState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(
            context,
            add_outcomes=["waypoint_traverse", "single_fiducial"],
        )
        self.traj = None

    def evaluate(self, ud):
        # Check if a path has been generated and its associated with the same
        # waypoint as the previous one  
        if self.traj is None:
            self.traj = SearchTrajectory.spiral_traj()

        # if so continue executing this path from wherever it left off

        # otherwise we generate a new path

        # if we see the fiduicial, go to the fiducial state

        # if we finish the spiral without seeing the fiducial, move on with course
        return "waypoint_traverse"
