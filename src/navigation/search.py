from typing import List

import numpy as np

from context import Context
from waypoint import STOP_THRESH, DRIVE_FWD_THRESH, WaypointState
from drive import get_drive_command


SPIRAL_POINTS = 13
SPIRAL_DISTANCE = 3.0


class SearchState(WaypointState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['single_fiducial', 'done', 'search'],
            input_keys=['search_point_index', 'searchPoints'],
            output_keys=['search_point_index']
        )
        self.search_path = None
        self.cur_search_point_index = None

    def gen_square_spiral_search_pattern(self, center: np.ndarray, num_turns: int, distance: int) -> np.ndarray:
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
        return coordinates

    def evaluate(self, ud):
        # if first evaluation pass, generate the search pattern
        if self.search_path is None:
            print(self.rover_pose().position_vector())
            self.search_path = self.gen_square_spiral_search_pattern(
                self.rover_pose().position_vector(), SPIRAL_POINTS, SPIRAL_DISTANCE)
            self.cur_search_point_index = 0

        # we've completed the search and still haven't found the tag
        if self.cur_search_point_index >= len(self.search_path):
            return 'done'

        # TODO: test
        fid_pos = self.current_fid_pos(ud)
        # if we have seen the target, switch to the single_fiducial state
        if fid_pos is not None:
            self.search_path = None
            return 'single_fiducial'

        search_point = self.search_path[self.cur_search_point_index]
        cmd_vel, arrived = get_drive_command(
            search_point, self.rover_pose(), STOP_THRESH, DRIVE_FWD_THRESH)
        # move onto next search point
        if arrived:
            self.cur_search_point_index += 1
        self.context.vel_cmd_publisher.publish(cmd_vel)
        return 'search'
