from typing import List

import numpy as np

from context import Context
from waypoint import STOP_THRESH, DRIVE_FWD_THRESH, BaseWaypointState
from drive import get_drive_command


SPIRAL_POINTS = 13
SPIRAL_DISTANCE = 3.0


class SearchState(BaseWaypointState):
    def __init__(self, context: Context):
        super().__init__(
            context,
            outcomes=['single_fiducial', 'done', 'search'],
            input_keys=['search_point_index', 'searchPoints'],
            output_keys=['search_point_index']
        )
        self.search_path = None
        self.cur_search_point_index = None

    def gen_square_spiral_search_pattern(self, center: np.ndarray, points: int, distance: int) -> List[np.ndarray]:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param points:      number of points (int)
        :param distance:    initial distance and increment (int)
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """
        # TODO: refactor (I just copied the current spiral generation code for now but it needs to be vectorized)
        directions = [(0, 1), (-1, 0), (0, -1), (1, 0)]
        coordinates = [np.array([center[0], center[1], 0.0])]
        new_distance = distance
        for i in range(0, points):
            coordinates.append(np.array([coordinates[-1][0]+new_distance*directions[i %
                               4][0], coordinates[-1][1]+new_distance*directions[i % 4][1], 0.0]))
            new_distance += (i % 2)*distance
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
