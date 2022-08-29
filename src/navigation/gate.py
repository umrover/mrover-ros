from __future__ import annotations
from typing import ClassVar
from unicodedata import normalize
from navigation.context import Post

import numpy as np

from context import Context, Environment
from state import BaseState
from dataclasses import dataclass
from drive import get_drive_command
from util.np_utils import normalized, perpendicular_2d

STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.95


@dataclass
class GateTrajectory:
    # Associated Post IDs for the gate
    post1_id: int
    post2_id: int

    @classmethod
    def gate_trajectory(cls, approach_distance: float, post1_id: int, post2_id: int) -> GateTrajectory:
        """
        Generates a square spiral search pattern around a center position, assumes rover is at the center position
        :param center:      position to center spiral on (np.ndarray)
        :param num_turns:   number of times to spiral around
        :param distance:    initial distance and increment (int)
        :return:            list of positions for the rover to traverse List(np.ndarray)
        """

        #first we get the positions of the two posts
        post1 = get_post_position_rover_relative(post1_id)
        post2 = get_post_position_rover_relative(post2_id)

        center = (post1 + post2) / 2
        post_direction = normalized(post2 - post1)
        perpendicular = perpendicular_2d(post_direction)
        
        possible_approach_points = np.array([approach_distance * perpendicular + center, 
                                    -approach_distance * perpendicular + center])
        
        possible_preparation_points = np.array([(2 * approach_distance * perpendicular) + post1,
                              (2 * approach_distance * perpendicular) + post2,
                              (-2 * approach_distance * perpendicular) + post1,
                              (-2 * approach_distance * perpendicular) + post2])
        
        #get closest prepration point
        closest_prep_point = 0 #TODO

        #get closest approach point (to selected prep point), set other one to victory point
        closest_approach_point = 0 #TODO
        victory_point = 0 #TODO

        coordinates = [closest_prep_point, closest_approach_point, victory_point]
        return GateTrajectory(coordinates, post1_id, post2_id)


# class GateState(BaseState):
#     def __init__(
#         self,
#         context: Context,
#     ):
#         super().__init__(
#             context,
#             add_outcomes=["waypoint_traverse", "single_fiducial", "search"],
#         )
#         self.traj = None

#     def evaluate(self, ud):
#         # Check if a path has been generated and its associated with the same
#         # waypoint as the previous one. Generate one if not
#         waypoint = self.context.course.current_waypoint()
#         if self.traj is None or self.traj.fid_id != waypoint.fiducial_id:
#             self.traj = SearchTrajectory.spiral_traj(
#                 self.context.rover.get_pose().position_vector()[0:2],
#                 5,
#                 2,
#                 waypoint.fiducial_id,
#             )

#         # continue executing this path from wherever it left off
#         target_pos = self.traj.get_cur_pt()
#         cmd_vel, arrived = get_drive_command(
#             target_pos,
#             self.context.rover.get_pose(),
#             STOP_THRESH,
#             DRIVE_FWD_THRESH,
#         )
#         if arrived:
#             # if we finish the spiral without seeing the fiducial, move on with course
#             if self.traj.increment_point():
#                 return "waypoint_traverse"

#         self.context.rover.send_drive_command(cmd_vel)
#         # if we see the fiduicial, go to the fiducial state
#         current_waypoint = self.context.course.current_waypoint()
#         if current_waypoint.fiducial_id != Environment.NO_FIDUCIAL and self.context.env.current_fid_pos() is not None:
#             return "single_fiducial"

#         return "search"
