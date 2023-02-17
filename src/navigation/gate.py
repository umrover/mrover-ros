from __future__ import annotations
from typing import ClassVar, Optional
from unicodedata import normalize
from context import Gate, Context

import numpy as np

from aenum import Enum, NoAlias
from state import BaseState
from trajectory import Trajectory
from dataclasses import dataclass
from drive import get_drive_command
from util.np_utils import normalized, perpendicular_2d
from shapely.geometry import LineString

STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.34  # 20 degrees

APPROACH_DISTANCE = 2.0


@dataclass
class GateTrajectory(Trajectory):
    @classmethod
    def spider_gate_trajectory(cls, approach_distance: float, gate: Gate, rover_position: np.ndarray) -> GateTrajectory:
        """
        Generates the "Spider" path through the gate, see https://github.com/umrover/mrover-ros/wiki/Navigation#searchtrajectory
        :param approach_distance: distance to the point straight out from the gate that the rover will drive to
        :param gate:    Gate object representing the gate that the rover will drive through
        :param rover_position: position vector of the rover
        :return:            GateTrajectory object containing the coordinates the rover will need to traverse
        """

        # first we get the positions of the two posts
        post1 = gate.post1
        post2 = gate.post2

        center = (post1 + post2) / 2
        # the direction of the post is just the normalized vector from post1 to post2
        post_direction = normalized(post2 - post1)
        perpendicular = perpendicular_2d(post_direction)

        # approach points are the points that are directly out from the center (a straight line) of
        # the gate "approach_distance" away
        possible_approach_points = [
            approach_distance * perpendicular + center,
            -approach_distance * perpendicular + center,
        ]

        # prep points are points that are direclty out in either direction from the posts
        # the idea here is that if we go to the closest prep point then an approach point,
        # we will never collide with the post
        possible_preparation_points = np.array(
            [
                (2 * approach_distance * perpendicular) + post1,
                (2 * approach_distance * perpendicular) + post2,
                (-2 * approach_distance * perpendicular) + post1,
                (-2 * approach_distance * perpendicular) + post2,
            ]
        )

        # get closest prepration point
        prep_distance_to_rover = np.linalg.norm(possible_preparation_points - rover_position[0:2], axis=1)
        prep_idx = np.argmin(prep_distance_to_rover)
        closest_prep_point = possible_preparation_points[prep_idx]

        # get closest approach point (to selected prep point), set other one to victory point
        approach_dist_to_prep = np.linalg.norm(possible_approach_points - closest_prep_point, axis=1)
        approach_idx = np.argmin(approach_dist_to_prep)
        closest_approach_point = possible_approach_points[approach_idx]
        victory_point = possible_approach_points[1 - approach_idx]

        coordinates = GateTrajectory.gate_select_path(
            rover_position, closest_prep_point, closest_approach_point, center, victory_point, gate
        )

        # put the list of coordinates together
        return GateTrajectory(coordinates)

    def gate_select_path(
        rover_position: np.ndarray,
        prep: np.ndarray,
        approach: np.ndarray,
        center: np.ndarray,
        done: np.ndarray,
        gate: Gate,
    ):
        """
        Here is a reference for the points mentioned below https://github.com/umrover/mrover-ros/wiki/Navigation#searchtrajectory
        :param rover_position: position vector of the rover,
        :param prep: This is the closest prep point given from spider_gate_trajectory,
        :param approach: This is the closest approach point given from spider_gate_trajectory ,
        :param center: This is the mid point of the two gate posts,
        :param done: This is the victory point given from spider_gate_trajectory,
        :param gate: Gate object representing the gate that the rover will drive through,
        :return coordinates: This is a np.array which represents the coordinates of the selected path
        """

        # Get the shapes of both the posts
        post_one_shape, post_two_shape = gate.get_post_shape()

        rover = rover_position[:2]

        # try paths with successively more points until we have one that won't intersect
        all_pts = np.vstack((prep, approach, center, done))
        start_index = 2
        path = make_shapely_path(rover, all_pts[start_index:, :])
        while path.intersects(post_one_shape) or path.intersects(post_two_shape):
            start_index -= 1
            if start_index == 0:
                break
            path = make_shapely_path(rover, all_pts[start_index:])

        coordinates = np.array(all_pts[start_index:])
        coordinates = np.hstack((coordinates, np.zeros(coordinates.shape[0]).reshape(-1, 1)))
        return coordinates


def make_shapely_path(rover, path_pts):
    """
    :param rover: position vector of the rover
    :param pathPts: This is a np.array that has the coordinates of the path
    :return: It returns the shapely object of LineString which put the path points given into
    one cohesive line segments.
    """
    path_list = np.vstack((rover, path_pts))
    return LineString(path_list)


class GateTraverseStateTransitions(Enum):
    _settings_ = NoAlias

    no_gate = "SearchState"
    finished_gate = "DoneState"
    continue_gate_traverse = "GateTraverseState"


class GateTraverseState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(
            context,
            add_outcomes=[transition.name for transition in GateTraverseStateTransitions],  # type: ignore
        )
        self.traj: Optional[GateTrajectory] = None

    def evaluate(self, ud):
        # Check if a path has been generated and its associated with the same
        # waypoint as the previous one. Generate one if not
        gate = self.context.env.current_gate()
        if gate is None:
            return GateTraverseStateTransitions.no_gate.name  # type: ignore
        if self.traj is None:
            self.traj = GateTrajectory.spider_gate_trajectory(
                APPROACH_DISTANCE, gate, self.context.rover.get_pose().position
            )

        # continue executing this path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        cmd_vel, arrived = get_drive_command(
            target_pos,
            self.context.rover.get_pose(),
            STOP_THRESH,
            DRIVE_FWD_THRESH,
        )
        if arrived:
            # if we finish the gate path, we're done
            if self.traj.increment_point():
                self.traj = None
                self.context.course.increment_waypoint()
                return GateTraverseStateTransitions.finished_gate.name  # type: ignore

        self.context.rover.send_drive_command(cmd_vel)
        return GateTraverseStateTransitions.continue_gate_traverse.name  # type: ignore
