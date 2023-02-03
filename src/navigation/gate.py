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

        coordinates = GateTrajectory.gateSelectPath(
            rover_position, closest_prep_point, closest_approach_point, center, victory_point, gate
        )

        # put the list of coordinates together
        return GateTrajectory(coordinates)

    def gateSelectPath(
        rover_position: np.ndarray, pt1: np.ndarray, pt2: np.ndarray, pt3: np.ndarray, pt4: np.ndarray, gate: Gate
    ):
        # Get the shapes of both the posts
        postOneShape, postTwoShape = gate.getPostGeoShape()

        # Get points for path
        pt0 = np.array([rover_position[0], rover_position[1]])

        # Get paths
        (
            pathOne,
            pathTwo,
        ) = GateTrajectory.pathLineString(pt0, pt2, pt3, pt4)

        # Check if the lines intersect
        # First check if the path1 hits either posts
        if GateTrajectory.lineIntersectCheck(pathOne, postOneShape) or GateTrajectory.lineIntersectCheck(
            pathOne, postTwoShape
        ):
            if GateTrajectory.lineIntersectCheck(pathTwo, postOneShape) or GateTrajectory.lineIntersectCheck(
                pathTwo, postTwoShape
            ):
                coordinates = np.array([pt1, pt2, pt3, pt4])
                coordinates = np.hstack((coordinates, np.zeros(coordinates.shape[0]).reshape(-1, 1)))
            else:
                coordinates = np.array([pt2, pt3, pt4])
                coordinates = np.hstack((coordinates, np.zeros(coordinates.shape[0]).reshape(-1, 1)))
        else:
            coordinates = np.array([pt3, pt4])
            coordinates = np.hstack((coordinates, np.zeros(coordinates.shape[0]).reshape(-1, 1)))

        return coordinates

    def lineIntersectCheck(rover_path, postShape):
        return rover_path.intersects(postShape)

    def pathLineString(rover_pose: np.ndarray, p2: np.ndarray, p3: np.ndarray, p4: np.ndarray):
        # Find path1 (only has rover, point 3, point4)
        path1 = LineString([list(rover_pose), list(p3), list(p4)])
        assert path1.length > 0
        # Find path2 (only has rover, point2, point3, point4)
        assert rover_pose.shape == (2,)
        path2 = LineString([list(rover_pose), list(p2), list(p3), list(p4)])
        assert path2.length > 0
        return (path1, path2)


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
