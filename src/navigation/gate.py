from __future__ import annotations
from typing import ClassVar, Optional
from unicodedata import normalize
from context import Gate, Context

import numpy as np
import rospy

from context import Context, Environment, Rover, convert_cartesian_to_gps
from aenum import Enum, NoAlias
from state import BaseState
from trajectory import Trajectory
from dataclasses import dataclass
from drive import DriveController
from util.np_utils import normalized, perpendicular_2d
from util.ros_utils import get_rosparam
from shapely.geometry import LineString, Polygon, Point
from mrover.msg import GPSPointList

STOP_THRESH = get_rosparam("gate/stop_thresh", 0.2)
DRIVE_FWD_THRESH = get_rosparam("gate/drive_fwd_thresh", 0.34)  # 20 degrees

APPROACH_DISTANCE = get_rosparam("gate/approach_distance", 2.0)


@dataclass
class GatePath:
    """
    Represents a path through a gate. The reason this is not subclassing Trajectory is because it needs to dynamically
    update the path every time we have a new gate estimate. Trajectory subclasses are meant to be immutable and regenerating
    them leads to problems with reseting the cur_pt variable. This class maintains minimal state (direction through the gate)
    through keeping track of what approach and prep points it should use and also keeps track of whether we've passed through
    gate or not.
    The Client can update the values of the actual points in the trajectory upon calling the update() method
    Use this class by first constructing the object GatePath(rover_pos, gate) and then calling update every time you have a new
    gate estimate. Call get_cur_pt() to get the current path that you should be following (which will be None if we are done)
    """

    rover_pos: np.ndarray
    gate: Gate
    prep_idx: int
    approach_idx: int
    victory_idx: int
    prep_pts: np.ndarray
    approach_pts: np.ndarray
    center: np.ndarray
    passed_center: bool = False
    center_idx: int = 2
    path_index: int = 0

    def __init__(self, rover_pos: np.ndarray, gate: Gate):
        self.rover_pos = rover_pos[:2]
        self.gate = gate
        self.__update_pts()
        self.prep_idx = int(np.argmin(np.linalg.norm(self.prep_pts - self.rover_pos, axis=1)))
        self.approach_idx = int(np.argmin(np.linalg.norm(self.approach_pts - self.rover_pos, axis=1)))
        self.victory_idx = int(np.argmax(np.linalg.norm(self.approach_pts - self.rover_pos, axis=1)))
        self.update(rover_pos, gate)
        self.path_index = self.__optimize_path()

    def __update_center(self) -> None:
        self.center = (self.gate.post1 + self.gate.post2) / 2

    def __update_approach_pts(self) -> None:
        """
        Updates the approach points based on the current gate estimate
        """
        post1 = self.gate.post1
        post2 = self.gate.post2

        # the direction of the post is just the normalized vector from post1 to post2
        post_direction = normalized(post2 - post1)
        perpendicular = perpendicular_2d(post_direction)

        # approach points are the points that are directly out from the center (a straight line) of
        # the gate "approach_distance" away
        self.approach_pts = np.array(
            [
                APPROACH_DISTANCE * perpendicular + self.center,
                -APPROACH_DISTANCE * perpendicular + self.center,
            ]
        )

    def __update_prep_pts(self) -> None:
        """
        Updates the prep points based on the current gate estimate
        """
        post1 = self.gate.post1
        post2 = self.gate.post2

        # the direction of the post is just the normalized vector from post1 to post2
        post_direction = normalized(post2 - post1)
        perpendicular = perpendicular_2d(post_direction)

        # prep points are the points that are perpendicular to the center of the gate
        # "approach_distance" away
        # prep points are points that are directly out in either direction from the posts
        # the idea here is that if we go to the closest prep point then an approach point,
        # we will never collide with the post
        self.prep_pts = np.array(
            [
                (2 * APPROACH_DISTANCE * perpendicular) + post1,
                (2 * APPROACH_DISTANCE * perpendicular) + post2,
                (-2 * APPROACH_DISTANCE * perpendicular) + post1,
                (-2 * APPROACH_DISTANCE * perpendicular) + post2,
            ]
        )

    def __update_pts(self) -> None:
        """
        Updates the prep points, approach points, and center of the gate
        """
        self.__update_center()
        self.__update_approach_pts()
        self.__update_prep_pts()

    def __get_full_path(self) -> np.ndarray:
        """
        :returns: np.array which represents the coordinates of the path.
        """
        return np.array(
            [
                self.prep_pts[self.prep_idx],
                self.approach_pts[self.approach_idx],
                self.center,
                self.approach_pts[self.victory_idx],
            ]
        )

    def __optimize_path(self) -> int:
        """
        Here is a reference for the points mentioned below:
         https://github.com/umrover/mrover-ros/wiki/Navigation#searchtrajectory
        :returns: an integer representing the farthest along point on the path that we can
          drive to without intersecting the gate (while still driving through it)
        """
        if not self.__should_optimize():
            return 0
        # Get the shapes of both the posts
        post_one_shape, post_two_shape = self.gate.get_post_shapes()

        rover = self.rover_pos[:2]

        # try paths with successively more points until we have one that won't intersect
        all_pts = self.__get_full_path()
        num_pts_included = (
            2  # require that we include the victory point and the center point in the path to ensure traversal
        )
        path = self.__make_shapely_path(rover, all_pts[-num_pts_included:, :])
        while path.intersects(post_one_shape) or path.intersects(post_two_shape):
            num_pts_included += 1
            if num_pts_included == all_pts.shape[0]:
                break
            path = self.__make_shapely_path(rover, all_pts[-num_pts_included:])

        return all_pts.shape[0] - num_pts_included

    def __get_paint(self) -> Polygon:
        """
        Generates the 'paint' as a shapely polygon. The 'paint' is the region that the rover is allowed to optimize its path within
        """
        return Polygon(self.prep_pts)

    def __should_optimize(self) -> bool:
        """
        :returns: True if the rover should optimize its path, False otherwise.
        the rover should optimize its path if its position is within the paint
        """
        paint = self.__get_paint()
        return paint.contains(Point(self.rover_pos))

    def __make_shapely_path(self, rover: Rover, path_pts: np.ndarray) -> LineString:
        """
        :param rover: position vector of the rover
        :param pathPts: This is a np.array that has the coordinates of the path
        :returns: Returns a Shapeley (geometry limake_shabrary) object of LineString which put the path points given into
        one cohesive line segments.
        """
        path_list = np.vstack((rover, path_pts))
        return LineString(path_list)

    def update(self, rover_pos: np.ndarray, gate: Gate) -> None:
        """
        Updates the calculated prep, approach, and center points based on the current gate estimate
        """
        self.rover_pos = rover_pos[:2]
        self.gate = gate
        self.__update_pts()

    def get_cur_pt(self) -> Optional[np.ndarray]:
        """
        :returns: The point on the path that the rover should drive to next, or None if the rover has completed the path
        """
        # first get the full path with all the key points.
        full_path = self.__get_full_path()

        pt = full_path[self.path_index]
        # if we are close enough to the point, move on to the next one
        if np.linalg.norm(pt - self.rover_pos) < STOP_THRESH:
            self.path_index += 1
            # if we have reached the end of the path, return None
            if self.path_index >= len(full_path):
                return None
            pt = full_path[self.path_index]
        # append a 0.0 to the end so the point is in R^3
        return np.append(pt, 0.0)


class GateTraverseStateTransitions(Enum):
    _settings_ = NoAlias

    no_gate = "SearchState"
    finished_gate = "DoneState"
    continue_gate_traverse = "GateTraverseState"
    recovery_state = "RecoveryState"


class GateTraverseState(BaseState):

    STOP_THRESH = get_rosparam("gate/stop_thresh", 0.2)
    DRIVE_FWD_THRESH = get_rosparam("gate/drive_fwd_thresh", 0.34)  # 20 degrees
    APPROACH_DISTANCE = get_rosparam("gate/approach_distance", 2.0)

    traj: Optional[GatePath] = None

    def __init__(
        self,
        context: Context,
    ):
        own_transitions = [GateTraverseStateTransitions.continue_gate_traverse.name]  # type: ignore
        super().__init__(
            context,
            own_transitions=own_transitions,  # type: ignore
            add_outcomes=[transition.name for transition in GateTraverseStateTransitions],  # type: ignore
        )

    def reset(self) -> None:
        self.traj = None

    def evaluate(self, ud):
        # Check if a path has been generated and its associated with the same
        # waypoint as the previous one. Generate one if not
        gate = self.context.env.current_gate()
        if gate is None:
            return GateTraverseStateTransitions.no_gate.name  # type: ignore

        rover_position = self.context.rover.get_pose(in_odom_frame=True).position
        if self.traj is None:
            self.traj = GatePath(rover_position, gate)
        else:
            self.traj.update(rover_position, gate)

        if self.traj is None:
            return GateTraverseState.finished_gate.name  # type: ignore

        # continue executing this path from wherever it left off
        target_pos = self.traj.get_cur_pt()
        if target_pos is None:
            self.context.course.increment_waypoint()
            return GateTraverseStateTransitions.finished_gate.name  # type: ignore

        cmd_vel, _ = self.context.rover.driver.get_drive_command(
            target_pos,
            self.context.rover.get_pose(in_odom_frame=True),
            self.STOP_THRESH,
            self.DRIVE_FWD_THRESH,
            in_odom=self.context.use_odom,
        )

        if self.context.rover.stuck:
            self.context.rover.previous_state = GateTraverseStateTransitions.continue_gate_traverse.name  # type: ignore
            return GateTraverseStateTransitions.recovery_state.name  # type: ignore


        map_gate = self.context.env.current_gate(odom_override=False)
        if map_gate is not None:
            self.context.gate_point_publisher.publish(
                GPSPointList([convert_cartesian_to_gps(p) for p in [map_gate.post1, map_gate.post2]])
            )
        self.context.rover.send_drive_command(cmd_vel)
        return GateTraverseStateTransitions.continue_gate_traverse.name  # type: ignore
