from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from typing import Optional
from state import BaseState
from trajectory import Trajectory
from aenum import Enum, NoAlias
from context import Context
from util.ros_utils import get_rosparam
from util.np_utils import perpendicular_2d
from shapely.geometry import Point, LineString
from util.SE3 import SE3
import tf2_ros
import rospy

POST_RADIUS = get_rosparam("gate/post_radius", 0.7) * get_rosparam("single_fiducial/post_avoidance_multiplier", 1.42)
BACKUP_DISTANCE = get_rosparam("recovery/recovery_distance", 2.0)
STOP_THRESH = 0.2
DRIVE_FWD_THRESH = 0.95


@dataclass
class AvoidPostTrajectory(Trajectory):
    def avoid_post_trajectory(rover_pose: SE3, post_pos: np.ndarray, waypoint_pos: np.ndarray) -> AvoidPostTrajectory:
        """
        Generates a trajectory that avoids a post until the rover has a clear path to the waypoint
        """

        rover_pos = rover_pose.position
        rover_direction = rover_pose.rotation.direction_vector()
        print(rover_pos, post_pos, waypoint_pos)

        # Converting to 2d arrays
        post_pos = post_pos[:2]
        rover_pos = rover_pos[:2]
        rover_direction = rover_direction[:2]
        waypoint_pos = waypoint_pos[:2]

        # normalize rover_direction
        rover_direction = rover_direction / np.linalg.norm(rover_direction)

        # create a Shapeley point of raidus POST_RADIUS around the post
        post_circle = Point(post_pos[0], post_pos[1]).buffer(POST_RADIUS)

        # generate a point BACKUP_DISTANCE behind the rover
        backup_point = rover_pos - BACKUP_DISTANCE * rover_direction

        # generate a line from the backup point to the waypoint
        path = LineString([backup_point, waypoint_pos])
        coords: np.ndarray = np.array([])
        # check if the path intersects the post circle
        if path.intersects(post_circle):
            # get a vector perpendicular to vector from rover to post
            rover_to_post = post_pos - backup_point
            rover_to_post = rover_to_post / np.linalg.norm(rover_to_post)
            left_perp = perpendicular_2d(rover_to_post)  # (-y,x)
            right_perp = -left_perp
            avoidance_point_left = post_pos + BACKUP_DISTANCE * left_perp
            avoidance_point_right = post_pos + BACKUP_DISTANCE * right_perp
            left_dist = np.linalg.norm(avoidance_point_left - waypoint_pos)
            right_dist = np.linalg.norm(avoidance_point_right - waypoint_pos)
            if left_dist < right_dist:
                coords = np.array([backup_point, avoidance_point_left])
            else:
                coords = np.array([backup_point, avoidance_point_right])
        else:
            coords = np.array([backup_point])

        # add a z coordinate of 0 to all the coords
        coords = np.hstack((coords, np.zeros((coords.shape[0], 1))))
        print(coords)
        return AvoidPostTrajectory(coords)


class PostBackupTransitions(Enum):
    _settings_ = NoAlias
    # State Transitions
    finished_traj = "WaypointState"
    recovery_state = "RecoveryState"
    continue_post_backup = "PostBackupState"


class PostBackupState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(context, add_outcomes=[transition.name for transition in PostBackupTransitions])  # type: ignore
        self.traj: Optional[AvoidPostTrajectory] = None

    def evaluate(self, ud):
        try:
            if self.traj is None:
                if self.context.env.last_post_location is None:
                    rospy.logerr("PostBackupState: last_post_location is None")
                    return PostBackupTransitions.finished_traj.name  # type: ignore

                self.traj = AvoidPostTrajectory.avoid_post_trajectory(
                    self.context.rover.get_pose(),
                    self.context.env.last_post_location,
                    self.context.course.current_waypoint_pose().position,
                )
                self.traj.cur_pt = 0

            target_pos = self.traj.get_cur_pt()

            # we drive backwards to the first point in this trajectory
            point_index = self.traj.cur_pt
            drive_backwards = point_index == 0

            cmd_vel, arrived = self.context.rover.driver.get_drive_command(
                target_pos,
                self.context.rover.get_pose(),
                STOP_THRESH,
                DRIVE_FWD_THRESH,
                drive_back=drive_backwards,
            )
            if arrived:
                print(f"ARRIVED AT POINT {point_index}")
                if self.traj.increment_point():
                    self.traj = None
                    return PostBackupTransitions.finished_traj.name  # type: ignore

            if self.context.rover.stuck:
                self.context.rover.previous_state = PartialGateStateTransitions.partial_gate.name  # type: ignore
                self.traj = None
                return PostBackupTransitions.recovery_state.name  # type: ignore

            self.context.rover.send_drive_command(cmd_vel)
            return PostBackupTransitions.continue_post_backup.name  # type: ignore
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return PostBackupTransitions.continue_post_backup.name  # type: ignore
