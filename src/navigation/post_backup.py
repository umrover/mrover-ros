from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np
from shapely.geometry import Point, LineString

import rospy
from navigation import waypoint, recovery
from navigation.context import Context
from navigation.trajectory import Trajectory
from util.SE3 import SE3
from util.np_utils import perpendicular_2d
from util.state_lib.state import State

POST_RADIUS = rospy.get_param("single_tag/post_radius") * rospy.get_param("single_tag/post_avoidance_multiplier")
BACKUP_DISTANCE = rospy.get_param("recovery/recovery_distance")
STOP_THRESH = rospy.get_param("search/stop_threshold")
DRIVE_FWD_THRESH = rospy.get_param("search/drive_forward_threshold")


@dataclass
class AvoidPostTrajectory(Trajectory):
    @staticmethod
    def avoid_post_trajectory(rover_pose: SE3, post_pos: np.ndarray, waypoint_pos: np.ndarray) -> AvoidPostTrajectory:
        """
        Generates a trajectory that avoids a post until the rover has a clear path to the waypoint
        :param rover_pose:      The current pose of the rover
        :param post_pos:        The position of the post
        :param waypoint_pos:    The position of the waypoint
        :return:                A trajectory that avoids the post, including the first point as a backup point that the rover MUST drive backwards towards

        Summary on how trajectory works:
        we first generate a backup point that is directly behind
        the rover some BACKUP_DISTANCE behind it. Then we see if
        we can drive directly to the waypoint without intersecting
        a circle of size POST_RADIUS ( < BACKUP_DISTANCE)
        if we can then the trajectory is just the backup point.
        If we cannot then we generate an avoidance point BACKUP_DISTANCE away
        from the post at a 90 degree angle from the rover relative to the post
        either to the left or the right depending on which is closer to the waypoint.
        Then the trajectory is backup_point, avoidance_point where we drive backwards to
        the backup_point and forwards to the avoidance_point.
        """
        rover_pos = rover_pose.position
        rover_direction = rover_pose.rotation.direction_vector()

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
        return AvoidPostTrajectory(coords)


class PostBackupState(State):
    traj: Optional[AvoidPostTrajectory]

    def on_exit(self, context: Context) -> None:
        self.traj = None

    def on_enter(self, context: Context) -> None:
        assert context.course is not None

        if context.env.last_target_location is None:
            self.traj = None
        else:
            self.traj = AvoidPostTrajectory.avoid_post_trajectory(
                context.rover.get_pose(),
                context.env.last_target_location,
                context.course.current_waypoint_pose().position,
            )
            self.traj.cur_pt = 0

    def on_loop(self, context: Context) -> State:
        if self.traj is None:
            return waypoint.WaypointState()

        target_pos = self.traj.get_cur_pt()

        # we drive backwards to the first point in this trajectory
        point_index = self.traj.cur_pt
        drive_backwards = point_index == 0

        cmd_vel, arrived = context.rover.driver.get_drive_command(
            target_pos,
            context.rover.get_pose(),
            STOP_THRESH,
            DRIVE_FWD_THRESH,
            drive_back=drive_backwards,
        )
        if arrived:
            rospy.loginfo(f"Arrived at point indexed: {point_index}")
            if self.traj.increment_point():
                self.traj = None
                return waypoint.WaypointState()

        if context.rover.stuck:
            context.rover.previous_state = self
            self.traj = None
            return recovery.RecoveryState()

        context.rover.send_drive_command(cmd_vel)
        return self
