from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from typing import Optional
from state import BaseState
from trajectory import Trajectory
from aenum import Enum, NoAlias
from context import Context
from utils.ros_utils import get_rosparam
from utils.np_utils import perpendicular_2d
from shapely.geometry import Point, LineString
from util.SE3 import SE3

POST_RADIUS = get_rosparam("gate/post_radius", default=0.7) * 4 # add a big buffer to the post radius, 4 is somewhat arbitrary but it doesn't really matter
BACKUP_DISTANCE = get_rosparam("recovery/recovery_distance", 1.0)
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

        # Converting to 2d arrays
        post_pos = post_pos[:2]
        rover_pos = rover_pos[:2]
        rover_direction = rover_direction[:2]
        waypoint_pos = waypoint_pos[:2]

        #normalize rover_direction
        rover_direction = rover_direction / np.linalg.norm(rover_direction)

        #create a Shapeley point of raidus POST_RADIUS around the post
        post_circle = Point(post_pos[0], post_pos[1]).buffer(POST_RADIUS)
        
        #generate a point BACKUP_DISTANCE behind the rover
        backup_point = rover_pos - BACKUP_DISTANCE * rover_direction

        #generate a line from the backup point to the waypoint
        path = LineString([backup_point, waypoint_pos])
        coords = np.array()
        #check if the path intersects the post circle
        if path.intersects(post_circle):
            #get a vector perpendicular to rover direction
            left_perp = perpendicular_2d(rover_direction)  # (-y,x)
            avoidance_point = post_pos + POST_RADIUS * left_perp
            coords = np.array([backup_point, avoidance_point, waypoint_pos])
        else:
            coords = np.array([backup_point, waypoint_pos])

        #add a z coordinate of 0 to all the coords
        coords = np.hstack((coords, np.zeros((coords.shape[0], 1))))
        return AvoidPostTrajectory(coords)


class PostBackupTransitions(Enum):
    _settings_ = NoAlias
    # State Transitions
    finished_traj = "WaypointState"
    recovery_state = "RecoveryState"
    continue_post_backup = "PostBackup"


class PostBackupState(BaseState):
    def __init__(
        self,
        context: Context,
    ):
        super().__init__(context, add_outcomes=[transition.name for transition in PostBackupTransitions])  # type: ignore
        self.traj: Optional[AvoidPostTrajectory] = None

    def evaluate(self, ud):
        if self.traj is None:
            self.traj = AvoidPostTrajectory.avoid_post_trajectory(
                self.context.rover.get_pose(in_odom_frame=self.context.use_odom),
                self.context.env.current_fid_pos(),
                self.context.course.current_waypoint_pose().position,
            )
            self.traj.cur_pt = 0

        target_pos = self.traj.get_cur_pt()

        #we drive backwards to the first point in this trajectory
        point_index = self.traj.cur_pt
        drive_backwards = point_index == 0

        cmd_vel, arrived = self.context.rover.driver.get_drive_command(
            target_pos,
            self.context.rover.get_pose(in_odom_frame=True),
            STOP_THRESH,
            DRIVE_FWD_THRESH,
            in_odom=self.context.use_odom,
            drive_back=drive_backwards,
        )
        if arrived:
            if self.traj.increment_point():
                self.traj = None
                return PostBackupTransitions.finished_traj.name  # type: ignore

        if self.context.rover.stuck:
            self.context.rover.previous_state = PartialGateStateTransitions.partial_gate.name  # type: ignore
            self.traj = None
            return PartialGateStateTransitions.recovery_state.name  # type: ignore

        self.context.rover.send_drive_command(cmd_vel)
        return PartialGateStateTransitions.continue_post_backup.name  # type: ignore
