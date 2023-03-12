from typing import List, Tuple, ClassVar
from dataclasses import dataclass
import numpy as np
from shapely.geometry import Polygon, Point

from geometry_msgs.msg import Twist
from util.SE3 import SE3

from context import Context
from util.np_utils import angle_to_rotate
from failure_zone import FailureZone
from path_planner import PathPlanner

MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 10.0

@dataclass
class Driver:
    ctx: ClassVar[Context]
    planner: ClassVar[PathPlanner] = PathPlanner()
    
    def add_failure_zone(self, failure_zone: Polygon) -> None:
        """
        Add a newly-detected failure zone to the PathPlanner. 
        """
        self.planner.add_failure_zone(FailureZone(failure_zone))

    def get_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        Gets the drive command for the rover currently at rover_pose and intending to
        reach target_pos while avoiding failure zones (if possible).  

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort command 
                                        to an intermediate target, and a bool indicating 
                                        whether the final destination has been reached. 
        """        
        cmd_vel, reached = self.get_intermediate_target_drive_command(target_pos,
                                                                      rover_pose, completion_thresh, turn_in_place_thresh)
        # if intermediate target already reached, then mark intermediate target
        # as complete and get command to next intermediate target
        while(reached and not self.planner.is_path_complete()):
            self.planner.complete_intermediate_target() # mark target as completed
            cmd_vel, reached = self.get_intermediate_target_drive_command(target_pos,
                                                                    rover_pose, completion_thresh, turn_in_place_thresh)
        return cmd_vel, reached

    def get_intermediate_target_drive_command(
            self, 
            target_pos: np.ndarray, 
            rover_pose: SE3, 
            completion_thresh: float, 
            turn_in_place_thresh: float
    ) -> Tuple[Twist, bool]:
        """
        Given a final target_pos and the current rover_pose, uses the planner to return a drive command to the current intermediate target that is on the way to the ultimate goal point. The planner avoids failure zones. 

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort 
                                        command to the intermediate target, and a bool indicating whether this target has been reached. 
        """
        source_point = Point(rover_pose.position[0:2])  
        target_point = Point(target_pos[0:2])

        curr_target = self.planner.get_intermediate_target(source_point, target_point)
        curr_target_pos = np.ndarray([curr_target.x, curr_target.y, 0])
        return self.get_clear_path_drive_command(curr_target_pos, 
                                                 rover_pose, completion_thresh, turn_in_place_thresh)
        

    def get_clear_path_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        Gets the drive command to the given target_pos. Assumes that the path to the target_pos is clear and free of failure zones. 

        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than
                                        this, stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target 
                                        and current heading in order to drive forward. When below, turn in place.

        :return:                        A tuple containing the rover drive effort 
                                        command, and a bool indicating whether the target 
                                        has been reached 
        """
        if not (0.0 < turn_in_place_thresh < 1.0):
            raise ValueError(f"Argument {turn_in_place_thresh} should be between 0 and 1")
        rover_pos = rover_pose.position
        rover_dir = rover_pose.rotation.direction_vector()
        rover_dir[2] = 0

        # Get vector from rover to target
        target_dir = target_pos - rover_pos
        print(
            f"rover direction: {rover_dir}, target direction: {target_dir}, rover position: {rover_pos} , goal: {target_pos}"
        )

        target_dist = np.linalg.norm(target_dir)
        if target_dist == 0:
            target_dist = np.finfo(float).eps

        alignment = angle_to_rotate(rover_dir, target_dir)

        if target_dist < completion_thresh:
            return Twist(), True

        cmd_vel = Twist()
        full_turn_override = True
        if abs(alignment) < turn_in_place_thresh:
            # We are pretty aligned so we can drive straight
            error = target_dist
            cmd_vel.linear.x = np.clip(error, 0.0, MAX_DRIVING_EFFORT)
            full_turn_override = False

        # we want to drive the angular offset to zero so the error is just 0 - alignment
        error = alignment
        cmd_vel.angular.z = (
            np.sign(error) if full_turn_override else np.clip(error * TURNING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
        )
        print(cmd_vel.linear.x, cmd_vel.angular.z)
        return cmd_vel, False