from typing import Optional, List, Tuple, ClassVar
from context import Context, FailureZone
import numpy as np

from geometry_msgs.msg import Twist
from util.SE3 import SE3
from util.np_utils import angle_to_rotate

MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 10.0

@dataclass
class Driver:
    ctx: Context

    curr_target_pos: ClassVar[np.ndarray] = None 
    curr_path: ClassVar[np.ndarray] = None
    visibility_graph: ClassVar[np.ndarray] = None          

    """
    Function called after a new FailureZone is added to Environment
    """
    def update_map(self):
        # add new edges
        for idx, fz in enumerate(self.ctx.env.failure_zones[:-1]):
            pass


        # remove old edges


    def shortest_path(source, dest):
        pass

    def get_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than this stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target and current heading
                                        in order to drive forward. When below, turn in place.
        :return:                        Rover drive effort command.
        """
        pass
    
    
    def get_clear_path_drive_command(
        self,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
    ) -> Tuple[Twist, bool]:
        """
        :param target_pos:              Target position to drive to.
        :param rover_pose:              Current rover pose.
        :param completion_thresh:       If the distance to the target is less than this stop.
        :param turn_in_place_thresh     Minimum cosine of the angle in between the target and current heading
                                        in order to drive forward. When below, turn in place.
        :return:                        Rover drive effort command.
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
