import numpy as np
from geometry_msgs.msg import Twist
from typing import Tuple

MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 100.0

def get_drive_command(target_pos : np.ndarray, rover_pos : np.ndarray, rover_dir : np.ndarray, completion_tolerance : float, turn_in_place_tolerance_rad : float) -> Tuple[Twist, bool]:
    """generalized drive to target command, returns a 
        :param: target_pos :  target position ndarray, rover_pos : current rover position ndarray, rover_dir : current rover rotatation (must be normalized) ndarray, completion tolerance: float scalar distance threshold, drive_fwd_thresh : float angular threshold for turning vs driving
        :return: Twist, Bool : twist is how to get to a particular pos along with a boolean of whether it is done or not
    """
    # Get vector from rover to target
    target_dir = target_pos - rover_pos
    target_dist = np.linalg.norm(target_dir)
    if target_dist == 0:
        target_dist = np.finfo(float).eps
    # Normalize direction
    target_dir /= target_dist
    # Both vectors are unit vectors so the dot product magnitude is 0-1
    # 0 alignment is perpendicular, 1 is parallel (fully aligned)
    alignment = np.dot(target_dir, rover_dir)

    if target_dist < completion_tolerance:
       return Twist(), True
    
    cmd_vel = Twist()
    if alignment > turn_in_place_tolerance_rad:
        # We are pretty aligned so we can drive straight
        error = target_dist
        cmd_vel.linear.x = np.clip(error, 0.0, MAX_DRIVING_EFFORT)
    # Determine the sign of our effort by seeing if we are to the left or to the right of the target
    # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
    perp_alignment = rover_dir[0] * -target_dir[1] + rover_dir[1] * target_dir[0]
    sign = -np.sign(perp_alignment)
    # 1 is target alignment (dot product of two normalized vectors that are parrallel is 1)
    error = 1.0 - alignment
    cmd_vel.angular.z = np.clip(error * TURNING_P * sign, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
    return cmd_vel, False