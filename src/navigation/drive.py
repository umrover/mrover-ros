from typing import Tuple

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from util.SE3 import SE3
import data_collection
from util.np_utils import angle_to_rotate

rospy.logerr(f"Make DataCollector in drive.py")
collector = data_collection.DataCollector()
MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 10.0


def get_drive_command(
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
        # getting commanded velocity into the data collection
        rospy.logerr(f"Called make_cmd_vel_obj from drive.py")
        collector.make_cmd_vel_obj(Twist())
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
    # Determine the sign of our effort by seeing if we are to the left or to the right of the target
    # This is done by dotting rover_dir and target_dir rotated 90 degrees ccw
    perp_alignment = target_dir[0] * -rover_dir[1] + target_dir[1] * rover_dir[0]
    sign = np.sign(perp_alignment)
    # 1 is target alignment (dot product of two normalized vectors that are parallel is 1)
    error = 1.0 - alignment
    cmd_vel.angular.z = np.clip(error * TURNING_P * sign, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
    # getting commanded velocity into the data collection
    rospy.logerr(f"Called make_cmd_vel_obj from drive.py")
    collector.make_cmd_vel_obj(cmd_vel)
    return cmd_vel, False
