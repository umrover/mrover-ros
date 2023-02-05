from typing import Tuple

import numpy as np
import rospy

from geometry_msgs.msg import Twist
from util.SE3 import SE3
from data_collection import DataManager
import util.np_utils as npu
# from util.np_utils import angle_to_rotate
# from util.np_utils import angle_to_rotate

collector = DataManager()
MAX_DRIVING_EFFORT = 1
MIN_DRIVING_EFFORT = -1
TURNING_P = 10.0


def get_drive_command(
    target_pos: np.ndarray,
    rover_pose: SE3,
    completion_thresh: float,
    turn_in_place_thresh: float,
) -> Tuple[Twist, bool, bool]:
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

    if collector.collector_context.rover.stuck and collector.collector_context.rover.move_back:
        rover_dir *= -1

    alignment = npu.angle_to_rotate(rover_dir, target_dir)
    rospy.logerr(f"Alignment: {alignment}")

    rospy.logerr(f"TARGET DIST {target_dist}")
    if target_dist < completion_thresh:
        # getting commanded velocity into the data collection
        # rospy.logerr(f"Called make_cmd_vel_obj from drive.py")
        rospy.logerr("WITHIN COMPLETION_THRESH")
        collector.make_cmd_vel_dataframe(Twist())
        return Twist(), True, collector.collector_context.rover.watchdog.is_stuck()

    cmd_vel = Twist()
    full_turn_override = True
    if abs(alignment) < turn_in_place_thresh:
        # We are pretty aligned so we can drive straight
        rospy.logerr(f"ALIGNED")
        error = target_dist
        cmd_vel.linear.x = np.clip(error, 0.0, MAX_DRIVING_EFFORT)
        if collector.collector_context.rover.stuck and collector.collector_context.rover.move_back:
            rospy.logerr(f"GO BACKWARDS")
            cmd_vel.linear.x *= -1 #Go backwards
        full_turn_override = False

    # we want to drive the angular offset to zero so the error is just 0 - alignment
    error = alignment
    if collector.collector_context.rover.stuck and not collector.collector_context.rover.move_back:
        rospy.logerr(f"Turning in place\n")
        cmd_vel.angular.z = -0.5 #Turn in place by some amount
    else:
        rospy.logerr(f"Setting angular.z\n")
        cmd_vel.angular.z = (
            np.sign(error) if full_turn_override else np.clip(error * TURNING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
        )
    # print(cmd_vel.linear.x, cmd_vel.angular.z)
    # full_turn_override = False

    # # we want to drive the angular offset to zero so the error is just 0 - alignment
    # error = alignment
    # cmd_vel.angular.z = (
    #     np.sign(error) if full_turn_override else np.clip(error * TURNING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
    # )

    # getting commanded velocity into the data collection
    # rospy.logerr(f"Called make_cmd_vel_obj from drive.py")
    collector.make_cmd_vel_dataframe(cmd_vel)
    print(cmd_vel.linear.x, cmd_vel.angular.z)
    return cmd_vel, False, collector.collector_context.rover.watchdog.is_stuck()
