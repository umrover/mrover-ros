from __future__ import annotations
from typing import Tuple, Optional
import numpy as np
import rospy
from enum import Enum

from geometry_msgs.msg import Twist
from util.SE3 import SE3
from util.np_utils import angle_to_rotate
from util.ros_utils import get_rosparam

default_constants = {
    "max_driving_effort": 1.0,
    "min_driving_effort": -1.0,
    "turning_p": 10.0,
    "max_turning_effort": 1.0,
    "min_turning_effort": -1.0,
    "driving_p": 1.0,
}
ODOM_CONSTANTS = get_rosparam("drive/odom", default_constants)
MAP_CONSTANTS = get_rosparam("drive/map", default_constants)


class Driver:
    _last_angular_error: Optional[float] = None

    class DriveMode(Enum):
        TURN_IN_PLACE = 1
        DRIVE_FORWARD = 2
        STOPPED = 3

    _driver_state: DriveMode = DriveMode.STOPPED

    def _is_complete(self, target_pos: np.ndarray, rover_pose: SE3, completion_thresh: float) -> bool:
        rover_pos = rover_pose.position
        target_dir = target_pos - rover_pos
        target_dist = np.linalg.norm(target_dir)
        return bool(target_dist < completion_thresh)

    def get_drive_command(
        self: Driver,
        target_pos: np.ndarray,
        rover_pose: SE3,
        completion_thresh: float,
        turn_in_place_thresh: float,
        drive_back: bool = False,
        in_odom: bool = False,
    ) -> Tuple[Twist, bool]:

        constants = ODOM_CONSTANTS if in_odom else MAP_CONSTANTS
        MAX_DRIVING_EFFORT = constants["max_driving_effort"]
        MIN_DRIVING_EFFORT = constants["min_driving_effort"]
        MAX_TURNING_EFFORT = constants["max_turning_effort"]
        MIN_TURNING_EFFORT = constants["min_turning_effort"]
        TURNING_P = constants["turning_p"]
        DRIVING_P = constants["driving_p"]

        rover_dir = rover_pose.rotation.direction_vector()
        rover_dir[2] = 0

        if drive_back:
            rover_dir *= -1
        
        rover_pos = rover_pose.position
        rover_pos[2] = 0

        target_pos[2] = 0
        target_dir = target_pos - rover_pos

        angle_error = angle_to_rotate(rover_dir, target_dir)

        output = Twist(), False
        print(f"drive state: {self._driver_state}")
        if self._driver_state == self.DriveMode.STOPPED:
            print("stopped")
            if self._is_complete(target_pos, rover_pose, completion_thresh):
                output = Twist(), True
            else:
                self._driver_state = self.DriveMode.TURN_IN_PLACE
                output = Twist(), False

        elif self._driver_state == self.DriveMode.TURN_IN_PLACE:
            print("turn in place")
            if self._is_complete(target_pos, rover_pose, completion_thresh):
                self._driver_state = self.DriveMode.STOPPED
                output = Twist(), True

            if abs(angle_error) < turn_in_place_thresh:
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                output = Twist(), False

            if self._last_angular_error is not None and np.sign(self._last_angular_error) != np.sign(angle_error):
                self._driver_state = self.DriveMode.DRIVE_FORWARD
                output = Twist(), False

            cmd_vel = Twist()
            cmd_vel.angular.z = np.clip(angle_error * TURNING_P, MIN_TURNING_EFFORT, MAX_TURNING_EFFORT)
            output = cmd_vel, False

        elif self._driver_state == self.DriveMode.DRIVE_FORWARD:
            print("drive forward")
            if self._is_complete(target_pos, rover_pose, completion_thresh):
                self._driver_state = self.DriveMode.STOPPED
                output = Twist(), True

            last_angular_was_inside = (
                self._last_angular_error is not None and abs(self._last_angular_error) < turn_in_place_thresh
            )
            cur_angular_is_outside = abs(angle_error) >= turn_in_place_thresh
            if cur_angular_is_outside and last_angular_was_inside:
                self._driver_state = self.DriveMode.TURN_IN_PLACE
                output = Twist(), False

            cmd_vel = Twist()
            distance_error = np.linalg.norm(target_dir)
            cmd_vel.linear.x = np.clip(distance_error * DRIVING_P, MIN_DRIVING_EFFORT, MAX_DRIVING_EFFORT)
            cmd_vel.angular.z = np.clip(angle_error * TURNING_P, MIN_TURNING_EFFORT, MAX_TURNING_EFFORT)
            output = cmd_vel, False

        if drive_back:
            cmd_vel.linear.x *= -1
        
        self._last_angular_error = angle_error
        return output
