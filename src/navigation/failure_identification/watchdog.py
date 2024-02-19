from typing import Tuple

import rospy
import numpy as np
import pandas as pd

from util.SO3 import SO3
from util.ros_utils import get_rosparam

WINDOW_SIZE = get_rosparam("watchdog/window_size", 100)
ANGULAR_THRESHOLD = get_rosparam("watchdog/angular_threshold", 0.001)
LINEAR_THRESHOLD = get_rosparam("watchdog/linear_threshold", 0.55)


class WatchDog:
    def get_start_end_positions(self, dataframe: pd.DataFrame) -> Tuple[np.ndarray, np.ndarray]:
        start_x, start_y, start_z = dataframe["x"].iloc[0], dataframe["y"].iloc[0], dataframe["z"].iloc[0]
        start_pos = np.array([start_x, start_y, start_z])
        end_x, end_y, end_z = dataframe["x"].iloc[-1], dataframe["y"].iloc[-1], dataframe["z"].iloc[-1]
        end_pos = np.array([end_x, end_y, end_z])
        return start_pos, end_pos

    def get_start_end_rotations(self, dataframe: pd.DataFrame) -> Tuple[SO3, SO3]:
        start_rot = np.array(
            [
                dataframe["rot_x"].iloc[0],
                dataframe["rot_y"].iloc[0],
                dataframe["rot_z"].iloc[0],
                dataframe["rot_w"].iloc[0],
            ]
        )
        end_rot = np.array(
            [
                dataframe["rot_x"].iloc[-1],
                dataframe["rot_y"].iloc[-1],
                dataframe["rot_z"].iloc[-1],
                dataframe["rot_w"].iloc[-1],
            ]
        )
        return SO3(start_rot), SO3(end_rot)

    def get_start_end_time(self, dataframe: pd.DataFrame) -> Tuple[float, float]:
        start_time = dataframe["time"].iloc[0]
        end_time = dataframe["time"].iloc[-1]
        return start_time, end_time

    def check_angular_stuck(self, delta_time, delta_rot, dataframe) -> bool:
        # check to make sure all cmd_vel_twist values are the same sign and cmd_vel_x values are 0
        turn_are_all_same_sign = (
            dataframe["cmd_vel_twist"].apply(np.sign).eq(np.sign(dataframe["cmd_vel_twist"].iloc[0])).all()
        )
        linear_are_all_zero = dataframe["cmd_vel_x"].eq(0).all()
        angular_are_non_zero = dataframe["cmd_vel_twist"].ne(0).all()
        rospy.logdebug(
            f"Turn all same sign: f{turn_are_all_same_sign}, linear all zero: f{linear_are_all_zero}, angular all nonzero: f{angular_are_non_zero}"
        )
        if not turn_are_all_same_sign or not linear_are_all_zero:
            rospy.logdebug("Not turning")
            return False
        if not angular_are_non_zero:
            rospy.logdebug("Not turning")
            return False
        # check if the angular velocity is less than the threshold for the entire dataframe
        angular_velocity = delta_rot / delta_time
        rospy.logdebug(f"{angular_velocity=}, {ANGULAR_THRESHOLD=}")
        return abs(angular_velocity) < ANGULAR_THRESHOLD

    def check_linear_stuck(self, delta_time, delta_pos, dataframe) -> bool:
        # check to make sure all cmd_vel_x values are the same sign and cmd_vel_twist values are 0
        linear_are_all_same_sign = (
            dataframe["cmd_vel_x"].apply(np.sign).eq(np.sign(dataframe["cmd_vel_x"].iloc[0])).all()
        )
        linear_are_all_non_zero = dataframe["cmd_vel_x"].ne(0).all()
        rospy.logdebug(f"{linear_are_all_same_sign=}, {linear_are_all_non_zero=}")
        if not linear_are_all_same_sign or not linear_are_all_non_zero:
            rospy.logdebug("Not driving straight")
            return False
        # check if the average linear velocity is less than the threshold for the entire dataframe
        linear_velocity = delta_pos / delta_time
        linear_velocity = np.linalg.norm(linear_velocity)
        rospy.logdebug(f"{linear_velocity=}, {LINEAR_THRESHOLD=}")
        return linear_velocity < LINEAR_THRESHOLD

    def is_stuck(self, dataframe: pd.DataFrame) -> bool:
        if len(dataframe) > WINDOW_SIZE:
            dataframe_sliced = dataframe.tail(WINDOW_SIZE)
            # get the start and end position and rotation
            start_pos, end_pos = self.get_start_end_positions(dataframe_sliced)
            start_rot, end_rot = self.get_start_end_rotations(dataframe_sliced)
            # get the start and end time
            start_time, end_time = self.get_start_end_time(dataframe_sliced)
            rospy.logdebug(f"{start_time=}, {end_time=}")
            # get the delta time and delta position
            delta_time = end_time - start_time
            delta_pos = end_pos - start_pos
            # use SO3 distance to find the delta rotation
            delta_rot = start_rot.rot_distance_to(end_rot)
            # check if the rover is stuck
            if self.check_angular_stuck(delta_time, delta_rot, dataframe_sliced) or self.check_linear_stuck(
                delta_time, delta_pos, dataframe_sliced
            ):
                return True
        return False
