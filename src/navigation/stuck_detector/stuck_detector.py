#!/usr/bin/env python3

from typing import Optional

import numpy as np

import message_filters
import rospy
from geometry_msgs.msg import Twist
from mrover.msg import StateMachineStateUpdate
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

WINDOW_SIZE = rospy.get_param("stuck_detector/window_size")
POST_RECOVERY_GRACE_PERIOD = rospy.get_param("stuck_detector/post_recovery_grace_period")
ANGULAR_THRESHOLD = rospy.get_param("stuck_detector/angular_threshold")
LINEAR_THRESHOLD = rospy.get_param("stuck_detector/linear_threshold")


class StuckDetector:
    cmd_vel: np.ndarray
    real_vel: np.ndarray
    last_update: rospy.Time
    last_trigger_time: Optional[rospy.Time]

    def __init__(self) -> None:
        nav_status_sub = message_filters.Subscriber("nav_state", StateMachineStateUpdate)
        odometry_sub = message_filters.Subscriber("odometry", Odometry)
        cmd_vel_sub = message_filters.Subscriber("cmd_vel", Twist)

        ts = message_filters.ApproximateTimeSynchronizer(
            [nav_status_sub, cmd_vel_sub, odometry_sub], 10, 1.0, allow_headerless=True
        )
        ts.registerCallback(self.update)

        self.stuck_publisher = rospy.Publisher("nav_stuck", Bool, queue_size=1)

        self.last_trigger_time = None

        self.reset()

    def reset(self) -> None:
        self.cmd_vel = np.zeros((2, WINDOW_SIZE))
        self.real_vel = np.zeros((2, WINDOW_SIZE))
        self.last_update = rospy.Time.now()

    def update(self, nav_status: StateMachineStateUpdate, cmd_vel: Twist, odometry: Odometry) -> None:
        if self.last_update and rospy.Time.now() - self.last_update > rospy.Duration(1):
            rospy.logwarn("Resetting failure identification due to long time since last update")
            self.reset()

        # Shift over all columns and replace the first column with the new values
        # This essentially advances forward te window
        self.cmd_vel = np.roll(self.cmd_vel, 1, axis=1)
        self.real_vel = np.roll(self.real_vel, 1, axis=1)
        self.cmd_vel[:, 0] = [cmd_vel.linear.x, cmd_vel.angular.z]
        self.real_vel[:, 0] = [odometry.twist.twist.linear.x, odometry.twist.twist.angular.z]

        is_trying_to_move = np.count_nonzero(self.cmd_vel) >= WINDOW_SIZE
        linear_speed_average = np.average(np.abs(self.real_vel[0, :]))
        angular_speed_average = np.average(np.abs(self.real_vel[1, :]))
        is_not_moving = linear_speed_average < LINEAR_THRESHOLD and angular_speed_average < ANGULAR_THRESHOLD
        is_outside_grace = self.last_trigger_time is None or rospy.Time.now() - self.last_trigger_time > rospy.Duration(
            POST_RECOVERY_GRACE_PERIOD
        )
        is_not_recovery = nav_status.state != "RecoveryState"

        if is_trying_to_move and is_not_moving and is_outside_grace and is_not_recovery:
            rospy.logwarn("Detecting rover being stuck!")
            is_stuck = True
            self.last_trigger_time = rospy.Time.now()
        else:
            is_stuck = False

        self.stuck_publisher.publish(is_stuck)

        self.last_update = rospy.Time.now()


def main() -> None:
    rospy.init_node("stuck_detector")
    StuckDetector()
    rospy.spin()


if __name__ == "__main__":
    main()
