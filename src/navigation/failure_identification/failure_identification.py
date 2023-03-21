#!/usr/bin/env python3
import rospy
import message_filters
from mrover.msg import MotorsStatus 
from geometry_msgs.msg import Twist, Odometry
from smach_msgs.msg import SmachContainerStatus
from std_msgs.msg import Bool
import pandas as pd
from pandas import DataFrame 
from watchdog import WatchDog

class FailureIdentifier:
    
    stuck_publisher: rospy.Publisher
    _df: DataFrame
    watchdog: WatchDog

    def __init__(self):
        nav_status_sub = message_filters.Subscriber("nav_status", SmachContainerStatus)
        cmd_vel_sub = message_filters.Subscriber("cmd_vel", Twist)
        drive_status_sub = message_filters.Subscriber("drive_status", MotorsStatus)
        odometry_sub = message_filters.Subscriber("global_ekf/odometry", Odometry)
        ts = message_filters.ApproximateTimeSynchronizer([nav_status_sub, cmd_vel_sub, drive_status_sub, odometry_sub], 10, 0.5)
        ts.registerCallback(self.update)

        self.stuck_publisher = rospy.Publisher("/nav_stuck", Bool, queue_size=1)
        position_variables = ["x", "y", "z"]
        rotation_variables = [f'rot_{comp}' for comp in ['x', 'y', 'z', 'w']]
        velocity_variables = ["linear_velocity", "angular_velocity"]
        self.wheel_effort_variables = [f"wheel_{wheel_num}_effort" for wheel_num in range(6)]
        self.wheel_velocity_variables = [f"wheel_{wheel_num}_velocity" for wheel_num in range(6)]
        command_variables = ["cmd_vel_x", "cmd_vel_twist"]
        self.data_collecting_mode = True
        self._df = pd.DataFrame(columns=["time", "stuck"] + position_variables + rotation_variables + velocity_variables + wheel_effort_variables + wheel_velocity_variables + command_variables)

    def update(self, nav_status: SmachContainerStatus, cmd_vel : Twist, drive_status : MotorsStatus, odometry : Odometry):
        cur_row = {}
        cur_row["cmd_vel_x"] = cmd_vel.linear.x
        cur_row["cmd_vel_twist"] = cmd_vel.angular.z

        self._df
        self.stuck_publisher.publish(Bool(self.watchdog.is_stuck(self._df)))
        


def main():
    rospy.loginfo("===== failure identification starting =====")
    rospy.init_node("failure_id")
    # collector.set_context(context)


if __name__ == "__main__":
    main()
