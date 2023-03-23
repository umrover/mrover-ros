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
import numpy as np
import os
from pathlib import Path

class FailureIdentifier:
    """
    Used to identify if the rover is stuck. Owns a data frame that is updated with the latest data from the rover
    and then used to determine if the rover is stuck
    """
    stuck_publisher: rospy.Publisher
    _df: DataFrame
    watchdog: WatchDog
    actively_collecting: bool

    def __init__(self):
        nav_status_sub = message_filters.Subscriber("nav_status", SmachContainerStatus)
        cmd_vel_sub = message_filters.Subscriber("cmd_vel", Twist)
        drive_status_sub = message_filters.Subscriber("drive_status", MotorsStatus)
        odometry_sub = message_filters.Subscriber("global_ekf/odometry", Odometry)
        stuck_button_sub = message_filters.Subscriber("/rover_stuck", Bool)
        ts = message_filters.ApproximateTimeSynchronizer([nav_status_sub, cmd_vel_sub, drive_status_sub, odometry_sub, stuck_button_sub], 10, 0.5)
        ts.registerCallback(self.update)

        self.stuck_publisher = rospy.Publisher("/nav_stuck", Bool, queue_size=1)
        position_variables = ["x", "y", "z"]
        rotation_variables = [f'rot_{comp}' for comp in ['x', 'y', 'z', 'w']]
        velocity_variables = ["linear_velocity", "angular_velocity"]
        wheel_effort_variables = [f"wheel_{wheel_num}_effort" for wheel_num in range(6)]
        wheel_velocity_variables = [f"wheel_{wheel_num}_velocity" for wheel_num in range(6)]
        command_variables = ["cmd_vel_x", "cmd_vel_twist"]
        self.data_collecting_mode = True
        self.actively_collecting = False
        self._df = pd.DataFrame(columns=["time", "stuck"] + position_variables + rotation_variables + velocity_variables + wheel_effort_variables + wheel_velocity_variables + command_variables)
    
    def write_to_csv(self):
        """
        Writes the data frame to a csv file marked with timestamp
        """
        home = str(Path.home())
        path = os.path.join(home, "catkin_ws/src/mrover-workspace/src/navigation/failure_data")
        file_name = f"failure_data_{rospy.Time.now()}.csv"
        path = os.path.join(path, file_name)
        self._df.to_csv(f"failure_data_{rospy.Time.now()}.csv")
        rospy.loginfo("===== failure data written to csv =====")
        

    def update(self, nav_status: SmachContainerStatus, cmd_vel : Twist, drive_status : MotorsStatus, odometry : Odometry, stuck_button : Bool):
        """
        Updates the current row of the data frame with the latest data from the rover
        then appends the row to the data frame
        @param nav_status: the current state of the rover, used to determine if the rover is already recovering
        @param cmd_vel: the current command velocity of the rover
        @param drive_status: the current status of the rovers motors, has velocity and effort data
        @param odometry: the current odometry of the rover, has position and velocity data

        publishes a message to the /nav_stuck topic indicating if the rover is stuck
        """

        #if the state is 'done' or 'off', write the data frame to a csv file if we were collecting
        if nav_status.active_states[0] == "done" or nav_status.active_states[0] == "off":
            if self.actively_collecting and self.data_collecting_mode:
                self.write_to_csv()
                self.actively_collecting = False
            #return to not collect any data
            return
        
        #create a new row for the data frame
        self.actively_collecting = True
        cur_row = {}
        cur_row["time"] = rospy.Time.now()

        #if the stuck button is pressed, the rover is stuck (as indicated by the GUI)
        if stuck_button.data:
            cur_row["stuck"] = 1
        else:
            cur_row["stuck"] = 0

        #get the command velocity from the cmd_vel message
        cur_row["cmd_vel_x"] = cmd_vel.linear.x
        cur_row["cmd_vel_twist"] = cmd_vel.angular.z

        #get the x, y, z position of the rover from odometry message
        cur_row["x"] = odometry.pose.pose.position.x
        cur_row["y"] = odometry.pose.pose.position.y
        cur_row["z"] = odometry.pose.pose.position.z

        #get the x, y, z, w rotation of the rover from odometry message
        cur_row["rot_x"] = odometry.pose.pose.orientation.x
        cur_row["rot_y"] = odometry.pose.pose.orientation.y
        cur_row["rot_z"] = odometry.pose.pose.orientation.z
        cur_row["rot_w"] = odometry.pose.pose.orientation.w

        #get the linear and angular velocity of the rover from odometry message
        linear_velocity_norm = np.linalg.norm(np.array([odometry.twist.twist.linear.x, odometry.twist.twist.linear.y, odometry.twist.twist.linear.z]))
        cur_row["linear_velocity"] = linear_velocity_norm
        cur_row["angular_velocity"] = odometry.twist.twist.angular.z

        #get the wheel effort and velocity from the drive status message
        for wheel_num in range(6):
            cur_row[f"wheel_{wheel_num}_effort"] = drive_status.joint_states.effort[wheel_num]
            cur_row[f"wheel_{wheel_num}_velocity"] = drive_status.joint_states.velocity[wheel_num]

        #update the data frame with the cur row
        self._df = self._df.append(cur_row, ignore_index=True)
       
        #publish the watchdog status if the nav state is not recovery
        if nav_status.active_states[0] != "recovery":
            self.stuck_publisher.publish(Bool(self.watchdog.is_stuck(self._df)))
        


def main():
    rospy.loginfo("===== failure identification starting =====")
    rospy.init_node("failure_id")
    FailureIdentifier()
    rospy.spin()

if __name__ == "__main__":
    main()
