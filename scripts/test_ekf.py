#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from mrover.msg import Waypoint, WaypointType
from smach_msgs.msg import SmachContainerStatus
import message_filters
from util.course_publish_helpers import convert_waypoint_to_gps, publish_waypoints
from util.SE3 import SE3


class EKF_Test:
    def __init__(self):
        self.raw_data = []
        self.ekf_data = []
        self.gt_data = []
        self.nav_state = ""

        # subscribe to both odometry topics and synchronize them
        raw_sub = message_filters.Subscriber("gps/pose", PoseWithCovarianceStamped)
        ekf_sub = message_filters.Subscriber("odometry/filtered", Odometry)
        truth_sub = message_filters.Subscriber("ground_truth", Odometry)
        rospy.Subscriber("smach/container_status", SmachContainerStatus, self.nav_status_callback)

        ts = message_filters.TimeSynchronizer([raw_sub, ekf_sub, truth_sub], 10)
        ts.registerCallback(self.odoms_callback)

    def odoms_callback(self, raw_pose_msg, ekf_odom_msg, truth_odom_msg):
        msgs = [raw_pose_msg, ekf_odom_msg, truth_odom_msg]
        datas = [self.raw_data, self.ekf_data, self.gt_data]

        for msg, data in zip(msgs, datas):
            pos = msg.pose.pose.position
            q = msg.pose.pose.orientation
            point = np.array([pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w])
            data.append(point)

    def nav_status_callback(self, status_msg):
        self.nav_state = status_msg.active_states[0]

    def execute_path(self):
        square_path = [
            (
                Waypoint(fiducial_id=0, tf_id="course0", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([3, 3, 0])),
            ),
            (
                Waypoint(fiducial_id=1, tf_id="course1", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([0, 6, 0])),
            ),
            (
                Waypoint(fiducial_id=2, tf_id="course2", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([-3, 3, 0])),
            ),
            (
                Waypoint(fiducial_id=3, tf_id="course3", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([0, 0, 0])),
            ),
        ]
        line_path = [
            (
                Waypoint(fiducial_id=0, tf_id="course0", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([3, 0, 0])),
            ),
        ]
        publish_waypoints([convert_waypoint_to_gps(waypoint) for waypoint in line_path])

        # wait until we reach the waypoint
        while self.nav_state != "DoneState":
            rospy.sleep(0.1)

        self.plot_data()

    def plot_data(self):
        raw_arr = np.vstack(self.raw_data)
        ekf_arr = np.vstack(self.ekf_data)
        gt_arr = np.vstack(self.gt_data)
        # TODO: do properly with timestamps
        times = np.arange(ekf_arr.shape[0])
        raw_pos_err = np.linalg.norm(gt_arr[:, :3] - raw_arr[:, :3], axis=1)
        pos_err = np.linalg.norm(gt_arr[:, :3] - ekf_arr[:, :3], axis=1)

        raw_pos_rmse = np.sqrt(np.sum(raw_pos_err**2) / raw_pos_err.shape[0])
        pos_rmse = np.sqrt(np.sum(pos_err**2) / pos_err.shape[0])
        # print(pos_err.shape)
        # ang_err = np.linalg.norm(gt_arr[:, 3:] - ekf_arr[:, 3:], axis=1)
        # print(ang_err.shape)

        fig, axs = plt.subplots(2, 2)
        axs[0, 0].plot(raw_arr[:, 0], raw_arr[:, 1], "tab:red", label="Raw GPS")
        axs[0, 0].plot(ekf_arr[:, 0], ekf_arr[:, 1], "tab:green", label="EKF")
        axs[0, 0].plot(gt_arr[:, 0], gt_arr[:, 1], "tab:blue", label="Ground Truth")
        axs[0, 0].set_xlabel("x (meters)")
        axs[0, 0].set_ylabel("y (meters)")
        axs[0, 0].set_title("Rover Path")
        axs[0, 0].legend()

        axs[0, 1].plot(times, pos_err, "tab:green", label=f"EKF, RMSE = {pos_rmse:.3f}")
        axs[0, 1].plot(times, raw_pos_err, "tab:red", label=f"Raw GPS, RMSE = {raw_pos_rmse:.3f}")
        # axs[1].plot(times, ang_err, "b-", label="angle error")
        axs[0, 1].set_xlabel("time (no unit)")
        axs[0, 1].set_ylabel("error (meters)")
        axs[0, 1].set_title("Position Error")
        axs[0, 1].legend()

        axs[1, 0].plot(times, ekf_arr[:, 0], "tab:green", label="EKF")
        axs[1, 0].plot(times, raw_arr[:, 0], "tab:red", label="Raw GPS")
        axs[1, 0].plot(times, gt_arr[:, 0], "tab:blue", label="Ground Truth")
        axs[1, 0].set_xlabel("time (no unit)")
        axs[1, 0].set_ylabel("x position")
        axs[1, 0].set_title("X Position vs Time")
        axs[1, 0].legend()

        axs[1, 1].plot(times, ekf_arr[:, 1], "tab:green", label="EKF")
        axs[1, 1].plot(times, raw_arr[:, 1], "tab:red", label="Raw GPS")
        axs[1, 1].plot(times, gt_arr[:, 1], "tab:blue", label="Ground Truth")
        axs[1, 1].set_xlabel("time (no unit)")
        axs[1, 1].set_ylabel("y position")
        axs[1, 1].set_title("Y Position vs Time")
        axs[1, 1].legend()

        plt.show()


def main():
    rospy.init_node("ekf_tester")
    t = EKF_Test()
    t.execute_path()


if __name__ == "__main__":
    main()
