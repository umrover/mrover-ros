#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from mrover.msg import Waypoint, WaypointType
from smach_msgs.msg import SmachContainerStatus
import message_filters
from util.course_publish_helpers import convert_waypoint_to_gps, publish_waypoints
from util.SE3 import SE3


class EKF_Test:
    def __init__(self):
        self.ekf_data = []
        self.gt_data = []
        self.nav_state = ""

        # subscribe to both odometry topics and synchronize them
        ekf_sub = message_filters.Subscriber("odometry/filtered", Odometry)
        truth_sub = message_filters.Subscriber("ground_truth", Odometry)
        rospy.Subscriber("smach/container_status", SmachContainerStatus, self.nav_status_callback)

        ts = message_filters.TimeSynchronizer([ekf_sub, truth_sub], 10)
        ts.registerCallback(self.odoms_callback)

    def odoms_callback(self, ekf_odom_msg, truth_odom_msg):
        ekf_pos = ekf_odom_msg.pose.pose.position
        ekf_q = ekf_odom_msg.pose.pose.orientation
        gt_pos = truth_odom_msg.pose.pose.position
        gt_q = truth_odom_msg.pose.pose.orientation

        ekf_point = np.array([ekf_pos.x, ekf_pos.y, ekf_pos.z, ekf_q.x, ekf_q.y, ekf_q.z, ekf_q.w])
        gt_point = np.array([gt_pos.x, gt_pos.y, gt_pos.z, gt_q.x, gt_q.y, gt_q.z, gt_q.w])

        self.ekf_data.append(ekf_point)
        self.gt_data.append(gt_point)

    def nav_status_callback(self, status_msg):
        self.nav_state = status_msg.active_states[0]

    def execute_path(self):
        waypoints = [
            (
                Waypoint(fiducial_id=0, tf_id="course0", type=WaypointType(val=WaypointType.NO_SEARCH)),
                SE3(position=np.array([-2, -2, 0])),
            ),
            # (Waypoint(fiducial_id=0, tf_id="course1"), SE3(position=np.array([-3, -3, -1]))),
            # (Waypoint(fiducial_id=0, tf_id="course2"), SE3(position=np.array([-5, -5, 0]))),
        ]
        publish_waypoints([convert_waypoint_to_gps(waypoint) for waypoint in waypoints])

        # wait until we reach the waypoint
        while self.nav_state != "DoneState":
            rospy.sleep(0.1)

        self.plot_data()

    def plot_data(self):
        ekf_arr = np.vstack(self.ekf_data)
        gt_arr = np.vstack(self.gt_data)
        # TODO: do properly with timestamps
        times = np.arange(ekf_arr.shape[0])
        pos_err = np.linalg.norm(gt_arr[:, :3] - ekf_arr[:, :3], axis=1)
        print(pos_err.shape)
        ang_err = np.linalg.norm(gt_arr[3:] - ekf_arr[3:], axis=1)

        fig, axs = plt.subplots(1, 2)
        axs[0].plot(ekf_arr[:, 0], ekf_arr[:, 1], "r-", label="EKF")
        axs[0].plot(gt_arr[:, 0], gt_arr[:, 1], "b-", label="Ground Truth")
        axs[0].legend()

        axs[1].plot(times, pos_err, "r-", label="position error")
        axs[1].legend()
        plt.show()


def main():
    rospy.init_node("ekf_tester")
    t = EKF_Test()
    t.execute_path()
    # rospy.spin()


if __name__ == "__main__":
    main()
