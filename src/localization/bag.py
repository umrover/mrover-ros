import rosbag
import csv
from tf.transformations import *
import numpy as np

bag_file_path = "../../offset_testing.bag"

bag = rosbag.Bag(bag_file_path)

topics = ["/linearized_pose", "/imu/data"]

csv_linearized_heading = "linearized_heading.csv"
csv_imu_heading = "imu_heading.csv"

with open(csv_linearized_heading, "w", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time", "linearized_heading"])

    for _, msg, _ in bag.read_messages(topics="/linearized_pose"):
        heading = euler_from_quaternion(np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]))[2]
        time = msg.header.stamp.secs + msg.header.stamp.nsecs * (0.000000001)
        csv_writer.writerow([time, heading])

with open(csv_imu_heading, "w", newline="") as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time", "imu_heading"])

    for _, msg, _ in bag.read_messages("/imu/data"):
        heading = euler_from_quaternion(np.array([msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z, msg.imu.orientation.w]))[2]
        time = msg.header.stamp.secs + msg.header.stamp.nsecs * (0.000000001)
        csv_writer.writerow([time, heading])

bag.close()