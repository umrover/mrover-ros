import matplotlib.pyplot as plt
import numpy as np

# from gps_linearization import GPSLinearization
# linearization = GPSLinearization()

from tf.transformations import *
from util.np_utils import numpify

# imu_orientationon = np.array([0.00025433645374749436, -0.0012950453686585608, 0.33316308291259367, 0.9428682932173706])
# gps_orientation = np.array([0,0,0.33316258923859826, 0.9428693913431663])

imu_orientation = np.array([0, 0, 0, 1])
gps_orientation = np.array([0, 0, 1, 0])

# find and compare offset between imu gps
imu_heading = euler_from_quaternion(imu_orientation)[2]
imu_heading_matrix = euler_matrix(0, 0, imu_heading, axes="sxyz")
# print(imu_heading_matrix)

gps_heading = euler_from_quaternion(gps_orientation)[2]
gps_heading_matrix = euler_matrix(0, 0, gps_heading, axes="sxyz")
print(gps_heading_matrix)
rtk_offset = np.matmul(inverse_matrix(gps_heading_matrix), imu_heading_matrix)
print(rtk_offset)


# if rtk_offset is set, apply offset, pose_msg contains imu quaternion
imu_rotation = euler_from_quaternion(imu_orientation)
imu_rotation_matrix = euler_matrix(ai=imu_rotation[0], aj=imu_rotation[1], ak=imu_rotation[2], axes="sxyz")

offsetted_rotation_matrix = np.matmul(imu_rotation_matrix, rtk_offset)
offsetted_euler = euler_from_matrix(offsetted_rotation_matrix)

imu_orientation = quaternion_from_euler(offsetted_euler[0], offsetted_euler[1], offsetted_euler[2], axes="sxyz")

# # imu quaternion
imu_quat = imu_orientation / np.linalg.norm(imu_orientation)
print("imu quat")
print(imu_quat)
