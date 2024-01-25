import matplotlib.pyplot as plt
from pymap3d.enu import geodetic2enu
import pandas as pd
import numpy as np

ref_lat = 42.293195
ref_lon = -83.7096706
ref_alt = 234.1
linearized_pose = pd.read_csv("/home/rahul/catkin_ws/src/mrover/rtk_linearized_pose.csv")
#right_gps = pd.read_csv("/Users/nehakankanala/PycharmProjects/plot_rtk_data/rtk_right_gps.csv")

lin_pose_x = linearized_pose["field.pose.pose.orientation.x"].to_numpy()
lin_pose_y = linearized_pose["field.pose.pose.orientation.y"].to_numpy()
lin_pose_z = linearized_pose["field.pose.pose.orientation.z"].to_numpy()
lin_pose_w = linearized_pose["field.pose.pose.orientation.w"].to_numpy()

time = linearized_pose["%time"].to_numpy()


rec_yaw = []
rec_time = []

for x, y, z, w, time in zip(lin_pose_x,lin_pose_y,lin_pose_z, lin_pose_w, time):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    rec_yaw.append(yaw)
    rec_time.append(time)
    print(yaw, time)


# rec_time = rec_time[20:]
# rec_yaw = rec_yaw[20:]

# for i in range(len(rec_time)):
#     rec_time[i] = rec_time[i] - rec_time[0]


plt.figure(figsize=(10, 10))
plt.plot(rec_time, rec_yaw, color='red', label='right')
plt.xlabel('time (s)')
plt.ylabel('yaw (rad)')
plt.title('yaw vs time')
plt.legend()
plt.savefig("rtk_plot.png")
plt.show()
