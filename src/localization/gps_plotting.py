# import matplotlib.pyplot as plt
# from pymap3d.enu import geodetic2enu
# import pandas as pd
# import numpy as np

# ref_lat = 42.293195
# ref_lon = -83.7096706
# ref_alt = 234.1

# linearized_pose = pd.read_csv("/home/daniel/catkin_ws/src/mrover/src/localization/linearized_pose.csv")


# lin_pose_x = linearized_pose["Orientation_X"].to_numpy()
# lin_pose_y = linearized_pose["Orientation_Y"].to_numpy()
# lin_pose_z = linearized_pose["Orientation_Z"].to_numpy()
# lin_pose_w = linearized_pose["Orientation_W"].to_numpy()

# time = linearized_pose["Timestamp"].to_numpy()


# rec_yaw = []
# rec_time = []

# for x, y, z, w, time in zip(lin_pose_x, lin_pose_y, lin_pose_z, lin_pose_w, time):
#     siny_cosp = 2 * (w * z + x * y)
#     cosy_cosp = 1 - 2 * (y * y + z * z)
#     yaw = np.arctan2(siny_cosp, cosy_cosp)

#     rec_yaw.append(yaw)
#     rec_time.append(time)

# for i in range(1, len(rec_time)):
#     rec_time[i] = rec_time[i] - rec_time[0]

# rec_time[0] = 0

# plt.figure(figsize=(10, 10))
# plt.plot(rec_time, rec_yaw, color="red", label="right")
# plt.xlabel("time (s)")
# plt.ylabel("yaw (rad)")
# plt.title("yaw vs time stationary")
# plt.legend()
# plt.savefig("rtk_plot.png")
# plt.show()

import matplotlib.pyplot as plt
from pymap3d.enu import geodetic2enu
import pandas as pd
import numpy as np

ref_lat = 42.293195
ref_lon = -83.7096706
ref_alt = 234.1
gps_readings = pd.read_csv("/home/daniel/catkin_ws/src/mrover/src/localization/stationary.csv")
left_fix = pd.read_csv("/home/daniel/catkin_ws/src/mrover/src/localization/left_fix_status.csv")
right_fix = pd.read_csv("/home/daniel/catkin_ws/src/mrover/src/localization/right_fix_status.csv")

left = left_fix["RTK_fix"].to_numpy()
right = right_fix["RTK_fix"].to_numpy()

left_arr = []
right_arr = []
for i in left_fix:
    left_arr.append(i)
for j in right_fix:
    right_arr.append(i)

gps_time = gps_readings["Timestamp"].to_numpy()
gps_type = gps_readings["GPS Type"].to_numpy()
gps_lat = gps_readings["Latitude"].to_numpy()
gps_long = gps_readings["Longitude"].to_numpy()
gps_alt = gps_readings["Altitude"].to_numpy()
print(len(left), len(right), len(gps_time))

left_time = []
right_time = []
left_x = []
left_y = []
left_z = []
right_x = []
right_y = []
right_z = []

for gps_type, lat, lon, alt, time in zip(gps_type, gps_lat, gps_long, gps_alt, gps_time):
    if gps_type == "/left_gps_driver/fix":
        pose = geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt, deg=True)
        left_x.append(pose[0])
        left_y.append(pose[1])
        left_z.append(pose[2])
        left_time.append(time)
    else:
        pose = geodetic2enu(lat, lon, alt, ref_lat, ref_lon, ref_alt, deg=True)
        right_x.append(pose[0])
        right_y.append(pose[1])
        right_z.append(pose[2])
        right_time.append(time)


plt.figure(figsize=(10, 10))
plt.scatter(left_time, left_x, left_y, color="black", label="left x and left y")
# plt.scatter(left_time, left_y, color="orange", label="left y")
plt.scatter(right_time, right_x, right_y, color="green", label="right x and right y")
# plt.scatter(right_time, right_y, color="blue", label="right x")
plt.scatter(left_time, left, color="red", label="left fix")
plt.scatter(right_time, right, color="violet", label="right fix")
# plt.scatter(left_x, left_y, color='blue', label='left', s=1)
plt.xlabel("time")
plt.ylabel("pos")
plt.title("RTK Stationary test")
plt.legend()
plt.show()
# plt.savefig("rtk_plot.png")