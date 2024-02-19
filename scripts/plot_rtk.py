import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymap3d.enu import geodetic2enu
import pandas as pd
import numpy as np

ref_lat = 42.293195
ref_lon = -83.7096706
ref_alt = 234.1
rtk = pd.read_csv("~/catkin_ws/src/mrover/bag/rtk_linearized_pose.csv")
no_rtk = pd.read_csv("~/catkin_ws/src/mrover/bag/no_rtk_linearized_pose.csv")

# right_gps_lat = right_gps["field.latitude"].to_numpy()
# right_gps_long = right_gps["field.longitude"].to_numpy()
# right_gps_alt = right_gps["field.altitude"].to_numpy()
# right_x = []
# right_y = []
# right_z = []
# for lat, lon, alt in zip(right_gps_lat,right_gps_long,right_gps_alt):
#     pose = geodetic2enu(lat,lon,alt, ref_lat,ref_lon, ref_alt, deg=True)
#     right_x.append(pose[0])
#     right_y.append(pose[1])
#     right_z.append(pose[2])

rtk_xs = rtk["field.pose.pose.position.x"].to_numpy()
rtk_ys = rtk["field.pose.pose.position.y"].to_numpy()
no_rtk_xs = no_rtk["field.pose.pose.position.x"].to_numpy()
no_rtk_ys = no_rtk["field.pose.pose.position.y"].to_numpy()

# plt.figure()
# plt.scatter(rtk_xs, rtk_ys, color='red', label='RTK', s=1)
# plt.scatter(no_rtk_xs, no_rtk_ys, color='blue', label='No RTK', s=1)
# plt.xlabel('x (m)')
# plt.ylabel('y (m)')
# plt.title('RTK vs No RTK')
# plt.legend()
# plt.grid(True)
# plt.show()

fig, ax = plt.subplots(1, 2)
scat1 = ax[0].scatter(rtk_xs[0], rtk_ys[0], color='red', label='RTK', s=3)
scat2 = ax[1].scatter(no_rtk_xs[0], no_rtk_ys[0], color='blue', label='No RTK', s=3)
# ax[0].set(xlim=(-50, 10), ylim=(-5, 90), xlabel='x (m)', ylabel='y (m)', title='RTK vs No RTK')
# ax[0].grid(True)
ax[0].set(xlim=(-50, 10), ylim=(-5, 90), xlabel='x (m)', ylabel='y (m)', title='RTK')
ax[0].grid(True)
ax[1].set(xlim=(-50, 10), ylim=(-5, 90), xlabel='x (m)', ylabel='y (m)', title='No RTK')
ax[1].grid(True)
# ax[2].legend()

def update(frame):
    rtk_x = rtk_xs[:frame]
    rtk_y = rtk_ys[:frame]
    data = np.stack((rtk_x, rtk_y)).T
    scat1.set_offsets(data)
    
    no_rtk_x = no_rtk_xs[:frame]
    no_rtk_y = no_rtk_ys[:frame]
    data = np.stack((no_rtk_x, no_rtk_y)).T
    scat2.set_offsets(data)

    return scat1, scat2

ani = animation.FuncAnimation(fig, update, frames=range(len(rtk_xs)), blit=True, interval=10)
plt.show()
