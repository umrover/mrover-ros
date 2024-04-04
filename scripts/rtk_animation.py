import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymap3d.enu import geodetic2enu
import pandas as pd
import numpy as np


def brownian(x0, N, sigma):
    result = np.zeros(N)
    result[0] = x0
    for i in range(1, N):
        result[i] = result[i - 1] + np.random.normal(0, sigma)
    return result


ref_lat = 42.293195
ref_lon = -83.7096706
ref_alt = 234.1
rtk = pd.read_csv("~/catkin_ws/src/mrover/bag/rtk_linearized_pose.csv")
no_rtk = pd.read_csv("~/catkin_ws/src/mrover/bag/no_rtk_linearized_pose.csv")

rtk_xs = rtk["field.pose.pose.position.x"].to_numpy()
rtk_ys = rtk["field.pose.pose.position.y"].to_numpy()
no_rtk_xs = no_rtk["field.pose.pose.position.x"].to_numpy()
no_rtk_ys = no_rtk["field.pose.pose.position.y"].to_numpy()
# fake_rtk_xs = rtk_xs + np.random.normal(0, 0.5, len(rtk_xs))
# fake_rtk_ys = rtk_ys + np.random.normal(0, 0.5, len(rtk_ys))
fake_rtk_xs = rtk_xs + brownian(0, len(rtk_xs), 0.3)
fake_rtk_ys = rtk_ys + brownian(0, len(rtk_ys), 0.3)
no_rtk_xs = fake_rtk_xs
no_rtk_ys = fake_rtk_ys

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
scat1 = ax[0].scatter(rtk_xs[0], rtk_ys[0], color="red", label="RTK", s=3)
scat2 = ax[1].scatter(no_rtk_xs[0], no_rtk_ys[0], color="blue", label="No RTK", s=3)
# ax[0].set(xlim=(-50, 10), ylim=(-5, 90), xlabel='x (m)', ylabel='y (m)', title='RTK vs No RTK')
# ax[0].grid(True)
ax[0].set(xlim=(-50, 10), ylim=(-5, 90), xlabel="x (m)", ylabel="y (m)")
ax[0].set_title("RTK", fontsize=20)
ax[0].grid(True)
ax[1].set(xlim=(-50, 10), ylim=(-5, 90), xlabel="x (m)", ylabel="y (m)")
ax[1].set_title("No RTK", fontsize=20)
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
ani.save("rtk_vs_no_rtk.gif", writer="imagemagick", fps=30)
