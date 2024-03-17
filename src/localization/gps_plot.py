import pandas as pd
import matplotlib.pyplot as plt

linearized_df = pd.read_csv("./linearized_heading.csv")
imu_df = pd.read_csv("./imu_heading.csv")

l_time = linearized_df["time"]
i_time = imu_df["time"]

linearized_roll = linearized_df["linearized_roll"]
imu_roll = imu_df["imu_roll"]

linearized_pitch = linearized_df["linearized_pitch"]
imu_pitch = imu_df["imu_pitch"]

linearized_heading = linearized_df["linearized_heading"]
imu_heading = imu_df["imu_heading"]

fig, (ax1, ax2, ax3) = plt.subplots(3, 1)

ax1.plot(l_time, linearized_roll, 'r-', label="linearized")
ax1.plot(i_time, imu_roll, 'b-', label="imu")
ax1.legend(loc="best")
ax1.set_title("Roll")

ax2.plot(l_time, linearized_pitch, 'r-', label="linearized")
ax2.plot(i_time, imu_pitch, 'b-', label="imu")
ax2.legend(loc="best")
ax2.set_title("Pitch")

ax3.plot(l_time, linearized_heading, 'r-', label="linearized")
ax3.plot(i_time, imu_heading, 'b-', label="imu")
ax3.legend(loc="best")
ax3.set_title("Heading")

plt.show()