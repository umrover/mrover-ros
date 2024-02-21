import pandas as pd
import matplotlib.pyplot as plt


linearized_heading_df = pd.read_csv("./linearized_heading.csv")
imu_heading_df = pd.read_csv("./imu_heading.csv")

l_time = linearized_heading_df["time"]
linearized_heading = linearized_heading_df["linearized_heading"]
imu_heading = imu_heading_df["imu_heading"]
i_time = imu_heading_df["time"]

plt.plot(l_time, linearized_heading, 'r-', label="linearized")
plt.plot(i_time, imu_heading, 'b-', label="imu")
plt.legend(loc='best')
plt.show()