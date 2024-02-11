import pandas as pd
import matplotlib.pyplot as plt
from pymap3d.enu import geodetic2enu
import numpy as np

ref_lat = 42.293195
ref_lon = -83.7096706
ref_alt = 234.1

left_fix = pd.read_csv("./left_position.csv")
right_fix = pd.read_csv("./right_position.csv")
left_fix_status = pd.read_csv("./left_fix_status.csv")
right_fix_status = pd.read_csv("./right_fix_status.csv")

# left_latitude = left_fix["Latitude"]
# left_longitude = left_fix["Longitude"]
right_latitude = right_fix["Latitude"]
right_longitude = right_fix["Longitude"]
# left_rtk_fix = left_fix_status["RTK_fix"]
# right_rtk_fix = right_fix_status["RTK_fix"]

left_x = []
left_y = []
right_x = []
right_y = []
left_rtk_fix = []
right_rtk_fix = []
m = np.array(['.', '+', '^'])
colors = np.array(["pink", "red", "orange"])

# plt.ion()
# plt.plot(left_x, left_y, )

for i in range(0, min(left_fix["Latitude"].size, left_fix["Longitude"].size, left_fix["Altitude"].size)):
    pose = geodetic2enu(left_fix["Latitude"][i], left_fix["Longitude"][i], left_fix["Altitude"][i], ref_lat, ref_lon, ref_alt, deg=True)
    left_x.append(pose[0])
    left_y.append(pose[1])

for i in range(0, min(right_fix["Latitude"].size, right_fix["Longitude"].size, right_fix["Altitude"].size)):
    pose = geodetic2enu(right_fix["Latitude"][i], right_fix["Longitude"][i], right_fix["Altitude"][i], ref_lat, ref_lon, ref_alt, deg=True)
    right_x.append(pose[0])
    right_y.append(pose[1])



# for i in range(0, left_fix_status["RTK_fix"].size):
#     if (left_fix_status["RTK_fix"][i] == 0):
#         left_rtk_fix.append('.')

#     elif (left_fix_status["RTK_fix"][i] == 1):
#         left_rtk_fix.append('+')
#     else:
#         left_rtk_fix.append('^')

# for i in range(0, right_fix_status["RTK_fix"].size):
#     if (right_fix_status["RTK_fix"][i] == 0):
#         right_rtk_fix.append('.')

#     elif (right_fix_status["RTK_fix"][i] == 1):
#         right_rtk_fix.append('+')
#     else:
#         right_rtk_fix.append('^')


# fig, ax = plt.subplots()

# for lx, ly, m in zip(left_x, left_y, left_rtk_fix):
#     ax.scatter([x],[y], marker=m)

# plt.show()

# for i in range(0, len(m)):
#     mask = [left_rtk_fix == m[i]]
#     left_x = np.array(left_x)
#     left_y = np.array(left_y)
#     plt.scatter(left_x[mask], left_y[mask], marker=m[i], color=colors[i])

# for marker in m:
#     mask = [right_rtk_fix == marker]
#     right_x = np.array(right_x)
#     right_y = np.array(right_y)
#     plt.scatter(right_x[mask], right_y[mask], marker=marker, color="blue")
plt.plot(left_x, left_y, "ro", right_x, right_y, "b+")
plt.show()