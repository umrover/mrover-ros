import numpy as np
# 2 antennas on the rover a fixed distance apart and pointing in the same direction
# antennas receive GNSS data (latitude, longitude)

# "mobile" RTK system in that the base station is also moving

# heading measurement is derived from differencing the 2 GNSS measurements
# at a single point in time

# vector connecting the two points (in latitude, longitude) can give accurate heading

# 2 antennas are close to each other, so RTK error subtracting can be applied

# bearing can be defined as the angle between the north-south line of the earth
# and the vector connecting the 2 GNSS antenna points


# bearing formula
# beta = atan2(X, Y)
# X = cos (latitude B) * sin(change in longitude)
# Y = cos (latitude A) * sin(latitude B) - sin(latitude A) * cos(latitude B) * cos(change in longitude)



def get_heading(antenna_point_A, antenna_point_B):
    latitude_A = np.radians(antenna_point_A[0])
    latitude_B = np.radians(antenna_point_B[0])
    longitude_A = np.radians(antenna_point_A[1])
    longitude_B = np.radians(antenna_point_B[1])
    
    x = np.cos(latitude_B) * np.sin(longitude_B - longitude_A)
    y = np.cos(latitude_A) * np.sin(latitude_B) - np.sin(latitude_A) * np.cos(latitude_B) * np.cos(longitude_B - longitude_A)

    bearing = np.arctan2(x, y)
    return bearing



test_bearing = get_heading(np.array([39.099912, -94.581213]), np.array([38.627089, -90.200203]))
print(test_bearing)
