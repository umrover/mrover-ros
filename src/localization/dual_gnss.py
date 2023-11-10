import numpy as np
import matplotlib.pyplot as plt


# def get_heading(antenna_point_A, antenna_point_B):
#     latitude_A = np.radians(antenna_point_A[0])
#     latitude_B = np.radians(antenna_point_B[0])
#     longitude_A = np.radians(antenna_point_A[1])
#     longitude_B = np.radians(antenna_point_B[1])

#     x = np.cos(latitude_B) * np.sin(longitude_B - longitude_A)
#     y = np.cos(latitude_A) * np.sin(latitude_B) - np.sin(latitude_A) * np.cos(latitude_B) * np.cos(
#         longitude_B - longitude_A
#     )

#     bearing = np.arctan2(x, y)
#     return bearing


# test_bearing = get_heading(np.array([39.099912, -94.581213]), np.array([38.627089, -90.200203]))
# print(np.degrees(test_bearing))


# reference point should be close, needs to be specified based on location
def spherical_to_cartesian(spherical_coord: np.ndarray, reference_coord: np.ndarray) -> np.ndarray:
    r = 6371000
    x = r * (np.radians(spherical_coord[1]) - np.radians(reference_coord[1])) * np.cos(np.radians(reference_coord[0]))
    y = r * (np.radians(spherical_coord[0]) - np.radians(reference_coord[0]))
    z = 0
    return np.array([x, y, z])


def get_heading_vector_from_angle(heading_angle):
    x = np.sin(heading_angle)
    y = np.cos(heading_angle)
    return np.array([x, y])


def get_heading_vector(point_L: np.array, point_R: np.array):
    vector_connecting = np.array([point_R[0] - point_L[0], point_R[1] - point_L[1]])
    vector_perp = np.zeros_like(vector_connecting)
    vector_perp[0] = -vector_connecting[1]
    vector_perp[1] = vector_connecting[0]
    # print(vector_perp)
    return vector_perp / np.linalg.norm(vector_perp)


def plot_vectors(P_1, P_2):
    result = P_1 - P_2
    slope_resultant = (P_2[1] - P_1[1]) / (P_2[0] - P_1[0])
    midpoint = np.array([((P_2[0] + P_1[0]) / 2), ((P_2[1] + P_1[1]) / 2)])
    slope_pd = -1 / slope_resultant

    plt.scatter([P_1[0], P_2[0]], [P_1[1], P_2[1]])
    x_intercept = (-midpoint[1] / slope_pd) + midpoint[0]
    x_pd = np.linspace(midpoint[0], x_intercept, 300)
    y_pd = slope_pd * (x_pd - midpoint[0]) + midpoint[1]

    plt.plot([P_1[0], P_2[0]], [P_1[1], P_2[1]])
    plt.plot(x_pd, y_pd)

    plt.axvline(x=midpoint[0], color="b")

    angle = np.degrees(np.arctan((midpoint[0] - x_intercept) / midpoint[1]))
    plt.xlabel("X-axis")
    plt.ylabel("Y-axis")
    plt.grid(True)
    plt.show()


def offset_lat_left(P_1, P_2, n):
    return P_1 - n * 0.0000001, P_2


def offset_lat_right(P_1, P_2, n):
    return P_1 + n * 0.0000001, P_2


def offset_long_left(P_1, P_2, n):
    return P_1, P_2 - n * 0.0000001


def offset_long_right(P_1, P_2, n):
    return P_1, P_2 + n * 0.0000001


# [1,0], [-1,0] if switched latitude 5th digit
# P_1 = np.array([42.30061, -83.71006])
# P_2 = np.array([42.30060, -83.71006])

# [0,-1], [0,1] if switched longitude 5th digit
P_1 = np.array([42.300611, -83.710071])
P_2 = np.array([42.300611, -83.710083])
ref = np.array([42.293195, -83.7096706])

for i in range(0, 170):  # graph changes based on these bounds due to reference coordinate
    new_p1, new_p2 = offset_lat_left(P_1, P_2, i)  # optimal bound is 0.0000001 (1e^-7)
    new_p1 = spherical_to_cartesian(new_p1, ref)
    new_p2 = spherical_to_cartesian(new_p2, ref)
    # print("P1")
    # print(new_p1)
    # print("P2")
    # print(new_p2)
    print("heading")
    print(get_heading_vector(new_p1, new_p2))
    test = get_heading_vector(new_p1, new_p2)
    plt.scatter(test[0], test[1])
plt.xlabel("heading x")
plt.ylabel("heading y")
plt.grid(True)
plt.show()

# print(get_heading_vector(np.array([0, 0]), np.array([3, 3])))
