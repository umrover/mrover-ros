#!/usr/bin/env python3

import numpy as np
from util.SE3 import SE3
import rospy
import tf2_ros
import matplotlib.pyplot as plt

GPSFILENAME = "recorded_gps2.npy"
VELFILENAME = "recorded_vel2.npy"


class getdata:
    def __init__(self):
        # save data when node dies
        rospy.on_shutdown(self.savedata)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        # create velocity and gps np arrays
        self.gps_data = []
        self.velocity_data = []

    def savedata(self):
        final_gps = np.hstack(self.gps_data)
        final_vel  = np.hstack(self.velocity_data)
        np.save(GPSFILENAME, final_gps)
        np.save(VELFILENAME, final_vel)

    def readtf(self):
        start = rospy.get_time()  # get start time to find time elapsed
        rospy.sleep(0.1)  # delay until data comes in

        rate = rospy.Rate(10)  # rate of data collection, 10 times per second

        # get data until node dies
        while not rospy.is_shutdown():
            currtime = rospy.get_time()
            self.velocity_data.append(np.array([[0], [currtime - start]]))
            currposition = SE3.from_tf_tree(self.buffer, parent_frame="map", child_frame="base_link")
            self.gps_data.append(currposition.position.reshape(3, 1))
            rate.sleep()


def data():
    g = getdata()
    g.readtf()


def plotData():
    # creates three plots of recorded (x,y,z) position vs time
    gps_data = np.load(GPSFILENAME, allow_pickle=True)
    velocity = np.load(VELFILENAME, allow_pickle=True)

    gps_data = gps_data[:,20:]
    velocity = velocity[:,20:]

    fig, ax = plt.subplots()
    ax.plot(gps_data[0, :], gps_data[1, :])

    ax.set(xlabel="x (m)", ylabel="y (m)", title="xpos vs ypos")

    fig.savefig("recordedxvsy.png")

    fig1, ax1 = plt.subplots()
    ax1.plot(velocity[1, :], gps_data[1, :])

    ax1.set(xlabel="time (s)", ylabel="position (m)", title="ypos vs time")

    fig1.savefig("recordedy.png")

    fig2, ax2 = plt.subplots()
    ax2.plot(velocity[1, :], gps_data[2, :])

    ax2.set(xlabel="time (s)", ylabel="position (m)", title="zpos vs time")

    fig2.savefig("recordedz.png")


# Since we assume zero-mean gaussian and gaussian distributions are defined by their mean and variance
# the only other information we need is the variance


def get_ground_truth(velocity, timestamps) -> np.ndarray:

    """
    initial_pos: an input position (X,Y,Z)
    velocity: 2xN velocity array (Vel, time)
    Returns ground truth for a moving rover over that time range

    assuming that velocity is constant, so just do v * t = pos
    """

    ground_truth = np.empty((3, timestamps.shape[1]), dtype=float)  # pre-allocate for runtime

    for i in range(timestamps.shape[1]):
        ground_truth[:,i] = velocity * timestamps[1,i]

    return ground_truth


def get_variance(gps_data, ground_truth, velocity) -> np.ndarray:
    """
    gps_data: the recorded gps position (X,Y,Z) at each timestep
    ground_truth: the actual gps position (X,Y,Z) at each timestep
    Returns the variance in each direction
    """
    # Var = Sum([X - u_x]^2 / (n - 1)
    # n-1 since we use a sample population

    plt.plot(velocity[1,:], gps_data[0,:], label = "gps data")
    plt.plot(velocity[1,:], ground_truth[0,:], label = "ground truth")
    plt.legend()
    plt.show()

    return np.sum(np.power(gps_data - ground_truth, [[2], [2], [2]]), axis=1) / (ground_truth.shape[1] - 1)


def get_std_dev(variance) -> np.ndarray:
    # Variance = (std_dev)^2
    return np.sqrt(variance)


def calcdata():

    time_taken = 8 #time by hand?, can change

    gps_data = np.load(GPSFILENAME)  # 2D array where each column is (x,y,z)
    velocity = np.load(VELFILENAME)  # 2D array where 1st row is a velocity value, 2nd row is a timestamp

    #get rid of random noise at start
    gps_data = gps_data[:,20:]
    velocity = velocity[:,20:]

    #normalize data based on start position
    for i in range(gps_data[1].size-1):
        gps_data[:,i+1] = gps_data[:,i+1] - gps_data[:,0]

    gps_data[:,0] = [0,0,0]

    #calculate time taken by rover
    time_taken = velocity[1,-1] - velocity[1,0]

    #calculate distance driven by using first and last gps data points
    end_pos = gps_data[:,-1]
    start_pos = gps_data[:,0]
    dist_driven = end_pos - start_pos

    #assuming constant velocity, calculates velocity
    calcvelocity = dist_driven / time_taken

    # will need to trim gps_data in order to obtain motion data
    motion_data = gps_data

    # Calculate the truth data for the moving rover based on its initial position and velocity at each timestep
    motion_truth = get_ground_truth(calcvelocity, velocity)

    # calculate stats
    motion_var = get_variance(motion_data, motion_truth, velocity)
    motion_std_dev = get_std_dev(motion_var)

    print(
        "Moving variance: X(%f), Y(%f), Z(%f); Moving Standard Deviation: X(%f), Y(%f), Z(%f)\n"
        % (motion_var[0], motion_var[1], motion_var[2], motion_std_dev[0], motion_std_dev[1], motion_std_dev[2])
    )


if __name__ == "__main__":
    rospy.init_node("gps_noise")
    #data()
    calcdata()
    #plotData()
    rospy.spin()
