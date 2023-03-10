#!/usr/bin/env python3

import numpy as np
from util.SE3 import SE3
import rospy
import tf2_ros
import matplotlib.pyplot as plt

GPSFILENAME = "recorded_gps.npy"
VELFILENAME = "recorded_vel.npy"

class getdata:
    
    def __init__(self):
        #save data when node dies
        rospy.on_shutdown(self.savedata)

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)

        #create velocity and gps np arrays
        self.live_gps_data = np.zeros([3,1])
        self.velocitytimes = np.zeros([2,1])


    def savedata(self):
        np.save(GPSFILENAME,self.live_gps_data)
        np.save(VELFILENAME,self.velocitytimes)

    def readtf(self):
        start = rospy.get_time() #get start time to find time elapsed
        rospy.sleep(0.1)         #delay until data comes in

        rate = rospy.Rate(10)    #rate of data collection, 10 times per second

        #get data until node dies
        while not rospy.is_shutdown():
            currtime = rospy.get_time()
            self.velocitytimes = np.hstack((self.velocitytimes, np.array([[0],[currtime-start]])))
            currposition = SE3.from_tf_tree(self.buffer, parent_frame="map", child_frame="base_link")
            self.live_gps_data = np.hstack((self.live_gps_data, np.reshape(currposition.position, (3,1))))
            rate.sleep()


def data():
    g = getdata()
    g.readtf()

def plotData():
    #creates three plots of recorded (x,y,z) position vs time
    gps_data = np.load(GPSFILENAME,allow_pickle=True)
    velocity = np.load(VELFILENAME, allow_pickle=True)

    fig, ax = plt.subplots()
    ax.plot(velocity[1,:], gps_data[0,:])

    ax.set(xlabel='time (s)', ylabel='position (m)',
       title='xpos vs time')

    fig.savefig("recordedx.png")

    fig1, ax1 = plt.subplots()
    ax1.plot(velocity[1,:], gps_data[1,:])

    ax1.set(xlabel='time (s)', ylabel='position (m)',
       title='ypos vs time')

    fig1.savefig("recordedy.png")

    fig2, ax2 = plt.subplots()
    ax2.plot(velocity[1,:], gps_data[2,:])

    ax2.set(xlabel='time (s)', ylabel='position (m)',
       title='zpos vs time')

    fig2.savefig("recordedz.png")

# Since we assume zero-mean gaussian and gaussian distributions are defined by their mean and variance
# the only other information we need is the variance

def get_position(gps_data):
    """
    gps_data: the recorded gps position (X,Y,Z) at each timestep
    Assumes that the GPS data is zero-mean gaussian and therefore the ground truth is the mean of gps_data
    Returns the GPS position as a 3x1 vector
    """
    return np.reshape(np.mean(gps_data, axis=1), [3,1])

def get_ground_truth(velocity):
    
    """
    initial_pos: an input position (X,Y,Z)
    velocity: 2x1 velocity array (Vel, time)
    Returns ground truth for a moving rover over that time range

    assuming that velocity is constant, so just do v * t = pos
    """
    ground_truth = np.empty((3,velocity.shape[1]), dtype=float) # pre-allocate for runtime

    for i in range(velocity.shape[1]):
        temp = np.array([velocity[0][i] * velocity[1][i],0,0])
        ground_truth[:,i] = temp


    return ground_truth

def get_variance(gps_data, ground_truth,velocity):
    """
    gps_data: the recorded gps position (X,Y,Z) at each timestep
    ground_truth: the actual gps position (X,Y,Z) at each timestep
    Returns the variance in each direction
    """
    # Var = Sum([X - u_x]^2 / (n - 1)
    # n-1 since we use a sample population

    """
    Uncomment to graph gps data and ground truth vs time
    plt.plot(velocity[1,:], gps_data[0,:], label = "gps data")
    plt.plot(velocity[1,:], ground_truth[0,:], label = "ground truth")
    plt.legend()
    plt.show()
    """
    return np.sum(np.power(gps_data - ground_truth, [[2],[2],[2]]), axis = 1) / (ground_truth.shape[1] - 1)

def get_std_dev(variance):
    # Variance = (std_dev)^2
    return np.sqrt(variance)        

def calcdata():
    
    calcvelocity = 1 #calculate velcoity by hand 
    
    gps_data = np.load(GPSFILENAME)# 2D array where each column is (x,y,z)
    velocity = np.load(VELFILENAME) # 2D array where 1st row is a velocity value, 2nd row is a timestamp
    

    velocity[0].fill(calcvelocity) #fill with predetermined velocity
    
    #will need to trim gps_data in order to obtain motion data
    motion_data = gps_data

    # Calculate the truth data for the moving rover based on its initial position and velocity at each timestep
    motion_truth = get_ground_truth(velocity)
    
    #calculate stats
    motion_var = get_variance(motion_data, motion_truth,velocity)
    motion_std_dev = get_std_dev(motion_var)

    print("Moving variance: X(%f), Y(%f), Z(%f); Moving Standard Deviation: X(%f), Y(%f), Z(%f)\n" % 
        (motion_var[0], motion_var[1], motion_var[2], motion_std_dev[0], motion_std_dev[1], motion_std_dev[2]))


if __name__ == "__main__":
    rospy.init_node("gps_noise")
    #data()
    calcdata()
    #plotData()
    rospy.spin()

