#!usr/bin/env python3
import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import JointState


# Since we assume zero-mean gaussian and gaussian distributions are defined by their mean and variance
# the only other information we need is the variance

class gps_noise:
    def __init__(self):
        rospy.Subscriber("gps/fix", JointState, self.velocity_callback) #TODO: get the actual topics for the joinstate msg and tf_tree msg
        tf2_ros::TransformListener tfListener(self.tf_callback);        
        gps_data = np.array([3,1])
        gps_times = np.array()
        vel_data = np.array([2,1])
        direction = np.array([[0],[1],[0]])

    tf_callback(msg: hmmmm):
        tf2_ros.lookupTransform([Whatever the GPS_linearization publishes to])
        np.append(gps_times,gps)
        np.append(gps_data, [[],[],[]])

    velocity_callback(msg: JointState):
        vel = np.mean(np.array(msg.velocity))
        time = header.stamp
        np.append(vel_data, [[vel],[time]])

    def get_position(self):
        """
        gps_data: the recorded gps position (X,Y,Z) at each timestep
        Assumes that the GPS data is zero-mean gaussian and therefore the ground truth is the mean of gps_data
        Returns the GPS position as a 3x1 vector
        """
        return np.reshape(np.mean(gps_data, axis=1), [3,1])

    # Assumes there is some stationary data
    def get_ground_truth(initial_pos, velocity, gps_times, direction):
        """
        initial_pos: an input position (X,Y,Z)
        velocity: 2x1 velocity array (Vel, time)
        Returns ground truth for a moving rover over that time range
        """
        # Do a trapezoidal approximation for position
        ground_truth = np.empty((3,velocity.shape[1]-1), dtype=float) # pre-allocate for SPEED
        prev_pos = initial_pos
        current_pos = np.reshape(initial_pos + direction * 0.5 * (velocity[0][0]+velocity[0][1]) * (velocity[1][1]-velocity[1][0]), (3,))
        vel_time_idx = 1
        idx = 0
        while(idx < len(gps_times)):
            while(gps_times[idx] < velocity[1,vel_time_idx]):
                ground_truth[:,idx] = prev_pos + np.reshape(direction * 0.5 * (prev_pos + current_pos) 
                                                                * (velocity[1][vel_time_idx - 1]-gps_times[idx]), (3,))
                idx += 1
            prev_pos = current_pos
            current_pos += np.reshape(direction * 0.5 * (velocity[0][vel_time_idx] + velocity[0][vel_time_idx+1]) 
                                        * (velocity[1][vel_time_idx] - velocity[1][vel_time_idx]), (3,))
            vel_time_idx += 1

        return ground_truth

    def get_variance(self, ground_truth):
        """
        gps_data: the recorded gps position (X,Y,Z) at each timestep
        ground_truth: the actual gps position (X,Y,Z) at each timestep
        Returns the variance in each direction
        """
        # Var = Sum([X - u_x]^2 / (n - 1)
        # n-1 since we use a sample population
        return np.sum(np.power(gps_data - ground_truth, [[2],[2],[2]]), axis = 1) / (ground_truth.shape[1] - 1)

    def get_std_dev(variance):
        # Variance = (std_dev)^2
        return np.sqrt(variance)

    # Finds the index in the velocity data where the rover starts moving
    def find_motion_start(velocity, threshold):
        """
        velocity: encoder values for the velocity (in some)
        """
        for v in range(velocity.shape[1]):
            if(velocity[0,v] > threshold):
                return v
        return None

def main():
    gps = gps_noise
    gps.__init__()

    #TODO: Actually get the gps_data and encoder values
    direction = np.array([[0],[1],[0]]) # hard code direction of the robot, TODO: maybe get this data from heading (might not be independent data though)
    gps_data = np.random.rand(3,10)*10 # 2D array of shape (3,N) where each row is a different X,Y,Z coordinate
    velocity = np.array([[0.0,0.0,0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0], [0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0]]) # 2D array where 1st row is a velocity value, 2nd row is a timestamp
    gps_times = np.array[0.01, 0.5, 0.7, 1.3, 2, 3.5, 6]

    # Find where we start moving so we can separate the stationary data from the moving data
    motion_start_index = find_motion_start(velocity, 0.1)
    if(not motion_start_index): motion_start_index = velocity.shape[1]
    
    # Split the GPS data into stationary/moving groups
    stationary_data = gps_data[:, 0:motion_start_index-1]
    motion_data = gps_data[:, motion_start_index:]

    initial_pos = get_position(stationary_data)

    # Calculate the truth data for the moving rover based on its initial position and velocity at each timestep
    motion_truth = get_ground_truth(initial_pos, velocity[:, motion_start_index-1:], direction)

    # Calculate the statistical properties of the noise and report them
    stationary_var = get_variance(stationary_data, initial_pos)
    motion_var = get_variance(motion_data, motion_truth)
    stationary_std_dev = get_std_dev(stationary_var)
    motion_std_dev = get_std_dev(motion_var)

    print("Moving variance: X(%d), Y(%d), Z(%d); Moving Standard Deviation: X(%d), Y(%d), Z(%d)\n" % 
        (motion_var[0], motion_var[1], motion_var[2], motion_std_dev[0], motion_std_dev[1], motion_std_dev[2]))
    print("Stationary variance: X(%f), Y(%f), Z(%f); Stationary Standard Deviation: X(%f), Y(%f), Z(%f)\n" % 
        (stationary_var[0], stationary_var[1], stationary_var[2], stationary_std_dev[0], stationary_std_dev[1], stationary_std_dev[2]))

if __name__ == "__main__":
    main()