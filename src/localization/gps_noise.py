#!usr/bin/env python3
import numpy as np

# Since we assume zero-mean gaussian and gaussian distributions are defined by their mean and variance
# the only other information we need is the variance

def get_position(gps_data):
    """
    gps_data: the recorded gps position (X,Y,Z) at each timestep
    Assumes that the GPS data is zero-mean gaussian and therefore the ground truth is the mean of gps_data
    Returns the GPS position as a 3x1 vector
    """
    return np.reshape(np.mean(gps_data, axis=1), [3,1])

def get_ground_truth(initial_pos, velocity)
    """
    initial_pos: an input position (X,Y,Z)
    velocity: 2x1 velocity array (Vel, time)
    returns ground truth for a moving rover over that time range
    """
    ground_truth = initial_pos
    for v in range(velocity.shape[1]):
        


def get_variance(gps_data, ground_truth):
    """
    gps_data: the recorded gps position (X,Y,Z) at each timestep
    ground_truth: the actual gps position (X,Y,Z) at each timestep
    Returns the variance in each direction
    """
    # Var = Sum([X - u_x]^2 / (n - 1)
    # n-1 since we use a sample population
    print(gps_data - ground_truth)
    print(np.power(gps_data - ground_truth, [[2],[2],[2]]))
    return np.sum(np.power(gps_data - ground_truth, [[2],[2],[2]]), axis = 1) / (ground_truth.shape[1] - 1)

def get_std_dev(variance):
    # Variance = (std_dev)^2
    return np.sqrt(variance)

def find_motion_start(velocity, threshold):
    """
    velocity: encoder values for the velocity (in some)
    """
    for v in range(velocity.shape[1]):
        if(velocity[0,v] > threshold):
            return v
    return None


def main():
    #TODO: Actually get the gps_data and encoder values
    gps_data = np.random.rand(3,6)*10 # 2D array of shape (3,N) where each row is a different X,Y,Z coordinate
    velocity = np.array([[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0], [0.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0]]) # 2D array where 1st row is a velocity value, 2nd row is a timestamp

    # Find where we start moving so we can separate the stationary data from the moving data
    motion_start_index = find_motion_start(velocity, 0.1) # Arbitrary threshold of 0.1, TODO: change when I know the format of encoder + gps data
    if(not motion_start_index): motion_start_index = velocity.shape[1]
    
    # Split the GPS data into stationary/moving groups
    stationary_data = gps_data[:, 0:motion_start_index-1]
    motion_data = gps_data[:, motion_start_index:]

    initial_pos = get_position(stationary_data)

    # Calculate the truth data for the moving rover based on its initial position and velocity at each timestep
    motion_truth = get_ground_truth(initial_pos, velocity[:, motion_start_index:])

    # Calculate the statistical properties of the noise and report them
    stationary_var = get_variance(stationary_data, np.repeat(initial_pos, stationary_data.shape[1], axis = 1))
    motion_var = get_variance(motion_data, motion_truth)
    stationary_std_dev = get_std_dev(stationary_var)
    motion_std_dev = get_std_dev(motion_var)

    # print("Moving variance: X(%d), Y(%d), Z(%d); Moving Standard Deviation: X(%d), Y(%d), Z(%d)\n" % 
    #     (motion_var[0], motion_var[1], motion_var[2], motion_std_dev[0], motion_std_dev[1], motion_std_dev[2]))
    print("Stationary variance: X(%f), Y(%f), Z(%f); Stationary Standard Deviation: X(%f), Y(%f), Z(%f)\n" % 
        (stationary_var[0], stationary_var[1], stationary_var[2], stationary_std_dev[0], stationary_std_dev[1], stationary_std_dev[2]))

if __name__ == "__main__":
    main()