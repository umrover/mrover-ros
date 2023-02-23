# Servo data and GPS data
# get tf tree latest map to base link transform, 
# average imu reading or middle imu reading since if you take the difference when the imu has just changed to a new direction it'll end your life
# only correct every X minutes, only correct off of N data points and for a certain amount of time
# 

import numpy as np
import rospy
import tf2_ros
from geometry_msgs import Twist
from util.SE3 import SE3

class GPS_Correction:
    def __init__(self):
        rospy.Subscriber("cmd_vel", Twist, self.velocity_callback)
        self.tf_buffer = tf2_ros.Buffer()

        # Tunable Parameters
        self.time_threshold = 3 # seconds
        self.num_points_threshold = 100
        self.cooling_off_period = 300 # seconds
        self.callback_rate = 10 # Hz
        self.linear_vel_threshold = 0.1 #TODO: definitely need to be tuned
        self.angular_vel_threshold = 0.1 #TODO: definitely need to be tuned

        self.gps_points = np.empty()
        self.current_vel = np.ndarray((2,3)) # unit linear and angular velocity vectors
        self.driving_straight = False

        self.last_update_time = 0
        self.last_heading_time = 0

        self.world_frame = rospy.get_param("gps_linearization/world_frame")
        self.rover_frame = rospy.get_param("gps_linearization/rover_frame")
    
    def get_heading_change(self, msg: Twist):
        """
        Updates the heading if either the linear velocity change or angular velocity magnitude is too large
        Returns true if the heading has changed, returns false if it has not
        """
        new_vel = np.array([msg.linear, msg.angular])
        new_vel[0] = new_vel[0] / np.linalg.norm(new_vel[0]) # only normalize the linear component
        if(self.current_vel.empty() or (np.dot(self.current_vel[0], new_vel[0]) > self.linear_vel_threshold) or 
            (np.linalg.norm(new_vel[0]) > self.angular_vel_threshold)):
            self.current_vel = new_vel
            self.driving_straight = False
        else:
            self.driving_straight = True
        return not(self.driving_straight)

    def velocity_callback(self, msg: Twist):
        if(not self.get_heading_change(msg)):
            self.get_new_readings() # Keep collecting points since the heading hasn't changed
            # TODO: does this work or will this block?
        else:
            # TODO: think about when we want to update the heading
            if((rospy.Time.now() - last_update_time > self.cooling_off_period) and 
              (self.ast_heading_time > self.time_threshold) and (self.gps_points.size > self.num_points_threshold)):
                  self.heading_correction = self.get_heading_correction()
                  last_update_time = rospy.Time.now()
            np.delete(self.gps_points, [:,:])
    
    # TODO: Think about how to structure this and which functions are checking what, have a flag to
    # TODO: problem here is unless I call get_heading_correction in the middle of this somewhere it will only correct heading at the end of driving straight 
    def get_new_readings(self):
        """
        Tries to read gps data at a rate specified by self.callback_rate
        Assumes that the rover is in a valid driving configuration so these points are be valid for heading correction
        """
        lookup_rate = rospy.Rate(self.callback_rate) # X Hz callback rate
        while(self.driving_straight):
            try:
                # Get linearized transform and append the position in the world frame to our gps_points array
                transform = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame)
                self.gps_points.append(transform.position, axis=1) #TODO: axis could be wrong
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            lookup_rate.sleep()

    def get_heading_correction(self):
        """
        Generates the correction matrix to multiply the IMU heading matrix by
        Assumes all conditions are acceptable to calculate the correction (time driving straight, number of datapoints, etc.)
        """
        heading = np.mean(self.gps_points, axis=2)
        heading_rotation = np.array([[heading[0], heading[1],  0],
                                    [heading[1], -heading[0], 0], 
                                    [0,          0,           1]])
        IMU_rotation = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame) #TODO: Gives us the last one, we want to look one up from the middle
        correction = np.matmul(np.linalg.inv(IMU_rotation), heading_rotation)
        return correction