import numpy as np
import rospy
import tf2_ros
from geometry_msgs import Twist
from geometry_msgs import Quaternion
from util.SE3 import SE3
from util.SO3 import SO3

# Notes:
# 1. Calculate and apply the transform as soon as we have a suitable number of points (self.num_points_threshold)
# 2. Only calculate the transform if we are moving fast enough (self.linear_vel_threshold)
# 3. Don't calculate the tranform if we are turning (self.angular_vel_threshold)

class GPS_Correction:
    def __init__(self):
        self.kill_flag = False      # Kill flag to stop collecting data completely

        rospy.Subscriber("cmd_vel", Twist, self.velocity_callback)
        self.correction_publisher = rospy.Publisher("imu/gps_correction", Quaternion)
        self.tf_buffer = tf2_ros.Buffer()

        self.transform_to_update = SE3()
        self.gps_points = np.empty([0,3])
        self.current_vel = np.zeros((2,3))
        self.driving_straight = False

        self.world_frame = rospy.get_param("gps_linearization/world_frame")
        self.rover_frame = rospy.get_param("gps_linearization/rover_frame")

        # Tunable Parameters
        self.time_threshold = 3             # seconds (tune this with the num_points_threshold)
        self.callback_rate = 10             # Hz

        self.num_points_threshold = self.time_threshold * self.callback_rate      # TODO: definitely needs to be tuned
        self.linear_vel_threshold = 0.1     # TODO: definitely need to be tuned
        self.angular_vel_threshold = 0.1    # TODO: definitely need to be tuned
        self.linear_angular_ratio = 10      # TODO: definitely needs to be tuned

    def is_driving_straight(self, new_vel):
        """
        Return false if we are turning (condition 1), not driving fast enough (condition 2), 
        """
        if((np.linalg.norm(new_vel[1]) > self.angular_vel_threshold) or
            (np.linalg.norm(new_vel[0]) < self.linear_vel_threshold)):
            return False
        return True
    
    def get_heading_change(self, msg: Twist):
        """
        Updates the heading if either the linear velocity change or angular velocity magnitude is too large
        Returns true if the heading has changed, returns false if it has not
        """
        new_vel = np.array([msg.linear, msg.angular])
        self.driving_straight = self.is_driving_straight(new_vel)
        self.current_vel = new_vel

        # If the robot is driving straight and fast enough, but the heading has changed, return true
        if(np.dot(self.current_vel[0], self.new_vel[0]) < (1 - self.linear_vel_threshold)):
            return True

        return (not self.driving_straight)

    def velocity_callback(self, msg: Twist):
        if(self.get_heading_change(msg)):
            if(self.gps_points.size > self.num_points_threshold):   # If we have enough points update heading before deleting all points
                  self.heading_correction = self.get_heading_correction()
            np.delete(self.gps_points, [0,1,2], axis=1)     # Delete all gps data from previous heading
    
    def get_new_readings(self):
        """
        Tries to read gps data at a rate specified by self.callback_rate
        Assumes that the rover is in a valid driving configuration so these points are be valid for heading correction
        """
        lookup_rate = rospy.Rate(self.callback_rate)    # X Hz callback rate
        while(self.driving_straight and (not self.kill_flag)):
            try:
                # Get linearized transform and append the position in the world frame to our gps_points array
                transform = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame)
                self.gps_points.append(transform.position, axis=0)      # TODO: might have to reshape transform.position to (1,3)
                
                # If it is the middle datapoint, get the transform that we will find the correction from
                if(self.gps_points.size == (self.num_points_threshold // 2)): 
                    self.transform_to_update = SE3.from_tf_tree(self.tf_buffer, self.world_frame, self.rover_frame)
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            if(self.gps_points.size() > self.num_points_threshold):   # compute the heading correction if we cross an upper points threshold
                self.get_heading_correction()

            lookup_rate.sleep()

    def get_heading_correction(self):
        """
        Generates the correction matrix to multiply the IMU heading matrix by
        Assumes all conditions are acceptable to calculate the correction (time driving straight, number of datapoints, etc.)
        """
        heading = np.mean(self.gps_points, axis=0)      # Take the mean along the same axis we append the points along
        heading = heading / np.linalg.norm(heading)     # Normalize the heading before using it in the rotation matrix
        heading_rotation = np.array([[heading[0], -heading[1],  0],
                                     [heading[1], heading[0],   0], 
                                     [0,          0,            1]])
        
        IMU_rotation = self.transform_to_update.rotation.rotation_matrix
        correct_matrix = np.matmul(np.linalg.inv(IMU_rotation), heading_rotation)
        correction = SO3.from_matrix(correct_matrix)

        self.correction_publisher.publish(correction.quaternion)

    def set_kill_flag(self, flag: bool):
        self.kill_flag = flag

    def stop_correction(self):
        self.correction_publisher.publish(np.array([0,0,0,1]))      # Publish the identity quaternion

def main():
    rospy.init_node("gps_correction")
    gps_correction = GPS_Correction()
    while(not rospy.is_shutdown): # Continuously get readings until the node is shutdown
        gps_correction.get_new_readings()

if __name__ == "__main__":
    main()

# Notes to myself:
# Servo data and GPS data
# get tf tree latest map to base link transform, 
# average imu reading or middle imu reading since if you take the difference when the imu has just changed to a new direction it'll end your life
# only correct every X minutes, only correct off of N data points and for a certain amount of time

# xval = 0 -> turning in place
# x velocity is some proportion of what it should be