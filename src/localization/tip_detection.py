#!/usr/bin/env python3

import tf2_ros
import numpy as np
import rospy
from std_msgs.msg import Bool
from util.SE3 import SE3
import time


class TipDetection:
    hit_count: int
    orientation_threshold: float
    hit_count_threshold: int
    reset_hit_count_threshold: int
    time_counter: time
    current_time: time
    tip_publisher: rospy.Publisher

    def __init__(self):
        self.tip_publisher = rospy.Publisher("tipping", Bool, queue_size=1)

        self.hit_count = 0
        self.orientation_threshold = 0.8
        self.hit_count_threshold = 10
        self.reset_hit_count_threshold = 3
        self.time_counter = time.time()
        self.current_time = time.time()

        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        self.world_frame = rospy.get_param("world_frame")
        self.rover_frame = rospy.get_param("rover_frame")
        self.in_loop()

    def in_loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # extract yaw
                self.old = SE3.from_tf_tree(self.buffer, self.world_frame, self.rover_frame).rotation.rotation_matrix()
                
                # multiply yaw by the z vector [0, 0, 1] to get new transform
                self.transform = np.dot(np.array([0, 0, 1]), self.old)

                # compare this new transform with our threshold to see if it's tipping, if so increment hit_count
                if self.transform[2] <= self.orientation_threshold:
                    self.hit_count += 1
                
                self.check_for_hit_count(self.hit_count)

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                print(e)
                rate.sleep()
            # reset the hit count time 
            self.current_time = time.time()
            self.reset_hit_count_time(self.reset_hit_count_threshold, self.time_counter)

    # check if hit_count is too big
    def check_for_hit_count(self, hit_count):
        # if hit_count exceeds threshold
        if hit_count > self.hit_count_threshold:
            # publishing into tip_publisher that rover is tipping, True
            self.tip_publisher.publish(True)
        else: # else publish False
            self.tip_publisher.publish(False)

    # reset hit_count each reset_hit_count_threshold seconds
    def reset_hit_count_time(self, reset_hit_count_threshold, time_counter):
        # if the amount of time that's passed since last reset > threshold, reset hit_count
        if time.time() - self.time_counter > self.reset_hit_count_threshold:
            self.hit_count = 0
            self.time_counter = time.time()

def main():
    rospy.init_node("tip_detection")
    TipDetection()

    rospy.spin()


if __name__ == "__main__":
    main()
    