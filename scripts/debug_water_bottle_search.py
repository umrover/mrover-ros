#!/usr/bin/env python3

import numpy as np

import rospy
import time
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from util.course_publish_helpers import publish_waypoints


"""
The purpose of this file is for testing the water bottle search state. 
Specifically the occupancy grid message. 
"""

if __name__ == "__main__":
    rospy.init_node("debug_water_bottle")
    try:
        # int8[] data
        test_grid = OccupancyGrid()
        test_grid.data = np.array([1, 2, 3, 4, 5, 6, 7, 8, 9], dtype=np.int8)

        # nav_msgs/MapMetaData info
        metadata = MapMetaData()
        metadata.map_load_time = rospy.Time.now()
        metadata.resolution = 3
        metadata.width = 3
        metadata.height = 3
        metadata.origin = Pose()
        test_grid.info = metadata

        # std_msgs/Header header
        header = Header()
        test_grid.header = header

        rospy.loginfo(f"Before publish")
        costpub = rospy.Publisher("costmap", OccupancyGrid, queue_size=1)
        for i in range(10):
            costpub.publish(test_grid)
            time.sleep(1)
        rospy.loginfo(f"After publish")
        rospy.spin()

    except rospy.ROSInterruptException as e:
        print(f"Didn't work to publish or retrieve message from ros")
