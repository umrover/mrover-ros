#!/bin/bash

export ROS_MASTER_URI=http://10.0.0.2:11311

source /opt/ros/noetic/setup.bash
source /home/mrover/catkin_ws/devel/setup.bash

exec "$@"
