#!/bin/bash
source /opt/ros/noetic/setup.bash
source /home/pi/catkin_ws/devel/setup.bash
export PYTHONPATH=/home/pi/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
export ROS_IP=10.0.0.3
roslaunch mrover brushless.launch