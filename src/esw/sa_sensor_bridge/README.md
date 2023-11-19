To install rosserial, do the following:
`
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial
`

Also make sure to do `sudo chmod 666 /dev/ttyACM0`

Run `rosrun rosserial_python serial_node.py /dev/ttyACM0`
