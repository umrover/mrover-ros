FROM ubuntu:20.04

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt install curl && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt update && \
    apt install -y ros-noetic-desktop python3-catkin-tools && \
    apt install -y ros-noetic-xacro ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher && \
    apt install -y ros-noetic-mapviz ros-noetic-gazebo-ros ros-noetic-hector-gazebo ros-noetic-teleop-twist-keyboard && \
    apt install -y ros-noetic-compressed-depth-image-transport ros-noetic-compressed-image-transport ros-noetic-theora-image-transport ros-noetic-tf2-geometry-msgs ros-noetic-tf2-ros ros-noetic-tf2 ros-noetic-visualization-msgs ros-noetic-vision-msgs ros-noetic-image-transport ros-noetic-sensor-msgs ros-noetic-cv-bridge ros-noetic-fiducial-msgs ros-noetic-dynamic-reconfigure ros-noetic-aruco-detect && \
    apt install -y ros-noetic-robot-localization ros-noetic-smach-ros

