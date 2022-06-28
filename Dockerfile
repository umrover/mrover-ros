FROM ros:noetic

RUN apt update && apt install -y \
    python3-catkin-tools \
#    ros-noetic-desktop \
    ros-noetic-xacro \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-mapviz \
    ros-noetic-gazebo-ros \
    ros-noetic-hector-gazebo \
    ros-noetic-teleop-twist-keyboard \
    ros-noetic-compressed-depth-image-transport \
    ros-noetic-compressed-image-transport \
    ros-noetic-theora-image-transport \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2 \
    ros-noetic-visualization-msgs \ 
    ros-noetic-vision-msgs \
    ros-noetic-image-transport \
    ros-noetic-sensor-msgs \
    ros-noetic-cv-bridge \
    ros-noetic-fiducial-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-aruco-detect \
    ros-noetic-robot-localization \
    ros-noetic-smach-ros

