FROM ros:noetic

RUN apt-get update && apt-get install -y \
    clang-format-12 \
    python3-catkin-tools \
    python3-pip

ADD . /catkin_ws/src/mrover

RUN rosdep update && rosdep install --from-paths /catkin_ws/src/ --ignore-src -y --rosdistro=noetic

RUN pip3 install -r /catkin_ws/src/mrover/requirements.txt
