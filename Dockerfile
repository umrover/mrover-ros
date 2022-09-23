FROM ros:noetic

RUN apt-get update && apt-get install -y \
    zsh \
    clang-format-12 \
    python3-catkin-tools \
    python3-pip

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
WORKDIR /home/mrover
USER mrover

ADD . ./catkin_ws/src/mrover

RUN rosdep update && rosdep install --from-paths ./catkin_ws/src/ --ignore-src -y --rosdistro=noetic

RUN pip3 install -r ./catkin_ws/src/mrover/requirements.txt
