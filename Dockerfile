FROM ros:noetic

RUN apt-get update && apt-get upgrade -y

# Add apt repo for latest version of Git
RUN apt-get install software-properties-common -y && add-apt-repository ppa:git-core/ppa -y

RUN apt-get update && apt-get install -y \
    zsh neovim sudo git git-lfs \
    clang-format-12 clang-tidy-12 \
    python3-catkin-tools python3-pip

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
# Give mrover user sudo access with no password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER mrover
WORKDIR /home/mrover

RUN mkdir -p ./catkin_ws/src    
ADD . ./catkin_ws/src/mrover

# Install ROS packages
RUN rosdep update && rosdep install --from-paths ./catkin_ws/src --ignore-src -y --rosdistro=noetic

# Install Python packags, sudo so it is a global install
RUN sudo pip3 install -r ./catkin_ws/src/mrover/requirements.txt

ENTRYPOINT [ "/bin/zsh" ]
