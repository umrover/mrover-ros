FROM ros:noetic

RUN apt-get update && apt-get install -y \
    zsh neovim sudo \
    clang-format-12 \
    python3-catkin-tools python3-pip

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER mrover
WORKDIR /home/mrover

ENV PATH="${PATH}:/home/mrover/.local/bin"

RUN mkdir -p ./catkin_ws/src
ADD . ./catkin_ws/src/mrover

RUN pip3 install -r ./catkin_ws/src/mrover/requirements.txt

RUN rosdep update && rosdep install --from-paths ./catkin_ws/src --ignore-src -y --rosdistro=noetic

ENTRYPOINT [ "/bin/zsh" ]
