FROM ros:noetic

RUN apt-get update -y && apt-get upgrade -y && apt-get install software-properties-common -y
RUN apt-add-repository ppa:ansible/ansible -y
RUN apt-add-repository ppa:git-core/ppa -y
RUN apt install -y ansible git git-lfs

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
# Give mrover user sudo access with no password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER mrover
RUN mkdir -p /home/mrover/catkin_ws/src/mrover
WORKDIR /home/mrover/catkin_ws/src/mrover
ADD ./package.xml .
ADD ./pyproject.toml .
ADD ./ansible ./ansible
ADD ./ansible.sh .
RUN ./ansible.sh build.yml

USER root
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER mrover
ENTRYPOINT [ "/bin/zsh" ]
