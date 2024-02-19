FROM ubuntu:focal

# DEBIAN_FRONTEND=noninteractive prevents apt from asking for user input
# software-properties-common is needed for apt-add-repository
# sudo is needed for ansible since it escalates from a normal user to root
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update -y && apt-get install software-properties-common sudo -y
RUN apt-add-repository ppa:ansible/ansible -y
RUN apt-get install -y ansible git git-lfs
ADD ./pkg /tmp/
RUN apt-get install -f /tmp/*.deb && rm /tmp/*.deb

RUN useradd --create-home --groups sudo --shell /bin/zsh mrover
# Give mrover user sudo access with no password
RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER mrover
RUN mkdir -p /home/mrover/catkin_ws/src/mrover
WORKDIR /home/mrover/catkin_ws/src/mrover
# Defines the APT packages that need to be installed
# rosdep is called from Ansible to install them
ADD --chown=mrover:mrover ./package.xml .
# Defines the Python packages that need to be installed
# pip is called from Ansible to install them
ADD --chown=mrover:mrover ./pyproject.toml .
# Define the NPM packages that need to be installed by Bun
ADD --chown=mrover:mrover ./src/teleoperation/frontend/package.json ./src/teleoperation/frontend/bun.lockb ./src/teleoperation/frontend/
# Copy over all Ansible files
ADD --chown=mrover:mrover ./ansible ./ansible
ADD --chown=mrover:mrover ./ansible.sh .
RUN ./ansible.sh ci.yml

USER root
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER mrover
ENTRYPOINT [ "/bin/zsh" ]
