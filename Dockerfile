FROM ubuntu:focal

# DEBIAN_FRONTEND=noninteractive and keyboard-configuration are needed to prevent stdin prompting later on
# This was super annoying to figure out because otherwise the build would hang
# software-properties-common is needed for apt-add-repository
RUN apt-get update -y && apt-get upgrade -y && DEBIAN_FRONTEND=noninteractive apt-get install software-properties-common keyboard-configuration -y
RUN apt-add-repository ppa:ansible/ansible -y
# RUN apt-add-repository ppa:git-core/ppa -y
RUN apt-get install -y ansible git git-lfs

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
RUN ./ansible.sh build.yml

USER root
# Dawn
ADD ./dawn.deb /tmp/
RUN apt-get install -f /tmp/dawn.deb -y && rm /tmp/dawn.deb
# Remove apt cache to free up space in the image
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

USER mrover
ENTRYPOINT [ "/bin/zsh" ]
