#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

echo "Ensuring SSH keys are set up ..."
if [ ! -f ~/.ssh/id_rsa ]; then
  echo "Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent"
  exit 1
fi

echo "Ensuring Git & Ansible are installed ..."
sudo apt-add-repository ppa:ansible/ansible -y
sudo apt-add-repository ppa:git-core/ppa -y
sudo apt update -y
sudo apt install -y ansible git git-lfs

readonly DEFAULT_CATKIN_PATH=~/catkin_ws

echo "Enter path to ROS workspace... [leave blank for ${DEFAULT_CATKIN_PATH}]:"
read -r CATKIN_PATH
if [ -z "${CATKIN_PATH}" ]; then
  CATKIN_PATH=${DEFAULT_CATKIN_PATH}
fi
echo "Using ${CATKIN_PATH} as ROS workspace"

readonly MROVER_PATH=${CATKIN_PATH}/src/mrover

if [ ! -d "${MROVER_PATH}" ]; then
  echo "Creating ROS workspace ..."
  mkdir -p ${CATKIN_PATH}/src
  git clone git@github.com:umrover/mrover-ros ${CATKIN_PATH}/src/mrover
fi

echo "Using Ansible to finish up ..."
ansible-playbook -i "localhost," -c local ${MROVER_PATH}/ansible/dev.yml --extra-vars "catkin_workspace=${CATKIN_PATH}"
