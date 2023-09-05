#!/usr/bin/env bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

echo "Ensuring SSH keys are set up ..."
if [ ! -f ~/.ssh/id_rsa ]; then
  echo "Please see: https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent"
  exit 1
fi

PPAS=(
  ansible/ansible
  git-core/ppa
)
NEED_APT_UPDATE=false
for PPA in "${PPAS[@]}"; do
  if ! grep -q "^deb .*${PPA}" /etc/apt/sources.list /etc/apt/sources.list.d/*;
  then
    echo "Adding PPA: ${PPA}"
    sudo apt-add-repository ppa:"${PPA}" -y
    NEED_APT_UPDATE=true
  fi
done

if [ "${NEED_APT_UPDATE}" = true ]; then
    sudo apt update
fi
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
