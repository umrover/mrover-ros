#!/usr/bin/env bash

# Run on a fresh Ubuntu 20.04 install.
# Installs Ansible and Git, then clones the mrover repo
# Ansible will be used to finish configuring the system

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

readonly BLUE='\033[0;34m'
readonly NC='\033[0m'

echo "Ensuring SSH keys are set up ..."
if [ ! -f ~/.ssh/id_ed25519 ] && [ ! -f ~/.ssh/id_rsa ]; then
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

echo "${BLUE}Enter path to ROS workspace... [leave blank for ${DEFAULT_CATKIN_PATH}]:${NC}"
read -r CATKIN_PATH
if [ -z "${CATKIN_PATH}" ]; then
  CATKIN_PATH=${DEFAULT_CATKIN_PATH}
fi
echo "Using ${CATKIN_PATH} as ROS workspace"

readonly MROVER_PATH=${CATKIN_PATH}/src/mrover
readonly FIRST_TIME_SETUP=[ ! -d ${MROVER_PATH}]

if [ "${FIRST_TIME_SETUP}" ]; then
  echo "Creating ROS workspace ..."
  mkdir -p ${CATKIN_PATH}/src
  git clone git@github.com:umrover/mrover-ros ${CATKIN_PATH}/src/mrover -b qd/ansible
fi

echo "Using Ansible to finish up ..."
${MROVER_PATH}/ansible.sh dev.yml

if [ ! -d ${MROVER_PATH}/venv ]; then
  echo "Setting up Python virtual environment ..."
  python3.10 -m venv ${MROVER_PATH}/venv
  source ${MROVER_PATH}/venv/bin/activate
  pip install -e "${MROVER_PATH}[dev]"
fi

if [ "${FIRST_TIME_SETUP}" ]; then
  echo "All done! Welcome to MRover!"
fi
