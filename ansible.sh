#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <playbook>"
    exit 1
fi

sudo -v # Ensure Ansible has sudo permission

readonly MROVER_PATH=$(dirname "$0")
readonly CATKIN_WORKSPACE_PATH=$(realpath "${MROVER_PATH}"/../..)
ansible-playbook -i "localhost," -c local "${MROVER_PATH}"/ansible/"$1" --extra-vars "catkin_workspace=${CATKIN_WORKSPACE_PATH}"
