#!/usr/bin/env bash

# Helper script to run Ansible playbooks

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <playbook>"
    exit 1
fi

MROVER_PATH=$(dirname "$0")
CATKIN_WORKSPACE_PATH=${MROVER_PATH}/../..
ansible-playbook -i "localhost," -c local "${MROVER_PATH}"/ansible/"$1" --ask-become-pass --extra-vars "catkin_workspace=${CATKIN_WORKSPACE_PATH}"
