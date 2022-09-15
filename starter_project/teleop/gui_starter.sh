#!/bin/bash

# Ensure proper NODE_PATH
export NODE_PATH=/usr/lib/nodejs:/usr/share/nodejs


node_executable=node
node_executable_path=$(which "$node_executable")
yarn_executable=yarn
yarn_executable_path=$(which "$yarn_executable")

if [ -z "$node_executable_path" ]
then
    echo "Node installation not found, please run src/teleop/gui/gui_install.sh"
    exit 1
fi
if [ -z "$yarn_executable_path" ]
then
    echo "Yarn installation not found, please run src/teleop/gui/gui_install.sh"
    exit 1
fi

# Check if node_modules up to date
yarn check --verify-tree

# If not up to date, install
if [ $? == 1 ]
then
    yarn install --check-files
fi

yarn run serve
