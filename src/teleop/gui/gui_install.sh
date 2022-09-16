#!/bin/bash

node_executable=node
node_executable_path=$(which "$node_executable")
yarn_executable=yarn
yarn_executable_path=$(which "$yarn_executable")

# Install node and yarn if not found
if [ -z "$node_executable_path" ]
then
    curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
    sudo apt update -y
    sudo apt install nodejs -y
fi
if [ -z "$yarn_executable_path" ]
then
    curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
    echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list
    sudo apt update -y
    sudo apt install yarn -y
fi

# Check if node_modules up to date
yarn check --verify-tree

# If not up to date, install
if [ $? == 1 ]
then
    yarn install --check-files
fi
