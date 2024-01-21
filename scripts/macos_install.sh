#!/bin/bash

if ! command -v mamba; then
    brew install mambaforge git-lfs ccache
    mamba init zsh
    echo "Please restart your shell and re-run"
    exit 1
fi
brew install cmake

# See: https://robostack.github.io/GettingStarted.html
mamba create -n ros_env
mamba activate ros_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults

mamba install compilers cmake pkg-config make opencv assimp bullet glfw fmt
mamba install catkin_tools ros-noetic-ros-base ros-noetic-rviz ros-noetic-xacro ros-noetic-robot-localization

git submodule update --init
mamba deactivate
./scripts/build_dawn.sh

mamba activate ros_env
pip3 install -e ".[dev]"
