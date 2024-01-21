#!/bin/bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

if ! command -v mamba; then
    brew install mambaforge
    mamba init zsh
    echo "Please restart your shell and re-run"
    exit 1
fi

brew install git-lfs ccache cmake ninja
git lfs install

mamba deactivate
./scripts/build_dawn.sh

# See: https://robostack.github.io/GettingStarted.html
mamba create python=3.9 -n ros_env
mamba activate ros_env
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults || true

mamba install catkin_tools ros-noetic-ros-base ros-noetic-rviz ros-noetic-xacro ros-noetic-robot-localization
mamba install compilers cmake pkg-config make opencv assimp bullet glfw fmt

git submodule update --init

mamba activate ros_env
pip3 install -e ".[dev]"

