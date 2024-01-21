#!/bin/bash

# See: https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

if ! command -v mamba; then
    brew install miniforge
    mamba init zsh
    echo "Please restart your shell and re-run"
    exit 1
fi

if ! command -v git-lfs; then
    brew install git-lfs
    git lfs install
fi

if ! conda info --envs | grep -q ros_env; then
    # See: https://robostack.github.io/GettingStarted.html
    mamba create python=3.9 -n ros_env
    mamba activate ros_env
    conda config --env --add channels conda-forge
    conda config --env --add channels robostack-staging
    conda config --env --remove channels defaults || true
else
    mamba activate ros_env
fi

mamba install catkin_tools ros-noetic-ros-base ros-noetic-rviz ros-noetic-xacro ros-noetic-robot-localization
mamba install compilers ccache ninja cmake pkg-config make opencv assimp bullet glfw fmt

# git submodule update --init deps/dawn
./scripts/build_dawn.sh

pip3 install -e ".[dev]"
