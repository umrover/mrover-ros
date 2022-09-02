#!/usr/bin/env python3

import subprocess
import sys

from typing import List
from pathlib import Path

BASE_APT_DEPS = ['curl', 'git', 'git-lfs', 'python3-pip']
ROS_APT_DEPS = ['ros-noetic-desktop', 'python3-catkin-tools', 'python3-rosdep']

DEFAULT_CATKIN_PATH = Path.home() / 'catkin_ws'


def run_command(args: List[str], **kwargs):
    check = input(f'Run: \"{" ".join(args)}\" [Y/n] ')
    if check not in {'N', 'n'}:
        # TODO: make stderr red
        process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            **kwargs)
        while process.stdout.readable():
            line = process.stdout.readline()
            if not line:
                break
            print(line.decode('utf-8'), end='')


def main() -> int:
    print('Checking for global system updates... this may take a while after a fresh install...')
    run_command(['sudo', 'apt-get', 'update', '-y'])
    run_command(['sudo', 'apt-get', 'upgrade', '-y'])
    print('Checking general system packages...')
    run_command(['sudo', 'apt-get', 'install'] + BASE_APT_DEPS + ['-y'])
    print('Checking ROS setup...')
    run_command(['sudo sh -c \'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list\''], shell=True)
    run_command(['curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -'], shell=True)
    print('Checking ROS packages... this also may take a while after fresh install...')
    run_command(['sudo', 'apt-get', 'update', '-y'])
    run_command(['sudo', 'apt-get', 'install'] + ROS_APT_DEPS + ['-y'])
    print('Checking ROS workspace...')
    workplace_path = input(f'Enter path to ROS workspace... [leave blank for {DEFAULT_CATKIN_PATH}] ')
    workplace_path = Path(workplace_path) if workplace_path else DEFAULT_CATKIN_PATH
    workplace_src_path = workplace_path / 'src'
    workplace_src_path.mkdir(exist_ok=True, parents=True)
    print(f'Selecting workspace path: {workplace_path}')
    run_command(['git', 'clone', 'https://github.com/umrover/mrover-ros.git','mrover'], cwd=workplace_src_path)
    run_command(['catkin', 'init'], cwd=workplace_path)
    print('Checking MRover dependency packages...')
    run_command(['sudo', 'rosdep', 'init'])
    run_command(['rosdep', 'update'])
    run_command(['rosdep', 'install', '--from-paths', str(workplace_src_path), '--ignore-src', '-y', '--rosdistro=noetic'])
    run_command(['catkin', 'build'], cwd=workplace_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
