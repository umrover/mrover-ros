#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path
from typing import List

BASE_APT_DEPS = ['curl', 'vim', 'git', 'git-lfs', 'python3-pip']
ROS_APT_DEPS = ['ros-noetic-desktop', 'python3-catkin-tools', 'python3-rosdep']

MROVER_ROS_GIT_URL = 'https://github.com/umrover/mrover-ros.git'
DEFAULT_CATKIN_PATH = Path.home() / 'catkin_ws'

ROS_APT_KEY_URL = 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc'
ROS_APT_SOURCE = 'deb https://packages.ros.org/ros/ubuntu focal main'


def run_bash_command(args: List[str], **kwargs):
    """
    Ask if a command should be run.
    Based on that exit or run it in the terminal environment.
    """
    check = input(f'Run: \"{" ".join(args)}\" [Y/n] ')
    if check not in {'N', 'n'}:
        process = subprocess.Popen(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,  # Combine stderr and stdout
            **kwargs)
        while process.stdout.readable():
            line = process.stdout.readline()
            if not line:
                break
            print(line.decode('utf-8'), end='')


def main() -> int:
    """
    Setup script to download the MRover repository and setup all dependencies.
    """
    try:
        print('Checking for global system updates... this may take a while after a fresh install...')
        run_bash_command(['sudo', 'apt-get', 'update', '-y'])
        run_bash_command(['sudo', 'apt-get', 'upgrade', '-y'])
        print('Checking general system packages...')
        run_bash_command(['sudo', 'apt-get', 'install'] + BASE_APT_DEPS + ['-y'])
        print('Checking ROS setup...')
        run_bash_command([f'sudo sh -c \'echo "{ROS_APT_SOURCE}" > /etc/apt/sources.list.d/ros-latest.list\''],
                         shell=True)
        run_bash_command([f'curl -s {ROS_APT_KEY_URL} | sudo apt-key add -'], shell=True)
        print('Checking ROS packages... this also may take a while after fresh install...')
        run_bash_command(['sudo', 'apt-get', 'update', '-y'])
        run_bash_command(['sudo', 'apt-get', 'install'] + ROS_APT_DEPS + ['-y'])
        print('Checking ROS workspace...')
        workplace_path = input(f'Enter path to ROS workspace... [leave blank for {DEFAULT_CATKIN_PATH}] ')
        workplace_path = Path(workplace_path) if workplace_path else DEFAULT_CATKIN_PATH
        workplace_src_path = workplace_path / 'src'
        workplace_src_path.mkdir(exist_ok=True, parents=True)
        print(f'Selecting workspace path: {workplace_path}')
        run_bash_command(['git', 'clone', MROVER_ROS_GIT_URL, 'mrover'], cwd=workplace_src_path)
        run_bash_command(['catkin', 'init'], cwd=workplace_path)
        print('Checking MRover dependency packages...')
        run_bash_command(['sudo', 'rosdep', 'init'])
        run_bash_command(['rosdep', 'update'])
        run_bash_command(
            ['rosdep', 'install', '--from-paths', str(workplace_src_path), '--ignore-src', '-y', '--rosdistro=noetic'])
        run_bash_command(['pip3', 'install', '-r', 'requirements.txt'], cwd=workplace_src_path)
        print(f'Repository and dependency setup successful! Now do:\n'
              f'\tcd {workplace_src_path}\n'
              '\tsource /opt/ros/noetic/setup.bash\n'
              f'\tsource {workplace_path}/devel/setup.bash\n'
              '\tcatkin build')
        return 0
    except Exception as exception:
        print(exception, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
