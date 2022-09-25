#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path
from typing import List

BASE_APT_DEPS = ["curl", "vim", "zsh", "git", "git-lfs", "python3-pip", "clang-format-12", "clangd-12", "nodejs", "yarn"]
ROS_APT_DEPS = ["ros-noetic-desktop", "python3-catkin-tools", "python3-rosdep"]

MROVER_ROS_GIT_URL = "https://github.com/umrover/mrover-ros.git"
WIKI_URL = "https://github.com/umrover/mrover-ros/wiki"
DEFAULT_CATKIN_PATH = Path.home() / "catkin_ws"

ROS_APT_KEY_URL = "https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc"
ROS_APT_SOURCE = "deb http://packages.ros.org/ros/ubuntu focal main"

NODE_SCRIPT_URL = "https://deb.nodesource.com/setup_18.x"
YARN_APT_KEY_URL = "https://dl.yarnpkg.com/debian/pubkey.gpg"
YARN_APT_SOURCE = "deb https://dl.yarnpkg.com/debian/ stable main"


def run_bash_command(args: List[str], **kwargs):
    """
    Ask if a command should be run.
    Based on that exit or run it in the terminal environment.
    """
    check = input(f'Run: "{" ".join(args)}" [Y/n] ')
    if check not in {"N", "n"}:
        process = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **kwargs  # Combine stderr and stdout
        )
        while process.stdout.readable():
            line = process.stdout.readline()
            if not line:
                break
            print(line.decode("utf-8"), end="")


def main() -> int:
    """
    Setup script to download the MRover repository and setup all dependencies.
    """

    def update_package_list():
        run_bash_command(["sudo", "apt-get", "update", "-y"])

    try:
        print("Checking for global system updates... this may take a while after a fresh install...")
        update_package_list()
        run_bash_command(["sudo", "apt-get", "upgrade", "-y"])
        print("Checking general system packages...")
        run_bash_command(["sudo", "add-apt-repository", "ppa:git-core/ppa"])  # Latest version of Git
        run_bash_command([f"curl -fsSL {NODE_SCRIPT_URL} | sudo -E bash -"], shell=True)
        run_bash_command([f"curl -sS {YARN_APT_KEY_URL} | sudo apt-key add -"], shell=True)
        run_bash_command([f'echo "{YARN_APT_SOURCE}" | sudo tee /etc/apt/sources.list.d/yarn.list'], shell=True)
        update_package_list()
        run_bash_command(["sudo", "apt-get", "install"] + BASE_APT_DEPS + ["-y"])
        print("Checking ROS setup...")
        run_bash_command([f'echo "{ROS_APT_SOURCE}" | sudo tee /etc/apt/sources.list.d/ros-latest.list'], shell=True)
        run_bash_command([f"curl -s {ROS_APT_KEY_URL} | sudo apt-key add -"], shell=True)
        print("Checking ROS packages... this also may take a while after fresh install...")
        update_package_list()
        run_bash_command(["sudo", "apt-get", "install"] + ROS_APT_DEPS + ["-y"])
        print("Checking ROS workspace...")
        workspace_path = input(f"Enter path to ROS workspace... [leave blank for {DEFAULT_CATKIN_PATH}] ")
        workspace_path = Path(workspace_path) if workspace_path else DEFAULT_CATKIN_PATH
        workspace_src_path = workspace_path / "src"
        workspace_src_path.mkdir(exist_ok=True, parents=True)
        print(f"Selecting workspace path: {workspace_path}")
        run_bash_command(["git", "clone", "--progress", MROVER_ROS_GIT_URL, "mrover"], cwd=workspace_src_path)
        run_bash_command(["catkin", "init"], cwd=workspace_path)
        print("Checking MRover dependency packages...")
        run_bash_command(["sudo", "rosdep", "init"])
        run_bash_command(["rosdep", "update"])
        run_bash_command(
            ["rosdep", "install", "--from-paths", str(workspace_src_path), "--ignore-src", "-y", "--rosdistro=noetic"]
        )
        mrover_repo_path = workspace_src_path / "mrover"
        run_bash_command(["pip3", "install", "-r", "requirements.txt"], cwd=mrover_repo_path)
        print(
            f"\nRepository and dependency setup successful! Now do:\n"
            f"\tcd {mrover_repo_path}\n"
            "\tsource /opt/ros/noetic/setup.bash\n"
            "\tcatkin build\n"
            f"\tsource {workspace_path}/devel/setup.bash"
        )
        print(
            "\nNote you will have to run the source commands for each terminal session!\n"
            f"\tSee: {WIKI_URL}/2.-Install-ROS#install-ros-natively to modify .bashrc to automatically do this"
        )
        print("To run a basic demo run: roslaunch mrover full.launch")
        print("\nAnd finally welcome to MRover!")
        return 0
    except KeyboardInterrupt:
        print("Exiting...")
        return 0
    except Exception as exception:
        print(exception, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
