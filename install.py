#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path
from typing import List

BASE_APT_DEPS = ["curl", "vim", "zsh", "git", "git-lfs", "python3-pip", "clang-format-12", "clang-tidy-12", "nodejs", "yarn"]
ROS_APT_DEPS = ["ros-noetic-desktop", "python3-catkin-tools", "python3-rosdep"]
DRONE_ROS_APT_DEPS = [f'ros-noetic-{dep}' for dep in ["mavlink", "mavros", "mavros-extras", "mavros-msgs"]]
DRONE_ROS_APT_DEPS += ["gstreamer1.0-plugins-bad", "gstreamer1.0-plugins-base", "gstreamer1.0-plugins-good",
                       "gstreamer1.0-plugins-ugly", "gstreamer1.0-libav", "libeigen3-dev", "libgstreamer-plugins-base1.0-dev",
                       "libimage-exiftool-perl", "libopencv-dev", "libxml2-utils", "pkg-config", "protobuf-compiler",
                       "libqt5gui5", "libfuse2"]

MROVER_ROS_GIT_URL = "https://github.com/umrover/mrover-ros.git"
PX4_GIT_URL = "https://github.com/px4/px4-autopilot"
WIKI_URL = "https://github.com/umrover/mrover-ros/wiki"
GROUND_CONTROL_URL = "https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage"
DEFAULT_GROUND_CONTROL_PATH = Path.home() / "Downloads"
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
        install_drone_deps = False
        if len(sys.argv) > 1 and sys.argv[1] == '--install-drone':
            install_drone_deps = True
            print("Install script will install drone dependencies")
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
        if install_drone_deps:
            run_bash_command(["git", "clone", "--progress", "--recursive", PX4_GIT_URL, "px4"], cwd=workspace_src_path)
        run_bash_command(["catkin", "init"], cwd=workspace_path)
        print("Checking MRover dependency packages...")
        run_bash_command(["sudo", "rosdep", "init"])
        run_bash_command(["rosdep", "update"])
        run_bash_command(
            ["rosdep", "install", "--from-paths", str(workspace_src_path), "--ignore-src", "-y", "--rosdistro=noetic"]
        )
        mrover_repo_path = workspace_src_path / "mrover"
        run_bash_command(["pip3", "install", "-r", "requirements.txt"], cwd=mrover_repo_path)
        if install_drone_deps:
            print("Installing drone dependencies...")
            run_bash_command(["sudo", "apt-get", "install", "-y"] + DRONE_ROS_APT_DEPS)
            print("Installing GeographicLib Datasets")
            run_bash_command(["wget -O - https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash"], shell=True)
            print("Granting permission to the serial ports...")
            run_bash_command(["sudo usermod -a -G dialout $USER"], shell=True)
            run_bash_command(["sudo", "apt-get", "remove", "modemmanager", "-y"])
            print("Downloading ground control software...")
            qgc_dir = input(f"Enter download location for QGroundControl software... [leave blank for {DEFAULT_GROUND_CONTROL_PATH}] ")
            qgc_dir = Path(qgc_dir) if qgc_dir else DEFAULT_GROUND_CONTROL_PATH
            qgc_path = qgc_dir/"QGroundControl"
            run_bash_command(["wget", GROUND_CONTROL_URL, "-O", str(qgc_path)])
            run_bash_command(["chmod", "+x", str(qgc_path)])

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
