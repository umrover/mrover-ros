#!/bin/bash
# copy_arm_pkg.sh

# Stop on errors
# See https://vaneyckt.io/posts/safer_bash_scripts_with_set_euxo_pipefail/
set -Eeuo pipefail

usage() {
    echo "Usage: $0 package direction"
    echo -e "    package: mrover_arm | sa_arm"
    echo -e "    direction:"
    echo -e "\tin: Copy from catkin workspace to mrover package"
    echo -e "\tout: Copy from mrover package to catkin workspace"
}

if [ $# -ne 2 ]; then
    usage
    exit 1
fi

rospack find mrover > /dev/null
declare mrover_pkg=`rospack find mrover`

# arg1: name of arm package
# arg2: name of directory containing the stl files
cp_in() {
    # verify args
    echo "$1 $2" > /dev/null

    # check that source directories exist
    if [ ! -d $mrover_pkg/../$1/ ]; then
        echo "Error: $1 package doesn't exist in catkin_ws/src/"
        exit 1
    fi

    if [ ! -d $mrover_pkg/../$1/rover_description/ ]; then
        echo "Error: $1 package is missing rover_description/"
        exit 1
    fi

    # remove existing directories
    if [ -d $mrover_pkg/src/teleop/$1/ ]; then
        rm -r $mrover_pkg/src/teleop/$1/
    fi

    if [ -d $mrover_pkg/rover_description/$2/ ]; then
        rm -r $mrover_pkg/rover_description/$2/
    fi

    # copy directories
    cp -r $mrover_pkg/../$1/. $mrover_pkg/src/teleop/$1/
    cp -r $mrover_pkg/src/teleop/$1/rover_description/. $mrover_pkg/rover_description/$2/
    rm -r $mrover_pkg/src/teleop/$1/rover_description/
}

# arg1: name of arm package
# arg2: name of directory containing the stl files
cp_out() {
    # verify args
    echo "$1 $2" > /dev/null

    # check that source directories exist
    if [ ! -d $mrover_pkg/src/teleop/$1/ ]; then
        echo "Error: $1/ directory doesn't exist in src/teleop/"
        exit 1
    fi

    if [ ! -d $mrover_pkg/rover_description/$2/ ]; then
        echo "Error: $2/ directory doesn't exist in rover_description/"
        exit 1
    fi

    # remove existing package
    if [ -d $mrover_pkg/../$1/ ]; then
        rm -r $mrover_pkg/../$1/
    fi

    # copy directories
    cp -r $mrover_pkg/src/teleop/$1/. $mrover_pkg/../$1/
    cp -r $mrover_pkg/rover_description/$2/. $mrover_pkg/../$1/rover_description/
}

# Parse argument.  $1 is the first argument
case $2 in
    "in")
        case $1 in
            "mrover_arm")
                cp_in mrover_arm arm
                ;;
            "sa_arm")
                cp_in mrover_sa_arm sa_arm
                ;;
            *)
                usage
                exit 1
                ;;
        esac
        ;;

    "out")
        case $1 in
            "mrover_arm")
                cp_out mrover_arm arm
                ;;
            "sa_arm")
                cp_out mrover_sa_arm sa_arm
                ;;
            *)
                usage
                exit 1
                ;;
        esac
        ;;
    
    *)
        usage
        exit 1
        ;;

esac
